// VSCode hints includes are evaluated based on the OS running on, not the OS
// compiled for, therefore the apple variable needs to be undefinded for hints
// on macOS.
#undef __APPLE__

#include <Arduino.h>
#include <FreeRTOS.h>
#include <driver/adc.h>
#include <driver/sdmmc_host.h>
#include <driver/sdspi_host.h>
#include <esp_err.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <sys/stat.h>
#include <unistd.h>

#include "wavfile.h"

// Timer
portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t writeWavTask;
hw_timer_t *adcTimer = NULL;

// ADC values
#define ADC_SAMPLES_COUNT 10000
#define ADC_SAMPLES_FLUSH 1000
int abuf[ADC_SAMPLES_COUNT];
int64_t abufPosWrite = 0;
int64_t abufPosRead = 0;

// Wavfile
WAVFILE *wf;
WavFileResult error;
char wavfile_error_buf[BUFSIZ];
int record_time_ms = 10000;

// SD card
#define MOSI gpio_num_t(15)
#define MISO gpio_num_t(2)
#define SCK gpio_num_t(14)
#define SD_CS gpio_num_t(13)

// https://www.toptal.com/embedded/esp32-audio-sampling
int IRAM_ATTR local_adc1_read(int channel) {
    uint16_t adc_value;
    SENS.sar_meas_start1.sar1_en_pad = (1 << channel);
    while (SENS.sar_slave_addr1.meas_status != 0)
        ;
    SENS.sar_meas_start1.meas1_start_sar = 0;
    SENS.sar_meas_start1.meas1_start_sar = 1;
    while (SENS.sar_meas_start1.meas1_done_sar == 0)
        ;
    adc_value = SENS.sar_meas_start1.meas1_data_sar;
    return adc_value;
}

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);

    // read adc and add to ringbuffer
    abuf[abufPosWrite % ADC_SAMPLES_COUNT] = local_adc1_read(ADC1_CHANNEL_0);
    abufPosWrite++;

    // flush sometimes
    if ((abufPosWrite % ADC_SAMPLES_FLUSH) == 0) {
        log_d("sample: %d, target 2048",
              abuf[(abufPosWrite - 1) % ADC_SAMPLES_COUNT]);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(writeWavTask, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }

    // if ((abufPosWrite % ADC_SAMPLES_COUNT) == 0) {
    //     log_d("read %d samples", abufPosWrite);
    // }

    portEXIT_CRITICAL_ISR(&timerMux);
}

void writeWav(void *param) {
    wavfile_data_t wav_buf;
    wav_buf.num_channels = 1;

    while (true) {
        // Sleep until notification (one second timeout)
        // uint32_t tcount =
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));

        // if the buffer contains new values, read
        while (abufPosRead < abufPosWrite) {
            // write data to wav
            int val = abuf[abufPosRead % ADC_SAMPLES_COUNT];
            wav_buf.channel_data[0] = (((double)val) / 0x0FFF);
            WavFileResult error = wavfile_write_data(wf, &wav_buf);

            if (error) {
                wavfile_result_string(error, wavfile_error_buf, BUFSIZ);
                log_e("error writing data: %s", wavfile_error_buf);
                timerAlarmDisable(adcTimer);
            }

            abufPosRead++;

            if ((abufPosRead % ADC_SAMPLES_COUNT) == 0) {
                // log_d("wrote %d samples", abufPosRead);
                // log_d("Example: %d", wav_buf.channel_data[0]);
            }

            // end recording after some time
            if (millis() > record_time_ms) {
                log_i("Recording finished.");
                timerAlarmDisable(adcTimer);
                wavfile_close(wf);
                return;
            }
        }
    }
}

int findWavPath(char *wavFilePath, size_t len) {
    for (int i = 0; i < 10000; i++) {
        snprintf(wavFilePath, 128, "/sdcard/recording_%04u.wav", i);

        if (access(wavFilePath, F_OK) == -1) {
            // file doesn't exists
            log_i("found %s", wavFilePath);
            return 0;
        } else {
            log_d("skip %s, exists", wavFilePath);
        }
    }

    return 1;
}

void setup() {
    // setup serial output
    Serial.begin(115200);
    log_v("ESP32 Recorder");

    // SD SPI config
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = MISO;
    slot_config.gpio_mosi = MOSI;
    slot_config.gpio_sck = SCK;
    slot_config.gpio_cs = SD_CS;

    // FS config
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};

    sdmmc_card_t *card;
    esp_err_t err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config,
                                            &mount_config, &card);
    if (err) {
        const char *err_msg = esp_err_to_name(err);
        log_e("SD card mount failed (%i): %s\n", err, err_msg);
        return;
    }

    // find wav path
    char wavFilePath[128];
    if (findWavPath(wavFilePath, 128)) {
        log_e("Could not find wav path.");
    }

    // init wavfile
    wavfile_info_t info;
    info.audio_format = 1;
    WAVFILE_INFO_AUDIO_FORMAT(&info) = 1;
    WAVFILE_INFO_NUM_CHANNELS(&info) = 1;
    WAVFILE_INFO_SAMPLE_RATE(&info) = 20000;
    WAVFILE_INFO_BYTE_RATE(&info) = 40000;
    WAVFILE_INFO_BLOCK_ALIGN(&info) = 2;
    WAVFILE_INFO_BITS_PER_SAMPLE(&info) = 16;

    wf = wavfile_open(wavFilePath, WavFileModeWrite, &error);
    if (error) {
        wavfile_result_string(error, wavfile_error_buf, BUFSIZ);
        log_e("error opening wav file: %s", wavfile_error_buf);
        return;
    }

    error = wavfile_write_info(wf, &info);
    if (error) {
        wavfile_result_string(error, wavfile_error_buf, BUFSIZ);
        log_e("error writing wav info: %s", wavfile_error_buf);
        return;
    }

    // configure adc
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);
    // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_get_raw(ADC1_CHANNEL_0);

    // ## Setup sound recorder
    xTaskCreate(writeWav, "Write WAV", 8192, NULL, 1, &writeWavTask);

    // 80 MHz / 80 = 1 MHz hardware clock for easy figuring
    adcTimer = timerBegin(3, 80, true);

    // Attaches the handler function to the timer
    timerAttachInterrupt(adcTimer, &onTimer, true);

    // Interrupts when counter == 50, i.e. 20.000 times a second
    timerAlarmWrite(adcTimer, 50, true);
    timerAlarmEnable(adcTimer);

    log_i("Init completed.");
}

void loop() {
    // put your main code here, to run repeatedly:
}