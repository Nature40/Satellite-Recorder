// VSCode hints includes are evaluated based on the OS running on, not the OS
// compiled for, therefore the apple variable needs to be undefinded for hints
// on macOS.
#undef __APPLE__

#include <Arduino.h>
#include <FreeRTOS.h>
#include <driver/adc.h>
#include <driver/i2s.h>
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

#define SAMPLE_RATE (500000)
#define SAMPLE_DEPTH I2S_BITS_PER_SAMPLE_16BIT
#define BUFFER_SIZE (50000)
#define BUFFER_BYTES (BUFFER_SIZE * (SAMPLE_DEPTH / 8))

// I2S driver
i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = SAMPLE_DEPTH,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    // .use_apll = 1,
    // .tx_desc_auto_clear = true,
    // .fixed_mclk = SAMPLE_RATE,
};

// wav info
wavfile_info_t info = {
    .audio_format = 1,
    .num_channels = 1,
    .sample_rate = SAMPLE_RATE,
    .byte_rate = SAMPLE_RATE * (SAMPLE_DEPTH / 8),
    .block_align = 2,
    .bits_per_sample = SAMPLE_DEPTH,
};

// sd mount options
esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024,
};

typedef struct {
    WAVFILE *wav;
    time_t duration_s;
} i2s_record_params_t;

void i2s_record(void *arg) {
    i2s_record_params_t *rec = (i2s_record_params_t *)arg;

    uint16_t i2s_read_buff[BUFFER_SIZE];
    size_t bytes_read;

    size_t bytes_missing = rec->duration_s * SAMPLE_RATE * SAMPLE_DEPTH / 8;
    log_i("Recording %u seconds, %lu bytes, buffer %p", rec->duration_s,
          bytes_missing, i2s_read_buff);

    i2s_adc_enable(I2S_NUM_0);

    while (bytes_missing > 0) {
        // record BUFFER_BYTES at maximum or the missing bytes
        bytes_read =
            (bytes_missing < BUFFER_BYTES) ? bytes_missing : BUFFER_BYTES;

        log_i("trying to read %u bytes", bytes_read);

        // read the values
        i2s_read(I2S_NUM_0, &i2s_read_buff, bytes_read, &bytes_read,
                 portMAX_DELAY);

        log_i("read %u bytes via I2S, samples: %i %i %i ... %i", bytes_read,
              i2s_read_buff[0], i2s_read_buff[1], i2s_read_buff[2],
              i2s_read_buff[(bytes_read / (SAMPLE_DEPTH * 8)) - 1]);

        // write bytes to sd card
        fwrite(i2s_read_buff, bytes_read, 1, rec->wav->fp);
        rec->wav->data_byte_count += bytes_read;
        rec->wav->data_checked = true;

        bytes_missing -= bytes_read;
    }

    i2s_adc_disable(I2S_NUM_0);

    log_i("Recording finished, wrote %u bytes total",
          rec->wav->data_byte_count);
    wavfile_close(rec->wav);
    free(rec);

    vTaskDelete(NULL);
}

TaskHandle_t start_record(char *file_path, time_t duration_s) {
    TaskHandle_t task = NULL;
    WAVFILE *wav;
    WavFileResult error;
    char error_buf[BUFSIZ];

    // init wavfile
    wav = wavfile_open(file_path, WavFileModeWrite, &error);
    if (error) {
        wavfile_result_string(error, error_buf, BUFSIZ);
        log_e("error opening wav file: %s", error_buf);
        return task;
    }

    error = wavfile_write_info(wav, &info);
    if (error) {
        wavfile_result_string(error, error_buf, BUFSIZ);
        log_e("error writing wav info: %s", error_buf);
        return task;
    }

    // install and start i2s driver
    esp_log_level_set("I2S", ESP_LOG_INFO);
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

    // init ADC pad
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);

    // init array
    i2s_record_params_t *params =
        (i2s_record_params_t *)malloc(sizeof(i2s_record_params_t));

    params->duration_s = duration_s;
    params->wav = wav;

    char task_name[BUFSIZ];
    snprintf(task_name, BUFSIZ, "i2s_record(%lu, \"%s\")", duration_s,
             file_path);

    // start actual recording task
    log_i("Starting recording task: %s", task_name);
    BaseType_t task_status = xTaskCreate(i2s_record, "task_name",
                                         BUFFER_BYTES + 2048, params, 5, &task);
    if (task_status != pdPASS)
        log_e("Record task failed, check memory.");

    return task;
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

    // setup SDMMC
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // mount FS
    sdmmc_card_t *card;
    esp_err_t err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config,
                                            &mount_config, &card);
    if (err) {
        const char *err_msg = esp_err_to_name(err);
        log_e("SD card mount failed (%i): %s\n", err, err_msg);
        return;
    } else {
        sdmmc_card_print_info(stdout, card);
    }

    // find wav path
    char wavFilePath[128];
    if (findWavPath(wavFilePath, 128)) {
        log_e("Could not find wav path.");
    }

    // start recording
    TaskHandle_t rec_task = start_record(wavFilePath, 5);

    log_i("Init completed.");
}

void loop() {
    // put your main code here, to run repeatedly:
}