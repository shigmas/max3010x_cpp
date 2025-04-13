#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "Max3010x.h"

#define DEFAULT_ITERATIONS 32
/* Defines */
#define TEMP_RATE_TASK_PERIOD (1000 / portTICK_PERIOD_MS)

static const char *TAG = "Max3010x";

extern "C" {
    void app_main(void);
}

Max3010x m;

static void temp_read_task(void *param) {
    int iterations = DEFAULT_ITERATIONS;

    ESP_LOGI(TAG, "I2C iterating: %d", iterations);
    for (int i = 0 ; i < iterations ; i++) {
        ESP_LOGI(TAG, "reading temp: %d", i);
        float temp = m.ReadTemperature();
        ESP_LOGI(TAG, "I2C read temp: %f", temp);
        vTaskDelay(TEMP_RATE_TASK_PERIOD);
    }

    /* Clean up at exit */
    vTaskDelete(NULL);
}    
void app_main(void)
{
    uint8_t powerLevel = 0x1F;
    uint8_t sampleAverage = 4;
    uint8_t ledMode = 3;
    int sampleRate = 400;
    int pulseWidth = 411;
    int adcRange = 4096;

    ESP_LOGI(TAG, "Verifying device");
    if (!m.VerifyDevice()) {
        ESP_LOGI(TAG, "Device failed verification. Exiting");
        return;
    }
        
    ESP_LOGI(TAG, "Setting up device");
    m.SetUpDevice(powerLevel, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    ESP_LOGI(TAG, "Starting read loop");
    xTaskCreate(temp_read_task, "Temp Read", 4*1024, NULL, 5, NULL);
    ESP_LOGI(TAG, "Returning from main");
}
