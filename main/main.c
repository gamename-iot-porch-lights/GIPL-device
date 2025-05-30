#include "cJSON.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h" // Include FreeRTOS timers header
#include "gecl-mqtt-manager.h"
#include "gecl-nvs-manager.h"
#include "gecl-ota-manager.h"
#include "gecl-time-sync-manager.h"
#include "gecl-wifi-manager.h"
#include "mbedtls/debug.h" // Add this to include mbedtls debug functions
#include "nvs_flash.h"
#include "error_handler.h"
#include "led_handler.h"
#include "mqtt_custom_handler.h"

static const char *TAG = "PORCH_LIGHTS";

void app_main(void)
{
    ESP_LOGI(TAG, "Init NVS");
    esp_err_t xEspErrRet = nvs_flash_init();
    if (xEspErrRet == ESP_ERR_NVS_NO_FREE_PAGES || xEspErrRet == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGE(TAG, "NVS partition error: %d", xEspErrRet);
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_LOGI(TAG, "Init WiFi");
    init_wifi();

    ESP_LOGI(TAG, "Init Time Sync");
    init_time_sync();

    ESP_LOGI(TAG, "Init MQTT");
    init_custom_mqtt();

    ESP_LOGI(TAG, "Init LED Handler");
    init_led_handler();

    ESP_LOGI(TAG, "Entering infinite loop");
    // Infinite loop to prevent exiting app_main
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow other tasks to run
    }
}
