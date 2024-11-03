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
    init_nvs();

    init_wifi();

    init_time_sync();

    init_custom_mqtt();

    // Infinite loop to prevent exiting app_main
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow other tasks to run
    }
}
