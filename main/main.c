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
#include "unity.h"
#include "driver/i2c.h"
#include "bh1750.h"

#define I2C_MASTER_SDA_IO 8       /*!< gpio number for I2C master data  */
#define I2C_MASTER_SCL_IO 9       /*!< gpio number for I2C master clock */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "PORCH_LIGHTS";
const char *device_name = CONFIG_WIFI_HOSTNAME;

TaskHandle_t ota_task_handle = NULL; // Task handle for OTA updating

esp_mqtt_client_handle_t mqtt_client_handle = NULL;

static bh1750_handle_t bh1750 = NULL;

char mac_address[18];

extern const uint8_t certificate[];
extern const uint8_t private_key[];
extern const uint8_t root_ca[];
const uint8_t *cert = certificate;
const uint8_t *key = private_key;
const uint8_t *ca = root_ca;

void error_stop_mqtt(esp_mqtt_client_handle_t mqtt_client)
{
    esp_err_t ret;

    ret = esp_mqtt_client_stop(mqtt_client);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "MQTT client stopped successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to stop MQTT client: %s", esp_err_to_name(ret));
    }

    // Optionally destroy the MQTT client to free up resources
    ret = esp_mqtt_client_destroy(mqtt_client);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "MQTT client destroyed successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to destroy MQTT client: %s", esp_err_to_name(ret));
    }
}

void error_stop_wifi()
{
    esp_err_t ret;

    // Stop Wi-Fi
    ret = esp_wifi_stop();
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Wi-Fi stopped successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to stop Wi-Fi: %s", esp_err_to_name(ret));
    }

    // Optionally deinitialize Wi-Fi
    ret = esp_wifi_deinit();
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Wi-Fi deinitialized successfully.");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to deinitialize Wi-Fi: %s", esp_err_to_name(ret));
    }
}

void error_reload()
{
    // Stop the MQTT client
    if (mqtt_client_handle != NULL)
    {
        error_stop_mqtt(mqtt_client_handle);
    }

    // Stop the Wi-Fi
    error_stop_wifi();

    // Restart the ESP32
    esp_restart();
}

void record_local_mac_address(char *mac_str)
{
    uint8_t mac[6];
    esp_err_t ret = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (ret == ESP_OK)
    {
        snprintf(mac_str, 18, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else
    {
        snprintf(mac_str, 18, "ERROR");
    }
}

void custom_handle_mqtt_event_connected(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    ESP_LOGI(TAG, "Custom handler: MQTT_EVENT_CONNECTED");
    int msg_id;

    msg_id = esp_mqtt_client_subscribe(client, CONFIG_MQTT_SUBSCRIBE_OTA_UPDATE_PORCH_LIGHTS_TOPIC, 0);
    ESP_LOGI(TAG, "Subscribed to topic %s, msg_id=%d", CONFIG_MQTT_SUBSCRIBE_OTA_UPDATE_PORCH_LIGHTS_TOPIC, msg_id);
}

void custom_handle_mqtt_event_disconnected(esp_mqtt_event_handle_t event)
{
    ESP_LOGI(TAG, "Custom handler: MQTT_EVENT_DISCONNECTED");
    if (ota_task_handle != NULL)
    {
        vTaskDelete(ota_task_handle);
        ota_task_handle = NULL;
    }
    // Reconnect logic
    int retry_count = 0;
    const int max_retries = 5;
    const int retry_delay_ms = 5000;
    esp_err_t err;
    esp_mqtt_client_handle_t client = event->client;

    // Check if the network is connected before attempting reconnection
    if (wifi_active())
    {
        do
        {
            ESP_LOGI(TAG, "Attempting to reconnect, retry %d/%d", retry_count + 1, max_retries);
            err = esp_mqtt_client_reconnect(client);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to reconnect MQTT client, retrying in %d seconds...", retry_delay_ms / 1000);
                vTaskDelay(pdMS_TO_TICKS(retry_delay_ms)); // Delay for 5 seconds
                retry_count++;
            }
        } while (err != ESP_OK && retry_count < max_retries);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to reconnect MQTT client after %d retries. Restarting", retry_count);
            error_reload();
        }
    }
    else
    {
        ESP_LOGE(TAG, "Network not connected, skipping MQTT reconnection");
        error_reload();
    }
}

bool extract_ota_url_from_event(esp_mqtt_event_handle_t event, char *local_mac_address, char *ota_url)
{
    bool success = false;
    cJSON *root = cJSON_Parse(event->data);
    cJSON *host_key = cJSON_GetObjectItem(root, local_mac_address);
    const char *host_key_value = cJSON_GetStringValue(host_key);

    if (!host_key || !host_key_value)
    {
        ESP_LOGW(TAG, "'%s' MAC address key not found in JSON", local_mac_address);
    }
    else
    {
        size_t url_len = strlen(host_key_value);
        strncpy(ota_url, host_key_value, url_len);
        ota_url[url_len] = '\0'; // Manually set the null terminator
        success = true;
    }

    cJSON_Delete(root); // Free JSON object
    return success;
}

void custom_handle_mqtt_event_ota(esp_mqtt_event_handle_t event, char *my_mac_address)
{
    if (ota_task_handle != NULL)
    {
        eTaskState task_state = eTaskGetState(ota_task_handle);
        if (task_state != eDeleted)
        {
            char log_message[256]; // Adjust the size according to your needs
            snprintf(log_message, sizeof(log_message),
                     "OTA task is already running or not yet cleaned up, skipping OTA update. task_state=%d",
                     task_state);

            ESP_LOGW(TAG, "%s", log_message);
            return;
        }
        else
        {
            // Clean up task handle if it has been deleted
            ota_task_handle = NULL;
        }
    }
    else
    {
        ESP_LOGI(TAG, "OTA task handle is NULL");
    }

    // Parse the message and get any URL associated with our MAC address
    assert(event->data != NULL);
    assert(event->data_len > 0);

    ota_config_t ota_config;
    ota_config.mqtt_client = event->client;

    if (!extract_ota_url_from_event(event, my_mac_address, ota_config.url))
    {
        ESP_LOGW(TAG, "OTA URL not found in event data");
        return;
    }

    // Pass the allocated URL string to the OTA task
    if (xTaskCreate(&ota_task, "ota_task", 8192, (void *)&ota_config, 5, &ota_task_handle) != pdPASS)
    {
        error_reload();
    }
}

void custom_handle_mqtt_event_data(esp_mqtt_event_handle_t event)
{

    ESP_LOGW(TAG, "Received topic %.*s", event->topic_len, event->topic);

    if (strncmp(event->topic, CONFIG_MQTT_SUBSCRIBE_OTA_UPDATE_PORCH_LIGHTS_TOPIC, event->topic_len) == 0)
    {
        // Use the global mac_address variable to pass the MAC address to the OTA function
        custom_handle_mqtt_event_ota(event, mac_address);
    }
    else
    {
        ESP_LOGE(TAG, "Un-Handled topic %.*s", event->topic_len, event->topic);
    }
}

void custom_handle_mqtt_event_error(esp_mqtt_event_handle_t event)
{
    ESP_LOGI(TAG, "Custom handler: MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_ESP_TLS)
    {
        ESP_LOGI(TAG, "Last ESP error code: 0x%x", event->error_handle->esp_tls_last_esp_err);
        ESP_LOGI(TAG, "Last TLS stack error code: 0x%x", event->error_handle->esp_tls_stack_err);
        ESP_LOGI(TAG, "Last TLS library error code: 0x%x", event->error_handle->esp_tls_cert_verify_flags);
    }
    else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
    {
        ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
    }
    else
    {
        ESP_LOGI(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
    }
    error_reload();
}

void app_main(void)
{
    bh1750_measure_mode_t cmd_measure;
    float bh1750_data;

    init_nvs();

    init_wifi();

    record_local_mac_address(mac_address);
    ESP_LOGW(TAG, "Burned-In MAC Address: %s", mac_address);

    init_time_sync();

    mqtt_set_event_connected_handler(custom_handle_mqtt_event_connected);
    mqtt_set_event_disconnected_handler(custom_handle_mqtt_event_disconnected);
    mqtt_set_event_data_handler(custom_handle_mqtt_event_data);
    mqtt_set_event_error_handler(custom_handle_mqtt_event_error);

    mqtt_config_t config = {.certificate = cert,
                            .private_key = key,
                            .root_ca = ca,
                            .broker_uri = CONFIG_AWS_IOT_ENDPOINT};

    mqtt_client_handle = init_mqtt(&config);

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");

    bh1750 = bh1750_create(I2C_MASTER_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    TEST_ASSERT_NOT_NULL_MESSAGE(bh1750, "BH1750 create returned NULL");

    bh1750_power_on(bh1750);
    cmd_measure = BH1750_CONTINUE_4LX_RES;

    // Infinite loop to prevent exiting app_main
    while (true)
    {
        ret = bh1750_set_measure_mode(bh1750, cmd_measure);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        vTaskDelay(30 / portTICK_PERIOD_MS);

        ret = bh1750_get_data(bh1750, &bh1750_data);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "bh1750 val(continuously mode): %f\n", bh1750_data);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay to allow other tasks to run
    }
}
