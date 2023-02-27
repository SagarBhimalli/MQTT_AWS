
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
/* Application headers */

#include "smartap_ota.h"
#include "smartap_common.h"
#include "smartap_mqttClient.h"

/* Standard headers */
#include <string.h>
#include <stdbool.h>
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "errno.h"
//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
#define CONFIG_EXAMPLE_OTA_RECV_TIMEOUT 5000
#define CONFIG_EXAMPLE_GPIO_DIAGNOSTIC 4

//--------------------------------------------------------------------------
// Typedef
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Static Variables
//--------------------------------------------------------------------------
/* global variables */

static const char *TAG = "STAP_App_OTA";
static char writeOtaData[BUFFSIZE + 1] = {0};

char *otaUrl = "https://smarttouchfwfile.s3.ap-south-1.amazonaws.com/smartap_4M/smartp.bin"; // AWS s3 bucket

bool avoidOta = false;
bool ledStatus = true;

//--------------------------------------------------------------------------
// Function Definition
//--------------------------------------------------------------------------

/**
 * @brief Initialize http_cleanup module.
 *
 * @param client Http client which needs to be cleaned and close connection
 */

static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

/**
 * @brief Initialize Secure Hash Algorithm.
 *
 * @param image_hash data from partation
 * @param label Name of data that needs to be printed
 */
static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
}

/**
 * @brief - Check for new application update via OTA.
 *        - Initialize HTTP client, check for running and boot partation, get data and write to second partation.
 *        - Change boot partation, and restart the device.
 */
void checkForOta(void) // Check OTA via Wi-Fi
{
    esp_err_t err;
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA");
    // get partition in which from device is boot-up.
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    // get currently running partition
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)", running->type, running->subtype, running->address);

    esp_http_client_config_t config =
        {
            .url = otaUrl,
            .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
            .keep_alive_enable = true,
        }; // http client configuration

    esp_http_client_handle_t client = esp_http_client_init(&config); // http client initilize
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        return;
    }
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        ESP_LOGE(TAG, "Skipping OTA");
    }
    if (err == ESP_OK)
    {
        esp_http_client_fetch_headers(client);

        // get the partition in which new image(firmware) will be downloaded and device will boot-up from this partition
        update_partition = esp_ota_get_next_update_partition(NULL);
        assert(update_partition != NULL);
        ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x", update_partition->subtype, update_partition->address);

        int binaryFileLenght = 0;
        bool isImageHeaderChecked = false;
        while (1)
        {
            // read data from http stream, first it will read header data
            int readHttpData = esp_http_client_read(client, writeOtaData, BUFFSIZE);
            if (readHttpData < 0)
            {
                ESP_LOGE(TAG, "Error: SSL data read error");
                http_cleanup(client);
            }
            else if (readHttpData > 0)
            {
                if (isImageHeaderChecked == false)
                {
                    esp_app_desc_t new_app_info;
                    ESP_LOGI(TAG, "Size of esp image header: %d", sizeof(esp_image_header_t));
                    ESP_LOGI(TAG, "esp_image_segment_header: %d", sizeof(esp_image_segment_header_t));
                    ESP_LOGI(TAG, "esp_app_desc: %d", sizeof(esp_app_desc_t));

                    if (readHttpData > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                    {
                        memcpy(&new_app_info, &writeOtaData[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));

                        ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                        esp_app_desc_t running_app_info;
                        // get description of partition from which the current application is running
                        if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                        {
                            ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                        }

                        const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                        esp_app_desc_t invalid_app_info;
                        if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                        {
                            ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                        }

                        if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                        {
                            ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                            checkForOtaPacket();
                            avoidOta = true;
                            break;
                        }
                        else
                        {
                            avoidOta = false;
                            isImageHeaderChecked = true;
                            // if available version on aws, diffrent from current running version OTA will begin
                            err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                            if (err != ESP_OK)
                            {
                                ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                                http_cleanup(client);
                            }
                            ESP_LOGI(TAG, "OTA succeeded");
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "received package is not fit len");
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                    }
                }
                if (avoidOta == false)
                {
                    // Writing OTA update data to next partition
                    err = esp_ota_write(update_handle, (const void *)writeOtaData, readHttpData);
                    if (err != ESP_OK)
                    {
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                    }
                    binaryFileLenght += readHttpData;
                    ESP_LOGD(TAG, "Written image length %d", binaryFileLenght);
                    ledStatus = !ledStatus;
                    gpio_set_level(GPIO_OUT_STS_LED, ledStatus);
                }
            }
            else if (readHttpData == 0 && avoidOta == false)
            {
                if (errno == ECONNRESET || errno == ENOTCONN)
                {
                    ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                    break;
                }
                if (esp_http_client_is_complete_data_received(client) == true)
                {
                    ESP_LOGI(TAG, "Connection closed");
                    break;
                }
            }
        }
        if (avoidOta == false)
        {
            ESP_LOGI(TAG, "Total Write binary data length: %d", binaryFileLenght);
            if (esp_http_client_is_complete_data_received(client) != true)
            {
                ESP_LOGE(TAG, "Error in receiving complete file");
                http_cleanup(client);
                esp_ota_abort(update_handle);
            }

            // If CRC bytes matches, OTA will be end.
            err = esp_ota_end(update_handle);
            if (err != ESP_OK)
            {
                if (err == ESP_ERR_OTA_VALIDATE_FAILED)
                {
                    ESP_LOGE(TAG, "Image validation failed, image is corrupted");
                }
                else
                {
                    ESP_LOGE(TAG, "OTA failed (%s)!", esp_err_to_name(err));
                }
                http_cleanup(client);
            }

            // Setting up pointer to next partition table in which new OTA update is downloaded as boot partition
            err = esp_ota_set_boot_partition(update_partition);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
                http_cleanup(client);
            }
            ESP_LOGW(TAG, "OTA done Successfully");
            ESP_LOGI(TAG, "Preparing to restart system!");
            // Device will restart and runs on new firmware
            esp_restart();
            return;
        }
    }
}

/**
 * @brief diagnose gpio for Secure Hash Algorithm 256.
 *
 */
static bool diagnostic(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Diagnostics (5 sec)...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    bool diagnostic_is_ok = gpio_get_level(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);

    gpio_reset_pin(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    return diagnostic_is_ok;
}

/**
 * @brief Get Secure Hash Algorithm 256 for bootloader, partation table and ota partation and diagnose.
 *
 */
void otaPartitionInit(void) // Setting up Partition for OTA
{
    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    // get sha256 digest for the partition table
    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for the partition table: ");

    // get sha256 digest for bootloader
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // get sha256 digest for running partition
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            bool diagnostic_is_ok = diagnostic();
            if (diagnostic_is_ok)
            {
                ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();
            }
            else
            {
                ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }
}
//--------------------------------------------------------------------------
// End of File
//--------------------------------------------------------------------------