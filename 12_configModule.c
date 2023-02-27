
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
#include "smartap_configModule.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
//#include "smartap_switchControl.h"
#include "Smart_Occupancy_SEnsor.h"
//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
#define CONFIG_MODULE SMART_TAP "_CONFIG"

#define FS_BASE_PATH "/spiflash"
#define FS_PARTITION_NAME "storage"
#define CFG_WIFI_STATUS FS_BASE_PATH "/wifi.cfg"
#define CFG_CONTROL_STATUS FS_BASE_PATH "/switch.cfg"
#define CFG_PIR_DELAY FS_BASE_PATH "/delay.cfg"
#define CFG_MODE_SELECT FS_BASE_PATH "/modes.cfg"
//--------------------------------------------------------------------------
// Static Variables
//--------------------------------------------------------------------------
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
//--------------------------------------------------------------------------
// Static Functions
//--------------------------------------------------------------------------
SENSOR_CONFIG_t sensorConfig;
//--------------------------------------------------------------------------
// Function Definations
//--------------------------------------------------------------------------
/**
 * @brief Initialize and Mount File system for reading/writing configuration
 *
 * @return esp_err_t
 */
esp_err_t InitConfigurationModule(void)
{
    esp_err_t retVal;

    ESP_LOGI(CONFIG_MODULE, "Initializing Configuration");
    ESP_LOGI(CONFIG_MODULE, "Mounting FAT filesystem");

    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 6,
        .format_if_mount_failed = true,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE};
    retVal = esp_vfs_fat_spiflash_mount(FS_BASE_PATH, FS_PARTITION_NAME, &mount_config, &s_wl_handle);
    if (retVal != ESP_OK)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to mount FATFS : %s", esp_err_to_name(retVal));
        return retVal;
    }

    ESP_LOGI(CONFIG_MODULE, "File System mounted");

    /* check if wifi config present */
    if (access(CFG_WIFI_STATUS, F_OK) != 0)
    {
        ESP_LOGW(CONFIG_MODULE, "Wifi configuration not available : writing default configuration");
        DefaultWifiConfig();
    }

    /* check if control status present */
    if (access(CFG_CONTROL_STATUS, F_OK) != 0)
    {
        ESP_LOGW(CONFIG_MODULE, "Control Status not available : writing default configuration");
        DefaultControlStatus();
    }

     /* check if mode select present */
    if (access(CFG_MODE_SELECT, F_OK) != 0)
    {
        ESP_LOGW(CONFIG_MODULE, "Mode select not available : writing default configuration");
        DefaultMode();
    }

     /* check if Delay present */
    if (access(CFG_PIR_DELAY, F_OK) != 0)
    {
        ESP_LOGW(CONFIG_MODULE, "Delay not available : writing default configuration");
        DefaultDelay();
    }

    

    return ESP_OK;
}

/**
 * @brief Unmount file system
 *
 */
void DeInitConfigurationModule(void)
{
    /* unmount file system */
    esp_vfs_fat_spiflash_unmount(FS_BASE_PATH, s_wl_handle);
}

/**
 * @brief Write given wifi configuration to file
 *
 * @param pWifiConfig
 * @return esp_err_t
 */
esp_err_t WriteWifiConfig(WIFI_CONFIG_t *pWifiConfig)
{
    ESP_LOGD(CONFIG_MODULE, "Writing Wifi Configuration : [ssid= %s], [passwd= %s]",
             pWifiConfig->ssid, pWifiConfig->password);

    FILE *pFile = NULL;
    pFile = fopen(CFG_WIFI_STATUS, "wb");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_WIFI_STATUS, strerror(errno));
        return ESP_FAIL;
    }

    if (fwrite(pWifiConfig, sizeof(WIFI_CONFIG_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to write : %s : [err= %s]", CFG_WIFI_STATUS, strerror(errno));
        fclose(pFile);
        return ESP_FAIL;
    }

    fclose(pFile);
    return ESP_OK;
}

/**
 * @brief Read wifi configurations
 *
 * @param pWifiConfig
 * @return esp_err_t
 */
esp_err_t ReadWifiConfig(WIFI_CONFIG_t *pWifiConfig)
{
    ESP_LOGI(CONFIG_MODULE, "Reading Wifi Configuration");

    FILE *pFile = NULL;
    pFile = fopen(CFG_WIFI_STATUS, "rb");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_WIFI_STATUS, strerror(errno));
        return ESP_FAIL;
    }

    if (fread(pWifiConfig, sizeof(WIFI_CONFIG_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to read : %s : [err= %s]", CFG_WIFI_STATUS, strerror(errno));
        fclose(pFile);
        return ESP_FAIL;
    }

    fclose(pFile);
    return ESP_OK;
}

/**
 * @brief Default Wifi Configuration
 *
 */
void DefaultWifiConfig(void)
{
    ESP_LOGD(CONFIG_MODULE, "Defaulting Wifi Configuration");

    WIFI_CONFIG_t tmpWifiConfig;
    sprintf((char *)tmpWifiConfig.ssid, "%s", MY_WIFI_SSID);
    sprintf((char *)tmpWifiConfig.password, "%s", MY_WIFI_PASSWD);

    FILE *pFile = NULL;
    pFile = fopen(CFG_WIFI_STATUS, "w+");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_WIFI_STATUS, strerror(errno));
        return;
    }

    if (fwrite(&tmpWifiConfig, sizeof(WIFI_CONFIG_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to write : %s : [err= %s]", CFG_WIFI_STATUS, strerror(errno));
        fclose(pFile);
        return;
    }

    fclose(pFile);
    return;
}

/**
 * @brief Write Given Control status configuration
 *
 * @param pControlStatus
 * @return esp_err_t
 */
esp_err_t WriteControlStatus(CONTROL_STATUS_t *pControlStatus)
{
    ESP_LOGD(CONFIG_MODULE, "Writing Control Status Configuration");

    FILE *pFile = NULL;
    pFile = fopen(CFG_CONTROL_STATUS, "wb");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_CONTROL_STATUS, strerror(errno));
        return ESP_FAIL;
    }

    if (fwrite(pControlStatus, sizeof(CONTROL_STATUS_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to write : %s : [err= %s]", CFG_CONTROL_STATUS, strerror(errno));
        fclose(pFile);
        return ESP_FAIL;
    }

    fclose(pFile);
    return ESP_OK;
}


/**
 * @brief Read Switch Control Status
 *
 * @param pControlStatus
 * @return esp_err_t
 */
esp_err_t ReadControlStatus(CONTROL_STATUS_t *pControlStatus)

{
    ESP_LOGD(CONFIG_MODULE, "Reading Control Status Configuration");

    FILE *pFile = NULL;
    pFile = fopen(CFG_CONTROL_STATUS, "rb");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_CONTROL_STATUS, strerror(errno));
        return ESP_FAIL;
    }

    if (fread(pControlStatus, sizeof(CONTROL_STATUS_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to read : %s : [err= %s]", CFG_CONTROL_STATUS, strerror(errno));
        fclose(pFile);
        return ESP_FAIL;
    }

    fclose(pFile);
    return ESP_OK;
}


/**
 * @brief Default Control status
 *
 */
void DefaultControlStatus(void)
{
    ESP_LOGD(CONFIG_MODULE, "Defaulting control status");
    CONTROL_STATUS_t tmpControlStatus = {0};

    FILE *pFile = NULL;
    pFile = fopen(CFG_CONTROL_STATUS, "wb+");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_CONTROL_STATUS, strerror(errno));
        return;
    }

    if (fwrite(&tmpControlStatus, sizeof(CONTROL_STATUS_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to write : %s : [err= %s]", CFG_CONTROL_STATUS, strerror(errno));
        fclose(pFile);
        return;
    }

    fclose(pFile);
    return;
}


/**
 * @brief Write Received delay from app to file
 *
 * @param psensorConfig
 * @return esp_err_t
 */
esp_err_t WritePIRDelay(SENSOR_CONFIG_t *psensorConfig)
{
    ESP_LOGD(CONFIG_MODULE, "Writing PIR Delay Configuration");

    FILE *pFile = NULL;
    pFile = fopen(CFG_PIR_DELAY, "wb");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_PIR_DELAY, strerror(errno));
        return ESP_FAIL;
    }

    if (fwrite(psensorConfig, sizeof(SENSOR_CONFIG_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to write : %s : [err= %s]", CFG_PIR_DELAY, strerror(errno));
        fclose(pFile);
        return ESP_FAIL;
    }

    fclose(pFile);
    return ESP_OK;
}

/**
 * @brief Read Received delay from  file
 *
 * @param psensorConfig
 * @return esp_err_t
 */
esp_err_t ReadPIRDelay(SENSOR_CONFIG_t *psensorConfig)

{
    ESP_LOGD(CONFIG_MODULE, "Reading PIR Delay Configuration");

    FILE *pFile = NULL;
    pFile = fopen(CFG_PIR_DELAY, "rb");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_PIR_DELAY, strerror(errno));
        return ESP_FAIL;
    }

    if (fread(psensorConfig, sizeof(SENSOR_CONFIG_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to read : %s : [err= %s]", CFG_PIR_DELAY, strerror(errno));
        fclose(pFile);
        return ESP_FAIL;
    }

    fclose(pFile);
    return ESP_OK;
}

/**
 * @brief Default PIR Dealy from file
 *
 */
void DefaultDelay(void)
{
    ESP_LOGD(CONFIG_MODULE, "Defaulting Delay status");
     sensorConfig.time_delay = 20;

    FILE *pFile = NULL;
    pFile = fopen(CFG_PIR_DELAY, "wb+");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_PIR_DELAY, strerror(errno));
        return;
    }

    if (fwrite(&sensorConfig.time_delay, sizeof(SENSOR_CONFIG_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to write : %s : [err= %s]", CFG_PIR_DELAY, strerror(errno));
        fclose(pFile);
        return;
    }

    fclose(pFile);
    return;
}

/**
 * @brief Write Sensor Modes to file
 *
 * @param psensorConfig
 * @return esp_err_t
 */

esp_err_t WriteSensorModes(SENSOR_CONFIG_t *psensorConfig)
{
    ESP_LOGI(CONFIG_MODULE, "Writing Mode Select Configuration");

    FILE *pFile = NULL;
    pFile = fopen(CFG_MODE_SELECT, "wb");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open Writing: %s : [err= %s]", CFG_MODE_SELECT, strerror(errno));
        return ESP_FAIL;
    }

    if (fwrite(psensorConfig, sizeof(SENSOR_CONFIG_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to write : %s : [err= %s]", CFG_MODE_SELECT, strerror(errno));
        fclose(pFile);
        return ESP_FAIL;
    }

    fclose(pFile);
    return ESP_OK;
}

/**
 * @brief Read Sensor Modes from file
 *
 * @param psensorConfig
 * @return esp_err_t
 */
esp_err_t ReadSensorModes(SENSOR_CONFIG_t *psensorConfig)
{
    ESP_LOGI(CONFIG_MODULE, "Reading Modes Select Configuration");

    FILE *pFile = NULL;
    pFile = fopen(CFG_MODE_SELECT, "rb");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open Reading : %s : [err= %s]", CFG_MODE_SELECT, strerror(errno));
        return ESP_FAIL;
    }

    if (fread(psensorConfig, sizeof(SENSOR_CONFIG_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to read : %s : [err= %s]", CFG_MODE_SELECT, strerror(errno));
        fclose(pFile);
        return ESP_FAIL;
    }
    fclose(pFile);
    return ESP_OK;
}

/**
 * @brief Read PIR Dealy from file
 *
 */
void DefaultMode(void)
{
    ESP_LOGD(CONFIG_MODULE, "Defaulting Selected Mode");
   sensorConfig.mode_select = 1;

    FILE *pFile = NULL;
    pFile = fopen(CFG_PIR_DELAY, "wb+");
    if (pFile == NULL)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to open : %s : [err= %s]", CFG_PIR_DELAY, strerror(errno));
        return;
    }

    if (fwrite(&sensorConfig.mode_select, sizeof(SELECT_MODES_t), 1, pFile) == 0)
    {
        ESP_LOGE(CONFIG_MODULE, "Failed to write : %s : [err= %s]", CFG_PIR_DELAY, strerror(errno));
        fclose(pFile);
        return;
    }

    fclose(pFile);
    return;
}

/**
 * @brief   Reset to factory settings
 *          Erase NVS flash, default configuration
 */
void DoFactoryReset(void)
{
    ESP_LOGI(CONFIG_MODULE, "Resetting to factory default settings");

    gpio_set_level(26, 0);
    gpio_set_level(27, 0);

    /* erase configuration written to NVS */
    nvs_flash_erase();

    /* Default configuration */
    DefaultWifiConfig();
    DefaultControlStatus();

    /* restart the system after resetting */
    ESP_LOGI(CONFIG_MODULE, "System is going down now...");
    vTaskDelay(3 * SEC_TO_MS / portTICK_RATE_MS);
    esp_restart();
}

/**
 * @brief   De-init modules and restart system
 *
 */
void DoSystemRestart(void)
{
    gpio_set_level(26, 0);
    gpio_set_level(27, 0);

    /* De-init file system */
    DeInitConfigurationModule();

    /* restarting the system */
    ESP_LOGI(CONFIG_MODULE, "System is going down now...");
    vTaskDelay(3 * SEC_TO_MS / portTICK_RATE_MS);
    esp_restart();
}

//--------------------------------------------------------------------------
// End of file
//--------------------------------------------------------------------------