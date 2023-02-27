
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
/* Application headers */
#include "smartap_networkManager.h"

/* Standard headers */
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "cJSON.h"
#include "driver/gpio.h"
//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
#define NW_MANAGER SMART_TAP "_NW_MNGR"
#define HW_RESET_PRESS_TIMEOUT (2 * SEC_TO_US)
#define GPIO_HW_RESET 19
/* Macro defines WiFi connection event flag bit, connection failure flag bit and smart distribution network flag bit */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define MAX_WIFI_RETRY 10              /* maximum retry */
#define PROVISIONING_TIME_OUT (5 * 60) /* wait 5 minutes for device provisioning */

/* TCP server settings */
#define TCP_SERVER_PORT 8881
#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

//--------------------------------------------------------------------------
// Typedef
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Static Variables
//--------------------------------------------------------------------------
WIFI_CONFIG_t config;
static EventGroupHandle_t wifiEventGrp;                  /* wifi connection event group */
static TaskHandle_t connMonitorTaskHandle = NULL;        /* connection monitor task handle */
static int retryCount = 0;                               /* Record the number of wifi reconnections */
bool wifiConnectionStatus = false;                       /* network connection status */
static TaskHandle_t deviceProvisioningTaskHandle = NULL; /* wifi AP device provisioning task handle */
bool led_Status = GPIO_LOW;
bool wifi_AP_Start = false;

bool wifi_STA_Disconnected = false;
bool is_WIFI_STA_Status = false;
bool start_sta = false;
bool isReg = false;
extern bool DeviceFactoryReset;
int isReset = false;
extern volatile int64_t startTime;
//--------------------------------------------------------------------------
// Static Functions
//--------------------------------------------------------------------------
static esp_err_t initWifiSTA(void);
static esp_err_t initDeviceProvisioning(void);
static void deviceProvisioningTask(void *pArg);
static void connectionEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void connectionMonitorLoop(void *pArg);
//--------------------------------------------------------------------------
// Function Definition
//--------------------------------------------------------------------------
/**
 * @brief Init Network Manager Module
 *
 * @return int
 */
int InitNetworkManager(void)
{
    esp_err_t retVal;

    /* Init non-volatile storage (NVS) */
    retVal = nvs_flash_init();
    if (retVal == ESP_ERR_NVS_NO_FREE_PAGES || retVal == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGE(NW_MANAGER, "NVS : exception occurred : erasing NVS");

        retVal = nvs_flash_erase();
        if (retVal != ESP_OK)
        {
            ESP_LOGE(NW_MANAGER, "NVS: failed to erase nvs : %s", esp_err_to_name(retVal));
            return MODULE_FAIL;
        }

        retVal = nvs_flash_init();
        if (retVal != ESP_OK)
        {
            ESP_LOGE(NW_MANAGER, "NVS: failed to init nvs : %s", esp_err_to_name(retVal));
            return MODULE_FAIL;
        }
    }
    ESP_LOGI(NW_MANAGER, "NVS initialized");

    /* init status led */
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_OUT_STS_LED);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUT_STS_LED, 1);
    vTaskDelay(SEC_TO_MS / portTICK_RATE_MS);
    gpio_set_level(GPIO_OUT_STS_LED, 1);

    /* Init wifi station mode */

    ESP_ERROR_CHECK(initWifiSTA());

    /* create connection monitor task */
    xTaskCreatePinnedToCore(connectionMonitorLoop, "connMonitor", NETWORK_MANAGER_STACK_SIZE_KB,
                            NULL, NETWORK_MANAGER_PRIORITY, &connMonitorTaskHandle, 0);
    if (connMonitorTaskHandle == NULL)
    {
        ESP_LOGE(NW_MANAGER, "Failed to create connection monitor task");
        return MODULE_FAIL;
    }
    return MODULE_SUCCESS;
}

/**
 * @brief Get the Wifi Connection Status object
 *
 * @return true - wifi is connected
 * @return false - wifi is not connected
 */
bool GetWifiConnectionStatus(void)
{
    return wifiConnectionStatus;
}

/**
 * @brief Init Wi-Fi Station Mode
 *
 * @return esp_err_t
 */
static esp_err_t initWifiSTA(void)
{
    esp_err_t retVal;

    /* Create an event flag group */
    wifiEventGrp = xEventGroupCreate();

    /* Initialize the underlying TCP/IP stack. This function should be called once when the application starts. */
    retVal = esp_netif_init();
    if (retVal != ESP_OK)
    {
        ESP_LOGE(NW_MANAGER, "esp_netif_init failed : %s", esp_err_to_name(retVal));
        return retVal;
    }

    /* Create a default event loop, */
    retVal = esp_event_loop_create_default();
    if (retVal != ESP_OK)
    {
        ESP_LOGE(NW_MANAGER, "esp_event_loop_create_default failed : %s", esp_err_to_name(retVal));
        return retVal;
    }

    ESP_LOGI(NW_MANAGER, "Creating default Wifi STA network interface");

    /* Create a default WIFI-STA network interface, if initialization error occurs, this API will be aborted. */
    esp_netif_create_default_wifi_sta();
    /* Create a default WIFI-AP network interface, if initialization error occurs, this API will be aborted. */
    esp_netif_create_default_wifi_ap();

    /* Use WIFI_INIT_CONFIG_DEFAULT() to get a default wifi configuration parameter structure variable*/
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    /* Initialize the resources required for the wifi connection according to the cfg parameters */
    retVal = esp_wifi_init(&cfg);
    if (retVal != ESP_OK)
    {
        ESP_LOGE(NW_MANAGER, "esp_wifi_init failed : %s", esp_err_to_name(retVal));
        return retVal;
    }

    /**
     * @note Spurious interrupts on GPIO36 and GPIO39 when using wifi with sleep mode enable
     *       Refer ECO_and_Workarounds_for_Bugs_in_ESP32__EN (Section 3.11)
     *       A hardware issue in esp32 module due to which when using RTC voltage spikes
     *       are observed in GPIO36/39 which give spurious interrupts.
     *       Work Around : Currently disabling the wifi the power saver mode resolves
     *                     the issue.
     *
     */

    /* disable wifi power saver mode */
    esp_wifi_set_ps(WIFI_PS_NONE);

    /* Register the event handler to the system default event loop, which are WiFi events, IP address events and smartconfig events */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &connectionEventHandler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connectionEventHandler, NULL));

    /* read previously stored wifi config */
    wifi_config_t wifiConfig;
    bzero(&wifiConfig, sizeof(wifi_config_t)); /* Clear the structure data */

    WIFI_CONFIG_t tmpWifiConfig;
    retVal = ReadWifiConfig(&tmpWifiConfig);
    if (retVal != ESP_OK)
    {
        ESP_LOGW(NW_MANAGER, "Failed to read old wifi configuration : going with default config");
        /* load default wifi config */
        sprintf((char *)wifiConfig.sta.ssid, "%s", MY_WIFI_SSID);
        sprintf((char *)wifiConfig.sta.password, "%s", MY_WIFI_PASSWD);
    }
    else
    {
        sprintf((char *)wifiConfig.sta.ssid, "%s", tmpWifiConfig.ssid);
        sprintf((char *)wifiConfig.sta.password, "%s", tmpWifiConfig.password);
        ESP_LOGD(NW_MANAGER, "read previous wifi configuration : [SSID: %s], [Password= %s]", wifiConfig.sta.ssid, wifiConfig.sta.password);
    }

    if ((strcmp((char *)wifiConfig.sta.ssid, (char *)"SmarTap_O") == 0))
    {
        ESP_LOGD(NW_MANAGER, "####### Provisioning Mode #########");
        initDeviceProvisioning();
    }
    else
    {
        /* Set the working mode of WiFi to STA */
        retVal = esp_wifi_set_mode(WIFI_MODE_STA);
        if (retVal != ESP_OK)
        {
            ESP_LOGE(NW_MANAGER, "esp_wifi_set_mode failed : %s", esp_err_to_name(retVal));
            return retVal;
        }
        else
        {
            start_sta = true;
        }

        /* Set the parameters of the WiFi connection, mainly ssid and password */
        retVal = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifiConfig);
        if (retVal != ESP_OK)
        {
            ESP_LOGE(NW_MANAGER, "esp_wifi_set_config failed : %s", esp_err_to_name(retVal));
            return retVal;
        }
        /* Start WiFi connection */
        retVal = esp_wifi_start();
        if (retVal != ESP_OK)
        {
            ESP_LOGE(NW_MANAGER, "esp_wifi_start failed : %s", esp_err_to_name(retVal));
            return retVal;
        }

        ESP_LOGI(NW_MANAGER, "Wi-fi started");
    }
    return ESP_OK;
}

/**
 * @brief Initialize device provisioning using WiFi AP mode
 *
 * @return esp_err_t
 */
static esp_err_t initDeviceProvisioning(void)
{
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = MY_WIFI_SSID,
            .ssid_len = strlen(MY_WIFI_SSID),
            .channel = 1,
            .password = MY_WIFI_PASSWD,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    if (strlen(MY_WIFI_PASSWD) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreate(deviceProvisioningTask, "wifiAPDevProv", AP_TASK_STACK_SIZE_KB, NULL, AP_TASK_PRIORITY, &deviceProvisioningTaskHandle);
    if (deviceProvisioningTaskHandle == NULL)
    {
        ESP_LOGE(NW_MANAGER, "Failed to start device provisioning task");
        return ESP_FAIL;
    }

    ESP_LOGI(NW_MANAGER, "WIFI-AP Device provisioning started");
    return ESP_OK;
}

/**
 * @brief Wifi AP device provisioning task
 *          - Create a TCP server socket and wait for mobile client to connect
 *          - after connection is established, get the credentials from the client in json format
 *            parse the data, provide acknowledgement and configure wifi station mode using received
 *            credentials.
 *
 * @param pArg
 */
static void deviceProvisioningTask(void *pArg)
{
    char addr_str[128];
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;
    int sock = -1;

    char device_name[32] = "";
    char wifiSSID[32] = "";
    char wifiPass[64] = "";
    char unique_id[12] = "";
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(TCP_SERVER_PORT);
    wifi_config_t wifiConfig;
    /* create socket */
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0)
    {
        ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Unable to create socket: [err= %s]", strerror(errno));
        close(listen_sock);
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Socket created");

    // do
    {
        /* bind socket */
        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0)
        {
            ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Socket unable to bind: [err= %s]", strerror(errno));
            //  break;
            close(listen_sock);
        }
        ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Socket bound, port %d", TCP_SERVER_PORT);

        /* listen for incoming connection */
        err = listen(listen_sock, 1);
        if (err != 0)
        {
            ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Error occurred during listen: [err= %s]", strerror(errno));
            // break;
            close(listen_sock);
        }
        ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Socket listening : wait for connection");

        while (1)
        {
            /* accept incoming connection */
            struct sockaddr_storage source_addr;
            socklen_t addr_len = sizeof(source_addr);
            sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
            if (sock < 0)
            {
                ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Unable to accept connection: [err= %s]", strerror(errno));
                close(listen_sock);
            }

            /* setup tcp keep-alive */
            setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
            if (source_addr.ss_family == PF_INET)
            {
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
            }
            ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Socket accepted ip address: %s", addr_str);

            int len;
            char rxTxBuffer[500];
        RX:
        {
            /* receive configuration data */
            len = recv(sock, rxTxBuffer, sizeof(rxTxBuffer) - 1, 0);
            if (len < 0)
            {
                ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Error occurred during receiving: [err= %s]", strerror(errno));
            }
            else if (len == 0)
            {
                ESP_LOGW(NW_MANAGER, "WifiAP provisioning : Connection closed");
            }
            else
            {
                isReg = false;
                ESP_LOGD(NW_MANAGER, "WifiAP provisioning : Received %d bytes: %s", len, rxTxBuffer);
                if (strcmp(rxTxBuffer, "getDeviceInfo") != 0)
                {
                    /* parse device configuration data */
                    cJSON *root = cJSON_Parse(rxTxBuffer);
                    if (root == NULL)
                    {
                        /* parsing error */
                        const char *errPtr = cJSON_GetErrorPtr();
                        if (errPtr != NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Error while parsing : %s", errPtr);
                            break;
                        }
                    }

                    if (cJSON_HasObjectItem(root, "deviceConfigure"))
                    {
                        cJSON *deviceConfig = cJSON_GetObjectItemCaseSensitive(root, "deviceConfigure");

                        cJSON *device = cJSON_GetObjectItemCaseSensitive(deviceConfig, "device_name");
                        ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Got device name : [%s]", device->valuestring);
                        snprintf(device_name, sizeof(device_name), "%s", device->valuestring);

                        cJSON *ssid = cJSON_GetObjectItemCaseSensitive(deviceConfig, "wifi_ssid");
                        ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Got ssid : [%s]", ssid->valuestring);
                        snprintf(wifiSSID, sizeof(wifiSSID), "%s", ssid->valuestring);

                        cJSON *pwd = cJSON_GetObjectItemCaseSensitive(deviceConfig, "password");
                        ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Got password : [%s]", pwd->valuestring);
                        snprintf(wifiPass, sizeof(wifiPass), "%s", pwd->valuestring);

                        cJSON *UniqueId = cJSON_GetObjectItemCaseSensitive(deviceConfig, "unique_id");
                        ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Got UniqueId : [%s]", UniqueId->valuestring);
                        snprintf(unique_id, sizeof(unique_id), "%s", UniqueId->valuestring);
                    }
                    cJSON_Delete(root);

                    /* Configure WiFi station with host credentials provided during provisioning */
                    bzero(&wifiConfig, sizeof(wifi_config_t)); /* Clear the structure data */
                    snprintf((char *)wifiConfig.sta.ssid, sizeof(wifiConfig.sta.ssid), "%s", wifiSSID);
                    snprintf((char *)wifiConfig.sta.password, sizeof(wifiConfig.sta.password), "%s", wifiPass);

                    bzero(&config, sizeof(config)); /* Clear the structure data */
                    snprintf((char *)config.ssid, sizeof(config.ssid), "%s", wifiSSID);
                    snprintf((char *)config.password, sizeof(config.password), "%s", wifiPass);

                    bzero(&rxTxBuffer, sizeof(rxTxBuffer));
                }

                if (strcmp(rxTxBuffer, "getDeviceInfo") == 0)
                {
                    {
                        cJSON *pRoot = cJSON_CreateObject();
                        cJSON *pDeviceInfo = cJSON_CreateObject();

                        if (cJSON_AddStringToObject(pDeviceInfo, "device_name", device_name) == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'device_name'");
                            break;
                        }

                        if (cJSON_AddStringToObject(pDeviceInfo, "wifi_ssid", wifiSSID) == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'wifi_ssid'");
                            break;
                        }

                        if (cJSON_AddStringToObject(pDeviceInfo, "password", wifiPass) == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'password'");
                            break;
                        }

                        /* get mac address to send as device serial number */
                        uint8_t macAdd[6] = {0};
                        char macAddrStr[20] = "";
                        esp_efuse_mac_get_default(macAdd);
                        sprintf(macAddrStr, MACSTR, MAC2STR(macAdd));

                        if (cJSON_AddStringToObject(pDeviceInfo, "device_serial_number", macAddrStr) == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'device_serial_number'");
                            break;
                        }

                        if (cJSON_AddStringToObject(pDeviceInfo, "mac_imei", macAddrStr) == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'device_name'");
                            break;
                        }

                        if (cJSON_AddStringToObject(pDeviceInfo, "vFirmwareVersion", FW) == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'firmware_version'");
                            break;
                        }

                        if (cJSON_AddStringToObject(pDeviceInfo, "vManufactureDate", MANUFACTURE_DATE) == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'manufacturing_date'");
                            break;
                        }

                        if (cJSON_AddStringToObject(pDeviceInfo, "device_type", "4") == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'firmware_version'");
                            break;
                        }

                        if (cJSON_AddStringToObject(pDeviceInfo, "vDesc", " ") == NULL)
                        {
                            ESP_LOGE(NW_MANAGER, "Failed to add element 'vDesc'");
                            break;
                        }

                        cJSON_AddItemToObject(pRoot, "getDeviceInfo", pDeviceInfo);

                        snprintf(rxTxBuffer, sizeof(rxTxBuffer), "%s", cJSON_Print(pRoot));
                        cJSON_Delete(pRoot);

                        int to_write = strlen(rxTxBuffer);
                        int written = 0;
                        int error_count = 0;
                        while (to_write > 0)
                        {
                            written = send(sock, rxTxBuffer + written, to_write, 0);
                            if (written < 0)
                            {
                                error_count++;
                                if (error_count == 20)
                                {
                                    error_count = 0;
                                    DoFactoryReset();
                                }
                                ESP_LOGE(NW_MANAGER, "Error occurred during sending: errno %d", errno);
                            }
                            to_write -= written;
                            if (to_write == 0)
                            {
                                ESP_LOGW(NW_MANAGER, "start_sta is true");
                                start_sta = true;
                                break;
                            }
                        }
                    }
                }
                else
                {
                    goto RX;
                }
                ESP_LOGD(NW_MANAGER, "WifiAP provisioning : Received %d bytes: %s", len, rxTxBuffer);
                len = 0;
                bzero(&rxTxBuffer, sizeof(rxTxBuffer)); /* Clear the structure data */
                ESP_LOGW(NW_MANAGER, "Writing Credentials");
                bzero(&config, sizeof(config)); /* Clear the structure data */
                snprintf((char *)config.ssid, sizeof(config.ssid), "%s", wifiSSID);
                snprintf((char *)config.password, sizeof(config.password), "%s", wifiPass);
                snprintf((char *)config.uniqueid, sizeof(config.uniqueid), "%s", unique_id);
                ESP_LOGW(NW_MANAGER, "wifi ssid : %s password : %s unique id : %s", config.ssid, config.password, config.uniqueid);
                WriteWifiConfig(&config);
            }
        }
            int retryCount = 0;
            if (start_sta == true)
            {
                close(listen_sock);
                ESP_LOGW(NW_MANAGER, "start_sta : %d", start_sta);
                if (esp_wifi_set_mode(WIFI_MODE_APSTA) != ESP_OK)
                {
                    ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Failed to set WiFi APSTA mode");
                    break;
                }
                else
                {
                    ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Success to set WiFi APSTA mode");
                }

                if (esp_wifi_set_config(WIFI_IF_STA, &wifiConfig) != ESP_OK)
                {
                    ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Failed to set WiFi configuration");
                    break;
                }
                else
                {
                    ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Success to set WiFi configuration");
                }
                // /* Restart WiFi */
                if (esp_wifi_start() != ESP_OK)
                {
                    ESP_LOGE(NW_MANAGER, "WifiAP provisioning : Failed to restart WiFi");
                    break;
                }
                else
                {
                    ESP_LOGI(NW_MANAGER, "WifiAP provisioning : Success to restart WiFi");
                }

                while ((GetWifiConnectionStatus() == false) && (retryCount < MAX_WIFI_RETRY))
                {
                    /* wait for connection with access-point */
                    vTaskDelay(3 * SEC_TO_MS / portTICK_RATE_MS);
                    retryCount++;
                }
                if (GetWifiConnectionStatus() == true)
                {
                    ESP_LOGW(NW_MANAGER, "GetWifiConnectionStatus() == true");
                    break;
                }
            }
        }
        close(listen_sock);
        shutdown(sock, 0);
        close(sock);
        ESP_LOGW(NW_MANAGER, "Socket close");
        /* close socket connection */

        ESP_LOGI(NW_MANAGER, "Device Provisioning Finished : going into station mode");

        /* disable the WIFI AP mode, start only station mode */
        if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK)
        {

            ESP_LOGE(NW_MANAGER, "Failed to set WiFi STA mode");
        }
        vTaskDelete(NULL);
    }
}
/**
 * @brief System event handler
 *
 * @param arg
 * @param eventBase
 * @param eventId
 * @param eventData
 */
static void connectionEventHandler(void *arg, esp_event_base_t eventBase, int32_t eventId, void *eventData)
{
    /* System event is WiFi event */
    if (eventBase == WIFI_EVENT)
    {
        if (eventId == WIFI_EVENT_STA_START) /* wifi start event */
        {
            ESP_LOGI(NW_MANAGER, "wifi station start event");

            /* connect to wifi */
            esp_wifi_connect();
        }
        else if (eventId == WIFI_EVENT_STA_DISCONNECTED) /* wifi station disconnected */
        {
            wifi_STA_Disconnected = WIFI_EVENT_STA_DISCONNECTED;
            ESP_LOGI(NW_MANAGER, "Device Disconnected");
            is_WIFI_STA_Status = false;
            retryCount++;

            if ((true == controlStatus.isDevicesActivated) && ((retryCount % 2) == 0))
            {
                wifi_AP_Start = false;

                if (isFactoryReset == false)
                {
                    ESP_LOGI(NW_MANAGER, "led_Status : %d", gpio_get_level(GPIO_OUT_STS_LED));
                    gpio_set_level(GPIO_OUT_STS_LED, led_Status);
                    led_Status = !led_Status;
                    ESP_LOGI(NW_MANAGER, "led_Status : %d", gpio_get_level(GPIO_OUT_STS_LED));
                }
                if (gpio_get_level(GPIO_HW_RESET) == 0)
                {
                    ESP_LOGI(SMART_TAP, "Hardware Reset Timer Started");
                    startTime = esp_timer_get_time();
                    isFactoryReset = true;
                    isReset = true;
                    gpio_set_level(GPIO_OUT_STS_LED, 1);
                }
                else
                {
                    if (((esp_timer_get_time() - startTime) > HW_RESET_PRESS_TIMEOUT) && (isReset == true))
                    {

                        DeviceFactoryReset = true;
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        ESP_LOGI(SMART_TAP, "Hardware Reset Timeout : Doing Factory Reset");
                        startTime = 0;
                        isFactoryReset = false;
                        gpio_set_level(GPIO_OUT_STS_LED, 1);
                        DoFactoryReset();
                    }
                    ESP_LOGI(SMART_TAP, "Hardware Reset switch released");
                    startTime = 0;
                    isReset = false;
                    isFactoryReset = false;
                }
                if ((gpio_get_level(GPIO_HW_RESET) == 1) && isFactoryReset == true)
                {
                    ESP_LOGI(SMART_TAP, "Hardware Reset switch released");
                    gpio_set_level(GPIO_OUT_STS_LED, 0);
                }
            }

            /* connect to wifi */
            esp_wifi_connect();
            if ((controlStatus.isDevicesActivated == false)) /* The number of WiFi reconnections is greater than 10 */
            {
                /* Set the WiFi connection failure event position of the WiFi connection event flag group to 1 */
                xEventGroupSetBits(wifiEventGrp, WIFI_FAIL_BIT);
                retryCount = 0;
            }
        }
        else if (eventId == WIFI_EVENT_AP_STACONNECTED) /* station connected */
        {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)eventData;
            ESP_LOGI(NW_MANAGER, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);

            gpio_set_level(GPIO_OUT_STS_LED, 0);
        }
        else if (eventId == WIFI_EVENT_AP_STADISCONNECTED) /* station disconnected */
        {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)eventData;
            ESP_LOGI(NW_MANAGER, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
        }
        else if (eventId == WIFI_EVENT_AP_START)
        {
            ESP_LOGI(NW_MANAGER, "DWIFI_EVENT_AP_START");

            wifi_AP_Start = true;
        }
        else if (eventId == WIFI_EVENT_AP_STOP)
        {
            ESP_LOGI(NW_MANAGER, "WIFI_EVENT_AP_STOP");
            gpio_set_level(GPIO_OUT_STS_LED, 0);
            wifi_AP_Start = false;
        }
    }
    /* The system event is an ip address event, and the event id is successfully obtaining an ip address */
    else if ((eventBase == IP_EVENT) && (eventId == IP_EVENT_STA_GOT_IP))
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)eventData; /* Get IP address information*/
        wifi_STA_Disconnected = 0;
        ESP_LOGI(NW_MANAGER, "got ip address from station : [IP= %d.%d.%d.%d]", IP2STR(&event->ip_info.ip));
        retryCount = 0; /* Clear the number of WiFi reconnections */

        /*Set the WiFi connection success event position of the WiFi connection event flag group to 1 */
        xEventGroupSetBits(wifiEventGrp, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief   Wait for connection/disconnection events and do action accordingly.
 *          Use the event flag group to wait for the connection establishment
 *          (WIFI_CONNECTED_BIT) or connection failure (WIFI_FAIL_BIT) event
 *
 * @return void
 */
static void connectionMonitorLoop(void *pArg)
{
    /* Define an event bit variable to receive the return value of the event flag group waiting function */
    EventBits_t eventBits;
    bool ledStatus = GPIO_LOW;

    while (1)
    {
        eventBits = xEventGroupWaitBits(wifiEventGrp,                       /* The handle of the event flag group to be waited for */
                                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, /* Event bit to wait for */
                                        pdTRUE,                             /* When it is pdFALSE, these event bits set before exiting this function will not change, if it is pdTRUE, it will be cleared*/
                                        pdFALSE,                            /* When it is pdFALSE, any one of these event bits set will be set to 1, and it will return if it is pdTRUE, it needs all 1 to return */
                                        1 * SEC_TO_MS / portTICK_RATE_MS);  /* Set to the longest blocking waiting time, the unit is clock beat */

        /* Get the WiFi connection status according to the return value of the event flag group waiting function */
        if (eventBits & WIFI_CONNECTED_BIT) /* WiFi connection success event */
        {
            ESP_LOGI(NW_MANAGER, "Smart Device is Connected to wifi");
            wifiConnectionStatus = true;
            is_WIFI_STA_Status = true;
            gpio_set_level(GPIO_OUT_STS_LED, GPIO_LOW);
        }
        else if (eventBits & WIFI_FAIL_BIT) /* WiFi connection failure event */
        {
            ESP_LOGW(NW_MANAGER, "Could not connect to configured wifi AP : Going into device provisioning");
            DoFactoryReset();
        }

        /* if connected to access-point then check for internet connection */
        if (wifiConnectionStatus == true)
        {
            //ESP_LOGE(NW_MANAGER, "**********WiFi Connection is TRue: connecting to internet**********");
            /* check internet connection */
            struct hostent *host;
            host = gethostbyname("google.com");
            if (host == NULL)
            {
                //ESP_LOGE(NW_MANAGER, "Can not resolve host : not connected to internet!");  Commented by Sagar
                if (DFactoryReset() == true)
                {
                    gpio_set_level(GPIO_OUT_STS_LED, !GPIO_HIGH);
                }
            }
            else
            {
                if (DFactoryReset() == true)
                {
                    gpio_set_level(GPIO_OUT_STS_LED, !GPIO_LOW); // for Hardware reset
                }
            }
        }

        if (true == wifi_AP_Start)
        {
            /* if wifi is not connected flash status led at 1 second interval */
            gpio_set_level(GPIO_OUT_STS_LED, ledStatus);
            ledStatus = !ledStatus;
        }
    }
}
//--------------------------------------------------------------------------
// End of File
//--------------------------------------------------------------------------
