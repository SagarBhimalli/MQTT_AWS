
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
/* Application headers */
#include "smartap_mqttClient.h"
#include "smartap_networkManager.h"
#include "Smart_Occupancy_Sensor.h"
#include "smartap_ota.h"

/* Standard headers */
#include "cJSON.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
#define MQTT_MANAGER SMART_TAP "_MQTT"
#define MQTT_CMD_TIMEOUT (20 * SEC_TO_MS)
#define MQTT_HS_TIMEOUT (5 * SEC_TO_MS)
#define MAX_TOPIC_LENGTH (150)

/* Smartap topics */
#define TOPIC_SMART_TOUCH "/topic_name"   /* topic prefix */
#define TOPIC_STATUS "/status/"           /* device status(bootup) (D->C) */
#define TOPIC_ACK_STATUS "/ack-status/"   /*  /smarttouch/{deviceid}/ack-status/ */
#define TOPIC_REQ_ACT_STS "/res-act/"     /* activation status request (D->C) */
#define TOPIC_ACK_ACT_STS "/ack-sts/"     /* activation status response (C->D) */
#define TOPIC_SW_CONTROL "/control/"      /* control device switches (C->D) */
#define TOPIC_REQ_GET_STS "/getswstatus/" /* get switch status request (C->D) */
#define TOPIC_RES_GET_STS "/swstatus/"    /* get switch status response (D->C) */
#define TOPIC_FACTORY_RESET "/restore/"   /* restore to factory settings (C->D) */
#define TOPIC_DEV_RESTART "/restart/"     /* restart device (C->D) */
// #define TOPIC_OTA_CHECK "/ota/"                               /* check for application update via OTA*/
#define TOPIC_OUT_DOOR_MODE "/outdoor-mode-settings/"         /*Out door mode (C-D)*/
#define TOPIC_ACK_OUT_DOOR_MODE "/outdoor-mode-settings-ack/" /*Ack Out door mode (D-C)*/
#define TOPIC_DEV_STANDBY_STATE "/standby-state/"             /*/smarttouch/{deviceid}/standby-state (D-C)*/
#define TOPIC_ACK_FACTORY_RESET "/pin-hole-reset/"            /* restore to factory settings (D->C) */
#define TOPIC_DEV_APPLIANCES "/device-appliances/"            /* get Appliance Detail (C->D) */
#define TOPIC_DEV_ACK_APPLIANCES "/ack-device-appliances/"    /* ACK Appliance Detail (D->C) */
#define TOPIC_DEV_ACTIVE_STATUS "/activation-status/"         /* get Activation Status Detail (C->D) */

// Added By Sagar
#define TOPIC_SENSOR_STATUS "/sensor-status/" /* Sensor status(bootup) (D->C) */
#define TOPIC_GET_DELAY "/get-delay/"         /* Get UnOccupancy delay  (C->D) */
#define TOPIC_GET_MODES "/modes/"             /* Get Modes selection  (C->D) */

/* json keys */
#define JSON_KEY_SWD "swd"
#define JSON_KEY_NAME "name"
#define JSON_KEY_VALUE "value"
#define JSON_KEY_SW "SW"
#define JSON_KEY_DMR "DMR"
#define JSON_KEY_DMR2 "DMR2" // Added by Sagar
#define JSON_KEY_STATUS "status"
#define JSON_KEY_OUT_DOOR "om"
#define JSON_KEY_R_STATE "rs"
#define JSIN_KEY_USB_STATE "USB2"
#define JSIN_KEY_USB_A_SATATE "USB1"
#define JSON_KEY_OM_SW "omsw"
#define JSON_KEY_DEVICE_STATUS "st"
#define JSON_KEY_SW01 "\"SW01\""
#define JSON_KEY_SW02 "\"SW02\""
#define JSON_KEY_SW03 "\"SW03\""
#define JSON_KEY_SW04 "\"SW04\""
#define JSON_KEY_SW05 "\"SW05\""
#define JSON_KEY_SW06 "\"SW06\""
#define JSON_KEY_SW07 "\"SW07\""
#define JSON_KEY_SW08 "\"SW08\""
#define JSON_KEY_USBC "\"USB2\""
#define JSON_KEY_USBA "\"USB1\""
#define JSON_KEY_APPLIANCES "dapp"
#define JSON_OD_DIM "dmr"
//--------------------------------------------------------------------------
// Typedef
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Static Variables
//--------------------------------------------------------------------------
/* CA Root certificate, device ("Thing") certificate and device */
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");
/* AWS MQTT host URL */
static char mqttHostUrl[255] = ""; //static char mqttHostUrl[255] = "a3gjsr00gjuzw5-ats.iot.us-west-1.amazonaws.com";
/* mqtt communication task handle */
static TaskHandle_t mqttCommTaskHandle = NULL;
/* mqtt client session handle */
static AWS_IoT_Client mqttClient;
/* MAC address string */
static char macAddrStr[20] = "";
/*Device DisConnect */
static bool Device_DisConnect = false;
/* Activation status bool */
static bool isAckStatusReceived = false;
/* Subscribed Topic */
static char Topic_Sw_Control[MAX_TOPIC_LENGTH] = "";
static char Topic_Req_Get_Sts[MAX_TOPIC_LENGTH] = "";
static char Topic_Factory_Reset[MAX_TOPIC_LENGTH] = "";
static char Topic_Dev_Restart[MAX_TOPIC_LENGTH] = "";
static char Topic_Ota_Check[MAX_TOPIC_LENGTH] = "";
static char Topic_Ack_Status[MAX_TOPIC_LENGTH] = "";
static char Topic_Out_Door[MAX_TOPIC_LENGTH] = "";
static char Topic_Device_Staus[MAX_TOPIC_LENGTH] = "";

// Added By Sagar
static char Topic_Sensor_Status[MAX_TOPIC_LENGTH] = "";
static char Topic_Get_Delay[MAX_TOPIC_LENGTH] = "";
static char Topic_Get_Modes[MAX_TOPIC_LENGTH] = "";

/* publish notify flag */
static bool doPublishStatus = false;
//--------------------------------------------------------------------------
// Static Functions
//--------------------------------------------------------------------------
static void mqttCommunicationTask(void *pArg);
static void mqttDisconnectCb(AWS_IoT_Client *pClient, void *data);
static void mqttReceiveCb(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen, IoT_Publish_Message_Params *params, void *pData);
static IoT_Error_t publishBootupPacket(void);

static void parseActivationStatus(char *pPayload);
static void parseSwitchControlRequest(char *pPayload);
static IoT_Error_t publishSwitchStatus(void);
static void parseActiveStatusRequest(char *pPayload);
static void parseOutdoorSetting(char *pPayload);
static void DeviceActivationStatus(char *pPayload);
static IoT_Error_t publishOutDoorModeSettingPacket(bool OD_state, char *pPayload);

static void parseGetDelayRequest(char *pPayload);
static void parseGetModesRequest(char *pPayload);

// Added By Sagar
static IoT_Error_t publishSensorStatus(void);
uint8_t Occ_Flag;
uint32_t ambient_light;
int Un_delay;
SENSOR_CONFIG_t sensorConfig;
SELECT_MODES_t selectModes;

uint8_t try_to_connect_count = 0;

extern bool wifiConnectionStatus;
extern bool wifi_STA_Disconnected;
extern bool is_WIFI_STA_Status;
extern bool outdoor_mode_state;
extern bool start_sta;

static const char *TAG = "SAGAR";
//--------------------------------------------------------------------------
// Function Definition
//--------------------------------------------------------------------------
/**
 * @brief Initialize MQTT client module, create MQTT communication task
 *
 * @return int
 */
int InitMqttClient(void)
{
    ReadWifiConfig(&config);
    /* create mqtt communication task */
    xTaskCreatePinnedToCore(mqttCommunicationTask, "mqttccccccComm", MQTT_CLIENT_STACK_SIZE_KB,
                            NULL, MQTT_CLIENT_PRIORITY, &mqttCommTaskHandle, 0);
    if (mqttCommTaskHandle == NULL)
    {
        ESP_LOGE(MQTT_MANAGER, "Failed to create mqtt communication task");
        return MODULE_FAIL;
    }

    return MODULE_SUCCESS;
}

/**
 * @brief   MQTT communication task
 *          - Initialize MQTT client session
 *          - Connect to MQTT server
 *          - Subscribe to topics
 *          - MQTT yield for receiving messages
 *
 * @param pArg
 */
IoT_MQTT_Will_Options will_opt;

static void mqttCommunicationTask(void *pArg)
{
    IoT_Error_t retVal = FAILURE;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params mqttConnParams = iotClientConnectParamsDefault;

    ESP_LOGI(MQTT_MANAGER, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);
    ESP_LOGI(MQTT_MANAGER, "controlStatus.isDevicesActivated : %d start_sta : %d ", controlStatus.isDevicesActivated, start_sta);

    /* Init mqtt parameters */
    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = mqttHostUrl;
    mqttInitParams.port = AWS_IOT_MQTT_PORT;
    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;
    mqttInitParams.mqttCommandTimeout_ms = MQTT_CMD_TIMEOUT;
    mqttInitParams.tlsHandshakeTimeout_ms = MQTT_HS_TIMEOUT;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = mqttDisconnectCb;
    mqttInitParams.disconnectHandlerData = NULL;
    mqttConnParams.isWillMsgPresent = true;

    uint8_t macAdd[6] = {0};
    /* get device MAC address */
    esp_efuse_mac_get_default(macAdd);
    sprintf(macAddrStr, MACSTR, MAC2STR(macAdd));

    /* Init connection parameters */
    mqttConnParams.keepAliveIntervalInSec = 10;
    mqttConnParams.isCleanSession = true;
    mqttConnParams.MQTTVersion = MQTT_3_1_1;
    mqttConnParams.pClientID = (const char *)macAddrStr; // aws_mqtt_clientid;
    ESP_LOGI(MQTT_MANAGER, "MAC address of device : [%s]", mqttConnParams.pClientID);
    mqttConnParams.clientIDLen = (uint16_t)strlen((const char *)macAddrStr);

    char WillTopic[MAX_TOPIC_LENGTH] = "";
    sprintf(WillTopic, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_STATUS);

    char will_message[50];
    sprintf(will_message, "{\"st\":\"0\",\"fwv\":\"%s\"}", FW);
    ESP_LOGI(MQTT_MANAGER, "Will Message : %s ", will_message);
    will_opt.isRetained = false;
    will_opt.msgLen = strlen(will_message);
    will_opt.pMessage = will_message;
    will_opt.pTopicName = WillTopic; // TOPIC_STATUS;
    will_opt.qos = 0;
    will_opt.topicNameLen = strlen(WillTopic);
    mqttConnParams.will = will_opt;

    /* end Will message packet */

    do
    {
        esp_task_wdt_init(25, false);
        /* Init mqtt client */
        retVal = aws_iot_mqtt_init(&mqttClient, &mqttInitParams);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to init mqtt client : [err= %d]", retVal);
            break;
        }

        ESP_LOGI(MQTT_MANAGER, "Waiting for wifi connection");

        /* Wait for wifi connection */
        while (!GetWifiConnectionStatus())
        {
            /* wait for 1 sec */
            vTaskDelay(SEC_TO_MS / portTICK_RATE_MS);
            esp_task_wdt_init(25, false);
        }

        ESP_LOGI(MQTT_MANAGER, "Connecting to AWS");

        /* try for aws connection */
        do
        {
            esp_task_wdt_init(25, false);
            retVal = aws_iot_mqtt_connect(&mqttClient, &mqttConnParams);
            ESP_LOGW(MQTT_MANAGER, "connecting to 1");
            if (SUCCESS != retVal)
            {
                ESP_LOGW(MQTT_MANAGER, "connecting to 2");
                if (isFactoryReset == false && start_sta == true)
                {
                    ESP_LOGW(MQTT_MANAGER, "start_sta : %d", start_sta);
                    gpio_set_level(GPIO_OUT_STS_LED, led_Status);
                    led_Status = !led_Status;
                }
                ESP_LOGW(MQTT_MANAGER, "connecting to 3");
                ESP_LOGW(MQTT_MANAGER, "connecting to [host= %s:%d] : [err= %d]",
                         mqttInitParams.pHostURL, mqttInitParams.port, retVal);
                vTaskDelay((2 * SEC_TO_MS) / portTICK_RATE_MS); /* sleep for 2000 ms */
                ESP_LOGW(MQTT_MANAGER, "connecting to 4");
            }
            ESP_LOGW(MQTT_MANAGER, "connecting to 5");
        } while (SUCCESS != retVal);
        gpio_set_level(GPIO_OUT_STS_LED, 0);
        ESP_LOGI(MQTT_MANAGER, "Connected to AWS service");

        /* Enable auto reconnect */
        retVal = aws_iot_mqtt_autoreconnect_set_status(&mqttClient, true);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Unable to set Auto Reconnect : [err= %d]", retVal);
            break;
        }
        ESP_LOGI(MQTT_MANAGER, "MAC address of device : [%s]", macAddrStr);

        /* subscribe to ( /smarttouch/{dev-id}/control ) */
        sprintf(Topic_Sw_Control, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_SW_CONTROL);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Sw_Control);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Sw_Control, strlen(Topic_Sw_Control), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Sw_Control, retVal);
            break;
        }

        /* subscribe to ( /smarttouch/{dev-id}/getswstatus ) */
        sprintf(Topic_Req_Get_Sts, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_REQ_GET_STS);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Req_Get_Sts);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Req_Get_Sts, strlen(Topic_Req_Get_Sts), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Req_Get_Sts, retVal);
            break;
        }

        /* subscribe to ( /smarttouch/{dev-id}/restore ) */
        sprintf(Topic_Factory_Reset, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_FACTORY_RESET);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Factory_Reset);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Factory_Reset, strlen(Topic_Factory_Reset), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Factory_Reset, retVal);
            break;
        }

        /* subscribe to ( /smarttouch/{dev-id}/restart ) */
        sprintf(Topic_Dev_Restart, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_DEV_RESTART);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Dev_Restart);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Dev_Restart, strlen(Topic_Dev_Restart), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Dev_Restart, retVal);
            break;
        }

        /* subscribe to ( /smarttouch/{dev-id}/ota ) */
        sprintf(Topic_Ota_Check, "%s/%s", TOPIC_SMART_TOUCH, TOPIC_OTA_CHECK);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Ota_Check);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Ota_Check, strlen(Topic_Ota_Check), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Ota_Check, retVal);
            break;
        }

        /* subscribe to ( /smarttouch/{dev-id}/outdoor-mode-settings ) */
        sprintf(Topic_Out_Door, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_OUT_DOOR_MODE);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Out_Door);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Out_Door, strlen(Topic_Out_Door), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Out_Door, retVal);
            break;
        }

        /* subscribe to ( /smarttouch/{dev-id}/ack-status ) */
        sprintf(Topic_Ack_Status, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_ACK_STATUS);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Ack_Status);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Ack_Status, strlen(Topic_Ack_Status), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Ack_Status, retVal);
            break;
        }

        /* subscribe to (/smarttouch/{deviceid}/activation-status/)*/
        sprintf(Topic_Device_Staus, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_DEV_ACTIVE_STATUS);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Device_Staus);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Device_Staus, strlen(Topic_Device_Staus), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Device_Staus, retVal);
            break;
        }

        // Added By Sagar
        /* subscribe to ( /smarttouch/{dev-id}/sensor-status ) */
        ESP_LOGI(TAG, "Subscribing...");
        sprintf(Topic_Sensor_Status, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_SENSOR_STATUS);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Sensor_Status);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Sensor_Status, strlen(Topic_Sensor_Status), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(TAG, "Error subscribing : %d ", retVal);
            abort();
        }

        /* subscribe to ( /smarttouch/{dev-id}/get-delay ) */
        ESP_LOGI(TAG, "Subscribing...");
        sprintf(Topic_Get_Delay, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_GET_DELAY);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Get_Delay);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Get_Delay, strlen(Topic_Get_Delay), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(TAG, "Error subscribing : %d ", retVal);
            abort();
        }

        /* subscribe to ( /smarttouch/{dev-id}/modes ) */
        ESP_LOGI(TAG, "Subscribing...");
        sprintf(Topic_Get_Modes, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_GET_MODES);
        ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Get_Modes);
        retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Get_Modes, strlen(Topic_Get_Modes), QOS0, mqttReceiveCb, NULL);
        if (SUCCESS != retVal)
        {
            ESP_LOGE(TAG, "Error subscribing : %d ", retVal);
            abort();
        }
        else
        {
        }

        /**
         * @note Currently max topic subscription is limited to 7.
         *       To add another increase the limit by changing AWS_IOT_MQTT_NUM_SUBSCRIBE_HANDLERS
         *       in aws_iot_config.h
         */
        ESP_LOGI(MQTT_MANAGER, "controlStatus.isDevicesActivated : %d start_sta : %d ", controlStatus.isDevicesActivated, start_sta);

        if (controlStatus.isDevicesActivated == true && start_sta == true)
        {
            ESP_LOGI(MQTT_MANAGER, "Publishing Boot-up packet");
            /* publish device status packet on bootup */
            publishBootupPacket();
            controlStatus.isDevicesActivated = true;

            WriteControlStatus(&controlStatus);
        }
        else
        {
            do
            {
                publishBootupPacket();
                vTaskDelay(2 * SEC_TO_MS / portTICK_PERIOD_MS);
            } while (controlStatus.isDevicesActivated == true);
        }
        retVal = SUCCESS;
    } while (0);

    while (1)
    {
        esp_task_wdt_init(25, false);
        /* Receive incoming message from server & MQTT keep-alive (ping request) */
        retVal = aws_iot_mqtt_yield(&mqttClient, 500);
        if ((SUCCESS != retVal) && wifi_STA_Disconnected == false)
        {
            aws_iot_mqtt_disconnect(&mqttClient);
            aws_iot_mqtt_free(&mqttClient);

            ESP_LOGW(MQTT_MANAGER, "MQTT NEW CONNECTION RETRY");

            Device_DisConnect = true;

            /* Init mqtt parameters */
            mqttInitParams.enableAutoReconnect = false; // We enable this later below
            mqttInitParams.pHostURL = mqttHostUrl;
            mqttInitParams.port = AWS_IOT_MQTT_PORT;
            mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
            mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
            mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;
            mqttInitParams.mqttCommandTimeout_ms = MQTT_CMD_TIMEOUT;
            mqttInitParams.tlsHandshakeTimeout_ms = MQTT_HS_TIMEOUT;
            mqttInitParams.isSSLHostnameVerify = true;
            mqttInitParams.disconnectHandler = mqttDisconnectCb;
            mqttInitParams.disconnectHandlerData = NULL;
            mqttConnParams.isWillMsgPresent = true;

            uint8_t macAdd[6] = {0};
            /* get device MAC address */
            esp_efuse_mac_get_default(macAdd);
            sprintf(macAddrStr, MACSTR, MAC2STR(macAdd));

            /* Init connection parameters */
            mqttConnParams.keepAliveIntervalInSec = 10;
            mqttConnParams.isCleanSession = true;
            mqttConnParams.MQTTVersion = MQTT_3_1_1;
            mqttConnParams.pClientID = (const char *)macAddrStr; // aws_mqtt_clientid;
            ESP_LOGI(MQTT_MANAGER, "MAC address of device : [%s]", mqttConnParams.pClientID);
            mqttConnParams.clientIDLen = (uint16_t)strlen((const char *)macAddrStr);

            char WillTopic[MAX_TOPIC_LENGTH] = "";
            sprintf(WillTopic, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_STATUS);

            char will_message[50];
            sprintf(will_message, "{\"st\":\"0\",\"fwv\":\"%s\"}", FW);

            will_opt.isRetained = false;
            will_opt.msgLen = strlen(will_message);
            will_opt.pMessage = will_message;
            will_opt.pTopicName = WillTopic; // TOPIC_STATUS;
            will_opt.qos = 0;
            will_opt.topicNameLen = strlen(WillTopic);
            mqttConnParams.will = will_opt;

            /* end Will message packet */

            /* Init mqtt client */
            retVal = aws_iot_mqtt_init(&mqttClient, &mqttInitParams);
            if (SUCCESS != retVal)
            {
                ESP_LOGE(MQTT_MANAGER, "Failed to init mqtt client : [err= %d]", retVal);
                break;
            }

            retVal = aws_iot_mqtt_connect(&mqttClient, &mqttConnParams);
            ESP_LOGW(MQTT_MANAGER, "retVal : %d", retVal);
            if (SUCCESS != retVal)
            {
                if (is_WIFI_STA_Status == true)
                {
                    gpio_set_level(GPIO_OUT_STS_LED, led_Status);
                    led_Status = !led_Status;
                }

                ESP_LOGW(MQTT_MANAGER, "#@#connecting to [host= %s:%d] : [err= %d]",
                         mqttInitParams.pHostURL, mqttInitParams.port, retVal);
                vTaskDelay((2 * SEC_TO_MS) / portTICK_RATE_MS); /* sleep for 2000 ms */
                esp_task_wdt_init(25, false);
            }
            else
            {
                esp_task_wdt_init(25, false);
                ESP_LOGI(MQTT_MANAGER, "Waiting for wifi connection");

                /* Wait for wifi connection */
                while (!GetWifiConnectionStatus())
                {
                    /* wait for 1 sec */
                    if (is_WIFI_STA_Status == true)
                    {
                        gpio_set_level(GPIO_OUT_STS_LED, led_Status);
                        led_Status = !led_Status;
                    }
                    vTaskDelay(SEC_TO_MS / portTICK_RATE_MS);
                    esp_task_wdt_init(25, false);
                }
                gpio_set_level(GPIO_OUT_STS_LED, 0);
                ESP_LOGI(MQTT_MANAGER, "Connected to AWS service");

                /* Enable auto reconnect */
                retVal = aws_iot_mqtt_autoreconnect_set_status(&mqttClient, true);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(MQTT_MANAGER, "Unable to set Auto Reconnect : [err= %d]", retVal);
                    break;
                }
                ESP_LOGI(MQTT_MANAGER, "MAC address of device : [%s]", macAddrStr);

                /* subscribe to ( /smarttouch/{dev-id}/control ) */
                sprintf(Topic_Sw_Control, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_SW_CONTROL);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Sw_Control);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Sw_Control, strlen(Topic_Sw_Control), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Sw_Control, retVal);
                    break;
                }

                /* subscribe to ( /smarttouch/{dev-id}/getswstatus ) */
                sprintf(Topic_Req_Get_Sts, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_REQ_GET_STS);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Req_Get_Sts);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Req_Get_Sts, strlen(Topic_Req_Get_Sts), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Req_Get_Sts, retVal);
                    break;
                }

                /* subscribe to ( /smarttouch/{dev-id}/restore ) */
                sprintf(Topic_Factory_Reset, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_FACTORY_RESET);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Factory_Reset);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Factory_Reset, strlen(Topic_Factory_Reset), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Factory_Reset, retVal);
                    break;
                }

                /* subscribe to ( /smarttouch/{dev-id}/restart ) */
                sprintf(Topic_Dev_Restart, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_DEV_RESTART);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Dev_Restart);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Dev_Restart, strlen(Topic_Dev_Restart), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Dev_Restart, retVal);
                    break;
                }

                /* subscribe to ( /smarttouch/{dev-id}/ota ) */
                sprintf(Topic_Ota_Check, "%s/%s", TOPIC_SMART_TOUCH, TOPIC_OTA_CHECK);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Ota_Check);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Ota_Check, strlen(Topic_Ota_Check), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Ota_Check, retVal);
                    break;
                }

                /* subscribe to ( /smarttouch/{dev-id}/outdoor-mode-settings ) */
                sprintf(Topic_Out_Door, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_OUT_DOOR_MODE);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Out_Door);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Out_Door, strlen(Topic_Out_Door), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Out_Door, retVal);
                    break;
                }

                /* subscribe to ( /smarttouch/{dev-id}/ack-status ) */
                sprintf(Topic_Ack_Status, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_ACK_STATUS);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Ack_Status);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Ack_Status, strlen(Topic_Ack_Status), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(MQTT_MANAGER, "Error subscribing topic [%s] : [err= %d]", Topic_Ack_Status, retVal);
                    break;
                }

                // Added By Sagar
                /* subscribe to ( /smarttouch/{dev-id}/sensor-status ) */
                ESP_LOGI(TAG, "Subscribing...");
                sprintf(Topic_Sensor_Status, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_SENSOR_STATUS);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Sensor_Status);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Sensor_Status, strlen(Topic_Sensor_Status), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(TAG, "Error subscribing : %d ", retVal);
                    abort();
                }

                /* subscribe to ( /smarttouch/{dev-id}/get-delay ) */
                ESP_LOGI(TAG, "Subscribing...");
                sprintf(Topic_Get_Delay, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_GET_DELAY);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Get_Delay);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Get_Delay, strlen(Topic_Get_Delay), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(TAG, "Error subscribing : %d ", retVal);
                    abort();
                }

                /* subscribe to ( /smarttouch/{dev-id}/modes ) */
                ESP_LOGI(TAG, "Subscribing...");
                sprintf(Topic_Get_Modes, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_GET_MODES);
                ESP_LOGI(MQTT_MANAGER, "Subscribing to topic [%s]", Topic_Get_Modes);
                retVal = aws_iot_mqtt_subscribe(&mqttClient, Topic_Get_Modes, strlen(Topic_Get_Modes), QOS0, mqttReceiveCb, NULL);
                if (SUCCESS != retVal)
                {
                    ESP_LOGE(TAG, "Error subscribing : %d ", retVal);
                    abort();
                }

                /**
                 * @note Currently max topic subscription is limited to 7.
                 *       To add another increase the limit by changing AWS_IOT_MQTT_NUM_SUBSCRIBE_HANDLERS
                 *       in aws_iot_config.h
                 */

                /* publish device status packet on bootup */
                publishBootupPacket();
            }
        }

        if (NETWORK_ATTEMPTING_RECONNECT == retVal)
        {
            ESP_LOGI(MQTT_MANAGER, "NETWORK_ATTEMPTING_RECONNECT");
            /* If the client is attempting to reconnect we will skip the rest of the loop */
            if (is_WIFI_STA_Status == true)
            {
                try_to_connect_count++;
                ESP_LOGI(MQTT_MANAGER, "TRY TO Connect");
                if (try_to_connect_count >= 4)
                {
                    gpio_set_level(GPIO_OUT_STS_LED, led_Status);
                    led_Status = !led_Status;
                }
                vTaskDelay((2 * SEC_TO_MS) / portTICK_RATE_MS); /* sleep for 2000 ms */
            }
            continue;
        }
        else if (NETWORK_RECONNECTED == retVal)
        {
            try_to_connect_count = 0;
            gpio_set_level(GPIO_OUT_STS_LED, 0);
            ESP_LOGI(MQTT_MANAGER, "Reconnected to MQTT");
            publishBootupPacket();
            continue;
        }
        /* after mqtt yield check if any event is pending to be send to cloud */
        if (doPublishStatus == true)
        {
            /*  if true publish the switch status */
            doPublishStatus = false;
            publishSwitchStatus();
            publishSensorStatus();
        }
        if (DeviceFactoryReset == true)
        {
            publishRestoreFactoryStatus();
            DeviceFactoryReset = false;
        }

        if (true == SwitchStatusUpdate)
        {
            publishSwitchStatus();
            SwitchStatusUpdate = false;
        }
    }

    ESP_LOGI(MQTT_MANAGER, "MQTT Return:%d", retVal);
    DoSystemRestart();
    ESP_LOGE(MQTT_MANAGER, "An error occurred in the main loop.");
    vTaskDelete(NULL);
}

/**
 * @brief MQTT Disconnect Call back
 *
 * @param pClient
 * @param data
 */
static void mqttDisconnectCb(AWS_IoT_Client *pClient, void *data)
{
    ESP_LOGW(MQTT_MANAGER, "MQTT client disconnected : reconnecting");
}

/**
 * @brief MQTT message receive callback
 *
 * @param pClient
 * @param topicName
 * @param topicNameLen
 * @param params
 * @param pData
 */
static void mqttReceiveCb(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen, IoT_Publish_Message_Params *params, void *pData)
{
    char payload[1000];
    snprintf(payload, params->payloadLen + 1, "%s", (char *)params->payload);
    ESP_LOGD(MQTT_MANAGER, "Payload_Received:\n%s", topicName);

    /*Newly subcribed topic ack-status*/
    if (strncmp(topicName, Topic_Sw_Control, strlen(Topic_Sw_Control)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Switch control request received");
        parseSwitchControlRequest((char *)params->payload);
    }
    else if (strncmp(topicName, Topic_Req_Get_Sts, strlen(Topic_Req_Get_Sts)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Request for get switch status received");
        publishSwitchStatus();
    }
    else if (strncmp(topicName, Topic_Factory_Reset, strlen(Topic_Factory_Reset)) == 0)
    {
        DoFactoryReset();
    }
    else if (strncmp(topicName, Topic_Dev_Restart, strlen(Topic_Dev_Restart)) == 0)
    {
        DoSystemRestart();
    }
    else if (strncmp(topicName, Topic_Ack_Status, strlen(Topic_Ack_Status)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Activation Status Response received");
        parseActiveStatusRequest((char *)params->payload);
    }
    else if (strncmp(topicName, Topic_Out_Door, strlen(Topic_Out_Door)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Outdoor mode setting received");
        parseOutdoorSetting((char *)params->payload);
    }
    else if (strncmp(topicName, Topic_Device_Staus, strlen(Topic_Device_Staus)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Activation Status received from cloud");
        DeviceActivationStatus((char *)params->payload);
    }
    else if (strncmp(topicName, Topic_Ota_Check, strlen(Topic_Ota_Check)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Request received for application update via OTA");
        otaPartitionInit();
        checkForOta();
        publishBootupPacket(); /*24082022 this line is added for sending same version acknowledgement to mobile app */
    }

    // Added By Sagar
    else if (strncmp(topicName, Topic_Sensor_Status, strlen(Topic_Sensor_Status)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Request for get switch status received");
        publishSensorStatus();
    }

    else if (strncmp(topicName, Topic_Get_Delay, strlen(Topic_Get_Delay)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Get Delay received from cloud");
        parseGetDelayRequest((char *)params->payload);
    }

    else if (strncmp(topicName, Topic_Get_Modes, strlen(Topic_Get_Modes)) == 0)
    {
        ESP_LOGD(MQTT_MANAGER, "Modes Selection received from cloud");
        parseGetModesRequest((char *)params->payload);
    }
}

// #endif
/**
 * @brief Publish boot up packet
 *
 */
static IoT_Error_t publishBootupPacket(void)
{
    IoT_Error_t retVal = FAILURE;
    char *publishPayload = NULL;
    cJSON *pRootObj = cJSON_CreateObject(); /* create json root object */
    if (pRootObj == NULL)
    {
        ESP_LOGE(MQTT_MANAGER, "Failed to create json object");
        return FAILURE;
    }

    do
    {
        ESP_LOGI(MQTT_MANAGER, "controlStatus.isDevicesActivated  controlStatus.isDevicesActivated : %d", controlStatus.isDevicesActivated);

        if (cJSON_AddStringToObject(pRootObj, "st", "1") == NULL)
        {
            ESP_LOGI(MQTT_MANAGER, "Failed to add element 'st' to object");
            break;
        }

        char fwVersion[5];
        sprintf(fwVersion, "%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR);

        if (cJSON_AddStringToObject(pRootObj, "fwv", FW) == NULL)
        {
            ESP_LOGI(MQTT_MANAGER, "Failed to add element 'fwv' to object");
            break;
        }

        if (cJSON_AddStringToObject(pRootObj, "uid", config.uniqueid) == NULL)
        {
            ESP_LOGI(MQTT_MANAGER, "Failed to add element 'unique_id' to object");
            break;
        }

        publishPayload = cJSON_Print(pRootObj);
        ESP_LOGD(MQTT_MANAGER, "payload:\n%s\n", publishPayload);

        IoT_Publish_Message_Params publishParam;
        publishParam.qos = QOS0;
        publishParam.isRetained = false;
        publishParam.payload = publishPayload;
        publishParam.payloadLen = strlen(publishPayload);

        char publishTopic[MAX_TOPIC_LENGTH] = "";
        sprintf(publishTopic, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_STATUS);
        ESP_LOGI(MQTT_MANAGER, "Publishing on topic [%s]", publishTopic);

        retVal = aws_iot_mqtt_publish(&mqttClient, publishTopic, strlen(publishTopic), &publishParam);
        if (retVal != SUCCESS)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to publish on topic [%s] : [err= %d]", publishTopic, retVal);
            break;
        }

        retVal = SUCCESS;
    } while (0);

    /* free payload */
    if (publishPayload != NULL)
    {
        free(publishPayload);
    }

    /* delete json object */
    cJSON_Delete(pRootObj);

    return retVal;
}

/**
 * @brief Publish packet after checking for OTA
 *
 */
IoT_Error_t checkForOtaPacket(void)
{
    IoT_Error_t retVal = FAILURE;
    char *publishPayload = NULL;
    cJSON *pRootObj = cJSON_CreateObject(); /* create json root object */
    if (pRootObj == NULL)
    {
        ESP_LOGE(MQTT_MANAGER, "Failed to create json object");
        return FAILURE;
    }

    do
    {
        if (cJSON_AddStringToObject(pRootObj, "fwv", FW) == NULL)
        {
            ESP_LOGI(MQTT_MANAGER, "Failed to add element 'fwv' to object");
            break;
        }

        if (cJSON_AddNumberToObject(pRootObj, "DT", DEVICE_TYPE == DEVICE_TYPE_SMARTAP_4 ? 4 : 8) == NULL)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to add element 'DT'");
            break;
        }

        publishPayload = cJSON_Print(pRootObj);
        ESP_LOGD(MQTT_MANAGER, "payload:\n%s\n", publishPayload);

        IoT_Publish_Message_Params publishParam;
        publishParam.qos = QOS0;
        publishParam.isRetained = false;
        publishParam.payload = publishPayload;
        publishParam.payloadLen = strlen(publishPayload);

        char publishTopic[MAX_TOPIC_LENGTH] = "";
        sprintf(publishTopic, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_STATUS);
        ESP_LOGI(MQTT_MANAGER, "Publishing on topic [%s]", publishTopic);

        retVal = aws_iot_mqtt_publish(&mqttClient, publishTopic, strlen(publishTopic), &publishParam);
        if (retVal != SUCCESS)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to publish on topic [%s] : [err= %d]", publishTopic, retVal);
            break;
        }

        retVal = SUCCESS;
    } while (0);

    /* free payload */
    if (publishPayload != NULL)
    {
        free(publishPayload);
    }

    /* delete json object */
    cJSON_Delete(pRootObj);

    return retVal;
}

/**
 * @brief Parse activation status received from cloud
 *        - Enable respective Switches/Dimmer
 *
 * @param pPayload
 */
static void parseActivationStatus(char *pPayload)
{
    if (false == Device_DisConnect)
    {
        cJSON *activeStatus = cJSON_Parse(pPayload);
        cJSON *switchDevices = NULL;
        cJSON *switchDevice = NULL;
        int om_state = 0;
        bool All_SW_Off = false;
        do
        {
            /* parsing error */
            if (activeStatus == NULL)
            {
                const char *errPtr = cJSON_GetErrorPtr();
                if (errPtr != NULL)
                {
                    ESP_LOGE(MQTT_MANAGER, "Error while parsing : %s", errPtr);
                }
                break;
            }

            if (cJSON_HasObjectItem(activeStatus, JSON_KEY_STATUS))
            {

                cJSON *state1 = cJSON_GetObjectItemCaseSensitive(activeStatus, JSON_KEY_R_STATE);
                if (state1->valuestring == NULL)
                {
                    ESP_LOGW(MQTT_MANAGER, "Error while parsing activation status");
                    break;
                }
                swReq.RetainState = atoi(state1->valuestring);
                ESP_LOGD(MQTT_MANAGER, "Retain Status  : [%s= %d]", JSON_KEY_STATUS, swReq.RetainState);

                /* Parse Outdoor mode state  */
                cJSON *om = cJSON_GetObjectItemCaseSensitive(activeStatus, JSON_KEY_OUT_DOOR);
                if (om->valuestring == NULL)
                {
                    ESP_LOGW(MQTT_MANAGER, "Error while parsing activation status");
                    break;
                }

                om_state = atoi(om->valuestring);
               // outdoor_mode_state = om_state;
                ESP_LOGD(MQTT_MANAGER, "Out Door Mode Status  : [%s= %d]", JSON_KEY_OUT_DOOR, om_state);
            }

            switchDevices = cJSON_GetObjectItemCaseSensitive(activeStatus, JSON_KEY_SWD);
            cJSON_ArrayForEach(switchDevice, switchDevices)
            {
                ESP_LOGW(MQTT_MANAGER, "@INTERNET CONNECT RECEIVE PACKET@");
                cJSON *swName = cJSON_GetObjectItemCaseSensitive(switchDevice, JSON_KEY_NAME);
                cJSON *swValue = cJSON_GetObjectItemCaseSensitive(switchDevice, JSON_KEY_VALUE);

                if ((swName == NULL) || (swName->valuestring == NULL))
                {
                    ESP_LOGW(MQTT_MANAGER, "Error while parsing switch index");
                    break;
                }

                if ((swValue == NULL) || (swValue->valuestring == NULL))
                {
                    ESP_LOGW(MQTT_MANAGER, "Error while parsing switch index");
                    break;
                }

                int switchId, switchValue;

                if (om_state == 1)
                {
                    controlStatus.outdoorMode = true;
                    cJSON *SW = cJSON_GetObjectItem(activeStatus, JSON_KEY_OM_SW);

                    char *val = cJSON_Print(SW);

                    ESP_LOGI(MQTT_MANAGER, "SW %s", val);

                    int size = cJSON_GetArraySize(SW);
                    ESP_LOGI(MQTT_MANAGER, "nArray %d", size);

                    cJSON *state1 = cJSON_GetObjectItemCaseSensitive(activeStatus, JSON_OD_DIM);
                    if (state1->valuestring == NULL)
                    {
                        ESP_LOGW(MQTT_MANAGER, "Error while parsing activation status");
                        break;
                    }

                    for (int i = 0; i < size; i++)
                    {
                        cJSON *elem = cJSON_GetArrayItem(SW, i);
                        char *arr = cJSON_Print(elem);
                        printf("\nelem  in Outdoor Mode %s\n", arr);
                        printf("\n#######elem  in Outdoor Mode %s\n", JSON_KEY_SW01);

                        if (strcmp(arr, JSON_KEY_SW01) == 0)
                        {
                            // gpio_set_level(GPIO_OUT_RELAY1, GPIO_HIGH);
                            controlStatus.switchStatus[SW01] = 1;
                            sw_state1 = 1;
                            ESP_LOGI(MQTT_MANAGER, "JSON_KEY_SW01 is On in Outdoor Mode %s", arr);
                        }
                    }
                    SwitchStatusUpdate = true;
                }
                else if (swReq.RetainState == true)
                {
                    sscanf(swName->valuestring, "SW%d", &switchId);
                    switchValue = atoi(swValue->valuestring);
                    ESP_LOGI(MQTT_MANAGER, "switch[%d] = %d", switchId, switchValue);

                    if ((switchId <= SW_MAX) && (switchId > 0))
                    {
                        if (switchId != SW_MAX) // Dimmer and USB c Switch Not Apply
                        {
                            swReq.Rs_switchId[(switchId - 1)] = (switchId - 1);
                            swReq.Rs_value[(switchId - 1)] = switchValue;
                            ESP_LOGI(MQTT_MANAGER, "#@#switch[%d] = %d", swReq.Rs_switchId[(switchId - 1)], swReq.Rs_value[(switchId - 1)]);
                        }
                    }
                }
                else
                {
                    All_SW_Off = true;
                }
            }
        } while (0);

        if (om_state == 1)
        {
        }
        else if (true == swReq.RetainState)
        {
            for (int sw = 0; sw < SW_MAX; sw++)
            {

                ESP_LOGW(MQTT_MANAGER, "RS_SW_ID[%d] = %d", swReq.Rs_switchId[sw], swReq.Rs_value[sw]);
            }

            EnqueueSwitchRequest(swReq);
        }

        if (true == All_SW_Off)
        {
            // gpio_set_level(GPIO_OUT_RELAY1, GPIO_LOW);

            sw_state1 = 0;
            bzero(controlStatus.switchStatus, sizeof(controlStatus.switchStatus));
            swReq.value = 0;
            All_SW_Off = false;
            SwitchStatusUpdate = true;
        }

        cJSON_Delete(activeStatus);
    }
    else
    {
        SwitchStatusUpdate = true;
        ESP_LOGW(MQTT_MANAGER, "DEVICE IS DISCONNECT NOT CONSIDER RETAIN MODE");
    }
}

/**
 * @brief Parse switch control request and enable/disable switch
 *
 * @param pPayload
 */
static void parseSwitchControlRequest(char *pPayload)
{
    cJSON *controlRequest = cJSON_Parse(pPayload);
    bool status = false;

    do
    {
        /* parsing error */
        if (controlRequest == NULL)
        {
            const char *errPtr = cJSON_GetErrorPtr();
            if (errPtr != NULL)
            {
                ESP_LOGE(MQTT_MANAGER, "Error while parsing : %s", errPtr);
            }
            break;
        }

        char tmpStr[20] = "";
        bool switchState = 0;

        /* find element in the json object */
        for (int sw = SW01; sw < SW_MAX; sw++)
        {
            sprintf(tmpStr, JSON_KEY_SW "0%d", sw + 1);
            if (cJSON_HasObjectItem(controlRequest, tmpStr))
            {
                cJSON *state = cJSON_GetObjectItemCaseSensitive(controlRequest, tmpStr);
                if (state->valuestring == NULL)
                {
                    ESP_LOGW(MQTT_MANAGER, "Error while parsing switch state");
                    break;
                }

                switchState = atoi(state->valuestring);
                ESP_LOGI(MQTT_MANAGER, "##Switch control : [%s= %d]", tmpStr, switchState);

                swReq.switchId = sw;

                swReq.value = switchState;

                EnqueueSwitchRequest(swReq);
                status = true;
                break;
            }
        }

        if (status == true)
        {
            ESP_LOGD(MQTT_MANAGER, "SW Index Found");
            break;
        }

    } while (0);

    cJSON_Delete(controlRequest);
}

/**
 * @brief Parse Activation Status Response from Cloud
 *
 * @param pPayload
 */

static void parseActiveStatusRequest(char *pPayload)
{
    cJSON *ackstatus = cJSON_Parse(pPayload);

    do
    {
        /* parsing error */
        if (ackstatus == NULL)
        {
            const char *errPtr = cJSON_GetErrorPtr();
            if (errPtr != NULL)
            {
                ESP_LOGE(MQTT_MANAGER, "Error while parsing : %s", errPtr);
            }
            break;
        }

        /* check if status key is present */
        if (cJSON_HasObjectItem(ackstatus, JSON_KEY_STATUS))
        {
            cJSON *state = cJSON_GetObjectItemCaseSensitive(ackstatus, JSON_KEY_STATUS);
            if (state->valuestring == NULL)
            {
                ESP_LOGW(MQTT_MANAGER, "Error while parsing activation status");
                break;
            }
            isAckStatusReceived = atoi(state->valuestring);
            ESP_LOGD(MQTT_MANAGER, "Activation Status  : [%s= %d]", JSON_KEY_STATUS, isAckStatusReceived);
        }

        /* if activation status is true */
        if (isAckStatusReceived == true)
        {
            /* publish switch status */
            /* parse device activation status and switch status */
            parseActivationStatus(pPayload);
        }
        else
        {
            if (controlStatus.isDevicesActivated == true)
            {
                /* send device status again */
                vTaskDelay(2 * SEC_TO_MS / portTICK_PERIOD_MS);
                publishBootupPacket();
            }
        }

    } while (0);

    cJSON_Delete(ackstatus);
}

/**
 * @brief Publish switch status
 *
 * @return IoT_Error_t
 */
static IoT_Error_t publishSwitchStatus(void)
{
    IoT_Error_t retVal = FAILURE;
    char *publishPayload = NULL;

    ESP_LOGW(MQTT_MANAGER, "SSID: %s PASS: %s", config.ssid, config.password);

    cJSON *pRootObj = cJSON_CreateObject();
    if (pRootObj == NULL)
    {
        ESP_LOGE(MQTT_MANAGER, "Failed to create json object");
        return retVal;
    }

    do
    {

        if (cJSON_AddNumberToObject(pRootObj, "DT", DEVICE_TYPE == DEVICE_TYPE_SMARTAP_4 ? 4 : 8) == NULL)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to add element 'DT'");
            break;
        }

        uint8_t SW_Ind = 0;
        SW_Ind = 1;
        char switchState[(3 * 2) + 1] = "";
        int strLen = 0;
        for (int sw = SW01; sw <= SW_Ind; sw++)
        {
            if (sw < (SW_Ind))
            {
                strLen += sprintf(&switchState[strLen], "%d,", GetSwitchValue(sw));
            }
            else
            {
                strLen += sprintf(&switchState[strLen], "%d", GetSwitchValue(sw));
            }
        }
        if (cJSON_AddStringToObject(pRootObj, "SW", switchState) == NULL)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to add element 'SW'");
            break;
        }

        publishPayload = cJSON_Print(pRootObj);
        ESP_LOGD(MQTT_MANAGER, "payload:\n%s\n", publishPayload);

        IoT_Publish_Message_Params publishParam;
        publishParam.qos = QOS0;
        publishParam.isRetained = false;
        publishParam.payload = publishPayload;
        publishParam.payloadLen = strlen(publishPayload);

        char publishTopic[MAX_TOPIC_LENGTH] = "";
        sprintf(publishTopic, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_RES_GET_STS);
        ESP_LOGI(MQTT_MANAGER, "Publishing on topic [%s]", publishTopic);

        retVal = aws_iot_mqtt_publish(&mqttClient, publishTopic, strlen(publishTopic), &publishParam);
        if (retVal != SUCCESS)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to publish on topic [%s] : [err= %d]", publishTopic, retVal);
            break;
        }

        retVal = SUCCESS;
    } while (0);

    /* free payload */
    if (publishPayload != NULL)
    {
        free(publishPayload);
    }

    /* delete json object */
    cJSON_Delete(pRootObj);

    return retVal;
}

/**
 * @brief Parse Device Activation Status
 *
 * @param pPayload
 */
static void DeviceActivationStatus(char *pPayload)
{
    uint8_t lu8_DeviceRegistrationStatus = 0;

    cJSON *DeviceActivationSatus = cJSON_Parse(pPayload);

    char *DeviceRegistrationStaus = cJSON_Print(DeviceActivationSatus);

    ESP_LOGW(MQTT_MANAGER, "Device Registraion packet %s", DeviceRegistrationStaus);

    do
    {
        /* parsing error */
        if (DeviceRegistrationStaus == NULL)
        {
            const char *errPtr = cJSON_GetErrorPtr();
            if (errPtr != NULL)
            {
                ESP_LOGE(MQTT_MANAGER, "Error while parsing : %s", errPtr);
            }
            break;
        }

        /* check if Device status  is present */
        if (cJSON_HasObjectItem(DeviceActivationSatus, JSON_KEY_DEVICE_STATUS))
        {
            cJSON *state = cJSON_GetObjectItemCaseSensitive(DeviceActivationSatus, JSON_KEY_DEVICE_STATUS);

            if (state->valuestring == NULL)
            {
                ESP_LOGE(MQTT_MANAGER, "Invalid value recevied Device Registration status");
                break;
            }

            lu8_DeviceRegistrationStatus = atoi(state->valuestring);
            controlStatus.isDevicesActivated = lu8_DeviceRegistrationStatus;
            WriteControlStatus(&controlStatus);
            ESP_LOGD(MQTT_MANAGER, "Device Registration state  : [%s= %d]", JSON_KEY_DEVICE_STATUS, atoi(state->valuestring));

            if (lu8_DeviceRegistrationStatus == 1)
            {
                ESP_LOGW(MQTT_MANAGER, "@Publishing Boot-up packet@");
                publishBootupPacket(); // Publish Activation Status
            }
        }

    } while (0);

    cJSON_Delete(DeviceActivationSatus);
}

/**
 * @brief Parse out door settings
 *
 * @param pPayload
 */
static void parseOutdoorSetting(char *pPayload)
{
    cJSON *outdoorSettings = cJSON_Parse(pPayload);

    char *OM = cJSON_Print(outdoorSettings);

    ESP_LOGE(MQTT_MANAGER, "OM packet %s", OM);

    do
    {
        /* parsing error */
        if (outdoorSettings == NULL)
        {
            const char *errPtr = cJSON_GetErrorPtr();
            if (errPtr != NULL)
            {
                ESP_LOGE(MQTT_MANAGER, "Error while parsing : %s", errPtr);
            }
            break;
        }

        /* check if status key is present */
        if (cJSON_HasObjectItem(outdoorSettings, JSON_KEY_OUT_DOOR))
        {
            cJSON *state = cJSON_GetObjectItemCaseSensitive(outdoorSettings, JSON_KEY_OUT_DOOR);

            if (state->valuestring == NULL)
            {
                ESP_LOGE(MQTT_MANAGER, "Invalid value recevied in outdoor settings");
                break;
            }

            ESP_LOGD(MQTT_MANAGER, "OUtdoor mode state  : [%s= %d]", JSON_KEY_OUT_DOOR, atoi(state->valuestring));
            // gpio_set_level(GPIO_OUT_RELAY1, GPIO_LOW);

            sw_state1 = 0;

            bzero(controlStatus.switchStatus, sizeof(controlStatus.switchStatus));
            swReq.value = 0;

            if (atoi(state->valuestring) == true)
            {
                controlStatus.outdoorMode = true;
                /* Disable zcd */

                cJSON *SW = cJSON_GetObjectItem(outdoorSettings, "SW");

                char *val = cJSON_Print(SW);
                ESP_LOGI(MQTT_MANAGER, "SW %s", val);

                int size = cJSON_GetArraySize(SW);
                ESP_LOGI(MQTT_MANAGER, "Array Size %d", size);

                for (int i = 0; i < size; i++)
                {
                    cJSON *elem = cJSON_GetArrayItem(SW, i);
                    char *arr = cJSON_Print(elem);
                    ESP_LOGI(MQTT_MANAGER, "in Outdoor Mode %s", arr);

                    if (strcmp(arr, JSON_KEY_SW01) == 0)
                    {
                        // gpio_set_level(GPIO_OUT_RELAY1, GPIO_HIGH);
                        controlStatus.switchStatus[SW01] = 1;
                        sw_state1 = 1;
                        ESP_LOGI(MQTT_MANAGER, "JSON_KEY_SW01 is On in Outdoor Mode %s", arr);
                    }
                }
            }
            else if (atoi(state->valuestring) == false)
            {
                cJSON *SW = cJSON_GetObjectItem(outdoorSettings, "SWd");

                char *val = cJSON_Print(SW);
                ESP_LOGI(MQTT_MANAGER, "SWd %s", val);

                int size = cJSON_GetArraySize(SW);
                ESP_LOGI(MQTT_MANAGER, "Array # Size %d", size);

                for (int i = 0; i < size; i++)
                {
                    cJSON *elem = cJSON_GetArrayItem(SW, i);
                    char *arr = cJSON_Print(elem);
                    ESP_LOGI(MQTT_MANAGER, "Outdoor Mode OFF %s", arr);

                    if (strcmp(arr, JSON_KEY_SW01) == 0)
                    {
                        // gpio_set_level(GPIO_OUT_RELAY1, GPIO_HIGH);
                        controlStatus.switchStatus[SW01] = 1;
                        sw_state1 = 1;
                        ESP_LOGI(MQTT_MANAGER, "JSON_KEY_SW01 is off in Outdoor Mode %s", arr);
                    }
                }

                controlStatus.outdoorMode = false;
                /* Write outdoor mode setting */
            }

            /* set publish flag for updating switch status to cloud */
            SetPublishFlag();

            /* Publishing OutDoor Settings Ack Packet */
            publishOutDoorModeSettingPacket(atoi(state->valuestring), OM);
        }

    } while (0);

    cJSON_Delete(outdoorSettings);
}

/**
 * @brief Set the Publish flag to notify the mqtt task that switch status has
 *        to be publish.
 *        - flag will be reset by mqtt task after publishing the status
 *
 */
void SetPublishFlag(void)
{
    doPublishStatus = true;
}

static IoT_Error_t publishOutDoorModeSettingPacket(bool OD_state, char *pPayload)
{
    IoT_Error_t retVal = FAILURE;
    char *publishPayload = NULL;
    publishPayload = pPayload;

    do
    {
        ESP_LOGD(MQTT_MANAGER, "payload:\n%s\n", publishPayload);

        IoT_Publish_Message_Params publishParam;
        publishParam.qos = QOS0;
        publishParam.isRetained = false;
        publishParam.payload = publishPayload;
        publishParam.payloadLen = strlen(publishPayload);

        char publishTopic[MAX_TOPIC_LENGTH] = "";
        sprintf(publishTopic, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_ACK_OUT_DOOR_MODE);
        ESP_LOGI(MQTT_MANAGER, "Publishing on topic [%s]", publishTopic);

        retVal = aws_iot_mqtt_publish(&mqttClient, publishTopic, strlen(publishTopic), &publishParam);
        if (retVal != SUCCESS)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to publish on topic [%s] : [err= %d]", publishTopic, retVal);
            break;
        }
    } while (0);
    return retVal;
}

IoT_Error_t publishRestoreFactoryStatus(void)
{
    IoT_Error_t retVal = FAILURE;
    char *publishPayload = NULL;

    cJSON *pRootObj = cJSON_CreateObject();

    if (pRootObj == NULL)
    {
        ESP_LOGE(MQTT_MANAGER, "Failed to create json object");
        return retVal;
    }

    do
    {
        if (cJSON_AddNumberToObject(pRootObj, "phr", 1) == NULL)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to add element 'phr'");
            break;
        }

        publishPayload = cJSON_Print(pRootObj);
        ESP_LOGD(MQTT_MANAGER, "payload:\n%s\n", publishPayload);

        IoT_Publish_Message_Params publishParam;
        publishParam.qos = QOS0;
        publishParam.isRetained = false;
        publishParam.payload = publishPayload;
        publishParam.payloadLen = strlen(publishPayload);

        char publishTopic[MAX_TOPIC_LENGTH] = "";

        sprintf(publishTopic, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_ACK_FACTORY_RESET);
        ESP_LOGI(MQTT_MANAGER, "Publishing on topic [%s]", publishTopic);

        retVal = aws_iot_mqtt_publish(&mqttClient, publishTopic, strlen(publishTopic), &publishParam);
        if (retVal != SUCCESS)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to publish on topic [%s] : [err= %d]", publishTopic, retVal);
            break;
        }

        retVal = SUCCESS;
    } while (0);

    /* free payload */
    if (publishPayload != NULL)
    {
        free(publishPayload);
    }

    /* delete json object */
    cJSON_Delete(pRootObj);

    return retVal;
}

// Added By sagar
static IoT_Error_t publishSensorStatus(void)
{
    IoT_Error_t retVal = FAILURE;
    char *publishPayload = NULL;

    ESP_LOGW(MQTT_MANAGER, "Sagar SSID: %s Sagar PASS: %s", config.ssid, config.password);

    cJSON *pRootObj = cJSON_CreateObject();
    if (pRootObj == NULL)
    {
        ESP_LOGE(MQTT_MANAGER, "Failed to create json object");
        return retVal;
    }

    do
    {
        if (cJSON_AddNumberToObject(pRootObj, "DT", 10) == NULL)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to add element 'DT'");
            break;
        }

        if (cJSON_AddNumberToObject(pRootObj, "Sensor_State", Occ_Flag) == NULL)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to add element 'SW'");
            break;
        }
        if (cJSON_AddNumberToObject(pRootObj, "LDR_Sensor_Value", ambient_light) == NULL)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to add element 'SW'");
            break;
        }

        publishPayload = cJSON_Print(pRootObj);
        ESP_LOGD(MQTT_MANAGER, "payload:\n%s\n", publishPayload);

        IoT_Publish_Message_Params publishParam;
        publishParam.qos = QOS1;
        publishParam.isRetained = false;
        publishParam.payload = publishPayload;
        publishParam.payloadLen = strlen(publishPayload);

        char publishTopic[MAX_TOPIC_LENGTH] = "";
        sprintf(publishTopic, "%s/%s%s", TOPIC_SMART_TOUCH, macAddrStr, TOPIC_SENSOR_STATUS);
        ESP_LOGI(MQTT_MANAGER, "Publishing on topic Sagar[%s]", publishTopic);

        retVal = aws_iot_mqtt_publish(&mqttClient, publishTopic, strlen(publishTopic), &publishParam);
        if (retVal != SUCCESS)
        {
            ESP_LOGE(MQTT_MANAGER, "Failed to publish on topic [%s] : [err= %d]", publishTopic, retVal);
            break;
        }

        retVal = SUCCESS;
    } while (0);

    /* free payload */
    if (publishPayload != NULL)
    {
        free(publishPayload);
    }

    /* delete json object */
    cJSON_Delete(pRootObj);

    return retVal;
}

// Added by Sagar
static void parseGetDelayRequest(char *pPayload)
{
    // const cJSON *delay = NULL;
    // int status = 0;
    ESP_LOGI(MQTT_MANAGER, "Sagar");
    cJSON *controlRequest = cJSON_Parse(pPayload);
    ESP_LOGI(TAG, "Payload_Received %s", pPayload);

    if (cJSON_GetObjectItem(controlRequest, "delay"))
    {
        char *delay = cJSON_GetObjectItem(controlRequest, "delay")->valuestring;
        ESP_LOGI(TAG, "delay=%s", delay);
        Un_delay = atoi(delay);
        ESP_LOGI(TAG, "Delay qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq %d", Un_delay);
        sensorConfig.time_delay = Un_delay;
        WritePIRDelay(&sensorConfig);
        ESP_LOGI(TAG, "Delay Time delay MQTT %lld", sensorConfig.time_delay);
    }

    cJSON_Delete(controlRequest);
}

static void parseGetModesRequest(char *pPayload)
{
    // const cJSON *modes = NULL;
    // int status = 0;
    cJSON *controlRequest = cJSON_Parse(pPayload);
    ESP_LOGI(TAG, "Payload_Received %s", pPayload);

    if (cJSON_GetObjectItem(controlRequest, "modes"))
    {
        char *modes = cJSON_GetObjectItem(controlRequest, "modes")->valuestring;
        int mode = atoi(modes);
        sensorConfig.mode_select = mode;
        WriteSensorModes(&sensorConfig);
        ESP_LOGI(TAG, "Occupancy Sensor Modes: %lld", sensorConfig.mode_select);
        esp_restart();
    }
    cJSON_Delete(controlRequest);
}
//--------------------------------------------------------------------------
// End of File
//--------------------------------------------------------------------------