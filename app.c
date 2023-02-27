
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
/* Application headers */
#include "smartap_networkManager.h"
#include "smartap_mqttClient.h"
// #include "smartap_switchControl.h"
#include "smartap_configModule.h"
#include "smartap_ota.h"
#include "Smart_Occupancy_Sensor.h"
//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
#define GPIO_HW_RESET 19
#define HW_RESET_PRESS_TIMEOUT (2 * SEC_TO_US)
//--------------------------------------------------------------------------
// Static Variables
//--------------------------------------------------------------------------
static char *deviceTypeString[DEVICE_TYPE_MAX] =
    {
        "DEVICE_TYPE_SMARTAP_4",
        "DEVICE_TYPE_NONE"};
static EventGroupHandle_t eventGroup;
volatile int64_t startTime = 0;
extern uint8_t AWS_MQTT_CLIENT_ID[6];
//--------------------------------------------------------------------------
// Function Definition
//--------------------------------------------------------------------------
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    int pin = (int)arg;

    if (pin == GPIO_HW_RESET)
    {
        xEventGroupSetBitsFromISR(eventGroup, BIT0, NULL);
    }
}

bool DFactoryReset(void)
{
    return isFactoryReset;
}

/**
 * @brief Application Main Function
 *
 */
void app_main(void)
{
    ESP_LOGI(SMART_TAP, "[----------Smartap Application Started----------]");
    ESP_LOGI(SMART_TAP, "[Device Type= %s]", deviceTypeString[DEVICE_TYPE]);
    // ESP_LOGI(SMART_TAP, "Firmware Version: [%d.%d]", FW_VERSION_MAJOR, FW_VERSION_MINOR);

    /* init Configuration module */
    if (MODULE_SUCCESS != InitConfigurationModule())
    {
        abort();
    }

    /* init switch control module */ //Commented By Sagar
    if (MODULE_SUCCESS != InitSwitchControl())
    {
        abort();
    }

    /* Smart Occupancy module */
    if (MODULE_SUCCESS != PIRStatus())
    {
        abort();
    }

    /* init network manager */
    if (MODULE_SUCCESS != InitNetworkManager())
    {
        abort();
    }

    /* init mqtt client */
    if (MODULE_SUCCESS != InitMqttClient())
    {
        abort();
    }

    /* create event group for hw reset interrupt */
    eventGroup = xEventGroupCreate();

    /* hardware reset routine */
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE; /* edge trigger */
    io_conf.pin_bit_mask = (1ULL << GPIO_HW_RESET);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    /* isr handler for hardware reset pin */
    gpio_isr_handler_add(GPIO_HW_RESET, gpio_isr_handler, (void *)GPIO_HW_RESET);

    EventBits_t bits;
    int isReset = false;

    ReadWifiConfig(&config); // Read Wi-Fi Data

    
    //ReadSensorModes(&selectModes);
    // ESP_LOGI(SMART_TAP, "Selected Mode::: %d", selectModes.mode_select);


    while (1)
    {
        /* wait for hw reset interrupt with timeout of 5 seconds */
        bits = xEventGroupWaitBits(eventGroup, BIT0, pdTRUE, pdFALSE, 5 * SEC_TO_MS / portTICK_RATE_MS);
        if (bits & BIT0)
        {
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
            if ((gpio_get_level(GPIO_HW_RESET) == 1))
            {
                ESP_LOGI(SMART_TAP, "Hardware Reset switch released");
                gpio_set_level(GPIO_OUT_STS_LED, 0);
            }
        }
    }
}
//--------------------------------------------------------------------------
// End of File
//--------------------------------------------------------------------------
