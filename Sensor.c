#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "smartap_common.h"
#include "smartap_mqttClient.h"
//#include "esp32idfDimmer.h"
#include "esp_task_wdt.h"
#include "Smart_Occupancy_Sensor.h"
#include "smartap_configModule.h"

#define TAG "Smart_Occupancy_Sensor"
#define PIRLDR "MODE2: PIR-LDR"
#define PWM_TASK_INIT "PWM_INITIALIZED"
#define SWITCH_CONTROL SMART_TAP "_SW_CNT"

static esp_adc_cal_characteristics_t adc1_chars;

#define RELAY 16
#define PIR_INTR 34
#define LDR 35

int power = 0;
int MIN_POWER = 0;
int MAX_POWER = 100;
int POWER_STEP = 2;

//dimmertyp *dimming;

// Dimmer Variables
#define PWM_PIN 22     /* output gpio for dimmer PWM */
#define GPIO_IN_ZCD 25 /* input gpio for zero-cross-detection */
#define SUPPLY_FREQ_HZ 50
#define DIMMER_TASK_STACK_SIZE_KB 2048
#define DIMMER_TASK_PRIORITY 1

#define GPIO_INPUT_MASK (1ULL << GPIO_IN_ZCD)
#define GPIO_OUTPUT_MASK (1ULL << GPIO_OUT_PWM)


/* switch control task handle */
static TaskHandle_t swcontrolTaskHandle = NULL;
static QueueHandle_t swControlQueue = NULL;

//--------------------------------------------------------------------------
// Static Functions
//--------------------------------------------------------------------------
static void switchControlTask(void *pArg);

/*PIR-LDR Sensors task  Handle*/
static TaskHandle_t dimmerTaskHandle = NULL;
volatile uint16_t zeroCross = 0;
volatile uint16_t pulseBegin = 0;
volatile int dutyCycle[] = {10000, 9300, 8700, 8000, 7000, 6000, 5200, 4700, 4200, 2500, 1000}; /* timer value in micro seconds */
static esp_timer_handle_t oneshot_timer;
static QueueHandle_t pirControlQueue = NULL;

/* queue size for modes request */
#define MAX_SW_REQ_QUEUE_SIZE 40

static int dimmerId;
static void dimmerTask(void *pArg);
void Light_On_Off();


/**
 * @brief Init Switch control module
 *
 * @return int
 */
int InitSwitchControl(void)
{
    /* create switch control task */
    xTaskCreatePinnedToCore(switchControlTask, "swcontrol", SW_CONTROL_STACK_SIZE_KB,
                            NULL, SW_CONTROL_TASK_PRIORITY, &swcontrolTaskHandle, 0);
    if (swcontrolTaskHandle == NULL)
    {
        ESP_LOGE(SWITCH_CONTROL, "Failed to create sw control task");
        return MODULE_FAIL;
    }

    ESP_LOGI(SWITCH_CONTROL, "Switch control initialized");
    return MODULE_SUCCESS;
}

static void switchControlTask(void *pArg)
{
    // if (initGPIO() != ESP_OK)
    // {
    //     ESP_LOGE(SWITCH_CONTROL, "Failed to init switch control GPIO");
    //     vTaskDelete(NULL);
    // }

    /* create queue for switch control requests */
    swControlQueue = xQueueCreate(MAX_SW_REQ_QUEUE_SIZE, sizeof(SW_CONTROL_REQ_t));
    if (swControlQueue == NULL)
    {
        ESP_LOGE(SWITCH_CONTROL, "Failed to create switch control queue");
        vTaskDelete(NULL);
    }

    while (true)
    {

        if (xQueueReceive(swControlQueue, &swReq, portMAX_DELAY) == pdTRUE)
        {
            if (AWSReq == true)
            {

                ESP_LOGW(SWITCH_CONTROL, "switch request received From Mobile");

                    if (swReq.switchId == 0)
                    {
                        sw_state1 = swReq.value;
                        sw_FinalState = swReq.value;
                    }
                    ESP_LOGD(SWITCH_CONTROL, "when request from mobile ---- sw_FinalState :%d ", sw_FinalState);

                    setSwitchState(swReq.switchId, swReq.value);
                    //  setSwitchState(swReq.switchId, sw_FinalState);
                    ESP_LOGI(SWITCH_CONTROL, "$$$$setSwitchState : [switch id : %d] [value : %d]  ", swReq.switchId, swReq.value);

                AWSReq = false;
            }
        }

        /* if control request is due to switch interrupt get gpio level or else use what is received in the queue */
        // if (swReq.isSwitchIntr == true)
        // {
        //     swReq.isSwitchIntr = false;
        //     ESP_LOGW(SWITCH_CONTROL, "SW Interrupet request received ");
        // }

        ESP_LOGI(SWITCH_CONTROL, "switch request received : [sw= %d], [value= %d]", swReq.switchId, swReq.value);

        //setSwitchState(swReq.switchId, swReq.value);

        /* set publish flag for updating switch status to cloud */
        SetPublishFlag();
    }
}


/**
 * @brief ISR handler for Zero cross detect interrupt
 *        - start one shot timer for configured dimmer value
 *
 * @param arg
 */
static void IRAM_ATTR zcd_isr(void *arg)
{
    if (zeroCross == 1)
    {
        esp_timer_start_once(oneshot_timer, pulseBegin);
    }
}
/**
 * @brief One shot timer callback
 *        - Timer callback to generate 20 us pulse to trigger the TRIAC
 * @param arg
 */
static void oneshot_timer_callback(void *arg)
{
    /* in case of value 5 set the PWM to high  */
    if (gpio_get_level(PWM_PIN) == 1)
    {
        gpio_set_level(PWM_PIN, GPIO_HIGH);
        return;
    }
    gpio_set_level(PWM_PIN, GPIO_HIGH);
    ets_delay_us(20);
    gpio_set_level(PWM_PIN, GPIO_LOW);
}

extern uint8_t Occ_Flag;
extern uint32_t ambient_light;
uint8_t dimmer_Flag = 0;

static TaskHandle_t pirTaskHandle = NULL;
static TaskHandle_t ldrTaskHandle = NULL;

// ESP32 Timer
#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert Counter value to seconds

static SemaphoreHandle_t s_timer_sem;
static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(s_timer_sem, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}

TaskHandle_t Occupancy_Handle;
TaskHandle_t LDR_Handle;
TaskHandle_t ldr_Occupancy_Handle;
TaskHandle_t PIR_LDR_Handle;

xQueueHandle interruptQueue;

void gpio_init()
{
    gpio_pad_select_gpio(RELAY);
    gpio_set_direction(RELAY, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY, 1);

    gpio_pad_select_gpio(PIR_INTR);
    gpio_set_direction(PIR_INTR, GPIO_MODE_INPUT);
}

void timer_task_init()
{
    ReadPIRDelay(&sensorConfig);
    int delayPir = sensorConfig.time_delay;
    ESP_LOGI(TAG, "Sagar ***** Inside Timer Task IniT ***** %d", delayPir);
    s_timer_sem = xSemaphoreCreateBinary();
    if (s_timer_sem == NULL)
    {
        printf("Binary semaphore can not be created");
    }
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN};
    timer_init(TIMER_GROUP_0, TIMER_1, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, TIMER_SCALE * delayPir); //
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, timer_group_isr_callback, NULL, 0);
    // timer_start(TIMER_GROUP_0, TIMER_0);
}

void LDR_Read()
{
    esp_task_wdt_init(25, false);
    gpio_init();

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11));

    while (1)
    {
        ambient_light = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_7), &adc1_chars);
        ESP_LOGI(TAG, "ADC1_CHANNEL_7: %d mV", ambient_light);

        int currentAmbient = ambient_light;
        if (ambient_light >= (currentAmbient + 50))
        {
            ESP_LOGI(TAG, "Changed Ambient: %d mV", currentAmbient);
            SetPublishFlag();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (ambient_light < 150 && dimmer_Flag == 0)
        {
            dimmer_Flag = 1;
            // timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
            ESP_LOGW(TAG, "Ambient Light is too low Turning on Light");

            // timer_start(TIMER_GROUP_0, TIMER_1);
            // SetPublishFlag();
            zeroCross = 1;
            for (int i = 0; i <= 10; i++)
            {
                pulseBegin = dutyCycle[i];
                vTaskDelay(0.1 * SEC_TO_MS / portTICK_RATE_MS);
            }
        }

        if (ambient_light > 150 && dimmer_Flag == 1)
        {
            dimmer_Flag = 0;
            // SetPublishFlag();
            // timer_deinit(TIMER_GROUP_0, TIMER_1);
            ESP_LOGW(TAG, "Turning Off Light");
            for (int i = 10; i >= 0; i--)
            {
                pulseBegin = dutyCycle[i];
                vTaskDelay(0.1 * SEC_TO_MS / portTICK_RATE_MS);
            }
            zeroCross = 0;
        }
    }
}

/*This is mode 1 where only PIR sensor will work to turn ON/OFF light*/
void PIR_Sensor_Read()
{
    esp_task_wdt_init(25, false);
    gpio_init();

    while (1)
    {
        if (gpio_get_level(PIR_INTR) == 0)
        {
            timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
            ESP_LOGW(TAG, "OCCUPANCY DETECTED");
            timer_start(TIMER_GROUP_0, TIMER_1);
            if (Occ_Flag == 0)
            {
                zeroCross = 1;
                Occ_Flag = 1;
                SetPublishFlag();

                ESP_LOGW(TAG, "Turning On Light");
                for (int i = 0; i <= 10; i++)
                {
                    pulseBegin = dutyCycle[i];
                    vTaskDelay(0.1 * SEC_TO_MS / portTICK_RATE_MS);
                }
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void PIR_Task()
{
    esp_task_wdt_init(25, false);

    xTaskCreate(&PIR_Sensor_Read, "PIR_Sensor_Read", 2048, NULL, 1, &Occupancy_Handle);

    while (1)
    {
        timer_task_init();
        if (xSemaphoreTake(s_timer_sem, portMAX_DELAY) == pdPASS)
        {
            if (Occ_Flag)
            {
                zeroCross = 1;
                Occ_Flag = 0;
                SetPublishFlag();
                timer_deinit(TIMER_GROUP_0, TIMER_1);
                printf("Timer Over !!! \n");
                ESP_LOGW(TAG, "Turning Off Light");
                for (int i = 10; i >= 0; i--)
                {
                    pulseBegin = dutyCycle[i];
                    vTaskDelay(0.1 * SEC_TO_MS / portTICK_RATE_MS);
                }
            }
            zeroCross = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/*This is mode 2 where PIR-LDR sensor will work together to turn ON/OFF light*/
void PIR_LDR_Read()
{
    esp_task_wdt_init(25, false);
    gpio_init();

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11));

    while (1)
    {
        ambient_light = esp_adc_cal_raw_to_voltage(adc1_get_raw(ADC1_CHANNEL_7), &adc1_chars);
        ESP_LOGI(PIRLDR, "ADC1_CHANNEL_7: %d mV", ambient_light);
        if (gpio_get_level(PIR_INTR) == 0 && (ambient_light >= 120 && ambient_light <= 250))
        {
            timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
            ESP_LOGW(PIRLDR, "OCCUPANCY DETECTED");
            timer_start(TIMER_GROUP_0, TIMER_1);
            if (Occ_Flag == 0)
            {
                zeroCross = 1;
                Occ_Flag = 1;
                SetPublishFlag();

                ESP_LOGW(TAG, "Turning On Light");
                for (int i = 0; i <= 10; i++)
                {
                    pulseBegin = dutyCycle[i];
                    vTaskDelay(0.1 * SEC_TO_MS / portTICK_RATE_MS);
                }
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void PIR_LDR_Task()
{
    esp_task_wdt_init(25, false);

    xTaskCreate(&PIR_LDR_Read, "PIR_LDR_Read", 2048, NULL, 1, &ldr_Occupancy_Handle);

    while (1)
    {
        timer_task_init();
        if (xSemaphoreTake(s_timer_sem, portMAX_DELAY) == pdPASS)
        {
            if (Occ_Flag)
            {
                zeroCross = 1;
                Occ_Flag = 0;
                SetPublishFlag();
                timer_deinit(TIMER_GROUP_0, TIMER_1);
                printf("Timer Over !!! \n");
                ESP_LOGW(TAG, "Turning Off Light");
                for (int i = 10; i >= 0; i--)
                {
                    pulseBegin = dutyCycle[i];
                    vTaskDelay(0.1 * SEC_TO_MS / portTICK_RATE_MS);
                }
            }
            zeroCross = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

int PIRStatus(void)
{
    esp_task_wdt_init(25, false);

    /* create dimmer task pinned to core 1 */
    xTaskCreatePinnedToCore(dimmerTask, "dimmerTask", DIMMER_TASK_STACK_SIZE_KB, NULL,
                            DIMMER_TASK_PRIORITY, &dimmerTaskHandle, 1);
    if (dimmerTaskHandle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create dimmer task");
        return MODULE_FAIL;
    }

    ReadSensorModes(&sensorConfig);
    ESP_LOGI(SMART_TAP, "::::::::::::::::::::::::::::::::::::::::::::::::::::::Selected Mode::: %lld", sensorConfig.mode_select);

    /* create PIR Sensor task */
    if (sensorConfig.mode_select == 1)
    {
        ESP_LOGI(SMART_TAP, ":::::::::::::::::STARTED PIR TASK::::::::::::::::::::::");
        xTaskCreatePinnedToCore(PIR_Task, "PIR_Task", 4096,
                                NULL, SW_CONTROL_TASK_PRIORITY, &pirTaskHandle, 0);

        if (pirTaskHandle == NULL)
        {
            ESP_LOGI(TAG, "Failed to create pirTaskHandle task");
            return MODULE_FAIL;
        }
    }

    /* create PIR-LDR Sensor task */
    else if (sensorConfig.mode_select == 2)
    {
        xTaskCreatePinnedToCore(PIR_LDR_Task, "PIR_LDR_Task", 4096,
                                NULL, 1, &PIR_LDR_Handle, 0);

        if (PIR_LDR_Handle == NULL)
        {
            ESP_LOGI(TAG, "Failed to create ldrTaskHandle task");
            return MODULE_FAIL;
        }
    }

    /* create LDR Sensor task */
    else if (sensorConfig.mode_select == 3)
    {
        xTaskCreatePinnedToCore(LDR_Read, "LDR_Read", 2048,
                                NULL, 1, &LDR_Handle, 0);

        if (LDR_Handle == NULL)
        {
            ESP_LOGI(TAG, "Failed to create ldrTaskHandle task");
            return MODULE_FAIL;
        }
    }
    else
    {
    }

    return MODULE_SUCCESS;
}

/**
 * @brief Dimmer Task pinned to core 1
 *        - Configure ZCD and PWM GPIOs
 *        - monitor change in dimmer switch and dimmer value
 *        - set the Output PWM duty cycle to vary the dimmer intensity
 *
 * @param pArg
 */
static void dimmerTask(void *pArg)
{
    /* configre ZCD pin */
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << GPIO_IN_ZCD);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = (gpio_pullup_t)1;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(GPIO_IN_ZCD, zcd_isr, (void *)GPIO_IN_ZCD);

    /* configure PWM pin */
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << PWM_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    io_conf.pull_down_en = (gpio_pulldown_t)1;
    gpio_config(&io_conf);

    /* initialize one shot timer */
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &oneshot_timer_callback,
        .name = "one-shot"};
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));

    ESP_LOGI(PWM_TASK_INIT, "Dimmer Task Initialized");

    while (1)
    {
        /* busy waiting */
        vTaskDelay(2 * SEC_TO_MS / portTICK_RATE_MS);
    }
}


/**
 * @brief Get the Switch Value object
 *
 * @param switchId
 * @return true
 * @return false
 */
bool GetSwitchValue(SWITCH_e switchId)
{
    if (switchId >= SW_MAX)
    {
        ESP_LOGE(SWITCH_CONTROL, "switch id out of bound : [switch= %d]", switchId);
        return false;
    }

    return controlStatus.switchStatus[switchId];
}

void setSwitchState(SWITCH_e switchId, bool state)
{
    if (switchId >= SW_MAX)
    {
        ESP_LOGE(SWITCH_CONTROL, "switch id out of bound : [switch= %d]", switchId);
        return;
    }

    if (switchId >= SW_MAX)
    {
        ESP_LOGE(SWITCH_CONTROL, "switch id out of bound : [switch= %d]", switchId);
        return;
    }

    else
    {
        ESP_LOGI(SWITCH_CONTROL, "Set switch[%d]: state[%d]", switchId, state);
        // if (gpio_set_level(switchGpio[switchId], state) != ESP_OK)
        // {
        //     ESP_LOGE(SWITCH_CONTROL, "Failed to set gpio[%d]", switchGpio[switchId]);
        // }
    }

    controlStatus.switchStatus[switchId] = state;
    swReq.value = state;
    // WriteControlStatus(&controlStatus);
}


/**
 * @brief
 *
 *
 * @param swReq
 */
void EnqueueSwitchRequest(SW_CONTROL_REQ_t swReq)
{
    ESP_LOGD(SWITCH_CONTROL, "Enqueue switch request");
    AWSReq = true;
    if (xQueueSend(swControlQueue, &swReq, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE(SWITCH_CONTROL, "Failed to enqueue switch request : [sw= %d], [value= %d]", swReq.switchId, swReq.value);
    }
}
