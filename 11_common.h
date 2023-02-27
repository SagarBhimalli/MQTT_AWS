
 */
#ifndef SMARTAP_COMM_HH
#define SMARTAP_COMM_HH
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG //Sagar

/* Standard headers */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <errno.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "driver/i2c.h"
//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------

#define SMARTAP_4 1

#ifndef DEVICE_TYPE
#define DEVICE_TYPE SMARTAP_4 // Set Macro As Per required Firmware
#endif

/* status led for connection status */
#define GPIO_OUT_STS_LED 26

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1
#define MANUFACTURE_DATE "2021/07/12" // yyyy/dd/mm

#define FW "1.0.0"
#define TOPIC_OTA_CHECK "ota/smartap_4M/" /* check for application update via OTA*/

#define WILL_MSG "/smarttouch/44:17:93:5b:de:64/status/"
#define SMART_TAP "STAP"

#define MODULE_SUCCESS 0
#define MODULE_FAIL -1

#define SEC_TO_MS 1000
#define MS_TO_US 1000
#define SEC_TO_US (SEC_TO_MS) * (MS_TO_US)

#define GPIO_HIGH 1 /* GPIO logic high */
#define GPIO_LOW 0  /* GPIO logic low */

/* Task Stack Size */
#define MQTT_CLIENT_STACK_SIZE_KB 9216
#define NETWORK_MANAGER_STACK_SIZE_KB 4096
#define SW_CONTROL_STACK_SIZE_KB 2048
#define AP_TASK_STACK_SIZE_KB 4096

/* Task Priority */
#define MQTT_CLIENT_PRIORITY 2
#define NETWORK_MANAGER_PRIORITY 3
#define SW_CONTROL_TASK_PRIORITY 5
#define AP_TASK_PRIORITY 3
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_HIGH 1 /* GPIO logic high */
#define GPIO_LOW 0  /* GPIO logic low */

//--------------------------------------------------------------------------
// Typedef
//--------------------------------------------------------------------------
typedef enum
{
    DEVICE_TYPE_SMARTAP_4,
    DEVICE_TYPE_NONE,
    DEVICE_TYPE_MAX
} DeviceType_e;
//--------------------------------------------------------------------------
// Function Declaration
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
//  global variables
//--------------------------------------------------------------------------

bool DFactoryReset(void);
bool isFactoryReset;
bool FactoryResetNotSuccess;
//--------------------------------------------------------------------------
// End of File
//--------------------------------------------------------------------------
#endif // SMARTAP_COMM_HH