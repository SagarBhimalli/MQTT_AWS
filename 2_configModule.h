
#ifndef CONFIG_MODULE_HH
#define CONFIG_MODULE_HH
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
#include "smartap_common.h"
//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
/* Macro definition of WiFi name and password */

#define MY_WIFI_SSID "SmarTap_O"
#define MY_WIFI_PASSWD "12345678"
//--------------------------------------------------------------------------
// Typedef
//--------------------------------------------------------------------------
typedef struct
{
    uint8_t ssid[32];
    uint8_t password[64];
    char uniqueid[12];
} WIFI_CONFIG_t;

typedef enum
{
    SW01 = 0,
    SW02 = 1,
    SW03 = 2,
    SW04 = 3,
    SW_MAX = 4,
} SWITCH_e;

typedef struct
{
    bool switchStatus[SW_MAX];
    int isDevicesActivated;
    int outdoorMode;
} CONTROL_STATUS_t;

typedef struct
{
    uint64_t mode_select;
} SELECT_MODES_t;

typedef struct
{
    uint64_t time_delay;
     uint64_t mode_select;
} SENSOR_CONFIG_t;

//--------------------------------------------------------------------------
// Function Declaration
//--------------------------------------------------------------------------
CONTROL_STATUS_t controlStatus;
SENSOR_CONFIG_t sensorConfig;
SELECT_MODES_t selectModes;

esp_err_t InitConfigurationModule(void);
void DeInitConfigurationModule(void);
esp_err_t WriteWifiConfig(WIFI_CONFIG_t *pWifiConfig);
esp_err_t ReadWifiConfig(WIFI_CONFIG_t *pWifiConfig);
void DefaultWifiConfig(void);
esp_err_t WriteControlStatus(CONTROL_STATUS_t *pContrlStatus);
esp_err_t ReadControlStatus(CONTROL_STATUS_t *pControlStatus);
esp_err_t WritePIRDelay(SENSOR_CONFIG_t *psensorConfig);
esp_err_t ReadPIRDelay(SENSOR_CONFIG_t *psensorConfig);
esp_err_t WriteSensorModes(SENSOR_CONFIG_t *psensorConfig);
esp_err_t ReadSensorModes(SENSOR_CONFIG_t *psensorConfig);
void DefaultMode(void);
void DefaultDelay(void);
void DefaultControlStatus(void);
void DoFactoryReset(void);
void DoSystemRestart(void);
int PIRStatus(void);
//--------------------------------------------------------------------------
// End of file
//--------------------------------------------------------------------------
#endif // CONFIG_MODULE_HH