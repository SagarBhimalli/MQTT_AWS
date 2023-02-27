

#ifndef SMARTAP_OTA_H

#define SMARTAP_OTA_H

//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
/* Application headers */

/* Standard headers */

//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif

#define BUFFSIZE 1024
#define HASH_LEN 32
#define OTA_URL_SIZE 256

//--------------------------------------------------------------------------
// Static Variables
//--------------------------------------------------------------------------
/* global variables */

char *otaUrl;
void otaPartitionInit(void);
void checkForOta(void);

#endif // SMARTAP_OTA_H