
 */
#ifndef MQTT_CLIENT_HH
#define MQTT_CLIENT_HH
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
/* Application headers */

#include "aws_iot_error.h"
#include "aws_iot_mqtt_client.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
extern uint64_t time_delay;
//--------------------------------------------------------------------------
// Typedef
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Function Declaration
//--------------------------------------------------------------------------
int InitMqttClient(void);
void SetPublishFlag(void);
bool GetDeviceActivationStatus(void);
bool DeviceFactoryReset;
char Dev_Appliances[25];
char Appliances_Group[2]; // only one character A or B will be there thats why take index 2
bool SwitchStatusUpdate;
IoT_Error_t publishRestoreFactoryStatus(void);
IoT_Error_t checkForOtaPacket(void);
//--------------------------------------------------------------------------
// End of File
//--------------------------------------------------------------------------
#endif // MQTT_CLIENT_HH