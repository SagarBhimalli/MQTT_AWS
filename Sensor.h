#include "smartap_configModule.h"


//--------------------------------------------------------------------------
// Typedef
//--------------------------------------------------------------------------



extern uint8_t Occ_Flag;
extern uint32_t ambient_light;

void timer_task_init();
void gpio_init();
void PIR_Sensor_Read();
void PIR_LDR_Read();
void PIR_Task();
int PIRStatus(void);

//--------------------------------------------------------------------------
// Typedef
//--------------------------------------------------------------------------
typedef struct
{
    uint32_t switchId;
    uint32_t value;
    volatile bool isSwitchIntr;
    uint32_t Rs_switchId[SW_MAX];
    uint32_t Rs_value[SW_MAX];
    volatile bool RetainState;
} SW_CONTROL_REQ_t;

typedef enum
{
    SW_MODE_COPY,
    SW_MODE_TOGGLE,
    SW_MODE_MAX
} SW_MODE_e;

SW_CONTROL_REQ_t swReq;
//--------------------------------------------------------------------------
// Function Declaration
//--------------------------------------------------------------------------

uint8_t sw_invalid;
unsigned char sw_state1;
unsigned char sw_state2;
unsigned char sw_state3;
unsigned char sw_state4;
unsigned char sw_FinalState;

bool AWSReq;
int InitSwitchControl(void);
bool GetSwitchValue(SWITCH_e switchId);
void EnqueueSwitchRequest(SW_CONTROL_REQ_t swReq);
void SetOutDoorMode(bool state);
esp_err_t initSwitches(uint8_t);
void setSwitchState(SWITCH_e switchId, bool state);