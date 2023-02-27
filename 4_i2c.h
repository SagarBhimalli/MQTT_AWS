
#ifndef SMARTAP_I2C_HH
#define SMARTAP_I2C_HH
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
#include "smartap_common.h"

//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------

/* I2C Device Address */
/* Touch key Device Address ATQT242120 */
#define TOUCH_KEY_DEVICE_ADDR 0x1C

/* AT42QT2120 Touch Sensor Registers */

//--------------------------------------------------------------------------
// AT24QT2120 Key Touch Chip Registers Macros Start here
//--------------------------------------------------------------------------

/* CHIP ID Read Only Register */
#define CHIP_ID_REG 00

/* Firmware Version Read Only Register */
#define FIRMWARE_VER_REG 01

/* Detection Status Read Only Register */
#define DETECTION_STATUS_REG 02

/* Key Status Read Only Register(KEY0 to KEY7) */
#define KEY_STATUS_K0_K7_REG 03

/* Key Status Read Only Register(KEY8 to KEY11) */
#define KEY_STATUS_K8_K11_REG 04

/* Slider position Read Only Register */
#define SLIDER_POSITION_REG 05

/* Calibration Read/Write Register */
#define CALIBRATION_REG 06

/* RESET Read/Write Register */
#define RESET_REG 07

/* Low Power Mode Read/Write Register */
#define LOW_POWER_MODE_REG 08

/* (TTD)Towards Touch Drift Compensation Read/Write Register */
#define TTD_COMPENSTN_REG 9

/* Away from Compensation Drift Register Read/Write Register */
#define ATD_COMPENSTN_REG 10

/* DI Read/Write Register */
#define DETECTION_INTIGRATION_REG 11

/* TRD Read/Write Register */
#define TOUCH_RECAL_DELAY_REG 12

/* DHT Read/Write Register */
#define DRIFT_HOLD_TIME_REG 13

/* Slider Option Read/Write Register */
#define SLIDER_OPTIONS_REG 14

/* Charge Time Register */
#define CHARGE_TIME_REG 15

/* Key Detect Threshold Read/Write Register */
#define KEY0_DETECT_THRESHOLD_REG 16
#define KEY1_DETECT_THRESHOLD_REG 17
#define KEY2_DETECT_THRESHOLD_REG 18
#define KEY3_DETECT_THRESHOLD_REG 19
#define KEY4_DETECT_THRESHOLD_REG 20
#define KEY5_DETECT_THRESHOLD_REG 21
#define KEY6_DETECT_THRESHOLD_REG 22
#define KEY7_DETECT_THRESHOLD_REG 23
#define KEY8_DETECT_THRESHOLD_REG 24
#define KEY9_DETECT_THRESHOLD_REG 25
#define KEY10_DETECT_THRESHOLD_REG 26
#define KEY11_DETECT_THRESHOLD_REG 27

/* Key Control Read/Write Register */
#define KEY0_CONNTROL_REG 28
#define KEY1_CONNTROL_REG 29
#define KEY2_CONNTROL_REG 30
#define KEY3_CONNTROL_REG 31
#define KEY4_CONNTROL_REG 32
#define KEY5_CONNTROL_REG 33
#define KEY6_CONNTROL_REG 34
#define KEY7_CONNTROL_REG 35
#define KEY8_CONNTROL_REG 36
#define KEY9_CONNTROL_REG 37
#define KEY10_CONNTROL_REG 38
#define KEY11_CONNTROL_REG 39

/* Key Pulse Scale Read/Write Register */
#define KEY0_PULSE_SCALE_REG 40
#define KEY1_PULSE_SCALE_REG 41
#define KEY2_PULSE_SCALE_REG 42
#define KEY3_PULSE_SCALE_REG 43
#define KEY4_PULSE_SCALE_REG 44
#define KEY5_PULSE_SCALE_REG 45
#define KEY6_PULSE_SCALE_REG 46
#define KEY7_PULSE_SCALE_REG 47
#define KEY8_PULSE_SCALE_REG 48
#define KEY9_PULSE_SCALE_REG 49
#define KEY10_PULSE_SCALE_REG 50
#define KEY11_PULSE_SCALE_REG 51

/* Key Singal Read Only Register  */
#define KEY_SIGNAL0_REG 52
#define KEY_SIGNAL1_REG 54
#define KEY_SIGNAL2_REG 56
#define KEY_SIGNAL3_REG 58
#define KEY_SIGNAL4_REG 60
#define KEY_SIGNAL5_REG 62
#define KEY_SIGNAL6_REG 64
#define KEY_SIGNAL7_REG 66
#define KEY_SIGNAL8_REG 68
#define KEY_SIGNAL9_REG 70
#define KEY_SIGNAL10_REG 72
#define KEY_SIGNAL11_REG 74

/* Reference Data Read Only Register */
#define REF_DATA0_REG 76
#define REF_DATA1_REG 78
#define REF_DATA2_REG 80
#define REF_DATA3_REG 82

/*GPIO EXPANDER Register */
#define GPIO_EX_PORT0_SET_LEVEL 0x02
#define GPIO_EX_PORT1_SET_LEVEL 0x03
#define GPIO_EX_PORT_CONFIGURE 0x06

//--------------------------------------------------------------------------
// AT24QT2120 Key touch Sense Chip Registers Macros Ends here
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Function Declarations
//--------------------------------------------------------------------------
esp_err_t i2c_master_init(void);
esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_wr, size_t size);
esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_rd, size_t size);

//--------------------------------------------------------------------------
// Start of Key Touch Chip registers
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// End of Key Touch Chip registers
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// End of file
//--------------------------------------------------------------------------
#endif // SMARTAP_I2C_HH
