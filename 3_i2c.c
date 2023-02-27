
 */
//--------------------------------------------------------------------------
// Include
//--------------------------------------------------------------------------
/* Application headers */
#include "smartap_common.h"
/* Standard headers */
#include "driver/i2c.h"
#include "smartap_i2c.h"

esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size);
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size);

//--------------------------------------------------------------------------
// Define
//--------------------------------------------------------------------------
#define CONFIG_I2C_MASTER_SCL 22
#define CONFIG_I2C_MASTER_SDA 21

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0                        /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

// I2C common protocol defines
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

uint8_t Databuf;
#define SIZE 0x01

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    ESP_LOGI(SMART_TAP, "i2c_param_config : ret : %s", esp_err_to_name(err));
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to read i2c slave device with registered interface
 * _______________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------|----------------------|--------------------|------|
 *
 */
esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_rd, size_t size)
{
    if (size == 0)
    {
        return ESP_OK;
    }
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    //  ESP_LOGI(SMART_TAP, "i2c_master_start : ret : %s", esp_err_to_name(ret));
    // first, send device address (indicating write) & register to be read
    ret = i2c_master_write_byte(cmd, (i2c_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    //  ESP_LOGI(SMART_TAP, "i2c_master_write_byte : ret : %s", esp_err_to_name(ret));

    ret = i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // ESP_LOGI(SMART_TAP, "i2c_master_write_byte : ret : %s", esp_err_to_name(ret));

    ret = i2c_master_start(cmd);
    // ESP_LOGI(SMART_TAP, "i2c_master_start : ret : %s", esp_err_to_name(ret));

    i2c_master_write_byte(cmd, i2c_addr << 1 | READ_BIT, ACK_CHECK_EN);
    // send register we want
    if (size > 1)
    {
        ret = i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
        // ESP_LOGI(SMART_TAP, "i2c_master_read : ret : %s", esp_err_to_name(ret));
    }
    ret = i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    // ESP_LOGI(SMART_TAP, "i2c_master_read_byte : ret : %s", esp_err_to_name(ret));
    ret = i2c_master_stop(cmd);
    // ESP_LOGI(SMART_TAP, "i2c_master_stop : ret : %s", esp_err_to_name(ret));
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    // ESP_LOGI(SMART_TAP, "i2c_master_cmd_begin : ret : %s", esp_err_to_name(ret));
    i2c_cmd_link_delete(cmd);
    // ESP_LOGI(SMART_TAP, "i2c_master_read successful : ret : %s", esp_err_to_name(ret));

    return ret;
}

esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t *data_wr, size_t size)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ret = i2c_master_start(cmd);
    // ESP_LOGI(SMART_TAP, "i2c_master_start : ret : %s", esp_err_to_name(ret));

    ret = i2c_master_write_byte(cmd, i2c_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    // ESP_LOGI(SMART_TAP, "i2c_master_write_byte : ret : %s", esp_err_to_name(ret));

    ret = i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // ESP_LOGI(SMART_TAP, "i2c_master_write_byte : ret : %s", esp_err_to_name(ret));

    for (int i = 0; i < size; i++)
    {
        ret = i2c_master_write_byte(cmd, data_wr[i], ACK_CHECK_EN);
        // ESP_LOGI(SMART_TAP, "i2c_master_write_byte : ret : %s", esp_err_to_name(ret));
    }

    ret = i2c_master_stop(cmd);
    // ESP_LOGI(SMART_TAP, "i2c_master_stop : ret : %s", esp_err_to_name(ret));

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    //  ESP_LOGI(SMART_TAP, "i2c_master_cmd_begin : ret : %s", esp_err_to_name(ret));

    i2c_cmd_link_delete(cmd);
    // ESP_LOGI(SMART_TAP, "i2c_master_write successful : ret : %s", esp_err_to_name(ret));

    return ret;
}
