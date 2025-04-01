
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include <string.h> // For memcpy


static const char *TAGscd = "scd30_control";

#define I2C_MASTER_SCL_IO           GPIO_NUM_11       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_10       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define SCD30_I2C_ADDR             0x61                /*!< I2C address of the SCD30 sensor */


/* --- Internal helper functions --- */

/**
 * @brief Compute the CRC8 for the given data.
 *
 * The SCD30 uses polynomial 0x31 (x^8 + x^5 + x^4 + 1) and an initial value of 0xFF.
 *
 * @param data Pointer to the data bytes.
 * @param len Number of bytes over which to compute the CRC.
 * @return uint8_t Calculated CRC8.
 */
static uint8_t scd30_crc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief Write a command (with optional parameters) to the sensor.
 *
 * This function builds a transmission buffer containing:
 * - The command (2 bytes)
 * - Zero or more parameters (each parameter is 2 bytes followed by its CRC)
 *
 * @param cmd The command code.
 * @param data Pointer to parameters data (can be NULL if no parameters).
 * @param data_len Number of bytes of parameters (must be a multiple of 2).
 * @return esp_err_t ESP_OK on success, or an error code.
 */
static esp_err_t scd30_write_command(uint16_t cmd, const uint8_t *data, size_t data_len)
{
    uint8_t buf[8] = {0}; // large enough for command + one parameter (adjust if needed)
    int idx = 0;
    buf[idx++] = (uint8_t)(cmd >> 8);
    buf[idx++] = (uint8_t)(cmd & 0xFF);

    /* Append each parameter (2 bytes + 1 CRC) */
    for (size_t i = 0; i < data_len; i += 2) {
        buf[idx++] = data[i];     // high byte
        buf[idx++] = data[i+1];   // low byte
        buf[idx++] = scd30_crc(&data[i], 2);
    }
    return i2c_master_write_to_device(I2C_MASTER_NUM, SCD30_I2C_ADDR,
                                      buf, idx, 1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Convert two 16-bit words (big-endian) into a float.
 *
 * The SCD30 sends IEEE 754 floats in big-endian order.
 * This helper converts the two words into a 32-bit value,
 * swaps the bytes for the little-endian system if necessary,
 * and then converts to a float.
 *
 * @param word1 First 16-bit word.
 * @param word2 Second 16-bit word.
 * @return float The resulting float.
 */
static float be_to_float(uint16_t word1, uint16_t word2)
{
    uint32_t raw = ((uint32_t)word1 << 16) | word2;
    float f;
    memcpy(&f, &raw, sizeof(f));
    return f;
}

/* --- Public API functions --- */

static esp_err_t scd30_init()
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGscd, "i2c_param_config failed");
        return ret;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGscd, "i2c_driver_install failed");
    }
    return ret;
}

static esp_err_t scd30_start_continuous_measurement(uint16_t ambient_pressure)
{
    /* Command 0x0010 starts continuous measurement.
       The sensor accepts an ambient pressure value in mbar for compensation.
       Pass ambient_pressure = 0 if no compensation is desired. */
    uint8_t data[2];
    data[0] = (uint8_t)(ambient_pressure >> 8);
    data[1] = (uint8_t)(ambient_pressure & 0xFF);
    return scd30_write_command(0x0010, data, sizeof(data));
}

static esp_err_t scd30_stop_continuous_measurement()
{
    /* Command 0x0104 stops continuous measurement */
    return scd30_write_command(0x0104, NULL, 0);
}

static esp_err_t scd30_read_measurement(float *co2, float *temperature, float *humidity)
{
    esp_err_t ret;
    /* Command 0x0300 reads the measurement */
    uint8_t cmd[2] = { 0x03, 0x00 };
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, SCD30_I2C_ADDR,
                                     cmd, sizeof(cmd), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGscd, "Failed to write read measurement command");
        return ret;
    }
    /* Allow some time for the sensor to respond */
    vTaskDelay(20 / portTICK_PERIOD_MS);

    uint8_t read_buf[18];  // 3 values x 6 bytes (2 data bytes + 1 CRC each for two words per measurement)
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, SCD30_I2C_ADDR,
                                      read_buf, sizeof(read_buf), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGscd, "Failed to read measurement data");
        return ret;
    }

    /* Verify CRC for each 2-byte word (each measurement consists of 2 words with CRC) */
    for (int i = 0; i < 3; i++) {
        int offset = i * 6;
        if (scd30_crc(&read_buf[offset], 2) != read_buf[offset + 2]) {
            ESP_LOGE(TAGscd, "CRC error in measurement %d (word1)", i);
            return ESP_ERR_INVALID_CRC;
        }
        if (scd30_crc(&read_buf[offset + 3], 2) != read_buf[offset + 5]) {
            ESP_LOGE(TAGscd, "CRC error in measurement %d (word2)", i);
            return ESP_ERR_INVALID_CRC;
        }
    }

    /* Convert the received data into float values.
       Each measurement (CO₂, temperature, humidity) is transmitted as two 16-bit words. */
    uint16_t co2_word1  = (read_buf[0]  << 8) | read_buf[1];
    uint16_t co2_word2  = (read_buf[3]  << 8) | read_buf[4];
    uint16_t temp_word1 = (read_buf[6]  << 8) | read_buf[7];
    uint16_t temp_word2 = (read_buf[9]  << 8) | read_buf[10];
    uint16_t hum_word1  = (read_buf[12] << 8) | read_buf[13];
    uint16_t hum_word2  = (read_buf[15] << 8) | read_buf[16];
    // log the raw data 
    ESP_LOGI(TAGscd, "Raw data: CO2: %04X %04X, Temp: %04X %04X, Hum: %04X %04X",
             co2_word1, co2_word2, temp_word1, temp_word2, hum_word1, hum_word2);

    *co2         = be_to_float(co2_word1, co2_word2);
    *temperature = be_to_float(temp_word1, temp_word2);
    *humidity    = be_to_float(hum_word1, hum_word2);

    return ESP_OK;
}

static esp_err_t scd30_set_measurement_interval(uint16_t interval)
{
    uint8_t data[2];
    data[0] = (uint8_t)(interval >> 8);
    data[1] = (uint8_t)(interval & 0xFF);
    return scd30_write_command(0x4600, data, sizeof(data));
}

static esp_err_t scd30_is_data_ready(bool *data_ready)
{
    esp_err_t ret;
    uint8_t cmd[2] = { 0x02, 0x02 };
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, SCD30_I2C_ADDR,
                                     cmd, sizeof(cmd), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGscd, "Failed to write data ready command");
        return ret;
    }
    /* Short delay to allow the sensor to prepare the response */
    vTaskDelay(6 / portTICK_PERIOD_MS);

    uint8_t read_buf[3];
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, SCD30_I2C_ADDR,
                                      read_buf, sizeof(read_buf), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGscd, "Failed to read data ready status");
        return ret;
    }
    if (scd30_crc(read_buf, 2) != read_buf[2]) {
        ESP_LOGE(TAGscd, "CRC error in data ready status");
        return ESP_ERR_INVALID_CRC;
    }
    uint16_t status = ((uint16_t)read_buf[0] << 8) | read_buf[1];
    *data_ready = (status != 0);
    return ESP_OK;
}