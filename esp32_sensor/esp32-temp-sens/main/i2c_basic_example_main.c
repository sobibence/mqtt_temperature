/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "led_control.c"
#include "scd30_control.c"
#include "zigbee_temp.c"
#define LED_DISABLE 1

void app_main(){
   configure_led();
   set_led_color(RED);

   scd30_init();
   scd30_start_continuous_measurement(0);
   scd30_set_measurement_interval(5);

   
   esp_zb_platform_config_t config = {
         .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
         .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
   };
   ESP_ERROR_CHECK(nvs_flash_init());
   ESP_ERROR_CHECK(esp_zb_platform_config(&config));
   set_led_color(YELLOW);
   /* Start Zigbee stack task */
   xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
   set_led_color(GREEN);
   bool data_ready = false;
   while (1) {
      scd30_is_data_ready(&data_ready);
      // Check if data is ready
      if (data_ready) {
         #ifdef LED_DISABLE
         set_led_color(BLACK);
         #else
         set_led_color(YELLOW);
         #endif
         // Read data from the sensor
         float co2, temperature, humidity;
         scd30_read_measurement(&co2, &temperature, &humidity);
         esp_app_temp_sensor_handler(temperature, humidity, co2);
         // log
         ESP_LOGI("SCD30", "CO2: %.2f ppm, Temperature: %.2f °C, Humidity: %.2f %%", co2, temperature, humidity);
         data_ready = false;
         #ifdef LED_DISABLE
         set_led_color(BLACK);
         #else
         set_led_color(GREEN);
         #endif
         // Simulate data ready condition
         vTaskDelay(pdMS_TO_TICKS(4300));
      }
      else 
      {
         #ifdef LED_DISABLE
         set_led_color(BLACK);
         #else
         set_led_color(ORANGE);
         #endif
         vTaskDelay(pdMS_TO_TICKS(200));
         scd30_is_data_ready(&data_ready);
      }
   }

   

}