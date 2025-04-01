
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#define GPIO_LED_PIN GPIO_NUM_8
#define LED_STRIP_LENGTH 1

static const char *TAG = "LED_CONTROL";

static led_strip_handle_t led_strip;

typedef enum {
    RED = 0x040000, // Red color
    PURPLE = 0x040004, // Purple color
    YELLOW = 0x080400, // Yellow color
    GREEN = 0x000400, // Green color
    BLUE = 0x000004, // Blue color
    WHITE = 0x040404, // White color
    BLACK = 0x000000, // Black color (off)
    ORANGE = 0x040100, // Orange color
} BasicColor_t;

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configuring LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_LED_PIN,
        .max_leds = LED_STRIP_LENGTH, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

static void set_led_color(BasicColor_t color)
{
    uint8_t red = (color >> 16) & 0xFF;
    uint8_t green = (color >> 8) & 0xFF;
    uint8_t blue = color & 0xFF;

    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}

static void led_rainbow()
{
    for (int i = 0; i < 256; i++) {
        set_led_color((BasicColor_t)((i << 16) | (i << 8) | i));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}