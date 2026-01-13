// I2C
#define I2C_SDA_GPIO      21
#define I2C_SCL_GPIO      22

// Vibration Motor
#define MOTOR_GPIO        13

// Servo (PWM)
#define SERVO_GPIO        12

// SD Card (SPI)
#define SD_MOSI_GPIO      23
#define SD_MISO_GPIO      19
#define SD_SCLK_GPIO      18
#define SD_CS_GPIO        5
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_err.h"

#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

/* ===================== SYSTEM CONFIG ===================== */
#define TAG "BEDSORE_SYS"

#define NUM_SENSORS                 8

#define TEMP_THRESHOLD_C            32.0f
#define PRESSURE_THRESHOLD_HPA      1001.0f
#define HUMIDITY_THRESHOLD_PCT      80.0f

#define I2C_MUX_ADDR                0x70
#define BME280_ADDR                 0x76
#define RTC_ADDR                    0x68

/* ===================== I2C INIT ===================== */
static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

/* ===================== I2C MUX ===================== */
static void mux_select(uint8_t channel)
{
    uint8_t data = (1 << channel);
    i2c_master_write_to_device(I2C_NUM_0,
                               I2C_MUX_ADDR,
                               &data,
                               1,
                               pdMS_TO_TICKS(100));
}

/* ===================== SENSOR READ (SIMPLIFIED) ===================== */
typedef struct {
    float temperature;
    float pressure;
    float humidity;
} sensor_data_t;

/* Placeholder read (replace with full BME280 driver later) */
static void read_bme280(sensor_data_t *data)
{
    data->temperature = 33.0f;
    data->pressure    = 1005.0f;
    data->humidity    = 82.0f;
}

/* ===================== MOTOR ===================== */
static void motor_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_GPIO),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_conf);
    gpio_set_level(MOTOR_GPIO, 0);
}

static void motor_vibrate(uint32_t duration_ms)
{
    gpio_set_level(MOTOR_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    gpio_set_level(MOTOR_GPIO, 0);
}

/* ===================== SERVO (LEDC PWM) ===================== */
static void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_16_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0
    };
    ledc_channel_config(&channel);
}

static void servo_set_angle(uint16_t angle)
{
    if (angle > 180) angle = 180;

    uint32_t duty = (500 + (angle * 2000 / 180)) * 65535 / 20000;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(300));
}

static void lateral_movement(void)
{
    servo_set_angle(45);
    motor_vibrate(200);

    servo_set_angle(90);
    motor_vibrate(200);

    servo_set_angle(135);
    motor_vibrate(200);

    servo_set_angle(90);
}

/* ===================== SD CARD ===================== */
static void sdcard_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5
    };

    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCLK_GPIO,
        .max_transfer_sz = 4000
    };

    spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_GPIO;

    esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config,
                            &mount_config, &card);
}

/* ===================== LOGGING ===================== */
static void log_event(uint8_t sensor_id,
                      const char *event,
                      sensor_data_t *data)
{
    FILE *f = fopen("/sdcard/log.txt", "a");
    if (!f) return;

    fprintf(f,
        "SENSOR=%d | T=%.2f | P=%.2f | H=%.2f | %s\n",
        sensor_id,
        data->temperature,
        data->pressure,
        data->humidity,
        event);

    fclose(f);
}

/* ===================== MAIN ===================== */
void app_main(void)
{
    ESP_LOGI(TAG, "System Init");

    i2c_init();
    motor_init();
    servo_init();
    sdcard_init();

    sensor_data_t data;

    while (1)
    {
        for (uint8_t i = 0; i < NUM_SENSORS; i++)
        {
            mux_select(i);
            read_bme280(&data);

            bool t = data.temperature > TEMP_THRESHOLD_C;
            bool p = data.pressure    > PRESSURE_THRESHOLD_HPA;
            bool h = data.humidity    > HUMIDITY_THRESHOLD_PCT;

            if (t || p || h)
            {
                motor_vibrate(1000);
                lateral_movement();
                log_event(i, "ALERT", &data);
            }

            if (t && p && h)
            {
                motor_vibrate(1500);
                lateral_movement();
                log_event(i, "CRITICAL", &data);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
