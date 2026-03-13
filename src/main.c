#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "IMU";

//SPI PINS
#define PIN_MOSI        23
#define PIN_MISO        19
#define PIN_CLK         18
#define PIN_CS           5
#define SPI_FREQ_HZ    1000000    

//MPU 9250 reg
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_ACCEL_XOUT_H  0x3B    // accel(6) + temp(2) + gyro(6) = 14 bytes
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_USER_CTRL     0x6A
#define REG_WHO_AM_I      0x75

#define MPU_READ          0x80    // bit7 = 1 → read

//SCALING sensitivity
#define ACCEL_SENS        8192.0f
#define GYRO_SENS         65.5f

//SAMPLING
#define SAMPLE_RATE_HZ    200
#define SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE_HZ)   // 5000 µs

//RING BUFFER: 200 Hz × 10 s = 2000 samples → power of 2 = 2048
#define RING_BUF_LEN      2048
#define RING_BUF_MASK     (RING_BUF_LEN - 1)

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp_raw;       // raw temperature (optional, included for free)
    int64_t timestamp_us;
} imu_sample_t;

//GLOBALS
static imu_sample_t       ring_buf[RING_BUF_LEN];
static volatile uint32_t  rb_head       = 0;
static volatile uint32_t  rb_tail       = 0;
static volatile uint32_t  sample_count  = 0;
static volatile uint32_t  overrun_count = 0;
static spi_device_handle_t spi_dev;

//RING BUFFER 
static inline uint32_t rb_size(void)
{
    return(rb_head-rb_tail)&RING_BUF_MASK;
}

static void rb_push(const imu_sample_t *s)
{
    uint32_t nxt=(rb_head+1)&RING_BUF_MASK;
    if(nxt==(rb_tail&RING_BUF_MASK))
    {            
        rb_tail=(rb_tail+1)&RING_BUF_MASK;
        overrun_count++;
    }
    ring_buf[rb_head]=*s;
    rb_head=nxt;
    sample_count++;
}

//SPI HELPERS
static void mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7F, val };   // bit7=0 → write
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = NULL,
    };
    spi_device_transmit(spi_dev, &t);
}

static uint8_t mpu_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { reg | MPU_READ, 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(spi_dev, &t);
    return rx[1];
}

// Burst read: reg + (len) bytes; tx[0] = reg | READ, rest zeros; rx[0] = dummy, rx[1..len] = data
static void mpu_read_burst(uint8_t reg, uint8_t *out, size_t len)
{
    size_t total = len + 1;
    uint8_t tx[total];
    uint8_t rx[total];
    memset(tx, 0, total);
    tx[0] = reg | MPU_READ;
    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(spi_dev, &t);
    memcpy(out, rx + 1, len);
}

//MPU-9250 INIT
static void mpu_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reset everything
    mpu_write(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Wake + use PLL (auto-select best clock)
    mpu_write(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Enable all accel + gyro axes
    mpu_write(REG_PWR_MGMT_2, 0x00);

    // Disable I2C, enable SPI only
    mpu_write(REG_USER_CTRL, 0x10);
    vTaskDelay(pdMS_TO_TICKS(10));

    // DLPF: 184 Hz bandwidth (gyro), internal rate = 1 kHz
    mpu_write(REG_CONFIG, 0x01);

    // Sample rate divider: ODR = 1000 / (1 + 4) = 200 Hz
    mpu_write(REG_SMPLRT_DIV, 0x04);

    // Gyro: ±500 °/s  (FCHOICE_B = 0 to use DLPF)
    mpu_write(REG_GYRO_CONFIG, 0x08);

    // Accel: ±4 g
    mpu_write(REG_ACCEL_CONFIG, 0x08);

    // Accel DLPF: 218 Hz bandwidth (ACCEL_FCHOICE_B = 0, A_DLPFCFG = 0)
    mpu_write(REG_ACCEL_CONFIG2, 0x00);

    // Verify
    uint8_t who = mpu_read_reg(REG_WHO_AM_I);
    // MPU-9250 WHO_AM_I = 0x71, MPU-6500 = 0x70, some clones = 0x68 or 0x73
    if (who == 0x71 || who == 0x70 || who == 0x68 || who == 0x73) {
        ESP_LOGI(TAG, "MPU-9250 detected  WHO_AM_I=0x%02X ✓", who);
    } else {
        ESP_LOGE(TAG, "Unknown WHO_AM_I=0x%02X — check wiring/CS pin", who);
    }
}

//TIMER CALLBACK — fires every 5 ms
// Reads 14 bytes: AX AY AZ TEMP GX GY GZ (all big-endian 16-bit)
static void sample_timer_cb(void *arg)
{
    uint8_t raw[14];
    mpu_read_burst(REG_ACCEL_XOUT_H, raw, 14);

    imu_sample_t s = {
        .ax           = (int16_t)((raw[0]  << 8) | raw[1]),
        .ay           = (int16_t)((raw[2]  << 8) | raw[3]),
        .az           = (int16_t)((raw[4]  << 8) | raw[5]),
        .temp_raw     = (int16_t)((raw[6]  << 8) | raw[7]),
        .gx           = (int16_t)((raw[8]  << 8) | raw[9]),
        .gy           = (int16_t)((raw[10] << 8) | raw[11]),
        .gz           = (int16_t)((raw[12] << 8) | raw[13]),
        .timestamp_us = esp_timer_get_time(),
    };

    rb_push(&s);
}

// ─── UART LOGGER TASK — 1 Hz
static void uart_logger_task(void *arg)
{
    uint32_t last_sc = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t sc       = sample_count;
        uint32_t oc       = overrun_count;
        uint32_t buffered = rb_size();
        uint32_t delta    = sc - last_sc;
        last_sc = sc;

        // Peek at newest sample (non-destructive)
        imu_sample_t s = ring_buf[(rb_head - 1) & RING_BUF_MASK];

        // Temperature formula from MPU-9250 datasheet:
        // Temp(°C) = (raw / 333.87) + 21.0
        float temp_c = (s.temp_raw / 333.87f) + 21.0f;

        ESP_LOGI(TAG,
            "[%"PRId64" us] total=%"PRIu32" (+%"PRIu32"/s) "
            "buf=%"PRIu32" overruns=%"PRIu32" | "
            "A(g): %.3f %.3f %.3f  "
            "G(dps): %.2f %.2f %.2f  "
            "T: %.1f°C",
            s.timestamp_us, sc, delta, buffered, oc,
            s.ax / ACCEL_SENS, s.ay / ACCEL_SENS, s.az / ACCEL_SENS,
            s.gx / GYRO_SENS,  s.gy / GYRO_SENS,  s.gz / GYRO_SENS,
            temp_c
        );
    }
}

//APP MAIN
void app_main(void)
{
    ESP_LOGI(TAG, "=== MPU-9250 SPI Acquisition starting ===");

    // 1. SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num    = PIN_MOSI,
        .miso_io_num    = PIN_MISO,
        .sclk_io_num    = PIN_CLK,
        .quadwp_io_num  = -1,
        .quadhd_io_num  = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 2. Add MPU-9250 device
    // MPU-9250 supports SPI mode 0 and mode 3
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_FREQ_HZ,
        .mode           = 3,
        .spics_io_num   = PIN_CS,
        .queue_size     = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_dev));

    // 3. Init sensor
    mpu_init();

    // 4. Logger task — low priority, core 1
    xTaskCreatePinnedToCore(uart_logger_task, "uart_log", 4096,
                            NULL, 2, NULL, 1);

    // 5. Periodic sampling timer — 200 Hz, dispatched via esp_timer task
    const esp_timer_create_args_t timer_args = {
        .callback        = sample_timer_cb,
        .arg             = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name            = "imu_sample",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, SAMPLE_INTERVAL_US));

    ESP_LOGI(TAG, "Sampling at %d Hz | Ring buffer: %d samples = %.1f s",
             SAMPLE_RATE_HZ, RING_BUF_LEN,
             (float)RING_BUF_LEN / SAMPLE_RATE_HZ);
}