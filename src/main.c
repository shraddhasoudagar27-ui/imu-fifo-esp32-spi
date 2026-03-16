#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define IMU_INT_PIN 4

static const char *TAG = "IMU";

/* SPI pins */
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_CLK  18
#define PIN_CS   5
#define SPI_FREQ_HZ 1000000

/* MPU registers */
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_USER_CTRL     0x6A
#define REG_WHO_AM_I      0x75

/* FIFO */
#define REG_FIFO_EN      0x23
#define REG_FIFO_COUNTH  0x72
#define REG_FIFO_COUNTL  0x73
#define REG_FIFO_R_W     0x74

/* Interrupt */
#define REG_INT_PIN_CFG  0x37
#define REG_INT_ENABLE   0x38
#define REG_INT_STATUS   0x3A

#define MPU_READ 0x80

/* Scaling */
#define ACCEL_SENS 8192.0f
#define GYRO_SENS  65.5f

/* Ring buffer */
#define RING_BUF_LEN 2048
#define RING_BUF_MASK (RING_BUF_LEN - 1)

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int64_t timestamp_us;
} imu_sample_t;

/* Globals */
static imu_sample_t ring_buf[RING_BUF_LEN];
static volatile uint32_t rb_head = 0;
static volatile uint32_t rb_tail = 0;
static volatile uint32_t sample_count = 0;
static volatile uint32_t overrun_count = 0;

static spi_device_handle_t spi_dev;
static volatile bool imu_data_ready = false;

/* Ring buffer helpers */
static inline uint32_t rb_size(void)
{
    return (rb_head - rb_tail) & RING_BUF_MASK;
}

static void rb_push(const imu_sample_t *s)
{
    uint32_t nxt = (rb_head + 1) & RING_BUF_MASK;

    if (nxt == (rb_tail & RING_BUF_MASK)) {
        rb_tail = (rb_tail + 1) & RING_BUF_MASK;
        overrun_count++;
    }

    ring_buf[rb_head] = *s;
    rb_head = nxt;
    sample_count++;
}

/* SPI helpers */
static void mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val};

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx
    };

    spi_device_transmit(spi_dev, &t);
}

static uint8_t mpu_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {reg | MPU_READ, 0x00};
    uint8_t rx[2] = {0};

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(spi_dev, &t);
    return rx[1];
}

static void mpu_read_burst(uint8_t reg, uint8_t *out, size_t len)
{
    size_t total = len + 1;

    uint8_t tx[total];
    uint8_t rx[total];

    memset(tx, 0, total);
    tx[0] = reg | MPU_READ;

    spi_transaction_t t = {
        .length = total * 8,
        .tx_buffer = tx,
        .rx_buffer = rx
    };

    spi_device_transmit(spi_dev, &t);

    memcpy(out, rx + 1, len);
}

/* FIFO count */
static uint16_t mpu_fifo_count(void)
{
    uint8_t h = mpu_read_reg(REG_FIFO_COUNTH);
    uint8_t l = mpu_read_reg(REG_FIFO_COUNTL);
    return (h << 8) | l;
}

/* Interrupt ISR */
static void IRAM_ATTR imu_isr(void *arg)
{
    imu_data_ready = true;
}

/* MPU init */
static void mpu_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    mpu_write(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(200));

    mpu_write(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    mpu_write(REG_PWR_MGMT_2, 0x00);

    mpu_write(REG_CONFIG, 0x01);
    mpu_write(REG_SMPLRT_DIV, 0x04);

    mpu_write(REG_GYRO_CONFIG, 0x08);
    mpu_write(REG_ACCEL_CONFIG, 0x08);
    mpu_write(REG_ACCEL_CONFIG2, 0x00);

    /* Reset FIFO */
    mpu_write(REG_USER_CTRL, 0x04);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Enable FIFO */
    mpu_write(REG_USER_CTRL, 0x40);

    /* Accel + gyro to FIFO */
    mpu_write(REG_FIFO_EN, 0x78);

    /* Configure interrupt */
    mpu_write(REG_INT_PIN_CFG, 0x10);
    mpu_write(REG_INT_ENABLE, 0x01);

    uint8_t who = mpu_read_reg(REG_WHO_AM_I);

    if (who == 0x71 || who == 0x70 || who == 0x68 || who == 0x73)
        ESP_LOGI(TAG, "MPU detected WHO_AM_I=0x%02X ✓", who);
    else
        ESP_LOGE(TAG, "Unknown WHO_AM_I=0x%02X", who);
}

/* IMU reader task */
static void imu_reader_task(void *arg)
{
    while (1)
    {
        if (imu_data_ready)
        {
            imu_data_ready = false;

            uint16_t count = mpu_fifo_count();

            while (count >= 12)
            {
                uint8_t raw[12];

                mpu_read_burst(REG_FIFO_R_W, raw, 12);

                imu_sample_t s = {
                    .ax = (raw[0] << 8) | raw[1],
                    .ay = (raw[2] << 8) | raw[3],
                    .az = (raw[4] << 8) | raw[5],
                    .gx = (raw[6] << 8) | raw[7],
                    .gy = (raw[8] << 8) | raw[9],
                    .gz = (raw[10] << 8) | raw[11],
                    .timestamp_us = esp_timer_get_time()
                };

                rb_push(&s);

                count -= 12;
            }
        }

        vTaskDelay(1);
    }
}

/* Logger */
static void uart_logger_task(void *arg)
{
    uint32_t last_sc = 0;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t sc = sample_count;
        uint32_t oc = overrun_count;
        uint32_t buffered = rb_size();
        uint32_t delta = sc - last_sc;

        last_sc = sc;

        imu_sample_t s = ring_buf[(rb_head - 1) & RING_BUF_MASK];

        ESP_LOGI(TAG,
            "[%"PRId64" us] total=%"PRIu32" (+%"PRIu32"/s) "
            "buf=%"PRIu32" overruns=%"PRIu32" | "
            "A(g): %.3f %.3f %.3f "
            "G(dps): %.2f %.2f %.2f",
            s.timestamp_us,
            sc,
            delta,
            buffered,
            oc,
            s.ax / ACCEL_SENS,
            s.ay / ACCEL_SENS,
            s.az / ACCEL_SENS,
            s.gx / GYRO_SENS,
            s.gy / GYRO_SENS,
            s.gz / GYRO_SENS);
    }
}

/* Main */
void app_main(void)
{
    ESP_LOGI(TAG, "=== MPU9250 FIFO + Interrupt Acquisition ===");

    /* Configure INT pin */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << IMU_INT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };

    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(IMU_INT_PIN, imu_isr, NULL);

    /* SPI bus */
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_FREQ_HZ,
        .mode = 3,
        .spics_io_num = PIN_CS,
        .queue_size = 4
    };

    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_dev));

    mpu_init();

    xTaskCreatePinnedToCore(uart_logger_task, "uart_log", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(imu_reader_task, "imu_reader", 4096, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "Interrupt-driven IMU sampling started");
}