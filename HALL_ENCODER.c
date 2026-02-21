#include <stdio.h>
#include "HALL_ENCODER.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/mcpwm_cap.h"
#include "math.h"

#define TAG "AS5600"

static TaskHandle_t as5600_task_handle = NULL;

volatile float encoder[100];

/* I2C configuration */
#define I2C_MASTER_NUM      I2C_NUM_1
#define I2C_MASTER_FREQ_HZ  400000

/* AS5600 */
#define AS5600_ADDR         0x36
#define AS5600_RAW_MSB      0x0C
#define AS5600_CONF_MSB     0x07
#define AS5600_CONF_LSB     0x08

#define AS5600_PWM_MIN 0.01f   // ~1%
#define AS5600_PWM_MAX 0.99f   // ~99%
#define AS5600_PWM_RANGE (AS5600_PWM_MAX - AS5600_PWM_MIN)

#define SENSOR_IZQ_BIT   (1 << 0)
#define SENSOR_DER_BIT   (1 << 1)

typedef struct {
    volatile uint32_t high;
    volatile uint32_t period;
    volatile uint32_t last_rise;
    volatile bool ready;
} as5600_pwm_t;

as5600_pwm_t as5600Izq = {0};
as5600_pwm_t as5600Der = {0};

esp_err_t as5600_enable_pwm(uint8_t frequency) {
    uint16_t conf = 0;

    // PM = 00 → NOM
    conf |= (0b00 << 0);
    // HYST = 00 → OFF
    conf |= (0b00 << 2);
    // OUTS = 10 → digital PWM
    conf |= (0b10 << 4);
    // PWMF = 01 → 230 Hz
    conf |= (frequency << 6);
    // SF = 11 → 2x (mínima latencia)
    conf |= (0b11 << 8);
    // FTH = 000 → slow filter only
    conf |= (0b000 << 10);
    // WD = 0 → watchdog OFF
    conf |= (0 << 13);

    uint8_t msb = (conf >> 8) & 0xFF;
    uint8_t lsb = conf & 0xFF;

    esp_err_t err;
    err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        AS5600_ADDR,
        (uint8_t[]){AS5600_CONF_MSB, msb},
        2,
        pdMS_TO_TICKS(50)
    );
    if (err != ESP_OK) return err;

    err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        AS5600_ADDR,
        (uint8_t[]){AS5600_CONF_LSB, lsb},
        2,
        pdMS_TO_TICKS(50)
    );

    return err;
}

static bool IRAM_ATTR pwm_capture_cb(
    mcpwm_cap_channel_handle_t chan,
    const mcpwm_capture_event_data_t *edata,
    void *user_data
) {
    as5600_pwm_t *sensor = (as5600_pwm_t*) user_data;
    BaseType_t hp_task_woken = pdFALSE;
    uint32_t notify_bit = 0;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        sensor->period = edata->cap_value - sensor->last_rise;
        sensor->last_rise = edata->cap_value;
        sensor->ready = true;

        if (sensor == &as5600Izq)
            notify_bit = SENSOR_IZQ_BIT;
        else if (sensor == &as5600Der)
            notify_bit = SENSOR_DER_BIT;

        xTaskNotifyFromISR(as5600_task_handle, notify_bit, eSetBits, &hp_task_woken);

        return hp_task_woken == pdTRUE;
    } else {
        sensor->high = edata->cap_value - sensor->last_rise;
    }

    return false;
}

void pwm_capture_init(gpio_num_t gpioIzq, gpio_num_t gpioDer) {
    mcpwm_cap_timer_handle_t cap_timer;
    mcpwm_cap_channel_handle_t cap_chan_izq;
    mcpwm_cap_channel_handle_t cap_chan_der;

    mcpwm_capture_timer_config_t timer_conf = {
        .clk_src = MCPWM_CAPTURE_CLK_SRC_APB,
        .group_id = 0,
    };
    mcpwm_new_capture_timer(&timer_conf, &cap_timer);
    mcpwm_capture_timer_enable(cap_timer);
    mcpwm_capture_timer_start(cap_timer);

    mcpwm_capture_channel_config_t ch_conf = {
        .gpio_num = gpioIzq,
        .prescale = 1,
        .flags.neg_edge = true,
        .flags.pos_edge = true,
    };
    mcpwm_new_capture_channel(cap_timer, &ch_conf, &cap_chan_izq);

    ch_conf.gpio_num = gpioDer;
    mcpwm_new_capture_channel(cap_timer, &ch_conf, &cap_chan_der);

    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = pwm_capture_cb,
    };
    mcpwm_capture_channel_register_event_callbacks(cap_chan_izq, &cbs, &as5600Izq);
    mcpwm_capture_channel_register_event_callbacks(cap_chan_der, &cbs, &as5600Der);
    mcpwm_capture_channel_enable(cap_chan_izq);
    mcpwm_capture_channel_enable(cap_chan_der);
}

static esp_err_t i2c_master_init(uint8_t gpioSCL, uint8_t gpioSDA) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = gpioSDA,
        .scl_io_num = gpioSCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_master_deInit() {
    return i2c_driver_delete(I2C_MASTER_NUM);
}

uint16_t as5600GetAnglePwm(as5600_pwm_t *sensor) {
    uint32_t high, period;

    // Evitar leer valores inconsistentes
    if (!sensor->ready)
        return 0;

    // Copia local atómica
    high = sensor->high;
    period = sensor->period;

    sensor->ready = false;

    float duty = (float) high / period;
    // clamp
    if (duty < AS5600_PWM_MIN) duty = AS5600_PWM_MIN;
    if (duty > AS5600_PWM_MAX) duty = AS5600_PWM_MAX;

    float norm = (duty - AS5600_PWM_MIN) / AS5600_PWM_RANGE;

    return (uint16_t)(norm * 4095.0f);
}

static esp_err_t as5600_read_raw(uint16_t *angle) {
    uint8_t reg = AS5600_RAW_MSB;
    uint8_t data[2];

    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        AS5600_ADDR,
        &reg,
        1,
        data,
        2,
        pdMS_TO_TICKS(100)
    );

    if (ret == ESP_OK) {
        *angle = ((data[0] << 8) | data[1]) & 0x0FFF;
    }

    return ret;
}

void as5600Task(void *arg) {
    uint16_t cont = 0, i;
    uint32_t notified_bits;
    as5600_config_t config = *(as5600_config_t *)arg;
    free(arg);

    ESP_ERROR_CHECK(i2c_master_init(config.gpioSclIzq, config.gpioSdaIzq));    
    ESP_ERROR_CHECK(as5600_enable_pwm(config.freq));
    ESP_ERROR_CHECK(i2c_master_deInit());
  
    ESP_ERROR_CHECK(i2c_master_init(config.gpioSclDer, config.gpioSdaDer));    
    ESP_ERROR_CHECK(as5600_enable_pwm(config.freq));
    ESP_ERROR_CHECK(i2c_master_deInit());

    pwm_capture_init(config.gpioInIzq, config.gpioInDer);
    ESP_LOGI(TAG, "AS5600 I2C initialized");

    while (1) {
        xTaskNotifyWait(0, ULONG_MAX, &notified_bits, portMAX_DELAY);

        if (notified_bits & SENSOR_IZQ_BIT) {
            uint16_t angleIzq = as5600GetAnglePwm(&as5600Izq);
            float angleIzqRad = angleIzq * (2.0f * M_PI / 4096.0f);

            as5600_queue_data_t sensorData = {
                .side = AS5600_IZQ,
                .radAngle = angleIzqRad
            };
            xQueueSend(config.queue, &sensorData, 0);
        }

        if (notified_bits & SENSOR_DER_BIT) {
            uint16_t angleDer = as5600GetAnglePwm(&as5600Der);
            float angleDerRad = angleDer * (2.0f * M_PI / 4096.0f);

            as5600_queue_data_t sensorData = {
                .side = AS5600_DER,
                .radAngle = angleDerRad
            };
            xQueueSend(config.queue, &sensorData, 0);
        }
    }
}

void as5600Init(as5600_config_t *config) {

    as5600_config_t *internal_cfg = malloc(sizeof(as5600_config_t));
    assert(internal_cfg != NULL);

    *internal_cfg = *config;  // copia interna para mantener la estructura en memoria

    gpio_set_direction(internal_cfg->gpioInIzq, GPIO_MODE_INPUT);
    gpio_set_direction(internal_cfg->gpioInDer, GPIO_MODE_INPUT);
    xTaskCreatePinnedToCore(as5600Task, "as5600 task", 4096, internal_cfg, internal_cfg->priority, &as5600_task_handle, APP_CPU_NUM);
}
