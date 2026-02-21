#ifndef __HALL_ENCODER_H__
#define __HALL_ENCODER_H__

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define AS5600_DER  1
#define AS5600_IZQ  2

typedef struct {
    float radAngle;
    uint8_t side;
} as5600_queue_data_t;

typedef enum {
    AS5600_PWM_115HZ = 0b00,
    AS5600_PWM_230HZ = 0b01,
    AS5600_PWM_460HZ = 0b10,
    AS5600_PWM_920HZ = 0b11
} as5600_pwm_freq_t;

typedef struct {
    uint8_t         gpioSclDer;
    uint8_t         gpioSdaDer;
    uint8_t         gpioInDer;
    uint8_t         gpioSclIzq;
    uint8_t         gpioSdaIzq;
    uint8_t         gpioInIzq;
    as5600_pwm_freq_t   freq;
    QueueHandle_t   queue;
    uint8_t         core;
    uint8_t         priority;
} as5600_config_t;

void as5600Init(as5600_config_t *config);

#endif
