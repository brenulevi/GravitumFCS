#ifndef TMP100_H
#define TMP100_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define TMP100_DEFAULT_ADDRESS 0x48 << 1  // Default I2C address for TMP100

#define TMP100_TEMP_REGISTER 0x00   // Temperature register
#define TMP100_CONFIG_REGISTER 0x01 // Configuration register

#define TMP100_SHUTDOWN_MODE (1 << 0)
#define TMP100_NORMAL_MODE   (0 << 0)

#define TMP100_THERMOSTAT_MODE_COMPARATOR (0 << 1)
#define TMP100_THERMOSTAT_MODE_INTERRUPT  (1 << 1)

#define TMP100_POLARITY_ACTIVE_LOW  (0 << 2)
#define TMP100_POLARITY_ACTIVE_HIGH (1 << 2)

#define TMP100_FAULT_QUEUE_1 (0 << 3)
#define TMP100_FAULT_QUEUE_2 (1 << 3)
#define TMP100_FAULT_QUEUE_4 (2 << 3)
#define TMP100_FAULT_QUEUE_6 (3 << 3)

#define TMP100_RESOLUTION_9BIT  (0 << 5)
#define TMP100_RESOLUTION_10BIT (1 << 5)
#define TMP100_RESOLUTION_11BIT (2 << 5)
#define TMP100_RESOLUTION_12BIT (3 << 5)

#define TMP100_ONE_SHOT_ENABLE  (1 << 7)
#define TMP100_ONE_SHOT_DISABLE (0 << 7)

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t address;
    uint8_t config;
} TMP100_Handle;

HAL_StatusTypeDef TMP100_Init(TMP100_Handle* tmp100);
HAL_StatusTypeDef TMP100_ReadTemperature(TMP100_Handle* tmp100, float* temperature);
HAL_StatusTypeDef TMP100_Configure(TMP100_Handle* tmp100, uint8_t config);

#endif // TMP100_H
