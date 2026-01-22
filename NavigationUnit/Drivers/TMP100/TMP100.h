#ifndef TMP100_H_
#define TMP100_H_

#include "stm32f4xx_hal.h"

#define TMP100_DEFAULT_ADDRESS 	(0x48 << 1)

#define TMP100_DATA_REGISTER 	0x00
#define TMP100_CONFIG_REGISTER 	0x01

typedef enum
{
	TMP100_MODE_NORMAL = 0,
	TMP100_MODE_SHUTDOWN = 1
} TMP100_Mode;

typedef enum
{
	TMP100_THERMOSTAT_COMPARATOR = 0,
	TMP100_THERMOSTAT_INTERRUPT = 1
} TMP100_Thermostat;

typedef enum
{
	TMP100_POLARITY_LOW = 0,
	TMP100_POLARITY_HIGH = 1
} TMP100_Polarity;

typedef enum
{
	TMP100_FAULT_QUEUE_1 = 0,
	TMP100_FAULT_QUEUE_2 = 1,
	TMP100_FAULT_QUEUE_4 = 2,
	TMP100_FAULT_QUEUE_6 = 3
} TMP100_FaultQueue;

typedef enum
{
	TMP100_RESOLUTION_9BITS = 0,
	TMP100_RESOLUTION_10BITS = 1,
	TMP100_RESOLUTION_11BITS = 2,
	TMP100_RESOLUTION_12BITS = 3
} TMP100_Resolution;

typedef enum
{
	TMP100_ONE_SHOT_OFF = 0,
	TMP100_ONE_SHOT_ON = 1
} TMP100_OneShot;

typedef struct
{
	TMP100_Mode mode;
	TMP100_Thermostat thermostat;
	TMP100_Polarity polarity;
	TMP100_FaultQueue fault_queue;
	TMP100_Resolution resolution;
	TMP100_OneShot one_shot;
} TMP100_Config;

typedef struct
{
	I2C_HandleTypeDef i2c_bus;
	uint8_t addr;

	TMP100_Config cfg;

	uint8_t buf[2];
} TMP100_Handle;

HAL_StatusTypeDef TMP100_Initialize(TMP100_Handle* t, I2C_HandleTypeDef* i2c_bus, uint8_t addr = TMP100_DEFAULT_ADDRESS);
HAL_StatusTypeDef TMP100_BLKConfigure(TMP100_Handle* t);
HAL_StatusTypeDef TMP100_DMAStartRead(TMP100_Handle* t);
float TMP100_GetTemperature(TMP100_Handle* t);
uint8_t TMP100_ConfigToValue(TMP100_Config c);

#endif /* TMP100_H_ */
