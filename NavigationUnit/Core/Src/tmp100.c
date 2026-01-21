#include "tmp100.h"

HAL_StatusTypeDef TMP100_Init(TMP100_Handle *tmp100)
{
	if(tmp100->hi2c == NULL) {
		return HAL_ERROR;
	}

    if(tmp100->address == 0) {
        tmp100->address = TMP100_DEFAULT_ADDRESS;
    }

    uint8_t config = TMP100_NORMAL_MODE |
                     TMP100_THERMOSTAT_MODE_COMPARATOR |
                     TMP100_POLARITY_ACTIVE_LOW |
                     TMP100_FAULT_QUEUE_1 |
                     TMP100_RESOLUTION_10BIT |
                     TMP100_ONE_SHOT_DISABLE;

    return TMP100_Configure(tmp100, config);
}

HAL_StatusTypeDef TMP100_ReadTemperature(TMP100_Handle *tmp100, float *temperature)
{
	uint8_t temp[2];
	uint16_t temp_raw;

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        tmp100->hi2c,
        tmp100->address,
        TMP100_TEMP_REGISTER,
        I2C_MEMADD_SIZE_8BIT,
        temp,
        2,
        HAL_MAX_DELAY
    );

	if(ret != HAL_OK)
		return ret;

	// Convert temperature
}

HAL_StatusTypeDef TMP100_Configure(TMP100_Handle *tmp100, uint8_t config)
{
	tmp100->config = config;

    return HAL_I2C_Mem_Write(
        tmp100->hi2c,
        tmp100->address,
        TMP100_CONFIG_REGISTER,
        I2C_MEMADD_SIZE_8BIT,
        &config,
        1,
        HAL_MAX_DELAY
    );
}
