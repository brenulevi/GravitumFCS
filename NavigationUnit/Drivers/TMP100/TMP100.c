#include "TMP100.h"

HAL_StatusTypeDef TMP100_Initialize(TMP100_Handle* t, I2C_HandleTypeDef* i2c_bus, uint8_t addr)
{
	if(!t)
		return HAL_ERROR;

	t->i2c_bus = i2c_bus;
	t->addr = addr;

	// Default configuration
	t->cfg.mode = TMP100_MODE_NORMAL;
	t->cfg.thermostat = TMP100_THERMOSTAT_COMPARATOR;
	t->cfg.polarity = TMP100_POLARITY_LOW;
	t->cfg.fault_queue = TMP100_FAULT_QUEUE_1;
	t->cfg.resolution = TMP100_RESOLUTION_10BITS;
	t->cfg.one_shot = TMP100_ONE_SHOT_OFF;

	// Apply default configuration
	return TMP100_BLKConfigure(t);
}

HAL_StatusTypeDef TMP100_BLKConfigure(TMP100_Handle* t)
{
	if(!t)
		return HAL_ERROR;

	uint8_t fmt_cfg = TMP100_ConfigToValue(t->cfg);

	return HAL_I2C_Mem_Write(
			t->i2c_bus,
			t->addr,
			TMP100_CONFIG_REGISTER,
			I2C_MEMADD_SIZE_8BIT,
			&fmt_cfg,
			1,
			HAL_MAX_DELAY
	);
}

HAL_StatusTypeDef TMP100_DMAStartRead(TMP100_Handle* t)
{
	if(!t)
		return HAL_ERROR;

	return HAL_I2C_Mem_Read_DMA(
		t->i2c_bus,
		t->addr,
		TMP100_DATA_REGISTER,
		I2C_MEMADD_SIZE_8BIT,
		buf,
		2
	);
}

float TMP100_GetTemperature(TMP100_Handle* t)
{

}

uint8_t TMP100_ConfigToValue(TMP100_Config c)
{
	uint8_t v = 0;

	v |= (cfg->mode        & 0x01) << 0; // SD
	v |= (cfg->thermostat  & 0x01) << 1; // TM
	v |= (cfg->polarity    & 0x01) << 2; // POL
	v |= (cfg->fault_queue & 0x03) << 3; // F1:F0
	v |= (cfg->resolution  & 0x03) << 5; // R1:R0
	v |= (cfg->one_shot    & 0x01) << 7; // OS

	return v;
}
