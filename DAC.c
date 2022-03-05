/*
 * ADC.c
 *
 *  Created on: 4 mar 2022
 *      Author: garci
 */
#include "DAC.h"

void DAC_init_config(dac_config_t *config)
{
    DAC_GetDefaultConfig(config);
    DAC_Init(DAC_USE, config);
    DAC_Enable(DAC_USE, true);             /* Enable output. */
    DAC_SetBufferReadPointer(DAC_USE, 0U); /* Make sure the read pointer to the start. */
}

void DAC_write_value(dac_config_t *config, uint32_t value)
{
	DAC_SetBufferValue(DAC_USE, 0U, value);
}

void DAC_write_value(dac_config_t *config, uint32_t value);
void DAC_init_config(dac_config_t *config);
