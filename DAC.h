/*
 * DAC.h
 *
 *  Created on: 4 mar 2022
 *      Author: garci
 */

#ifndef DAC_H_
#define DAC_H_

#include "fsl_dac.h"
#include "fsl_common.h"
#define DAC_USE DAC0

void DAC_init_config(dac_config_t *config);
void DAC_write_value(dac_config_t *config, uint32_t value);

#endif /* DAC_H_ */
