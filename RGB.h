/*
 * RGB.h
 *
 *  Created on: 15 feb 2022
 *      Author: garci
 */

#ifndef RGB_H_
#define RGB_H_

#include "MK64F12.h"
#include "GPIO.h"
#include "Bits.h"

#define led_red 22
#define led_green 26
#define led_blue 21


void red();
void green();
void blue();
void white();
void purple();
void yellow();
void off();

#endif /* RGB_H_ */
