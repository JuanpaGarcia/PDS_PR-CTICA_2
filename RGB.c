/*
 * RGB.c
 *
 *  Created on: 15 feb 2022
 *      Author: garci
 */


#include "RGB.h"

void red()
{
	GPIO_set_pin(GPIO_E,led_green);
	GPIO_set_pin(GPIO_B,led_blue);
	GPIO_clear_pin(GPIO_B,led_red);
}

void green()
{
	GPIO_set_pin(GPIO_B,led_red);
	GPIO_set_pin(GPIO_B,led_blue);
	GPIO_clear_pin(GPIO_E,led_green);
}

void blue()
{
	GPIO_set_pin(GPIO_E,led_green);
	GPIO_set_pin(GPIO_B,led_red);
	GPIO_clear_pin(GPIO_B,led_blue);
}

void white()
{
	GPIO_clear_pin(GPIO_E,led_green);
	GPIO_clear_pin(GPIO_B,led_blue);
	GPIO_clear_pin(GPIO_B,led_red);
}

void purple()
{
	GPIO_set_pin(GPIO_E,led_green);
	GPIO_clear_pin(GPIO_B,led_red);
	GPIO_clear_pin(GPIO_B,led_blue);
}

void yellow()
{
	GPIO_clear_pin(GPIO_E,led_green);
	GPIO_clear_pin(GPIO_B,led_red);
	GPIO_set_pin(GPIO_B,led_blue);
}
void off()
{
	GPIO_set_pin(GPIO_E,led_green);
	GPIO_set_pin(GPIO_B,led_blue);
	GPIO_set_pin(GPIO_B,led_red);
}

void red();
void green();
void blue();
void white();
void purple();
void yellow();
void off();
