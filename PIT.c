/*
 * PIT.c
 *
 *  Created on: 21 feb 2022
 *      Author: miel_
 */

#include "PIT.h"

static void (*g_PIT_callback)(void) = 0;
uint8_t g_PIT0_flag = FALSE;

void PIT_callback_init(PIT_timer_t pit_timer,void (*handler)(void))
{
	if(PIT_0 == pit_timer)
	{
		g_PIT_callback = handler;
	}
}

void PIT0_IRQHandler(void) /* ISR PIT_0*/
{
	uint8_t dummyRead;
	PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;	//Aquí se limpia interrupción
	dummyRead = PIT->CHANNEL[0].TCTRL;
	g_PIT0_flag = TRUE;
	if(g_PIT_callback != 0)
	{
		g_PIT_callback();
	}
}

void PIT_clock_gating(void)
{
	SIM->SCGC6 |= PIT_CLOCK_GATING;
}

void PIT_delay(PIT_timer_t pit_timer, My_float_pit_t system_clock , My_float_pit_t delay)
{
	PIT->CHANNEL[pit_timer].LDVAL= (uint32_t)(delay * system_clock -1);
	PIT_enable_interrupt(pit_timer);
}

void PIT_clear_interrupt_flag(void){
	g_PIT0_flag = FALSE;
}

void PIT_enable(void){
	PIT->MCR &= ~PIT_MDIS;	//Sets timers
	PIT->MCR |= PIT_FRZ_DB;	// Timers respond to debug session
}

void PIT_enable_interrupt(PIT_timer_t pit){
	PIT->CHANNEL[pit].TCTRL |= PIT_TCTRL_TIE_MASK;
	PIT->CHANNEL[pit].TCTRL |= PIT_TCTRL_TEN_MASK;
}

uint8_t PIT_get_interrupt_flag_status(void)
{
	return g_PIT0_flag;
}


void PIT_delay(PIT_timer_t pit_timer, My_float_pit_t system_clock , My_float_pit_t delay);
void PIT_clock_gating(void);
uint8_t PIT_get_interrupt_flag_status(void);
void PIT_clear_interrupt_flag(void);
void PIT_enable(void);
void PIT_enable_interrupt(PIT_timer_t pit);
