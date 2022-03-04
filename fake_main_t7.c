/*
 * main.c
 *
 *  Created on: 24 feb 2022
 *      Author: garci
 */
//#include "MK64F12.h"
//#include "PIT.h"
//#include "NVIC.h"
//#include "GPIO.h"
//#include "Bits.h"
//#include "RGB.h"
//
//#define GPIO_OFF_CONST (0xFFFFFFFFU)
//#define DELAY_TIME 2.0f
//#define SYSTEM_CLOCK (21000000U)
//#define Color_Quantity_Per_Funciton 3
//#define SYSTEM_CLOCK (21000000U)
//
//typedef struct
//{
//	void* ColorMatrix[Color_Quantity_Per_Funciton];
//	PIT_timer_t Timer;
//	void(*fptrcallback)(PIT_timer_t pit_timer, void (*handler)(void));
//	uint8_t next[4];
//}State_t;
//
//typedef enum {No_Color, Secuence_1, Secuence2, Secuence_3} Color_Secuences;
//
//const State_t FSM[4] =
//{
//		{{off, off, off}, PIT_0, PIT_callback_init, {No_Color, Secuence_1, Secuence2, Secuence_3}},
//		{{yellow, red, purple}, PIT_0, PIT_callback_init, {No_Color, Secuence_1, Secuence2, Secuence_3}},
//		{{green, red, white}, PIT_0, PIT_callback_init, {No_Color, Secuence_1, Secuence2, Secuence_3}},
//		{{blue, green, white}, PIT_0, PIT_callback_init, {No_Color, Secuence_1, Secuence2, Secuence_3}}
//};
//
//void init_interrupt();
//void init();
//
//
//int no_main()
//{
//	init();
//	init_interrupt();
//
//	uint8_t state = No_Color;
//	uint8_t turn = 0;
//	uint8_t sw_input = Not_Pressed;
//	void (*ftpr_selected_color)(void) = 0;
//	PIT_delay(PIT_0, SYSTEM_CLOCK, DELAY_TIME);
//
//
//	while(TRUE)
//	{
//
//		if (Color_Quantity_Per_Funciton <= turn) turn = 0; //Ensure not accesing an outbound value in Color Matrix
//		ftpr_selected_color = FSM[state].ColorMatrix[turn];
//		FSM[state].fptrcallback(FSM[state].Timer, ftpr_selected_color);
//
//		if(GPIO_get_irq_status(GPIO_C)){
//			if(BIT_OFF == GPIO_read_pin(GPIO_A, 4)){
// 				sw_input = SW2_SW3;
//				turn = 0;									//Resets count
//				GPIO_clear_irq_status(GPIO_A);
//				GPIO_clear_irq_status(GPIO_B);
//			}else{
//				sw_input = SW2;
//				turn = 0;									//Resets count
//				GPIO_clear_irq_status(GPIO_C);
//			}
//		}else{
//			if(GPIO_get_irq_status(GPIO_A)){
//				GPIO_clear_irq_status(GPIO_A);
//				if(BIT_OFF == GPIO_read_pin(GPIO_C, 6)){
//
//				}else{
//					sw_input = SW3;
//					turn = 0;								//Resets count
//				}
//			}
//		}
//
//		state = FSM[state].next[sw_input];
//
//		if(PIT_get_interrupt_flag_status())
//		{
//			turn++;
//			PIT_clear_interrupt_flag();
//		}
//	}
//	return 0;
//}
//
//void init()
//{
//	gpio_pin_control_register_t input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_FALLING_EDGE; // SW interrupt config
//	gpio_pin_control_register_t pcr_gpioe_pin_led = GPIO_MUX1;
//	gpio_pin_control_register_t pcr_gpiob_pin_led = GPIO_MUX1;
//
//	GPIO_clock_gating(GPIO_A);
//	GPIO_clock_gating(GPIO_B);
//	GPIO_clock_gating(GPIO_C);
//	GPIO_clock_gating(GPIO_E);
//
//	GPIO_pin_control_register(GPIO_A,bit_4, &input_intr_config);
//	GPIO_pin_control_register(GPIO_C,bit_6, &input_intr_config);
//
//	GPIO_pin_control_register(GPIO_B,bit_21,&pcr_gpiob_pin_led);
//	GPIO_pin_control_register(GPIO_B,bit_22,&pcr_gpiob_pin_led);
//	GPIO_pin_control_register(GPIO_E,bit_26,&pcr_gpioe_pin_led);
//
//	GPIO_write_port(GPIO_B, GPIO_OFF_CONST);
//	GPIO_write_port(GPIO_E, GPIO_OFF_CONST);
//
//	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, bit_21);
//	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, bit_22);
//	GPIO_data_direction_pin(GPIO_E, GPIO_OUTPUT, bit_26);
//	GPIO_data_direction_pin(GPIO_C, GPIO_INPUT, bit_6);
//	GPIO_data_direction_pin(GPIO_A, GPIO_INPUT, bit_4);
//
//	PIT_clock_gating();
//	PIT_enable();
//
//}
//
//
//
//void init_interrupt()
//{
//
//	/**Sets the threshold for interrupts, if the interrupt has higher priority constant that the BASEPRI, the interrupt will not be attended*/
//	NVIC_set_basepri_threshold(PRIORITY_10);
//	/**Enables and sets a particular interrupt and its priority*/
//	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_3);
//	/**Enables and sets a particular interrupt and its priority*/
//	NVIC_enable_interrupt_and_priotity(PORTC_IRQ, PRIORITY_4);
//	/**Enables and sets a particular interrupt and its priority*/
//	NVIC_enable_interrupt_and_priotity(PORTA_IRQ, PRIORITY_4);
//
//	NVIC_global_enable_interrupts;
//
//}
