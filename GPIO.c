/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	18/02/2019
	\todo
	    Interrupts are not implemented in this API implementation.
 */

#include "GPIO.h"

static volatile gpio_interrupt_flags_t g_intr_status_flag = {0};
static void (*gpio_C_callback)(int *volume) = 0;
static void (*gpio_A_callback)(int *volume) = 0;
static void (*gpio_B_callback)(void) = 0;
static int *volume_h;

void GPIO_callback_init(gpio_port_name_t port_name, void (*handler)(int*), int * volume)
{
	volume_h = volume;
	if(GPIO_A == port_name)
	{
		gpio_A_callback = handler;
	}
	else
	{
		gpio_C_callback = handler;
	}
}

void PORTC_IRQHandler(void)
{
	if(gpio_C_callback)
	{
		gpio_C_callback(volume_h);
	}
	g_intr_status_flag.flag_port_c = TRUE;
	GPIO_clear_interrupt(GPIO_C);
}

void PORTB_IRQHandler(void)
{
	if(gpio_B_callback)
	{
		gpio_B_callback();
	}
	g_intr_status_flag.flag_port_b = TRUE;
	GPIO_clear_interrupt(GPIO_B);
}

void PORTA_IRQHandler(void)
{
	if(gpio_A_callback)
	{
		gpio_A_callback(volume_h);
	}
	g_intr_status_flag.flag_port_a = TRUE;
	GPIO_clear_interrupt(GPIO_A);
}

void GPIO_clear_irq_status(gpio_port_name_t gpio)
{
	if(GPIO_A == gpio)
	{
		g_intr_status_flag.flag_port_a = FALSE;
	}
	else
	{
		if(GPIO_C == gpio){
		g_intr_status_flag.flag_port_c = FALSE;
	}
		else{
			g_intr_status_flag.flag_port_b = FALSE;
		}
	}
}

uint8_t GPIO_get_irq_status(gpio_port_name_t gpio)
{
	uint8_t status = 0;

	if(GPIO_A == gpio)
	{
		status = g_intr_status_flag.flag_port_a;
	}
	else
	{
		if(GPIO_C == gpio){
			status = g_intr_status_flag.flag_port_c;
	}
		else{
			status = g_intr_status_flag.flag_port_b;
		}
	}
	return(status);
}

void GPIO_clear_interrupt(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			PORTA->ISFR=0xFFFFFFFF;
			break;
		case GPIO_B: /** GPIO B is selected*/
			PORTB->ISFR=0xFFFFFFFF;
			break;
		case GPIO_C: /** GPIO C is selected*/
			PORTC->ISFR = 0xFFFFFFFF;
			break;
		case GPIO_D: /** GPIO D is selected*/
			PORTD->ISFR=0xFFFFFFFF;
			break;
		default: /** GPIO E is selected*/
			PORTE->ISFR=0xFFFFFFFF;
			break;

	}// end switch
}

uint8_t GPIO_clock_gating(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
			{
				case GPIO_A: /** GPIO A is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
					break;
				case GPIO_B: /** GPIO B is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
					break;
				case GPIO_C: /** GPIO C is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
					break;
				case GPIO_D: /** GPIO D is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
					break;
				default: /**If doesn't exist the option*/
					return(FALSE);
			}// end switch
	/**Successful configuration*/
	return(TRUE);
}// end function

uint8_t GPIO_pin_control_register(gpio_port_name_t port_name, uint8_t pin,const gpio_pin_control_register_t*  pin_control_register)
{

	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] = *pin_control_register;
			break;
		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] = *pin_control_register;
			break;
		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] = *pin_control_register;
			break;
		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] = *pin_control_register;
			break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin]= *pin_control_register;
		default:/**If doesn't exist the option*/
			return(FALSE);
		break;
		}
	/**Successful configuration*/
	return(TRUE);
}

void GPIO_write_port(gpio_port_name_t port_name, uint32_t data)//function for writing data on selected GPIO PORT
{
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PSOR = data;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PSOR = data;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PSOR = data;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PSOR = data;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PSOR = data;
		default:/**If doesn't exist the option*/
		break;
		}
	}

void GPIO_data_direction_port(gpio_port_name_t port_name ,uint32_t direction)
{
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR |= direction;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR |= direction;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR |= direction;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR |= direction;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PDDR |= direction;
		default:/**If doesn't exist the option*/
		break;
		}

}

void GPIO_data_direction_pin(gpio_port_name_t port_name, uint8_t state, uint8_t pin)//function for configuring pins as I/O
{
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR |= state<<pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR |= state<<pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR |= state<<pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR |= state<<pin;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PDDR |= state<<pin;
		default:/**If doesn't exist the option*/
		break;
		}
}

uint32_t GPIO_read_port(gpio_port_name_t port_name)
{
	uint32_t return_value;
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			return_value = (GPIOA->PDIR);
			break;
		case GPIO_B:/** GPIO B is selected*/
			return_value = (GPIOB->PDIR);
			break;
		case GPIO_C:/** GPIO C is selected*/
			return_value = (GPIOC->PDIR);
			break;
		case GPIO_D:/** GPIO D is selected*/
			return_value = (GPIOD->PDIR);
			break;
		case GPIO_E: /** GPIO E is selected*/
			return_value = (GPIOE->PDIR);
		default:/**If doesn't exist the option*/
		break;
		}
	return return_value;
}

uint8_t GPIO_read_pin(gpio_port_name_t port_name, uint8_t pin)
{
	uint8_t return_value;
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			return_value = ((GPIOA->PDIR)&((1<<pin)));
			break;
		case GPIO_B:/** GPIO B is selected*/
			return_value = ((GPIOB->PDIR)&((1<<pin)));
			break;
		case GPIO_C:/** GPIO C is selected*/
			return_value = ((GPIOC->PDIR)&((1<<pin)));
			break;
		case GPIO_D:/** GPIO D is selected*/
			return_value = ((GPIOD->PDIR)&((1<<pin)));
			break;
		case GPIO_E: /** GPIO E is selected*/
			return_value = ((GPIOE->PDIR)&((1<<pin)));
		default:/**If doesn't exist the option*/
		break;
		}
	return return_value;
}

void GPIO_clear_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PCOR = 1<<pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PCOR = 1<<pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PCOR = 1<<pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PCOR = 1<<pin;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PCOR = 1<<pin;
		default:/**If doesn't exist the option*/
		break;
		}
}



void GPIO_set_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PSOR = 1<<pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PSOR = 1<<pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PSOR = 1<<pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PSOR = 1<<pin;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PSOR = 1<<pin;
		default:/**If doesn't exist the option*/
		break;
		}
}

void GPIO_toogle_pin(gpio_port_name_t port_name, uint8_t pin)
{
	switch(port_name)
		{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PTOR = 1<<pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PTOR = 1<<pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PTOR = 1<<pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PTOR = 1<<pin;
			break;
		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PTOR = 1<<pin;
		default:/**If doesn't exist the option*/
		break;
		}
}

void GPIO_write_port(gpio_port_name_t port_name, uint32_t data);
uint32_t GPIO_read_port(gpio_port_name_t port_name);
uint8_t GPIO_read_pin(gpio_port_name_t port_name, uint8_t pin);
void GPIO_set_pin(gpio_port_name_t port_name, uint8_t pin);
void GPIO_clear_pin(gpio_port_name_t port_name, uint8_t pin);
void GPIO_toogle_pin(gpio_port_name_t port_name, uint8_t pin);
void GPIO_data_direction_port(gpio_port_name_t port_name ,uint32_t direction);
void GPIO_data_direction_pin(gpio_port_name_t port_name, uint8_t state, uint8_t pin);
void GPIO_callback_init(gpio_port_name_t port_name, void (*handler)(int *volume), int * volume);
void GPIO_clear_irq_status(gpio_port_name_t gpio);
uint8_t GPIO_get_irq_status(gpio_port_name_t gpio);
void GPIO_clear_interrupt(gpio_port_name_t port_name);

