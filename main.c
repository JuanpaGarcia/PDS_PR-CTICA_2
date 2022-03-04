
#include <stdio.h>
#include "GPIO.h"
#include "PIT.h"
#include "NVIC.h"
#include "Bits.h"

#define x_n_lenght (7)//
#define h_n_lenght (7)
#define y_n_lenght ((x_n_lenght) + (h_n_lenght) - 1)
#define GPIO_OFF_CONST (0xFFFFFFFFU)
#define DELAY_TIME 0.045351473f
#define SYSTEM_CLOCK (21000000U)
#define vol_const 20


void pointer_void_funct(float *x, float *h,float *y);
void init();
void init_interrupt();
void volume_up(int *volume);
void volume_down(int *volume);

uint8_t state_select();

int main(void) {
	float h_1 [h_n_lenght] = {-0.1050, -0.1448, -0.1721, 0.8182, -0.1721, -0.1448, -0.1050};
	float h_2 [h_n_lenght] = {0.0802, 0.0860, 0.0897, 0.0909, 0.0897, 0.0860, 0.0802};
	float x[x_n_lenght] = {0};
	float y[y_n_lenght] = {0};
	int state = 0, turn = 0, volume = 0;
	float read_value;
	init();
	init_interrupt();
	void (*ptr_to_f)(int *)= volume_up;
	GPIO_callback_init(GPIO_A, ptr_to_f,&volume);
	ptr_to_f = volume_down;
	GPIO_callback_init(GPIO_C, ptr_to_f,&volume);
	PIT_delay(PIT_0, SYSTEM_CLOCK, DELAY_TIME);// EMPEZAMOS LA FREQ DE MUESTREO


    while(1) {
    	if(GPIO_get_irq_status(GPIO_B))
    		{
    			state_select();
    			GPIO_clear_irq_status(GPIO_B);
    		}
    	if(PIT_get_interrupt_flag_status){
    		//leear del adc y agregar a read_value

    		x[turn]= read_value + volume;
    		turn++;
    		PIT_clear_interrupt_flag();
    	}
    	if(y_n_lenght+1 == turn){
    		turn = 0;
    		pointer_void_funct(x, h_1, y);
    		int i;
    		//pasar los valores al ADC
    		for (i=0;i<y_n_lenght;i++)
    		{
    			//print en adc en y[i]
    		}
    	}
    }
    return 0 ;
}

void init()
{





	//configurar entradas switches puerto b como
	gpio_pin_control_register_t input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_FALLING_EDGE; // SW interrupt config
	gpio_pin_control_register_t pcr_gpioe_pin_led = GPIO_MUX1;
	gpio_pin_control_register_t pcr_gpiob_pin_led = GPIO_MUX1;
	gpio_pin_control_register_t input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_EITHER_EDGE;//interrupcion sw puerto b

	GPIO_clock_gating(GPIO_A);//VOLUMEN SW3
	GPIO_clock_gating(GPIO_B);//SWITHCES
	GPIO_clock_gating(GPIO_C);//VOLUMEN SW2


	GPIO_pin_control_register(GPIO_A,bit_4, &input_intr_config);
	GPIO_pin_control_register(GPIO_C,bit_6, &input_intr_config);


	GPIO_data_direction_pin(GPIO_C, GPIO_INPUT, bit_6);
	GPIO_data_direction_pin(GPIO_A, GPIO_INPUT, bit_4);

	PIT_clock_gating();
	PIT_enable();

}

void init_interrupt()
{

	/**Sets the threshold for interrupts, if the interrupt has higher priority constant that the BASEPRI, the interrupt will not be attended*/
	NVIC_set_basepri_threshold(PRIORITY_10);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_2);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enable_interrupt_and_priotity(PORTC_IRQ, PRIORITY_4);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enable_interrupt_and_priotity(PORTA_IRQ, PRIORITY_4);
	/*Puerto para la selección de estado, swithces*/
	NVIC_enable_interrupt_and_priotity(PORTB_IRQ, PRIORITY_5);


	NVIC_global_enable_interrupts;

}

uint8_t state_select()
{
	int a;
	uint8_t return_state = 0;
	a = GPIO_read_port(GPIO_B) &(0x0C);
	switch(a){
	case 0x0C:
			return_state = 3;
			break;
	case 0x04:
		return_state = 2;
		break;
	case 0x08:
		return_state = 1;
		break;
	default:
		return_state = 0;
		break;

	}
	return return_state;
}

void volume_up(int *volume)
{
	*volume += vol_const;
}

void volume_down(int *volume)
{
	*volume -= vol_const;
}
void pointer_void_funct(float *x, float *h,float *y)
{
    int i,j;
    for(i=0; i<y_n_lenght; i++) {
        y[i]=0;
        for(j=0; j < h_n_lenght; j++) {
            if(i-j < 0 || i-j >= x_n_lenght) continue;
            y[i] = y[i]+h[j]*x[i-j];
        }
    }
}
