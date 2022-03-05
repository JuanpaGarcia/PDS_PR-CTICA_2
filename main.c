
#include <stdio.h>
#include "GPIO.h"
#include "PIT.h"
#include "NVIC.h"
#include "Bits.h"
#include "DAC.h"
#include "RGB.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_dac.h"
#include <stdio.h>
#include<math.h>

#include "fsl_common.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_adc16.h"

#define DEMO_ADC16_BASE          ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL  12U

#define x_n_lenght (7)//
#define h_n_lenght (7)
#define y_n_lenght ((x_n_lenght) + (h_n_lenght) - 1)
#define GPIO_OFF_CONST (0xFFFFFFFFU)
#define DELAY_TIME 0.045351473f
#define SYSTEM_CLOCK (21000000U)
#define vol_const 300
#define DAC_USAGE DAC0

void pointer_void_funct(float *x, float *h,float *y);
void init();
void init_interrupt();
void volume_up(int *volume);
void volume_down(int *volume);

uint8_t state_select();

const uint32_t g_Adc16_12bitFullRange = 4096U;

int main(void) {

	float h_0 [h_n_lenght] = {1, 0, 0, 0, 0, 0, 0};	//Respuesta 1 para reproducri cacnión normal
	float h_1 [h_n_lenght] = {-0.1050, -0.1448, -0.1721, 0.8182, -0.1721, -0.1448, -0.1050};//Respuesta h1
	float h_2 [h_n_lenght] = {0.0802, 0.0860, 0.0897, 0.0909, 0.0897, 0.0860, 0.0802};//Respuesta h2
	float *h = &h_0;
	float x[x_n_lenght+3] = {0};
	float *x1 = &x, *x2 = &x[7];
	float y;
	int state = 0, turn = 0, volume = 0;
	int conv;
	init();
	init_interrupt();
	void (*ptr_to_f)(int *)= volume_up;
	GPIO_callback_init(GPIO_A, ptr_to_f,&volume);
	ptr_to_f = volume_down;
	GPIO_callback_init(GPIO_C, ptr_to_f,&volume);


    uint8_t ready_flag;
    dac_config_t dacConfigStruct;
    uint32_t dacValue;

    DAC_init_config(&dacConfigStruct);


    adc16_config_t adc16ConfigStruct;
    adc16_channel_config_t adc16ChannelConfigStruct;
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
#ifdef BOARD_ADC_USE_ALT_VREF
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    adc16ChannelConfigStruct.channelNumber                        = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

    ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);

    red();
    PIT_delay(PIT_0, SYSTEM_CLOCK, DELAY_TIME);// EMPEZAMOS LA FREQ DE MUESTREO

    while(1) {
    	if(GPIO_get_irq_status(GPIO_B))
    		{
    			state++;
    			if(state>3) state = 0;
    	    	switch(state)
    	    	{
    	    	case 0:
    	    		// se escucha la señal original
    	    		h = &h_0[0];
    	    		red();
    	    		break;
    	    	case 1:	//Respuesta al impulso 1
    	    		h = &h_1[0];
    	    		blue();
    	    		break;
    	    	case 2:		//Respuesta al impulso 2
    	    		h = &h_2[0];
    	    		yellow();
    	    		break;
    	    	default:
    	    		white();
    	    		break;
    	    	}
    			GPIO_clear_irq_status(GPIO_B);
    		}
    	if(PIT_get_interrupt_flag_status()){
    		//leear del adc y agregar a read_value
    		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
    	    for(int i=x_n_lenght;i>0;i--)	// este for recorre el arreglo y las muestras pasadas para pushear una nueva
    		{

    			x[i] = x[i-1];

    		}
            while (0U == (kADC16_ChannelConversionDoneFlag &
                          ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
            {
            }
    		x[0]= ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP) + volume;	//no se si sea el sumar o que miltiplique
    		//void pointer_void_funct(float *x, float *h,float *y)
    		pointer_void_funct(x,h,&y);
    		conv = floor(y);
    		if(4094>conv) conv=4094;
    		//void DAC_write_value(dac_config_t *config, uint32_t value);
    		DAC_write_value(&dacConfigStruct,conv);
    		PIT_clear_interrupt_flag();
    	}
    }
    return 0 ;
}


void init()
{

	BOARD_InitBootClocks();
	//configurar entradas switches puerto b como
	gpio_pin_control_register_t input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_FALLING_EDGE; // SW interrupt config
	gpio_pin_control_register_t input_intr_config_dp_sw = GPIO_MUX1|GPIO_PS|INTR_EITHER_EDGE;//interrupcion sw puerto b

	GPIO_clock_gating(GPIO_A);//VOLUMEN SW3
	GPIO_clock_gating(GPIO_B);//SWITHCES
	GPIO_clock_gating(GPIO_C);//VOLUMEN SW2
	GPIO_clock_gating(GPIO_E);

	gpio_pin_control_register_t pcr_gpioe_pin_led = GPIO_MUX1;
	gpio_pin_control_register_t pcr_gpiob_pin_led = GPIO_MUX1;

	GPIO_pin_control_register(GPIO_B,bit_21,&pcr_gpiob_pin_led);
	GPIO_pin_control_register(GPIO_B,bit_22,&pcr_gpiob_pin_led);
	GPIO_pin_control_register(GPIO_E,bit_26,&pcr_gpioe_pin_led);

	GPIO_pin_control_register(GPIO_A,bit_4, &input_intr_config);
	GPIO_pin_control_register(GPIO_C,bit_6, &input_intr_config);

	GPIO_pin_control_register(GPIO_B, bit_3,&input_intr_config_dp_sw);
	GPIO_pin_control_register(GPIO_B, bit_10,&input_intr_config_dp_sw);

	GPIO_data_direction_pin(GPIO_C, GPIO_INPUT, bit_6);
	GPIO_data_direction_pin(GPIO_A, GPIO_INPUT, bit_4);

	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, bit_3);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, bit_10);
	//
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, bit_21);
	GPIO_data_direction_pin(GPIO_B, GPIO_OUTPUT, bit_22);
	GPIO_data_direction_pin(GPIO_E, GPIO_OUTPUT, bit_26);

	//
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
    int i;
    float count;
    for (i=0;i<x_n_lenght;i++)
    {
    	count = count + x[i]*h[i];
    }
    *y = count;
}
