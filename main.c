
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
#include "fsl_dmamux.h"
#include "fsl_edma.h"


#define DEMO_ADC16_CHANNEL       12U
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_BASEADDR      ADC0
#define DEMO_DMAMUX_BASEADDR     DMAMUX0
#define DEMO_DMA_CHANNEL         0U
#define DEMO_DMA_ADC_SOURCE      40U
#define DEMO_DMA_BASEADDR        DMA0
#define ADC16_RESULT_REG_ADDR    0x4003b010U
#define DEMO_DMA_IRQ_ID          DMA0_IRQn
#define DEMO_ADC16_SAMPLE_COUNT 16U /* The ADC16 sample count. */

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

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Initialize the EDMA.
 */
static void EDMA_Configuration(void);

/*!
 * @brief Initialize the DMAMUX.
 */
static void DMAMUX_Configuration(void);

/*!
 * @brief Initialize the ADC16.
 */
static void ADC16_Configuration(void);

/*!
 * @brief Process ADC values.
 */
static void ProcessSampleData(void);

/*!
 * @brief Callback function for EDMA.
 */
static void Edma_Callback(edma_handle_t *handle, void *userData, bool transferDone, uint32_t tcds);

volatile bool g_Transfer_Done = false; /* DMA transfer completion flag. */
uint32_t g_adc16SampleDataArray[DEMO_ADC16_SAMPLE_COUNT];
uint32_t g_avgADCValue = 0U; /* Average ADC value .*/
edma_handle_t g_EDMA_Handle; /* Edma handler. */
edma_transfer_config_t g_transferConfig;
const uint32_t g_Adc16_16bitFullRange = 65536U;

int main(void) {

	float h_0 [h_n_lenght] = {1, 0, 0, 0, 0, 0, 0};	//Respuesta 1 para reproducri cacnión normal
	float h_1 [h_n_lenght] = {-0.1050, -0.1448, -0.1721, 0.8182, -0.1721, -0.1448, -0.1050};//Respuesta h1
	float h_2 [h_n_lenght] = {0.0802, 0.0860, 0.0897, 0.0909, 0.0897, 0.0860, 0.0802};//Respuesta h2
	float *h=&h_0;
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



    EDMA_Configuration();   /* Initialize EDMA. */
    DMAMUX_Configuration(); /* Initialize DMAMUX. */
    ADC16_Configuration();  /* Initialize ADC16. */

    adcChnConfig.channelNumber = DEMO_ADC16_CHANNEL;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adcChnConfig.enableDifferentialConversion = false;
#endif
    adcChnConfig.enableInterruptOnConversionCompleted = false;
    ADC16_SetChannelConfig(DEMO_ADC16_BASEADDR, DEMO_ADC16_CHANNEL_GROUP, &adcChnConfig);
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
    	    for(int i=x_n_lenght;i>0;i--)	// este for recorre el arreglo y las muestras pasadas para pushear una nueva
    		{

    			x[i] = x[i-1];

    		}
    		ProcessSampleData();
    		x[0]= g_avgADCValue + volume;	//no se si sea el sumar o que miltiplique
    		//void pointer_void_funct(float *x, float *h,float *y)
    		pointer_void_funct(x,h,&y);
    		conv = floor(y);
    		if(4094>conv) conv=4094;
    		//void DAC_write_value(dac_config_t *config, uint32_t value);
    		DAC_write_value(&dacConfigStruct,conv);
    		PIT_clear_interrupt_flag();
    	}
    return 0 ;
}


void init()
{

	BOARD_InitBootClocks();
	//configurar entradas switches puerto b como
	gpio_pin_control_register_t input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_FALLING_EDGE; // SW interrupt config
	gpio_pin_control_register_t input_intr_config_dp_sw = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_EITHER_EDGE;//interrupcion sw puerto b

	GPIO_clock_gating(GPIO_A);//VOLUMEN SW3
	GPIO_clock_gating(GPIO_B);//SWITHCES
	GPIO_clock_gating(GPIO_C);//VOLUMEN SW2


	GPIO_pin_control_register(GPIO_A,bit_4, &input_intr_config);
	GPIO_pin_control_register(GPIO_C,bit_6, &input_intr_config);

	GPIO_pin_control_register(GPIO_B, bit_3,&input_intr_config_dp_sw);
	GPIO_pin_control_register(GPIO_B, bit_10,&input_intr_config_dp_sw);

	GPIO_data_direction_pin(GPIO_C, GPIO_INPUT, bit_6);
	GPIO_data_direction_pin(GPIO_A, GPIO_INPUT, bit_4);

	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, bit_3);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, bit_10);
	//


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
