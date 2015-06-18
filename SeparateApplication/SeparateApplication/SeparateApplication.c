/*
 * SeparateApplication.c
 *
 * Created: 17.06.2015 12:40:57
 *  Author: Alex
 */ 

//------------------------------------
// Definitions
//------------------------------------
#define D1_CH1 0		// Detector 1
#define D1_CH2 4
#define D1_CH3 0
#define D1_CH4 2

#define D2_CH1 4		// Detector 2
#define D2_CH2 0
#define D2_CH3 4
#define D2_CH4 2

#define ENCD_A 0		// Encoder
#define ENCD_B 4
#define ENCD_Z 2
#define ENCD_LINE_COUNT 8000

#define TST_PIN_CNT1 0			// Pin for tests
#define TST_PIN_CNT2 1
#define TST_PIN_CNT3 2
#define TST_PIN_CNT4 3
#define TST_PIN_CNT5 4
#define TST_PIN_CNT6 5
#define TST_PIN_CNT7 6
#define TST_PIN_CNT8 7
#define TST_PIN_QDEC 0

#define PER_COUNTER1 999		// period of counters
#define PER_COUNTER2 999
#define PER_COUNTER3 999
#define PER_COUNTER4 999
#define PER_COUNTER5 999
#define PER_COUNTER6 999
#define PER_COUNTER7 999
#define PER_COUNTER8 999

//------------------------------------
// Includes
//------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>

//------------------------------------
// Global variables
//------------------------------------
unsigned int counter1 = 0;
unsigned int counter2 = 0;

//------------------------------------
// Interrupt handlers
//------------------------------------
ISR(TCC0_OVF_vect)
{
	cli();
	PORTJ_OUT |= (1 << TST_PIN_CNT1);
	PORTJ_OUT &= ~(1 << TST_PIN_CNT1);
}

ISR(TCC1_OVF_vect)
{
	cli();
	PORTJ_OUT |= (1 << TST_PIN_CNT2);
	PORTJ_OUT |= (1 << TST_PIN_CNT2);
}

ISR(TCD0_OVF_vect)
{
	cli();
	PORTJ_OUT |= (1 << TST_PIN_CNT3);
	PORTJ_OUT &= ~(1 << TST_PIN_CNT3);
}

ISR(TCD1_OVF_vect)
{
	cli();
	PORTJ_OUT |= (1 << TST_PIN_CNT4);
	PORTJ_OUT &= ~(1 << TST_PIN_CNT4);
}

ISR(TCE0_OVF_vect)
{
	cli();
	PORTJ_OUT |= (1 << TST_PIN_CNT5);
	PORTJ_OUT &= ~(1 << TST_PIN_CNT5);
}

ISR(TCE1_OVF_vect)
{
	cli();
	PORTJ_OUT |= (1 << TST_PIN_CNT6);
	PORTJ_OUT &= ~(1 << TST_PIN_CNT6);
}

ISR(PORTC_INT0_vect)
{
	cli();
	if(counter1 == PER_COUNTER7)
	{
		counter1 = 0;
		PORTJ_OUT |= (1 << TST_PIN_CNT7);
		PORTJ_OUT &= ~(1 << TST_PIN_CNT7);
	}
	else
	{
		++counter1;
	}
}

ISR(PORTE_INT0_vect)
{
	cli();
	if(counter2 == PER_COUNTER8)
	{
		counter2 = 0;
		PORTJ_OUT |= (1 << TST_PIN_CNT8);
		PORTJ_OUT &= ~(1 << TST_PIN_CNT8);
	}
	else
	{
		++counter2;
	}
}

ISR(TCF0_OVF_vect)
{
	cli();
	PORTK_OUT |= (1 << TST_PIN_QDEC);
	PORTK_OUT &= ~(1 << TST_PIN_QDEC);
}


//------------------------------------
// Prototypes
//------------------------------------
void Initialize();

//------------------------------------
// Entry point
//------------------------------------
int main(void)
{
	Initialize();
	sei();
	
	volatile int i = 0;
	
    while(1)
    {
        //TODO:: Please write your application code 
		if (i > 10)
			continue;
    }
}

//-------------------------------------
// Functions
//-------------------------------------
void Initialize()
{
	//---------------Detector Initialization-----------------------
	// Pins direction: input
	PORTC_DIR &= ~((1 << D1_CH1) | (1 << D1_CH2) | (1 << D1_CH4));
	PORTD_DIR &= ~((1 << D1_CH3) | (1 << D2_CH1));
	PORTE_DIR &= ~((1 << D2_CH2) | (1 << D2_CH3) | (1 << D2_CH4));
	
	// Sense configuration: rising
	PORTC_PIN0CTRL = PORT_ISC_RISING_gc;
	PORTC_PIN2CTRL = PORT_ISC_RISING_gc;
	PORTC_PIN4CTRL = PORT_ISC_RISING_gc;
	PORTD_PIN0CTRL = PORT_ISC_RISING_gc;
	PORTD_PIN4CTRL = PORT_ISC_RISING_gc;
	PORTE_PIN0CTRL = PORT_ISC_RISING_gc;
	PORTE_PIN2CTRL = PORT_ISC_RISING_gc;
	PORTE_PIN4CTRL = PORT_ISC_RISING_gc;
	
	// Set event multiplexers
	EVSYS_CH2MUX = EVSYS_CHMUX_PORTC_PIN0_gc;
	EVSYS_CH3MUX = EVSYS_CHMUX_PORTC_PIN4_gc;
	EVSYS_CH4MUX = EVSYS_CHMUX_PORTD_PIN0_gc;
	EVSYS_CH5MUX = EVSYS_CHMUX_PORTD_PIN4_gc;
	EVSYS_CH6MUX = EVSYS_CHMUX_PORTE_PIN0_gc;
	EVSYS_CH7MUX = EVSYS_CHMUX_PORTE_PIN4_gc;
	
	// Enable filtering on events
	EVSYS_CH2CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
	EVSYS_CH3CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
	EVSYS_CH4CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
	EVSYS_CH5CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
	EVSYS_CH6CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
	EVSYS_CH7CTRL = EVSYS_DIGFILT_1SAMPLE_gc;
	
	// Set the period of counters(edge)
	TCC0_PER = PER_COUNTER1;
	TCC1_PER = PER_COUNTER2;
	TCD0_PER = PER_COUNTER3;
	TCD1_PER = PER_COUNTER4;
	TCE0_PER = PER_COUNTER5;
	TCE1_PER = PER_COUNTER6;
	
	// Select event channel as clock source for timers
	TCC0_CTRLA = TC_CLKSEL_EVCH2_gc;
	TCC1_CTRLA = TC_CLKSEL_EVCH3_gc;
	TCD0_CTRLA = TC_CLKSEL_EVCH4_gc;
	TCD1_CTRLA = TC_CLKSEL_EVCH5_gc;
	TCE0_CTRLA = TC_CLKSEL_EVCH6_gc;
	TCE1_CTRLA = TC_CLKSEL_EVCH7_gc;
	
	// Set interrupt masks
	PORTC_INT0MASK = (1 << D1_CH4);
	PORTE_INT0MASK = (1 << D2_CH4);
	
	// Enable port interrupt
	PORTC_INTCTRL |= PORT_INT0LVL_HI_gc;
	PORTE_INTCTRL |= PORT_INT0LVL_HI_gc;
	
	
	//---------------Motor initialization-----------------------
	// Pins direction: input
	PORTF_DIR &= ~((1 << ENCD_A) | (1 << ENCD_B) | (1 << ENCD_Z));
	
	// Sense configuration
	PORTF_PIN0CTRL = PORT_ISC_LEVEL_gc;
	PORTF_PIN2CTRL = PORT_ISC_LEVEL_gc;
	PORTF_PIN4CTRL = PORT_ISC_BOTHEDGES_gc;
	
	// Select ENCD_A as multiplexer input for event channel 0
	EVSYS_CH0MUX = EVSYS_CHMUX_PORTF_PIN0_gc;
	EVSYS_CH1MUX = EVSYS_CHMUX_PORTF_PIN4_gc;
	
	// Set the  Quadrature Index Enable bit in event channel 0
	EVSYS_CH0CTRL |= (1 << EVSYS_QDIEN_bp);
	
	// Select the Index Recognition mode for event channel 0
	EVSYS_CH0CTRL |= EVSYS_QDIRM_gp;						// set, after testing
	
	//  Enable quadrature decoding and digital filtering in event channel 0
	EVSYS_CH0CTRL |= (1 << EVSYS_QDEN_bp);
	
	// Set Quadrature decoding as event action for timer
	TCF0_CTRLD |= TC_EVACT_QDEC_gc;
	
	// Select event channel 0 as event source for timer
	TCF0_CTRLD |= TC_EVSEL_CH0_gc;
	
	// Set the period register of timer (n * 4 - 1)
	TCF0_PER = (ENCD_LINE_COUNT * 4 - 1);
	
	// Enable timer by setting CLKSEL to a CLKSEL_DIV1
	TCF0_CTRLA = TC_CLKSEL_DIV1_gc;
	
	// Enable overflow interrupt of timer
	TCF0_INTCTRLA |= TC_OVFINTLVL_HI_gc;
	
	
	//---------------Interrupt initialization-----------------------
	// Enable high level interrupt
	PMIC_CTRL |= (1 << PMIC_HILVLEN_bp);
	
	//---------------Initialization for tests-----------------------
	// Pin direction: output
	PORTJ_DIR |= (1 << TST_PIN_CNT1) | (1 << TST_PIN_CNT2) | (1 << TST_PIN_CNT3) | (1 << TST_PIN_CNT4)
	| (1 << TST_PIN_CNT5) | (1 << TST_PIN_CNT6) | (1 << TST_PIN_CNT7) | (1 << TST_PIN_CNT8);
	PORTK_DIR |= (1 << TST_PIN_QDEC);
}