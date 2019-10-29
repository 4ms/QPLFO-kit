#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
/*
  FUSES =
    {
        .low = 0xe2,
	.high = 0xd7
    };
*/

/***********************
 *  COMPILE SETTINGS   *
 ***********************/


/*
CALIBRATE 
	comment this out for normal operations
	when enabled, each trigger on the PING jack will 
	increment the PWM output by 1 (LSB)
*/

//#define CALIBRATE

/*
DEBUG
*/
//#define DEBUG

//#define FREERUN

#define MIN_PW 10

extern uint32_t udiv32(uint32_t divisor);

volatile uint32_t tmrms=0;
volatile uint32_t reset_tmr=0;
volatile uint32_t ping_irq_timestamp=0;

/************************
 * Mnuemonics		*
 ************************/

#define WAIT 0
#define RISE 1
#define FALL 2


/********************
 * GLOBAL VARIABLES *
 ********************/


#define ADC_DRIFT 2
#define PLUCKY_CURVE_ADC 7
#define USER_INPUT_POLL_TIME 200
#define SKEW_TRACKING_TIME 50000

/*******************
 * PIN DEFINITIONS *
 *******************/

#ifdef DEBUG
#define DEBUG_pin PB4
#define DEBUG_init DDRB |= (1<<DEBUG_pin)
#define DEBUGFLIP PORTB ^= (1<<DEBUG_pin)
#define DEBUGHIGH PORTB |= (1<<DEBUG_pin)
#define DEBUGLOW PORTB &= ~(1<<DEBUG_pin)
#endif

#define PING_pin PB0
#define PING_init DDRB &= ~(1<<PING_pin); PORTB &= ~(1<<PING_pin)
#define PING (PINB & (1<<PING_pin))

#define RESET_pin PB2
#define RESET_init DDRB &= ~(1<<RESET_pin); PORTB &= ~(1<<RESET_pin)
#define RESET (PINB & (1<<RESET_pin)) 

#define ADC_DDR DDRB
#define ADC_PORT PORTB
#define ADC_pin PB3
#define ADC_mask (1<<ADC_pin)


#define PWM_MSB OCR1A
#define PWM_LSB OCR1B

volatile char timer_overflowed=0;


SIGNAL (TIMER0_OVF_vect){
	tmrms++;
	reset_tmr++;
//	TCNT0 = 0;
	timer_overflowed++;
}


long gettmrms(void){
	uint32_t result;
	cli();
	result = (tmrms << 8) | TCNT0;
	sei();
	return result;
}



void inittimer(void){
	//Normal mode, TOP at 0xFF, OC0A and OC0B disconnected, Prescale @ FCK/8
	TCCR0A=(0<<WGM01) | (0<<WGM00) ;
	TCCR0B= (0<<WGM02) | (0<<CS00) | (1<<CS01) | (0<<CS02);

	TCNT0=0;

	TIMSK |= (1<<TOIE0); 					// Enable timer overflow interrupt
	sei();
}


void init_PWM(void){
//Timer 1

//init PLL clock
	PLLCSR = (1<<PLLE);					//enable PLL
	while (!(PLLCSR & (1<<PLOCK))) ;	//wait for the PLL to lock
	PLLCSR |= (1<<PCKE);				//Set Timer1's clock to the PLL

	//Set OC1A on compare match
	//PCK = 64MHz PWM clock
	TCCR1 = (1<<PWM1A) | (0<<COM1A0) | (1<<COM1A1) | (0<<CS13) | (0<<CS12) | (0<<CS11) | (1<<CS10);
	DDRB |= (1<<PB1); //set OC1A to output

	//Set OC1B on compare match
#ifndef DEBUG
	GTCCR |= (1<<PWM1B) | (0<<COM1B0) | (1<<COM1B1);
	DDRB |= (1<<PB4); //set OC1B to output
#endif


	PWM_MSB=0x00;
	PWM_LSB=0x00;

	OCR1C = 0xFF;
}

inline uint8_t diff(uint8_t a, uint8_t b);
inline uint8_t diff(uint8_t a, uint8_t b){
	if (a>b) return (a-b);
	else return (b-a);
}

void init_pins(void){
	PING_init;
	RESET_init;
#ifdef DEBUG_init
	DEBUG_init;
#endif
}


void init_adc(void){
	//init the ADC:
	ADC_DDR &= ~(ADC_mask); //adc input
	ADC_PORT &= ~(ADC_mask); //disable pullup
	ADCSRA = (1<<ADEN);	//Enable ADC
	ADMUX = (1<<ADLAR) | (ADC_pin);	//Left-Adjust, MUX to the ADC_pin
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //prescale = clk/128 = 125kHz
	ADCSRA |= (1<<ADSC);//set the Start Conversion Flag in the ADC Status Register
}

SIGNAL (PCINT0_vect){
	if (PING){
		ping_irq_timestamp = (tmrms << 8) | TCNT0;
	}
}

void init_extinterrupt(void){
	PCMSK = (1<<PCINT0); //pin change on PCINTO = PB0
	GIMSK = (1<<PCIE); //enable pin change interrupt

}


uint32_t calc_pw(uint8_t pw_adc, uint32_t period){
	uint32_t t;

	if (pw_adc<4) t=(period>>6); //1.0625%
	else if (pw_adc<14) t=(period>>5); //3.125%
	else if (pw_adc<24) t=(period>>4); //6.25%
		//	else if (pw_adc<34) t=((period>>4)+(period>>6)); //7.8125%
	else if (pw_adc<34) t=((period>>4)+(period>>5)); //9.375%
		//	else if (pw_adc<44) t=((period>>4)+(period>>5)+(period>>6)); //10.9375%
	else if (pw_adc<44) t=(period>>3); //12.5%
	else if (pw_adc<54) t=((period>>3)+(period>>5)); //15.5%
	else if (pw_adc<64) t=((period>>3)+(period>>4)); //18.75%
	else if (pw_adc<74) t=((period>>3)+(period>>4)+(period>>5)); //21.875%
	else if (pw_adc<85) t=(period>>2); //25%
	else if (pw_adc<94) t=((period>>2)+(period>>4)); //31.25%
	else if (pw_adc<104) t=((period>>2)+(period>>3)); //37.5%
	else if (pw_adc<114) t=((period>>2)+(period>>3)+(period>>4)); //43.75%

	else if (pw_adc<140) t=(period>>1); //50%

	else if (pw_adc<150) t=((period>>1)+(period>>5)); //53.125%
	else if (pw_adc<160) t=((period>>1)+(period>>4)); //56.25%
	else if (pw_adc<170) t=((period>>1)+(period>>4)+(period>>5)); //59.375%
	else if (pw_adc<180) t=((period>>1)+(period>>3)); //62.5%
	else if (pw_adc<190) t=((period>>1)+(period>>3)+(period>>5)); //65.5%
	else if (pw_adc<200) t=((period>>1)+(period>>3)+(period>>4)); //68.75%
	else if (pw_adc<210) t=((period>>1)+(period>>3)+(period>>4)+(period>>5)); //71.875%
	else if (pw_adc<220) t=((period>>1)+(period>>2)); //75%
	else if (pw_adc<230) t=((period>>1)+(period>>2)+(period>>4)); //81.25%
	else if (pw_adc<240) t=((period>>1)+(period>>2)+(period>>3)); //87.5%
	else if (pw_adc<250) t=((period>>1)+(period>>2)+(period>>3)+(period>>4)); //93.75%
	else t=period-(period>>5); //96.875%

	if (period>(30000)){   //period is at least 30ms (lower than 33Hz) so we should use MIN_PW as a min/max
		if (pw_adc<4 || t<MIN_PW) t=MIN_PW;
		if (pw_adc>=250 || t>(period-MIN_PW)) t=period-MIN_PW; 

	}

	return(t);

}

/***************************************************
 *             MAIN() FUNCTION                     *
 *                                                 *
 ***************************************************/


int main(void){

	uint8_t env_state=WAIT;

	uint8_t reset_now_flag=0;

	long last_clk=0;
	uint32_t clk_time=0;
	uint32_t high_time=0;

	uint8_t reset_up=0;

	uint8_t pw_adc=127; 

	unsigned char adch=0;
	char poll_user_input=0;

	char t=0;

	int16_t t_dacout=0;

	uint32_t t32=0;

	/** Initialize **/



	inittimer();
	init_pins();
	init_adc();
	init_extinterrupt();
	init_PWM();

	clk_time=200000;
	high_time=calc_pw(127,clk_time);

	_delay_ms(5);
	


	/** Main loop **/
	while(1){

		/***************** READ PING *********************
		 *														*
		 *  On rising edge of input ping, record the time	 	*
		 *  since the last ping into clk_time 					*
		 * 														*
		 ********************************************************/


		if (PING && ping_irq_timestamp){
			#ifdef CALIBRATE
			calibrate_ctr++;
			#endif

			if (last_clk==0){
				last_clk=gettmrms();
				reset_now_flag=0;
			} else {

				clk_time=ping_irq_timestamp-last_clk;

				last_clk=ping_irq_timestamp;

				//cli();
					//reset_tmr=0;
				//sei();

				ping_irq_timestamp=0;

				high_time=calc_pw(pw_adc,clk_time);
			
			}
		
		} 



		if (RESET){
			if (!reset_up){
				
				cli();
					reset_tmr=0;
				sei();

				reset_now_flag=1;
				//reset_up=1;
				env_state=RISE;
			}
		}
		else {
			reset_up=0;
		}
		



		/********************* READ SKEW ************************
		 *														*
		 * 		Read the ADC for the Skew value.				*
		 *		If it's changed more than ADC_DRIFT,			*
		 *		re-cacluate the rise and fall times				*
		 * 														*
		 ********************************************************/

		if ((++poll_user_input>USER_INPUT_POLL_TIME) && (ADCSRA & (1<<ADIF))){
			poll_user_input=0;

			/** READ ADC **/

			ADCSRA |= (1<<ADIF);		// Clear the flag by sending a logical "1"
			adch=ADCH;

			ADMUX = (1<<ADLAR) | ADC_pin; //Setup for next conversion
			ADCSRA |= (1<<ADSC);		//Start Conversion

			if (pw_adc>adch) t=pw_adc-adch;
			else t=adch-pw_adc;

			if (t>ADC_DRIFT){
				pw_adc=adch;

				high_time=calc_pw(pw_adc,clk_time);

			}
				
		}


		/**************** UPDATE THE ENVELOPE *******************
		 ********************************************************/


		if (timer_overflowed || reset_now_flag){

			// Handle the reset_now_flag by resetting the envelope
			if (reset_now_flag){
				reset_now_flag=0;
			}

			cli();
				t32=(reset_tmr<<8);
			sei();

			if (env_state==RISE)
				t_dacout=4095;

			if ((env_state==RISE) && (t32>high_time)){
				t_dacout=1;
				env_state=FALL;
			}

			if (t32>clk_time){
				t_dacout=1;
				env_state=WAIT;
			}
			PWM_MSB=t_dacout>>4;		//top 8 bits
			PWM_LSB=((t_dacout & 0x0F)<<4);	//bottom 4 bits

			timer_overflowed=0;
		}



	} //main loop

} //void main()





