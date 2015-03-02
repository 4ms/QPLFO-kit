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

#define WAIT_WHEN_DONE_EARLY_BECAUSE_OF_SKEW_MOD


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

/***************************************************
 *             MAIN() FUNCTION                     *
 *                                                 *
 ***************************************************/


int main(void){

	uint8_t env_state=RISE;

	uint8_t reset_now_flag=0;

	long last_clk=0;
	uint32_t clk_time=0;
	uint32_t old_clk_time=0;
	uint32_t rise_time=0;
	uint32_t fall_time=0;

	uint32_t reset_offset_time=0;
	uint8_t ready_to_reset=0;

	uint8_t reset_up=0;
	uint8_t got_ping=0;

	uint8_t skew_adc=127; //0..255, with 0 being fall only, 127 being rise/fall equal, 255 being rise only


	uint8_t do_plucky_curve=0;

	unsigned char adch=0;
	char poll_user_input=0;

	char t=0;

	int16_t t_dacout=0;
	uint32_t sample_ctr=0;
	uint32_t rise_inc=0;
	uint32_t fall_inc=0;
	uint32_t accum=0;

	char is_modulating_skew=0;

	uint32_t t32=0;

	#ifdef CALIBRATE
		uint16_t calibrate_ctr=0;
	#endif

	/** Initialize **/

	accum=0;


	inittimer();
	init_pins();
	init_adc();
	init_extinterrupt();
	init_PWM();



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

				cli();
					reset_tmr=0;
				sei();

				//clk_time changed, so see if reset_offset_time is still valid
				//reset it to 0 if it's greater than the ping period or 2ms under the period 
				if ((reset_offset_time+2000)>=clk_time) {
					reset_offset_time=0;
				}
				ping_irq_timestamp=0;

				ready_to_reset=1;

				old_clk_time=clk_time;

				if (skew_adc<=PLUCKY_CURVE_ADC) {
					rise_time=64;
				}
				else if (skew_adc>254) {
					rise_time=clk_time-256;
				}
				else if ((skew_adc>122) && (skew_adc<132)){
					rise_time=clk_time>>1;
				} else if (skew_adc<=10) {
					rise_time=64;
				} else {
					rise_time=(skew_adc-10) * (clk_time >> 8);
				}

				fall_time=clk_time-rise_time;
		
				rise_inc=udiv32(rise_time>>5);
				fall_inc=udiv32(fall_time>>5);

			
				got_ping=1;

			}
		
		} 



		if (RESET){
			if (!reset_up){
				#ifdef CALIBRATE
				calibrate_ctr=0;
				#endif
				
			//	if (env_state!=WAIT){
					cli();
						reset_offset_time=(reset_tmr<<8);
					sei();

					//see if the new reset_offset_time is still valid
					//reset it to 0 if it's greater than the ping period or 2ms under the period 
					if ((reset_offset_time+2000)>=clk_time) {
						reset_offset_time=0;
					}
			//	}

				reset_now_flag=1;
				reset_up=1;
				ready_to_reset=0;
			}
			else {
				if (env_state!=WAIT)
					ready_to_reset=0; //disable resetting if RESET is held high "analog mode"

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

			if (skew_adc>adch) t=skew_adc-adch;
			else t=adch-skew_adc;

			if (t>ADC_DRIFT){
				is_modulating_skew=250;
				skew_adc=adch;

				if (skew_adc<=PLUCKY_CURVE_ADC) {	
					rise_time=64;
				}
				else if (skew_adc>254) { 
					rise_time=clk_time-256;
				}
				else if ((skew_adc>122) && (skew_adc<132)){ //2.40V - 2.60V
					rise_time=clk_time>>1;
				} else if (skew_adc<=10) {
					rise_time=64;
				} else {
					rise_time=(skew_adc-10) * (clk_time >> 8);
				}
				fall_time=clk_time-rise_time;

				rise_inc=udiv32(rise_time>>5);
				fall_inc=udiv32(fall_time>>5);

			} else {
				if (is_modulating_skew>0)
					is_modulating_skew--;
			}
				
		}

//		if (is_modulating_skew>0) DEBUGHIGH;
//		else DEBUGLOW;





		/*******************
		*
		*
		*	Reset Lock Point
		*
		*	
		********************/


		cli();
			t32=(reset_tmr<<8);
		sei();

		if (ready_to_reset && (t32>reset_offset_time)){

			//If we've haven't recently modulated skew, then force a reset
			if (is_modulating_skew==0)
				reset_now_flag=1;

			if (env_state==WAIT)
				reset_now_flag=1;
			ready_to_reset=0;
		}




		/**************** UPDATE THE ENVELOPE *******************
		 *														*
		 *  Update only when timer has overflowed				*
		 * -restart if needed									*
		 * -calculate new position								*
		 * -change curve step (RISE/FALL)						*
		 * 								 		*
		 *														*
		 ********************************************************/


		if (timer_overflowed || reset_now_flag){
		

			// Handle the reset_now_flag by resetting the envelope
			if (reset_now_flag){
				reset_now_flag=0;

				env_state=RISE;

				if (skew_adc<=PLUCKY_CURVE_ADC){
					if (skew_adc<=1) do_plucky_curve=1;
					else do_plucky_curve=skew_adc;
				} else {
					do_plucky_curve=0;
				}

				accum=0;
				sample_ctr=0;
			}

			t_dacout=0;
			sample_ctr++;
			switch (env_state){
				case(RISE):
					accum+=rise_inc*timer_overflowed;
					t_dacout=accum>>16;

					if (t_dacout>=0x0FFF){
							accum=0x0FFF0000;
							t_dacout=0x0FFF;
							env_state=FALL;
					}
				break;


				case(FALL):
  				

					if (accum > fall_inc)
					accum-=fall_inc*timer_overflowed;
					else accum=0;

					t_dacout=accum>>16;

					if ((t_dacout<1) || (t_dacout>0x0FFF)){
							accum=0;
							t_dacout=0;

#ifndef FREERUN	

							env_state=WAIT;

//when modulating skew with a slower LFO:
//usually we don't get to env_state=RISE because of the clk_time comparison, but sometimes it's because is_mod_skew==0

							if (RESET || (is_modulating_skew>0)){
								cli();
									t32=((reset_tmr)<<7 ); //half the time since the last ping
								sei();
								if (t32<clk_time){ //if time since last ping is less than twice the LFO period
									env_state=RISE;

									if (skew_adc<=PLUCKY_CURVE_ADC){
										if (skew_adc<=1) do_plucky_curve=1;
										else do_plucky_curve=skew_adc;
									} else {
										do_plucky_curve=0;
									}
									sample_ctr=0;

								}
							}
#else
							reset_now_flag=1;
#endif
							got_ping=0;

					}
				break;

				default:
					env_state=WAIT;
				
					accum=0;
					t_dacout=0;
				break;

			}
			#ifdef CALIBRATE
			t_dacout = calibrate_ctr;
			#else

			if (do_plucky_curve){

				if (do_plucky_curve==1){

					if (sample_ctr<34) t_dacout=4095;
					else t_dacout=1;
				} else {

					t_dacout=(((int32_t)t_dacout)*((int32_t)t_dacout))>>12;
					if (do_plucky_curve<=5)
					t_dacout=(((int32_t)t_dacout)*((int32_t)t_dacout))>>12;
					if (do_plucky_curve<=3)
					t_dacout=(((int32_t)t_dacout)*((int32_t)t_dacout))>>12;
				}


			}

			#endif
		
			PWM_MSB=t_dacout>>4;		//top 8 bits
			PWM_LSB=((t_dacout & 0x0F)<<4);	//bottom 4 bits
		//	PWM_MSB=t_dacout>>8;		//top 8 bits
		//	PWM_LSB=t_dacout & 0xFF;	//bottom 8 bits

			timer_overflowed=0;
		}



	} //main loop

} //void main()





