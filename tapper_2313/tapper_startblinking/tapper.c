#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*
  FUSES =
    {
        .low = 0xe4,
		.high = 0xd7
    };
*/

/***********************
 *  COMPILE SETTINGS   *
 ***********************/

/*
DEBUG
*/
//#define DEBUG



volatile uint32_t tapin1_tmr;
volatile uint32_t tapin2_tmr;
volatile uint32_t tapin3_tmr;
volatile uint32_t tapin4_tmr;
volatile uint32_t tapout1_tmr;
volatile uint32_t tapout2_tmr;
volatile uint32_t tapout3_tmr;
volatile uint32_t tapout4_tmr;



/************************
 * Mnuemonics		*
 ************************/




/********************
 * GLOBAL VARIABLES *
 ********************/
 //at timer prescaled to fck/8, it runs at 1MHz, divide by 256, so 7812 is 2 seconds. Actually 3606 is 2 seconds!

#define HOLDTIMECLEAR 3606 

#define STARTING_TIME 400000
/*******************
 * PIN DEFINITIONS *
 *******************/

#ifdef DEBUG
#define DEBUG_pin PA1
#define DEBUG_init DDRA |= (1<<DEBUG_pin)
#define DEBUGFLIP PORTA ^= (1<<DEBUG_pin)
#define DEBUGHIGH PORTA |= (1<<DEBUG_pin)
#define DEBUGLOW PORTA &= ~(1<<DEBUG_pin)
#endif

#define TAP1_pin PD0
#define TAP1_init DDRD &= ~(1<<TAP1_pin); PORTD |= (1<<TAP1_pin)
#define TAP1 (!(PIND & (1<<TAP1_pin)))

#define TAP2_pin PB7
#define TAP2_init DDRB &= ~(1<<TAP2_pin); PORTB |= (1<<TAP2_pin)
#define TAP2 (!(PINB & (1<<TAP2_pin)))

#define TAP3_pin PD5
#define TAP3_init DDRD &= ~(1<<TAP3_pin); PORTD |= (1<<TAP3_pin)
#define TAP3 (!(PIND & (1<<TAP3_pin)))

#define TAP4_pin PD6
#define TAP4_init DDRD &= ~(1<<TAP4_pin); PORTD |= (1<<TAP4_pin)
#define TAP4 (!(PIND & (1<<TAP4_pin)))


#define TAPOUT1_pin PD1
#define TAPOUT1_init DDRD |= (1 << TAPOUT1_pin)
#define TAPOUT1_ON PORTD |= (1 << TAPOUT1_pin)
#define TAPOUT1_OFF PORTD &= ~(1 << TAPOUT1_pin)

#define TAPOUT2_pin PB6
#define TAPOUT2_init DDRB |= (1 << TAPOUT2_pin)
#define TAPOUT2_ON PORTB |= (1 << TAPOUT2_pin)
#define TAPOUT2_OFF PORTB &= ~(1 << TAPOUT2_pin)

#define TAPOUT3_pin PD4
#define TAPOUT3_init DDRD |= (1 << TAPOUT3_pin)
#define TAPOUT3_ON PORTD |= (1 << TAPOUT3_pin)
#define TAPOUT3_OFF PORTD &= ~(1 << TAPOUT3_pin)

#define TAPOUT4_pin PB0
#define TAPOUT4_init DDRB |= (1 << TAPOUT4_pin)
#define TAPOUT4_ON PORTB |= (1 << TAPOUT4_pin)
#define TAPOUT4_OFF PORTB &= ~(1 << TAPOUT4_pin)

#define PING1_pin PD2
#define PING1_init DDRD &= ~(1<<PING1_pin)
#define PING1 (PIND & (1<<PING1_pin))

#define PING2_pin PB5
#define PING2_init DDRB &= ~(1<<PING2_pin)
#define PING2 (PINB & (1<<PING2_pin))

#define PING3_pin PD3
#define PING3_init DDRD &= ~(1<<PING3_pin)
#define PING3 (PIND & (1<<PING3_pin))

#define PING4_pin PB1
#define PING4_init DDRB &= ~(1<<PING4_pin)
#define PING4 (PINB & (1<<PING4_pin))

#define JUMP1_pin PA1
#define JUMP1_init DDRA &= ~(1<<JUMP1_pin); PORTA |= (1<<JUMP1_pin)
#define JUMP1 (!(PINA & (1<<JUMP1_pin)))

#define JUMP2_pin PA0
#define JUMP2_init DDRA &= ~(1<<JUMP2_pin); PORTA |= (1<<JUMP2_pin)
#define JUMP2 (!(PINA & (1<<JUMP2_pin)))

#define JUMP3_pin PB4
#define JUMP3_init DDRB &= ~(1<<JUMP3_pin); PORTB |= (1<<JUMP3_pin)
#define JUMP3 (!(PINB & (1<<JUMP3_pin)))

#define JUMP4_pin PB3
#define JUMP4_init DDRB &= ~(1<<JUMP4_pin); PORTB |= (1<<JUMP4_pin)
#define JUMP4 (!(PINB & (1<<JUMP4_pin)))


SIGNAL (TIMER0_OVF_vect){
	tapin1_tmr++;
	tapin2_tmr++;
	tapin3_tmr++;
	tapin4_tmr++;

	tapout1_tmr++;
	tapout2_tmr++;
	tapout3_tmr++;
	tapout4_tmr++;
}









/***************************************************
 *             MAIN() FUNCTION                     *
 *                                                 *
 ***************************************************/


int main(void){

	uint32_t tapout1_clk_time=STARTING_TIME;
	uint8_t tapin1_up=0;
	uint8_t ping1_high=0;
	char running1=2,running2=2,running3=2,running4=2;

	uint32_t tapout2_clk_time=STARTING_TIME;
	uint8_t tapin2_up=0;
	uint8_t ping2_high=0;

	uint32_t tapout3_clk_time=STARTING_TIME;
	uint8_t tapin3_up=0;
	uint8_t ping3_high=0;

	uint32_t tapout4_clk_time=STARTING_TIME;
	uint8_t tapin4_up=0;
	uint8_t ping4_high=0;

	uint32_t now=0;
	uint32_t t=0;


	/** Initialize **/

//	tapin1_tmr=0;

	//Normal mode, TOP at 0xFF, OC0A and OC0B disconnected, Prescale @ FCK/8
	TCCR0A=(0<<COM0A0) | (0<<COM0A1) | (0<<COM0B0) | (0<<COM0B1) | (0<<WGM01) | (0<<WGM00) ;
	TCCR0B= (0<<WGM02) | (0<<CS00) | (1<<CS01) | (0<<CS02);

	TCNT0=0;

	TIMSK |= (1<<TOIE0); 


//	tapout1_tmr=0;
/*
	tapout2_tmr=0;
	tapin2_tmr=0;
	tapout3_tmr=0;
	tapin3_tmr=0;
	tapout4_tmr=0;
	tapin4_tmr=0;
*/


	TAP1_init;
	TAP2_init;
	TAP3_init;
	TAP4_init;
	PING1_init;
	PING2_init;
	PING3_init;
	PING4_init;
	TAPOUT1_init;
	TAPOUT2_init;
	TAPOUT3_init;
	TAPOUT4_init;
#ifdef DEBUG_init
	DEBUG_init;
#endif
	JUMP1_init;
	JUMP2_init;
	JUMP3_init;
	JUMP4_init;

	TAPOUT1_OFF;
	TAPOUT2_OFF;
	TAPOUT3_OFF;
	TAPOUT4_OFF;


	_delay_ms(100);

	/** Main loop **/
	while(1){

		if (TAP1 || (PING1 && JUMP1)){

			if (!(tapin1_up)){
				if (running1==1) running1=2;
				if (running1==0) running1=1;

				TAPOUT1_ON;
				tapin1_up=1;
				cli();
					tapout1_clk_time=(tapin1_tmr << 8) | TCNT0;
					tapin1_tmr=0;tapout1_tmr=0;
				sei();

			} else {
				if (TAP1 && (tapin1_tmr > HOLDTIMECLEAR)){ //button has been down for more than 2 seconds
					tapout1_clk_time=0;
					if (running1) TAPOUT1_OFF;
					running1=0;
				}
			}

		} else {
			if (tapin1_up){
				tapin1_up=0;
				TAPOUT1_OFF;
			}
		}

		if (running1==2){
			cli();now = (tapout1_tmr << 8) | TCNT0;sei();

			if (now>=(tapout1_clk_time>>1)){
				TAPOUT1_OFF;
			}
			if (now>tapout1_clk_time){
				t=(now-tapout1_clk_time)>>8;
				cli();
				tapout1_tmr=t;
				sei();

				TAPOUT1_ON;
			}
 		}


		if (PING1 && !JUMP1){
			if (!ping1_high){
				TAPOUT1_ON;
				running1=0;
				ping1_high=1;
			}
		} else {
			if (ping1_high){
				TAPOUT1_OFF;
				ping1_high=0;
			}
		}




		if (TAP2 || (PING2 && JUMP2)){

			if (!(tapin2_up)){
				if (running2==1) running2=2;
				if (running2==0) running2=1;

				TAPOUT2_ON;
				tapin2_up=1;
				cli();
					tapout2_clk_time=(tapin2_tmr << 8) | TCNT0;
					tapin2_tmr=0;tapout2_tmr=0;
				sei();

			} else {

				if (TAP2 && (tapin2_tmr > HOLDTIMECLEAR)){ //button has been down for more than 2 seconds
					tapout2_clk_time=0;
					if (running2) TAPOUT2_OFF;
					running2=0;
				}
			}

		} else {
			if (tapin2_up){
				tapin2_up=0;
				TAPOUT2_OFF;
			}
		}

		if (running2==2){
			cli();now = (tapout2_tmr << 8) | TCNT0;sei();


			if (now>=(tapout2_clk_time>>1)){
				TAPOUT2_OFF;
			}
			if (now>tapout2_clk_time){
				t=(now-tapout2_clk_time)>>8;
				cli();
				tapout2_tmr=t;
				sei();

				TAPOUT2_ON;
			}
 		}


		if (PING2 && !JUMP2){
			if (!ping2_high){
				TAPOUT2_ON;
				running2=0;
				ping2_high=1;
			}
		} else {
			if (ping2_high){
				TAPOUT2_OFF;
				ping2_high=0;
			}
		}



		if (TAP3 || (PING3 && JUMP3)){

			if (!(tapin3_up)){
				if (running3==1) running3=2;
				if (running3==0) running3=1;

				TAPOUT3_ON;
				tapin3_up=1;
				cli();
					tapout3_clk_time=(tapin3_tmr << 8) | TCNT0;
					tapin3_tmr=0;tapout3_tmr=0;
				sei();

			} else {

				if (TAP3 && (tapin3_tmr > HOLDTIMECLEAR)){ //button has been down for more than 2 seconds
					tapout3_clk_time=0;
					if (running3) TAPOUT3_OFF;
					running3=0;
				}
			}

		} else {
			if (tapin3_up){
				tapin3_up=0;
				TAPOUT3_OFF;
			}
		}

		if (running3==2){
			cli();now = (tapout3_tmr << 8) | TCNT0;sei();


			if (now>=(tapout3_clk_time>>1)){
				TAPOUT3_OFF;
			}
			if (now>tapout3_clk_time){
				t=(now-tapout3_clk_time)>>8;
				cli();
				tapout3_tmr=t;
				sei();

				TAPOUT3_ON;
			}
 		}


		if (PING3 && !JUMP3){
			if (!ping3_high){
				TAPOUT3_ON;
				running3=0;
				ping3_high=1;
			}
		} else {
			if (ping3_high){
				TAPOUT3_OFF;
				ping3_high=0;
			}
		}




		if (TAP4 || (PING4 && JUMP4)){

			if (!(tapin4_up)){
				if (running4==1) running4=2;
				if (running4==0) running4=1;

				TAPOUT4_ON;
				tapin4_up=1;
				cli();
					tapout4_clk_time=(tapin4_tmr << 8) | TCNT0;
					tapin4_tmr=0;tapout4_tmr=0;
				sei();

			} else {

				if (TAP4 && (tapin4_tmr > HOLDTIMECLEAR)){ //button has been down for more than 2 seconds
					tapout4_clk_time=0;
					if (running4) TAPOUT4_OFF;
					running4=0;
				}
			}

		} else {
			if (tapin4_up){
				tapin4_up=0;
				TAPOUT4_OFF;
			}
		}

		if (running4==2){
			cli();now = (tapout4_tmr << 8) | TCNT0;sei();


			if (now>=(tapout4_clk_time>>1)){
				TAPOUT4_OFF;
			}
			if (now>tapout4_clk_time){
				t=(now-tapout4_clk_time)>>8;
				cli();
				tapout4_tmr=t;
				sei();

				TAPOUT4_ON;
			}
 		}


		if (PING4 && !JUMP4){
			if (!ping4_high){
				TAPOUT4_ON;
				running4=0;
				ping4_high=1;
			}
		} else {
			if (ping4_high){
				TAPOUT4_OFF;
				ping4_high=0;
			}
		}


	} //main loop

} //void main()





