#include "msp430g2553.h"

/*
 * EC450 Final Project
 *	Leveling device
 *	Fall 2013
 *
 * Conor McEwen & Patrick W. Crawford
 *
 */



#define LED_center	0x01	// on or off, level is flat indicator.
#define LED_Xaxis	0x02	// x-axis/horizontal LED array (local axis) (Timer 1 output) P2 output!
#define LED_Yaxis	0x04	// y-axis/vertical LED array (local axis) (Timer 0 output) P1 output!
#define BUTTON_BIT 	0x08	// button bit

#define CS 0x10			//Chip select port
#define SCLK 0x20		//UCB0CLK 1 + 1
#define SDO  0x40		//UCB0SOMI 1 + 1
#define SDI 0x80		//UCB0SIMO 1 + 1

#define DATAX0 0x32	 	//data address of accelerometer
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37

#define wait_tm while ((UCB0STAT & UCBUSY)); // wait for end of transmission
#define wait_btw __delay_cycles(10);		

volatile char val, xval0, xval1, yval0, yval1, zval0, zval1;	//accelerometer intermediate variables
volatile char count;				// counting variable to wait for a time for the device being level before turning on the center LED.
const char scale = 200;				// scaling factor to make brightness of LED's vary more over a small range
const int threshold = 0x1000;		// threshold factor for how off from "flat" it can be while allowing the center LED to turn on

/* declarations of functions defined later */
void init_adc(void); // adc setup
void init_timer(void); // timer setup
void init_button(void); // button setup
void init_SPI();
void init_accel(void);	// set up the accelerometer
char read_byte(char addr);
void write_byte(char addr, char b);

/* global variables */
volatile unsigned int xaxis_calibrate;	// calibration of the x-axis (used as a centering offset)
volatile unsigned int yaxis_calibrate;	// calibration of the y-axis (used as a centering offset)
// note that "center" or 50-50 or level, is the value 32756.

// the 3 axis stored/received from the accelerometer... or should it be signed for our reference?
volatile signed int x_axis;
volatile signed int y_axis;
volatile signed int z_axis;

// main function
void main(){
	WDTCTL = WDTPW + WDTTMSEL+ WDTCNTCL + 0; // setup using the watchdog timer, using the 8MHz clock, 00 us /32768
	BCSCTL1 = CALBC1_8MHZ;    // 8Mhz calibration for clock
	DCOCTL  = CALDCO_8MHZ;

	init_timer(); 						// initialize timer
	init_button(); 						// initialize the button

	// clear/initialize the global variables
	xaxis_calibrate=0;
	yaxis_calibrate=0;
	x_axis=0;
	y_axis=0;
	z_axis=1;
	count= 100;

	P1REN = BUTTON_BIT;
	P1DIR |= LED_center;	//(BUTTON_BIT+LED_Xaxis+LED_Yaxis+LED_center);
	P1OUT &= ~LED_center;
	
	// initialize accelerometer and onboard communication protocols
	init_SPI();
	init_accel();

	IE1 |= WDTIE;			//enable the WDT interrupt

	_bis_SR_register(GIE+LPM0_bits);	// enable general interrupts and power down CPU
}




// Sound Production System
void init_timer(){              // initialization and start of timer

	// setup of timer 1 & 0 overall
	TA0CTL |= TACLR;             	// reset clock
	TA0CTL = TASSEL_2+ID_0+MC_2; 	// clock source = SMCLK (TASSEL_2)
									// clock divider=1 (ID_3 is /8)
									// continuous mode (MC_2, MC_1 is up mode)
									// timer A interrupt off (turn on, add TAIE interrupt enable)
	TA1CTL |= TACLR;             	// reset clock
	TA1CTL = TASSEL_2+ID_0+MC_2;


	// setup the individual channels on Timer1
	TA1CCTL1=OUTMOD_7; 		// second channel settings
	P2SEL|= LED_Xaxis; 		// connect timer output to pin	// connect other timer output to pin
	P2DIR|= LED_Xaxis;

	// setup the individual channels on Timer0
	TA0CCTL1=OUTMOD_7;		// second channel settings (channel 2 is not accessible)
	P1SEL|= LED_Yaxis; 		// connect timer output to pin	// connect other timer output to pin
	P1DIR|= LED_Yaxis;

	// registers set behavior for all of timer 0.
	TA0CCR0 = 0xFFF0; 		// in up mode TAR=0... TACCRO-1
	TA0CCR1 = 0x0FFF;

	// registers set behavior for all of timer 1.
	TA1CCR0 = 0xFFF0;
	TA1CCR1 = 0x0FFF;

}

// Button input System
void init_button(){
	P1OUT |= BUTTON_BIT; // pullup
	P1REN |= BUTTON_BIT; // enable resistor
	P1IFG &= ~(BUTTON_BIT);// clear interrupt flag
	P1IE  |= BUTTON_BIT; // enable interrupt
}




// button handler, for calibration
void interrupt button_handler(){
	if (P1IFG & BUTTON_BIT){
		// reset flag
		P1IFG &= ~BUTTON_BIT;
		// here we  set the calibration values
		// must also consider change from previous calibration offset, and "0" or "level" is 32768
		xaxis_calibrate = TA1CCR1-32768+xaxis_calibrate;
		yaxis_calibrate = TA0CCR1-32768+yaxis_calibrate;
	}
}
ISR_VECTOR(button_handler, ".int02")





// communication setup, SPI
void init_SPI(void){
	UCB0CTL1 = UCSWRST; // reset USCI logic
	UCB0CTL1 |= UCSSEL_2; // SMCLK is clock source
	UCB0BR0= 32; // USCI clock divisor low -- from parameters file
	UCB0BR1=0; // USCI clock divisor high
	UCB0CTL0 = UCCKPL
			  +UCMSB // MSB first
			  +UCMST  // Master
			  +UCMODE_0 // 3 pin SPI
			  +UCSYNC;// sync mode (needed for SPI)
	// configure port to connect to UCB0 (not the MISO pin though
	P1SEL |= SCLK + SDO + SDI;
	P1SEL2 |= SCLK + SDO  + SDI;
	P1DIR |= CS;
	P1OUT |= CS;
	UCB0CTL1 &= ~(UCSWRST); // exit reset state for USCI

}


void init_accel(void){
	val = read_byte(0x00); //read dev id
	write_byte(0x2D, 0x08); // set to measurement mode
	write_byte(0x31, 0x00); // set data format - +/- 2g, right justified with sign extended

}

void write_byte(char addr, char b) {
	P1OUT &= ~CS;  			//CS low to turn on chip
	while((IFG2&UCB0TXIFG)==0); //wait for TX flag to be set
	UCB0TXBUF=addr; //put address into TX buffer to write it
	wait_tm;
	UCB0TXBUF=b; //write data
	wait_tm;
	P1OUT |= CS;
}

char read_byte(char addr) {
	P1OUT &= ~CS;
	while((IFG2&UCB0TXIFG)==0); //wait for TX flag to be set
	UCB0TXBUF=addr+0x80; //put address into TX buffer to write it
	wait_tm;
	UCB0TXBUF=0xFF; //dummy data to get read value
	wait_tm;
	P1OUT |= CS;
	return UCB0RXBUF; //return read value
}


interrupt void WDT_interval_handler(){

	// send/get the data from the accelerometer
	xval0 = read_byte(DATAX0);
	xval1 = read_byte(DATAX1);
	x_axis = xval0 + (xval1<<8); //concatenate two bytes into an int

	yval0 = read_byte(DATAY0);
	yval1 = read_byte(DATAY1);
	y_axis = yval0 + (yval1<<8);

	zval0 = read_byte(DATAZ0);
	zval1 = read_byte(DATAZ1);
	z_axis = zval0 + (zval1<<8);


	// state machine for the accelerometer
	if (z_axis > 170 || z_axis < -170){
		// it is flat on the table, set the x and y values to the timers
		TA1CCR1 = (x_axis*scale + 32768);
		TA0CCR1 = (-y_axis*scale + 32768);
	}
	else if (x_axis > 170 || x_axis < -170){
		// turned 90 degrees
		TA1CCR1 = (z_axis*scale + 32768);
		TA0CCR1 = (-y_axis*scale + 32768);
	}
	else if (y_axis > 170 || y_axis < -170){
		// 90 degree turn on the other axis
		TA1CCR1 = (x_axis*scale + 32768);
		TA0CCR1 = (-z_axis*scale + 32768);
	}
	else{
		// other in-between cases, have a neutral output
		TA1CCR1 = 0x8000;
		TA0CCR1 = 0x8000;
	}


	// calibration, affect the currently selected axis in the timers
	TA1CCR1 -= xaxis_calibrate;
	TA0CCR1 -= yaxis_calibrate;


	// logic for turning on the center LED
	if ( (TA1CCR1 < 32768+threshold) && (TA1CCR1 > 32768-threshold) && (TA0CCR1 < 32768+threshold) && (TA0CCR1 > 32768-threshold)){
		if (count--==0){
			// LED on
			P1OUT|=LED_center;
			count = 100;
		}

	}
	else{
		//LED off
		count = 100;
		P1OUT&= ~LED_center;
	}

}
ISR_VECTOR(WDT_interval_handler, ".int10")	//declare watchdog interrupt



