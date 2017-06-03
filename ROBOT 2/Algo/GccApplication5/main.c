
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"
#include "shortest_path.c"



void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
//int start=1, dest=5;// start is start node. dest in destination node
int count=1,data=0,played=-1,prsnt2=1,nxt2=0;
	int nodes[33],h=0;
//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}
void interrupt_switch_config(void)
{
	DDRE=DDRE & 0x7F;
	PORTE=PORTE | 0x80;
	
}
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}


//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00;
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}
//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	interrupt_switch_config();
	 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	motion_pin_config();
	buzzer_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}
/*
Function Name:linear_distance_mm()
Input:distance to move in mm
Output:null
Logic:uses shaft count to move the bot by a specific distance
Example call:linear_distance_mm(100);
*/
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void backward_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}


void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count

	forward_mm(60);
	stop();
	_delay_ms(100);
	left(); //Turn left
	angle_rotate(Degrees);
}

/*
Function Name:right_degrees()
Input:angle
Output:NULL
Logic:uses pulse count in the robot to turn the bot to a specific angle on the axis of the axle
Example call:right_degrees(90);
*/

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	forward_mm(60);
	stop();
	_delay_ms(100);
	right(); //Turn right
	angle_rotate(Degrees);
}
void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	// UBRR0L = 0x47; //11059200 Hz
	UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}


// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}


//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

/*
Function Name:forward()
Input:null
Output:null
Logic:moves the bot forward
Example call:forward();
*/
void forward (void)
{
  motion_set (0x06);
}
void back (void) //both wheels backward
{
	motion_set(0x09);
}

void right(void)
{
	motion_set (0x0A);
}
void left(void)
{
	motion_set (0x05);
}
void stop (void)
{
  motion_set (0x00);
}
void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
		uart0_init(); //Initailize UART1 for serial communiaction
	 timer1_init();

	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}
void strike_left()
{
	servo_1(105);
	_delay_ms(500);
	servo_1(84);
	_delay_ms(500);
}

void strike_right()
{
	servo_1(67);
	_delay_ms(500);
	servo_1(84);
	_delay_ms(500);
}
void horizontal()
{
	servo_1(84);
}
SIGNAL(USART0_RX_vect) 		// ISR for receive complete interrupt
{
	
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable
	if(data==255)
	{
		played++;			//for synchronization
	}
	else if((data>99) && (data<=150))
	{
		prsnt2=(data-100);		//prsnt location of other robot in case of obstacle
	}	
	else if((data>200) && (data<=250))
	{
		nxt2=(data-200);			//nxt location of other robot in case of obstacle
	}
	else if((data >0) && (data<=50))
	{
		nodes[h]=data;		//full nodes array
		h++;
	}
	
	
}

void decision(int prev,int prsnt,int nxt)
{
	if(prev==2 && prsnt==3 && nxt==35)
	{
		right_degrees(63);
	}
	if(prev==35 && prsnt==3 && nxt==2)
	{
		left_degrees(63);
	}
	if(prev==2 && prsnt==3 && nxt==36)
	{
		right_degrees(123);
	}
	if(prev==36 && prsnt==3 && nxt==2)
	{
		left_degrees(125);
	}
	if(prev==4 && prsnt==3 && nxt==35)
	{
		left_degrees(123);
	}
	if(prev==35 && prsnt==3 && nxt==4)
	{
		right_degrees(125);
	}
	if(prev==4 && prsnt==3 && nxt==36)
	{
		left_degrees(63);
	}
	if(prev==36 && prsnt==3 && nxt==4)
	{
		right_degrees(63);
	}
	if(prev==36 && prsnt==3 && nxt==35)
	{
		right_degrees(123);
	}
	if(prev==35 && prsnt==3 && nxt==36)
	{
		left_degrees(123);
	}
	if(prev==3 && prsnt==35 && nxt==36)
	{
		right_degrees(123);
	}
	if(prev==36 && prsnt==35 && nxt==3)
	{
		left_degrees(123);
	}
	if(prev==40 && prsnt==35 && nxt==36)
	{
		left_degrees(63);
	}
	if(prev==36 && prsnt==35 && nxt==40)
	{
		right_degrees(63);
	}
	if(prev==35 && prsnt==40 && nxt==39)
	{
		right_degrees(63);
	}
	if(prev==39 && prsnt==40 && nxt==35)
	{
		left_degrees(63);
	}
	if(prev==35 && prsnt==40 && nxt==34)
	{
		left_degrees(63);
	}
	if(prev==34 && prsnt==40 && nxt==35)
	{
		right_degrees(63);
	}
	if(prev==34 && prsnt==40 && nxt==39)
	{
		left_degrees(63);
	}
	if(prev==39 && prsnt==40 && nxt==34)
	{
		right_degrees(63);
	}
	if(prev==39 && prsnt==40 && nxt==41)
	{
		right_degrees(123);
	}
	if(prev==41 && prsnt==40 && nxt==39)
	{
		left_degrees(123);
	}
	if(prev==34 && prsnt==40 && nxt==41)
	{
		left_degrees(123);
	}
	if(prev==41 && prsnt==40 && nxt==34)
	{
		right_degrees(123);
	}
	if(prev==35 && prsnt==36 && nxt==37)
	{
		left_degrees(63);
	}
	if(prev==37 && prsnt==36 && nxt==35)
	{
		right_degrees(63);
	}
	if(prev==3 && prsnt==36 && nxt==35)
	{
		left_degrees(123);
	}
	if(prev==35 && prsnt==36 && nxt==3)
	{
		left_degrees(123);
	}
	if(prev==36 && prsnt==37 && nxt==38)
	{
		left_degrees(63);
	}
	if(prev==38 && prsnt==37 && nxt==36)
	{
		right_degrees(63);
	}
	if(prev==25 && prsnt==37 && nxt==48)
	{
		right_degrees(123);
	}
	if(prev==48 && prsnt==37 && nxt==25)
	{
		left_degrees(123);
	}
	if(prev==36 && prsnt==37 && nxt==25)
	{
		right_degrees(63);
	}
	if(prev==25 && prsnt==37 && nxt==36)
	{
		left_degrees(63);
	}
	if(prev==38 && prsnt==37 && nxt==48)
	{
		left_degrees(123);
	}
	if(prev==48 && prsnt==37 && nxt==38)
	{
		left_degrees(123);
	}
	if(prev==25 && prsnt==37 && nxt==38)
	{
		right_degrees(63);
	}
	if(prev==38 && prsnt==37 && nxt==25)
	{
		left_degrees(63);
	}
	if(prev==24 && prsnt==23 && nxt==25)
	{
		left_degrees(123);
	}
	if(prev==25 && prsnt==23 && nxt==24)
	{
		right_degrees(125);
	}
	if(prev==24 && prsnt==23 && nxt==26)
	{
		left_degrees(63);
	}
	if(prev==26 && prsnt==23 && nxt==24)
	{
		right_degrees(63);
	}
	if(prev==22 && prsnt==23 && nxt==26)
	{
		right_degrees(123);
	}
	if(prev==26 && prsnt==23 && nxt==22)
	{
		left_degrees(125);
	}
	if(prev==22 && prsnt==23 && nxt==25)
	{
		right_degrees(63);
	}
	if(prev==25 && prsnt==23 && nxt==22)
	{
		left_degrees(63);
	}
	if(prev==25 && prsnt==23 && nxt==26)
	{
		left_degrees(123);
	}
	if(prev==26 && prsnt==23 && nxt==25)
	{
		right_degrees(123);
	}
	if(prev==26 && prsnt==48 && nxt==47)
	{
		left_degrees(63);
	}
	if(prev==47 && prsnt==48 && nxt==26)
	{
		right_degrees(63);
	}
	if(prev==26 && prsnt==48 && nxt==37)
	{
		left_degrees(123);
	}
	if(prev==37 && prsnt==48 && nxt==26)
	{
		right_degrees(123);
	}
	if(prev==26 && prsnt==48 && nxt==27)
	{
		right_degrees(63);
	}
	if(prev==27 && prsnt==48 && nxt==26)
	{
		left_degrees(63);
	}
	if(prev==27 && prsnt==48 && nxt==47)
	{
		right_degrees(63);
	}
	if(prev==47 && prsnt==48 && nxt==27)
	{
		left_degrees(63);
	}
	if(prev==37 && prsnt==48 && nxt==47)
	{
		left_degrees(123);
	}
	if(prev==47 && prsnt==48 && nxt==37)
	{
		right_degrees(123);
	}
	if(prev==28 && prsnt==27 && nxt==48)
	{
		right_degrees(63);
	}
	if(prev==48 && prsnt==27 && nxt==28)
	{
		left_degrees(63);
	}
	if(prev==19 && prsnt==27 && nxt==28)
	{
		right_degrees(123);
	}
	if(prev==28 && prsnt==27 && nxt==19)
	{
		left_degrees(123);
	}
	if(prev==19 && prsnt==28 && nxt==27)
	{
		left_degrees(123);
	}
	if(prev==27 && prsnt==28 && nxt==19)
	{
		right_degrees(123);
	}
	if(prev==45 && prsnt==28 && nxt==27)
	{
		right_degrees(63);
	}
	if(prev==27 && prsnt==28 && nxt==45)
	{
		left_degrees(63);
	}
	if(prev==20 && prsnt==19 && nxt==27)
	{
		left_degrees(123);
	}
	if(prev==27 && prsnt==19 && nxt==20)
	{
		right_degrees(125);
	}
	if(prev==20 && prsnt==19 && nxt==28)
	{
		left_degrees(63);
	}
	if(prev==28 && prsnt==19 && nxt==20)
	{
		right_degrees(63);
	}
	if(prev==18 && prsnt==19 && nxt==27)
	{
		right_degrees(63);
	}
	if(prev==27 && prsnt==19 && nxt==18)
	{
		left_degrees(63);
	}
	if(prev==18 && prsnt==19 && nxt==28)
	{
		right_degrees(123);
	}
	if(prev==28 && prsnt==19 && nxt==18)
	{
		left_degrees(125);
	}
	if(prev==27 && prsnt==19 && nxt==28)
	{
		left_degrees(123);
	}
	if(prev==28 && prsnt==19 && nxt==27)
	{
		right_degrees(123);
	}
	if(prev==28 && prsnt==45 && nxt==46)
	{
		left_degrees(63);
	}
	if(prev==46 && prsnt==45 && nxt==28)
	{
		right_degrees(63);
	}
	if(prev==28 && prsnt==45 && nxt==29)
	{
		right_degrees(63);
	}
	if(prev==29 && prsnt==45 && nxt==28)
	{
		left_degrees(63);
	}
	if(prev==46 && prsnt==45 && nxt==29)
	{
		left_degrees(63);
	}
	if(prev==29 && prsnt==45 && nxt==46)
	{
		right_degrees(63);
	}
	if(prev==44 && prsnt==45 && nxt==46)
	{
		right_degrees(123);
	}
	if(prev==46 && prsnt==45 && nxt==44)
	{
		left_degrees(123);
	}
	if(prev==44 && prsnt==45 && nxt==29)
	{
		left_degrees(123);
	}
	if(prev==29 && prsnt==45 && nxt==44)
	{
		right_degrees(123);
	}
	if(prev==29 && prsnt==15 && nxt==30)
	{
		left_degrees(123);
	}
	if(prev==30 && prsnt==15 && nxt==29)
	{
		right_degrees(123);
	}
	if(prev==29 && prsnt==15 && nxt==16)
	{
		right_degrees(125);
	}
	if(prev==16 && prsnt==15 && nxt==29)
	{
		left_degrees(123);
	}
	if(prev==29 && prsnt==15 && nxt==14)
	{
		left_degrees(63);
	}
	if(prev==14 && prsnt==15 && nxt==29)
	{
		right_degrees(63);
	}
	if(prev==16 && prsnt==15 && nxt==30)
	{
		left_degrees(63);
	}
	if(prev==30 && prsnt==15 && nxt==16)
	{
		right_degrees(63);
	}
	if(prev==14 && prsnt==15 && nxt==30)
	{
		left_degrees(123);
	}
	if(prev==30 && prsnt==15 && nxt==14)
	{
		right_degrees(125);
	}
	if(prev==30 && prsnt==44 && nxt==45)
	{
		left_degrees(123);
	}
	if(prev==45 && prsnt==44 && nxt==30)
	{
		right_degrees(123);
	}
	if(prev==30 && prsnt==44 && nxt==31)
	{
		right_degrees(63);
	}
	if(prev==31 && prsnt==44 && nxt==30)
	{
		left_degrees(63);
	}
	if(prev==30 && prsnt==44 && nxt==43)
	{
		left_degrees(63);
	}
	if(prev==43 && prsnt==44 && nxt==30)
	{
		right_degrees(63);
	}
	if(prev==43 && prsnt==44 && nxt==31)
	{
		left_degrees(63);
	}
	if(prev==31 && prsnt==44 && nxt==43)
	{
		right_degrees(63);
	}
	if(prev==43 && prsnt==44 && nxt==45)
	{
		right_degrees(123);
	}
	if(prev==45 && prsnt==44 && nxt==43)
	{
		left_degrees(123);
	}
	if(prev==44 && prsnt==31 && nxt==32)
	{
		left_degrees(63);
	}
	if(prev==32 && prsnt==31 && nxt==44)
	{
		right_degrees(63);
	}
	if(prev==32 && prsnt==31 && nxt==11)
	{
		left_degrees(123);
	}
	if(prev==11 && prsnt==31 && nxt==32)
	{
		right_degrees(123);
	}
	if(prev==11 && prsnt==32 && nxt==31)
	{
		left_degrees(123);
	}
	if(prev==31 && prsnt==32 && nxt==11)
	{
		right_degrees(123);
	}
	if(prev==31 && prsnt==32 && nxt==41)
	{
		left_degrees(63);
	}
	if(prev==41 && prsnt==32 && nxt==31)
	{
		right_degrees(63);
	}
	if(prev==32 && prsnt==11 && nxt==31)
	{
		right_degrees(123);
	}
	if(prev==31 && prsnt==11 && nxt==12)
	{
		right_degrees(125);
	}
	if(prev==12 && prsnt==11 && nxt==31)
	{
		left_degrees(123);
	}
	if(prev==10 && prsnt==11 && nxt==32)
	{
		right_degrees(123);
	}
	if(prev==32 && prsnt==11 && nxt==10)
	{
		left_degrees(125);
	}
	if(prev==12 && prsnt==11 && nxt==32)
	{
		left_degrees(63);
	}
	if(prev==32 && prsnt==11 && nxt==12)
	{
		right_degrees(63);
	}
	if(prev==10 && prsnt==11 && nxt==31)
	{
		right_degrees(63);
	}
	if(prev==31 && prsnt==11 && nxt==10)
	{
		left_degrees(63);
	}
	if(prev==42 && prsnt==41 && nxt==40)
	{
		left_degrees(123);
	}
	if(prev==40 && prsnt==41 && nxt==42)
	{
		right_degrees(123);
	}
	if(prev==42 && prsnt==41 && nxt==32)
	{
		left_degrees(123);
	}
	if(prev==32 && prsnt==41 && nxt==42)
	{
		left_degrees(63);
	}
	if(prev==42 && prsnt==41 && nxt==33)
	{
		left_degrees(63);
	}
	if(prev==33 && prsnt==41 && nxt==42)
	{
		right_degrees(63);
	}
	if(prev==32 && prsnt==41 && nxt==33)
	{
		right_degrees(63);
	}
	if(prev==33 && prsnt==41 && nxt==32)
	{
		left_degrees(63);
	}
	if(prev==33 && prsnt==41 && nxt==40)
	{
		right_degrees(123);
	}
	if(prev==40 && prsnt==41 && nxt==33)
	{
		left_degrees(123);
	}
	if(prev==6 && prsnt==7 && nxt==34)
	{
		right_degrees(123);
	}
	if(prev==34 && prsnt==7 && nxt==6)
	{
		left_degrees(125);
	}
	if(prev==6 && prsnt==7 && nxt==33)
	{
		right_degrees(63);
	}
	if(prev==33 && prsnt==7 && nxt==6)
	{
		left_degrees(63);
	}
	if(prev==34 && prsnt==7 && nxt==33)
	{
		right_degrees(123);
	}
	if(prev==33 && prsnt==7 && nxt==34)
	{
		left_degrees(123);
	}
	if(prev==8 && prsnt==7 && nxt==33)
	{
		left_degrees(123);
	}
	if(prev==33 && prsnt==7 && nxt==8)
	{
		right_degrees(125);
	}
	if(prev==8 && prsnt==7 && nxt==34)
	{
		left_degrees(63);
	}
	if(prev==34 && prsnt==7 && nxt==8)
	{
		right_degrees(63);
	}//extra conditions for hexagon
	if (prev==37 && prsnt==38 && nxt==39)
	{
		left_degrees(63);
	}
	if (prev==38 && prsnt==39 && nxt==40)
	{
		left_degrees(63);
	}
	if (prev==40 && prsnt==39 && nxt==38)
	{
		right_degrees(63);
	}
	if (prev==39 && prsnt==38 && nxt==37)
	{
		right_degrees(63);
	}
	if (prev==48 && prsnt==47 && nxt==46)
	{
		right_degrees(63);
	}
	if (prev==47 && prsnt==46 && nxt==45)
	{
		right_degrees(63);
	}
	if (prev==45 && prsnt==46 && nxt==47)
	{
		left_degrees(63);
	}
	if (prev==46 && prsnt==47 && nxt==48)
	{
		left_degrees(63);
	}
	if (prev==44 && prsnt==43 && nxt==42)
	{
		right_degrees(63);
	}
	if (prev==43 && prsnt==42 && nxt==41)
	{
		right_degrees(63);
	}
	if (prev==41 && prsnt==42 && nxt==43)
	{
		left_degrees(63);
	}
	if (prev==42 && prsnt==43 && nxt==44)
	{
		left_degrees(63);
	}
	
	
	if(prev==nxt)
	{
		right_degrees(186);
	}
	
}


//Main Function
int main()
{	
				
	init_devices();
	lcd_set_4bit();
	lcd_init();
	horizontal();
	float lprev=100;//stores the previous value of left white sensor
	float rprev=100;//stores the previous value of right white sensor
	float cprev=100;//stores the previous value of center white sensor
	float l,c,r;
	int p=0,i,j,k,threshold=40,start,dest,steps=1,boot=0,sharp;
	int prev=0,prsnt=13,nxt=0,len1,len2,z=0,temp_dest=0,temp_len1=0,temp_len2=0;// denotes the position of nodesi.e. previous, present , next
	int filtered_nodes[33];
	
		count=0;
		
		while(boot==0)
		{
			if ((PINE & 0x80)==0x80)
			{
				boot=0;
				
			}
			else
			{
				boot=1;
			}
		}
		if (boot==1)
		{

stop();
_delay_ms(300);
left_degrees(81);
backward_mm(90);
filtered_nodes[-1]=0;
for(int cc=0;cc<=33;cc++)		//filtered nodes
{
	 temp_dest=nodes[cc];
	dijkstra(prsnt,temp_dest);
	steps=1;
	for(i=0;i<30;i++)
	{
		if(path[i]!=prsnt)
		steps++;
		else
		break;
	}
	
	j = steps - 1;   // j will Point to last Element
	i = 0;       // i will be pointing to first element
	
	while (i < j) {
		temp = path[i];
		path[i] = path[j];
		path[j] = temp;
		i++;             // increment i
		j--;          // decrement j
	}
	 len1=steps;
	
	dijkstra(prsnt2,temp_dest);
	steps=1;
	for(i=0;i<30;i++)
	{
		if(path[i]!=prsnt2)
		steps++;
		else
		break;
	}
	
	j = steps - 1;   // j will Point to last Element
	i = 0;       // i will be pointing to first element
	
	while (i < j) {
		temp = path[i];
		path[i] = path[j];
		path[j] = temp;
		i++;             // increment i
		j--;          // decrement j
	}
	len2= steps;
	if ((z==0)&&(cc!=0))
	{
		len2=temp_len2+len2;
	}
	if((cc!=0)&&(z!=0))
	{	
		if(nodes[cc-1]==filtered_nodes[z-1])
		{
			len1=temp_len1+len1;
		}
		if(nodes[cc-1]!=filtered_nodes[z-1])
		{
			len2=temp_len2+len2;
		}
	}
	
	temp_len1=len1;
	temp_len2=len2;
	if(len1<len2)
	{	
		filtered_nodes[z]=temp_dest;
		z++;
		prsnt=temp_dest;
	}
	else{
		prsnt2=temp_dest;
	}
	
}
prsnt=13;
for(k=0;k<33;k++)
{		 

	start=prsnt;
	dest=filtered_nodes[k];
    steps=1;
	
	dijkstra(start,dest);
	// To reverse the path array
	steps=1;
	for(i=0;i<30;i++)
	{
		if(path[i]!=start)
		steps++;
		else
		break;
	}
	
	j = steps - 1;   // j will Point to last Element
	i = 0;       // i will be pointing to first element
	
	while (i < j) {
		temp = path[i];
		path[i] = path[j];
		path[j] = temp;
		i++;             // increment i
		j--;          // decrement j
	}
	  
	  
		


		  
	
while(1)
	{   //code to follow black line
horizontal();
		l = ADC_Conversion(3);	//Getting data of Left WL Sensor
		c = ADC_Conversion(2);	//Getting data of Center WL Sensor
		r = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		
		
		if ((l<threshold)&&(c>=threshold)&&(r<threshold))//white  black  white
		{
			forward();
			velocity(230,230);
		}
		if ((l>=threshold)&&(c<threshold)&&(r<threshold))//black  white  white
		{
			
			soft_left();
			velocity(230,230);
		}
		if ((l<threshold)&&(c<threshold)&&(r>=threshold))//W   W   B
		{
			
			soft_right();
			velocity(230,230);
		}
		if (((l>=threshold)&&(c>=threshold)&&(r>=threshold))||((l>=threshold)&&(c>=threshold)&&(r<threshold))||((l<threshold)&&(c>=threshold)&&(r>=threshold)))//NODES
		{
			stop();
			_delay_ms(200);
			backward_mm(5);
			stop();
			horizontal();
			
			
			
						
			
		//	prev=prsnt;
			prsnt=path[p];
			nxt=path[p+1];
			lcd_init();
			lcd_print(1,1,prev,3);
			lcd_print(1,5,prsnt,3);
			lcd_print(1,9,nxt,3);
			lcd_print(2,1,l,3);
			lcd_print(2,5,c,3);
			lcd_print(2,9,r,3);
			
			/*if (prsnt==nodes[2])
			{
				return 0;
			}*/

			if(prsnt==dest)
			{
				stop();
				while(dest!=nodes[played+1])
				{
					_delay_ms(100);
					stop();
				}
				//strike
				buzzer_on();
				_delay_ms(500);
				buzzer_off();
				played++;
				UDR0=255;		//for synchronisation
				
				p=0;
				break;

			}//dest ended
			/*
			Following are the condition list OF ALL THE ANGLES PRESENT IN THE GIVEN ARENA. THESE CONDITIONS TELL THE BOT AT WHICH ANGLE
			IT HAS TO ROTATE TO MOVE FROM 1 NODE TO OTHER.
           *THE NODES WHERE THE BOT CAN REAACH JUST BY LINE FOLLOWING ARE EXCLUDED IN THESE CONDITIONS.
           EX. SAY IF THE ROBOT IS COMING FROM A PREVIOUS NODE ,IT IS  ON PRESENT NODE AND IT HAS TO MOVE TO NEXT NODE THE IT SHOULD TAKE
               A TURN OF 'THIS' DEGREE.
			*/
			
			decision(prev,prsnt,nxt);
			// IF NONE OF THE CONDITIONS ABOVE MATCHES THEN THE BOT AS TO FOLLOW BLACK LINE
// sharp=ADC_Conversion(11);
 
/* if ((sharp<=158)&&(sharp>=70))
 {
	 
	 cost[prsnt-1][nxt-1]=9999;
	 cost[nxt-1][prsnt-1]=9999;
	 start=prsnt;
	// obstacle=1;
	 stop();
	 
	 dijkstra(start,dest);
	 steps=1;
	 //to reverse path array
	 for(i=0;i<30;i++)
	 {
		 if(path[i]!=start)
		 steps++;
		 else
		 break;
	 }
	 
	 j = steps - 1;   // j will Point to last Element
	 i = 0;       // i will be pointing to first element
	 
	 while (i < j) {
		 temp = path[i];
		 path[i] = path[j];
		 path[j] = temp;
		 i++;             // increment i
		 j--;          // decrement j
	 }
	 
	// right_degrees(186);
	// prev=nxt;
	 //nxt=path[1];
	 
	//prev=prsnt; 
	prsnt=path[0];
	nxt=path[1];			//pending-> double length path condition, angle turn followed by backward_mm ,count-1
	 p=0;
	 
	 decision(prev,prsnt,nxt);
	
	 
 }//sharp sensor if */
			
		  forward();
		  _delay_ms(300);
		  buzzer_off();
		  velocity(230,230);
 			prev=prsnt;

			p++;
			
			
			
		}
		if ((l<threshold)&&(c<threshold)&&(r<threshold))//  W  W  W
		{
			if ((lprev>=threshold)&&(cprev<threshold)&&(rprev<threshold))// PREVIOUS  B  W  W
			{
				
				soft_left();
				velocity(230,230);
			}
			if ((lprev<threshold)&&(cprev<threshold)&&(rprev>=threshold))//  PREVIOUS   W  W  B
			{
				
				soft_right();
				velocity(230,230);
			}
			if ((lprev<threshold)&&(cprev>=threshold)&&(rprev<threshold))//   PREVIOUS  W   B   W  SPECIAL CASE
			{
				
				soft_left();
				velocity(230,230);
			}
			if ((lprev<threshold)&&(cprev>=threshold)&&(rprev>=threshold))//   PREVIOUS  W  B  B
			{
				
				soft_right();
				velocity(230,230);
			}
			if ((lprev>=threshold)&&(cprev>=threshold)&&(rprev<threshold))//   PREVIOUS  B  B  W
			{
				
				soft_left();
				velocity(230,230);
			}
		}

		
		// The values of white line sensors are stores for feedback system
		lprev=l;
		rprev=r;
		cprev=c;

 }// while loop
	}//for loop
			}//if condition for boot switch
  
return 0;
}
		
