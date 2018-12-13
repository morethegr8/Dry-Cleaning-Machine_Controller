/*
 * Industrial Controller.c
 *
 * Created: 9/4/2016 12:08:20 AM
 * Author : Neutral Atom
 */ 

/************************************HARDWARE************************************

ATMEGA2560 with 16MHZ Crystal
PERIPHERALS:
* RTC
* ADC with SPI	
* 7 Segment Driver with SPI
* Communication modes: RS232, RS485
* DATA LATCH - 74HC373
* 32 RELAYS
* 8 DIGITAL I/Ps - OPTO ISOLATED
* 6X7 MATRIX KEYBOARD - 42 KEYS

########################PIN DEFINITION############################################
	LCD - 4 bit mode
	DATA PINS - DB4~DB7 - PORTG PIN 2~5
	EN - PG1
	RS - PG0 
#################################################################################
	Matrix Keyboard - 6 ROWS X 7 COLUMNS - PORTE & PORTJ
	Buzzer - PJ7
#########################################################
	UART
	RX - PE0
	TX - PE1
########################################################
	EEPROM - PORTD
	ADDRESS BUS - 
	A0 - PD4
	A1 - PD5
	A2 - PD6
	WP - PD7
#########################################################
	OCTAL DATA LATCH - ADDRESS BUS - PORTA PIN 0~7
	LATCH ENABLE & LATCH OC - PORTC - PIN 0~7
	OC1 - PC4
	ENC1- PC0
	OC2 - PC5
	ENC2 - PC1
	OC3 - PC6
	ENC3 - PC2
	OC4 - PC7
	ENC4 - PC3
##########################################################
	I2C Interface - PORTD 
	SDA - PD0
	SCL - PD1
	DEVICES CONNECTED ON I2C 
	# RTC 
	# EEPROM
#########################################################
	SPI Interface - PORTB
	MISO - PB3
	MOSI - PB2
	DEVICE CONNECTED ON SPI
	# AD7705 2 Nos.
	# MAX7219
	CHIP SELECTS
	T1_SS - PB4
	T2_SS - PB5
	LDRV_SS - PB0
#########################################################
	TEMP LED - PORTH
	TT1_LED - PH0
	TT2_LED - PH1
	TT3_LED - PH2
	TT4_LED - PH3
#########################################################
	INPUTS
	RTC - SQW OUT - PD2
	DIGI INPUTS - PORTL
	I/P1~8 - PL0~PL7 

******************************************************************************/


#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

//#include "comm/uart/uart.h"
#include "peri/rtc/ds1307.h"
#include "peri/lcd/lcd_NAPL.h"
#include "peri/rtc/i2c/i2cmaster.h"
#include "peri/spi_leddrv/leddrv.h"
//#include "peri/adc/adc.h"

//********************************************MACROS & DEFINITIONS*****************************************
#define OFF 0
#define ON 1
//********************GRP1*********************************************
#define TANK1IN 0x01
#define TANK2IN 0x04
#define TANK1OUT 0x02
#define TANK2OUT 0x08
#define DRUMIN	0x10
#define DRUMOUT 0x20
#define DIST_IN 0x40
#define TANK3OUT 0x80
//**************************GRP2*****************************************
#define DIST 0x01
#define PUMP 0x02
#define HEATER 0x04
#define CHILLER 0x08
#define DRUM_ROT 0x10
#define EXTRACT 0x20
#define DOOR_LOCK 0x40
#define BLOWER 0x80
//********************************GRP ENDS******************************************
#define UART_BAUD_RATE 2400
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= (~(1<<BIT)))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
//********************************************************************************
#define NUM_SAMPLES 10 /* change the number of data samples */
#define MAX_REG_LENGTH 2 /* this says that the max length of a register is 2 bytes */


//Don't Touch the lines below
//**********************************************************************************************************
#define ROW_PORT PORTE
#define ROW_DDR   DDRE
#define ROW_PIN   PINE
#define COL_PORT PORTJ
#define COL_DDR   DDRJ
#define COL_PIN   PINJ
//***********************************************************************************************************

// Program IDlcd
uint8_t program_author[]   = "NEUTRAL ATOM";
uint8_t program_version[]  = "V1_12112016";
uint8_t program_status[]     = "LOADING...";
uint8_t program_message[] = "**PLEASE  WAIT**";

char mode_auto,mode_manu,prog1,prog2,prog3,main_menuflag,loop_brk,sec_cnt,err_rst,rly_status,grp_chng,BUZZOFF,time_up,door_lck,door_status=0;    //flags setting for display 
uint8_t dig_ip=0;
uint8_t menu_key=0,key=0,bck_key=0;      // KEYPAD VARIABLE
uint8_t dly=0,prog_run=0,prog_no=0,step_no=1;
char store[NUM_SAMPLES*MAX_REG_LENGTH + 30];
char *datapointer = store;


//***************************TIME VARIABLES**************************************************************

uint8_t year = 0;
uint8_t month = 0;
uint8_t day = 0;
uint8_t hour = 0;
uint8_t minute = 0;
uint8_t second = 0;
uint8_t set_hr=0, set_min=0, set_sec=0;

//***********************************************FUNCTION DECLARATION************************************

uint8_t GetKeyPressed();
void Buzz1S();
void Pulse1S();
void display_time();
void dig_Err();
void Relay_set(uint8_t cond,uint8_t grp, uint8_t rly);
void dly_routine(uint8_t dly);
void prog_01(uint8_t the_step);
void prog_02(uint8_t the_step);
void prog_03(uint8_t the_step);
void prog_04();
void prog_05();
void prog_06();
void prog_07(uint8_t the_step);
void doorlck_ON(void);
void doorlck_OFF(void);


//**************************WATCH DOG*******************8**************

void WDT_Init(void)
{
	//disable interrupts
	cli();
	//reset watchdog
	wdt_reset();
	// Clear WDRF in MCUSR 
	MCUSR &= ~(1<<WDRF);
	//set up WDT interrupt
	WDTCSR = (1<<WDCE)|(1<<WDE);
	//Start watchdog timer with 8s prescaller
	WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP3);
	//Enable global interrupts
	sei();
}


ISR(WDT_vect)
{
	lcd_write_instruction_4d(lcd_Clear);
	_delay_us(80);
	lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
	_delay_us(80);
	lcd_write_string_4d("RUNTIME ERROR");
	_delay_us(80);
	lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
	_delay_us(80);
	lcd_write_string_4d("REBOOT MACHINE");
	_delay_us(80);
	rly_status=0;
	Relay_set(OFF,OFF,OFF);
	rly_status=1;
	loop_brk=1;
	//Burst of fice 0.1Hz pulses
	for (uint8_t i=0;i<10;i++)
	{
		//LED ON
		PORTJ|=(1<<PJ7);
		//~0.1s delay
		_delay_ms(20);
		//LED OFF
		PORTJ&=~(1<<PJ7);
		_delay_ms(80);
	}
}
//***********************************************************************
 /********************KEYPAD FUNCTION***********************

    Function return the keycode of keypressed
    on the Keypad. Keys are numbered as follows

    [00]~[41]

   Arguments:
      None

   Return:
      Any number between 0-41 depending on
      keypressed.

      255 (hex 0xFF) if NO keypressed.

   Precondition:
      None. Can be called without any setup.

*******************************************/
 
uint8_t GetKeyPressed()
    {
      uint8_t r,c;
      
      ROW_PORT|= 0XFC;
	  COL_PORT|= 0x00;

      for(c=0;c<7;c++)
      {
         COL_DDR&=~(0X7F);
		 ROW_DDR&=~(0xFC);
         
         COL_DDR|=(0X01<<c);
		 _delay_us(10);
		 wdt_reset();
         for(r=0;r<6;r++)
         {
            if(!(ROW_PIN & (0X04<<r)))
            {		
				 return (r*7+c);
            }
         }
      }
	 wdt_reset();
      return 0XFF;//Indicate No key pressed
   }
/****************************************************************************/

void Buzz1S(void) 
{   
     wdt_reset();
     if (PIND & 0x04)
	 PORTJ &= ~(0x80);
	 else
	 PORTJ |= 0x80;
}

//*************************************************************************************

void Pulse1S(void)
{   
	wdt_reset();
	if((set_sec==0)&&(set_min==0)&&(set_hr==0))
	{
		time_up=1;
	}
	
	if((set_sec==0)&&(set_min==0)&&(set_hr!=0))
	{
		set_hr--;
		set_min=59;
		set_sec=59;
	}
	
	if((set_sec==0)&&(set_min!=0))
	{
		set_sec=59;
		set_min--;
	}
	
	
	if ((PIND & 0x04)&&(sec_cnt==0))
	{
		if(set_sec!=0)
		{
			set_sec--;			
		}
		sec_cnt=1;
	}
	else if (!(PIND & 0x04))
	{
		sec_cnt=0;
	}
}

//*******************************************************************************
void display_time(void)
{
	wdt_reset();
	// GET TIME
	ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
	_delay_us(80);
	
	lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
	_delay_us(80);
	lcd_write_string_4d("TIME:");
	_delay_us(80);
	lcd_number_write(hour,10);
	_delay_us(80);
	lcd_write_character_4d(':');
	_delay_us(80);
	lcd_number_write(minute,10);
	_delay_us(80);
	lcd_write_character_4d(':');
	_delay_us(80);
	lcd_number_write(second,10);
	_delay_us(80);
	lcd_write_character_4d(' ');
	
}
//**************************************************************************
void dig_Err(void)
{
	if (dig_ip!=0)
		{    wdt_reset();
		     lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 rly_status=0;	
			 Relay_set(OFF,OFF,OFF);
			 
		do 
		{
				wdt_reset();
				rly_status=0;
				Relay_set(OFF,OFF,OFF);
				rly_status=1;
				
				lcd_write_instruction_4d(lcd_Home);
				_delay_us(80);
				lcd_write_string_4d("ERROR CODE:");
				_delay_us(80);
				lcd_write_string_4d("E0");
				_delay_us(80);
				lcd_number_write(dig_ip,10);
				_delay_us(80);
				
			    display_time();
				main_menuflag=0;
				
				menu_key=GetKeyPressed();
				
				if(menu_key==31)
				{
					BUZZOFF=1;
					PORTJ=(0<<PJ7);
				}
				
				if(BUZZOFF!=1)
				Buzz1S();
				
				dig_ip=PINL;
				if(dig_ip==0)
				{
					wdt_reset();
					CLEARBIT(PORTJ,PINJ7);
					lcd_write_instruction_4d(lcd_Clear);
					_delay_us(80);
					prog_run=0;
					err_rst=1;
					rly_status=0;
					Relay_set(OFF,OFF,OFF);
					rly_status=1;
					BUZZOFF=0;
				}
				
		} while (dig_ip!=0);
		
		}
}

//*********************************************************************************************
void dly_routine(uint8_t dly)
{   
	LCDGotoXY(11,0);
	lcd_write_string_4d("      ");
	do
		{
			wdt_reset();
			LCDGotoXY(11,0);
			lcd_write_string_4d("00:");
			lcd_number_write(dly,10);
			lcd_write_character_4d(' ');
			_delay_ms(80);                     // LOADING COUNTDWN 3 SEC
			dly--;		
					
		} while (!dly==0);
}

void doorlck_ON()
{
		door_status=1;
		SETBIT(PORTH,PINH3);
		wdt_reset();
		//PORTA&=0x00;
		PORTC&=0x08;
		_delay_us(10);
		PORTC|=0x08;
		
		_delay_us(10);
		PORTA|=0x04;
		_delay_us(10);
		PORTC&=0x00;
		wdt_reset();	
}

void doorlck_OFF()
{
	door_status=0;
	CLEARBIT(PORTH,PINH3);
	wdt_reset();
	//PORTA&=0x00;
	PORTC&=0x08;
	_delay_us(10);
	PORTC|=0x08;
	
	_delay_us(10);
	PORTA&=0x00;
	_delay_us(10);
	PORTC&=0x00;
	wdt_reset();
}
void Relay_set(uint8_t cond,uint8_t grp, uint8_t rly)
{   
	wdt_reset();
	if(rly_status!=1)
	{   
		wdt_reset();
		if(cond==ON)
			{	wdt_reset();
				
				if (grp!=grp_chng)
				    {
						wdt_reset();
						PORTA&=0x00;
						_delay_us(10);
					}
				   
				if (grp==1)
				{   
					wdt_reset();
					grp_chng=grp;
					//PORTA&=0x00;
					PORTC&=0x01;
					_delay_us(10);
					PORTC|=0x01;
				}
		
				if (grp==2)
				{   
					wdt_reset();
					grp_chng=grp;
					//PORTA&=0x00;
					PORTC&=0x02;
					_delay_us(10);
					PORTC|=0x02;
				}
		
				if (grp==3)
				{
					wdt_reset();
					grp_chng=grp;
					//PORTA&=0x00;
					PORTC&=0x04;
					_delay_us(10);
					PORTC|=0x04;
				}
		
				/*if (grp==4)
				{  
					wdt_reset();
					grp_chng=grp;
					//PORTA&=0x00;
					PORTC&=0x08;
					_delay_us(10);
					PORTC|=0x08;
				}*/

				
				_delay_us(10);
				PORTA|=rly;
				_delay_us(10);
				PORTC&=0x00;
				wdt_reset();				
			}   //if cond == ON
	
		if (cond==OFF)
			{
				if (grp==1)
				{	
					wdt_reset();
					//PORTA&=0x00;
					PORTC&=0x01;
					_delay_us(10);
					PORTC|=0x01;
					
					_delay_us(10);
					PORTA&=(~(rly));
					_delay_us(10);
					PORTC&=0x00;
				}
				
				if (grp==2)
				{
					wdt_reset();
					//PORTA&=0x00;
					PORTC&=0x02;
					_delay_us(10);
					PORTC|=0x02;
					
					_delay_us(10);
					PORTA&=(~(rly));
					_delay_us(10);
					PORTC&=0x00;
				}
				
				if (grp==3)
				{
					wdt_reset();
					//PORTA&=0x00;
					PORTC&=0x04;
					_delay_us(10);
					PORTC|=0x04;
					
					_delay_us(10);
					PORTA&=(~(rly));
					_delay_us(10);
					PORTC&=0x00;
				}
				
				/*if (grp==4)
				{
					wdt_reset();
					//PORTA&=0x00;
					PORTC&=0x08;
					_delay_us(10);
					PORTC|=0x08;
					
					_delay_us(10);
					PORTA&=(~(rly));
					_delay_us(10);
					PORTC&=0x00;
				}*/
		
		if((grp==OFF)&&(rly==OFF))
				{
					for (grp=0;grp<=4;grp++)
					{
						if (grp==1)
						{
							wdt_reset();
							//PORTA&=0x00;
							PORTC&=0x01;
							_delay_us(10);
							PORTC|=0x01;
						}
						
						if (grp==2)
						{
							wdt_reset();
							//PORTA&=0x00;
							PORTC&=0x02;
							_delay_us(10);
							PORTC|=0x02;
						}
						
						if (grp==3)
						{
							wdt_reset();
							//PORTA&=0x00;
							PORTC&=0x04;
							_delay_us(10);
							PORTC|=0x04;
						}
						
						/*if (grp==4)
						{
							wdt_reset();
							//PORTA&=0x00;
							PORTC&=0x08;
							_delay_us(10);
							PORTC|=0x08;
						}*/
							
							_delay_us(10);
							PORTA&=0x00;
							_delay_us(10);
							PORTC&=0x00;
						
						//PORTC&=0x00;
						
					}
				}  
			}  //if cond ==OFF
		}     // if rly_status!=1 
	}         // void Relay_set()

//***************************************************************************

void Writetoreg(int byteword)
{	
	wdt_reset();
	//int q;
	//SPCR = 0x3f;
	//SPCR = 0X7f; /* this sets the WiredOR mode(DWOM=1), Master mode(MSTR=1), SCK idles high(CPOL=1), /SS can be low
	//always (CPHA=1), lowest clock speed(slowest speed which is master clock /32 */
	//DDRD = 0x18; /* SCK, MOSI outputs 
	//q = SPSR;
	//q = SPDR; /* the read of the status register and of the data register is needed to clear the interrupt which tells
	//the user that the data transfer is complete*/
	//PORTB &= 0xDF; /* /CS is low */
	CLEARBIT(PORTB,PINB5);
	SPDR = byteword; /* put the byte into data register */
	while(!(SPSR & 0x80)); /* wait for /DRDY to go low */
	SETBIT(PORTB,PINB5);
	//PORTB |= 0x20; /* /CS high */
}

/*void Read(int amount, int reglength)
	{
		int q;
		//SPCR = 0x3f;
		//SPCR = 0x7f; /* clear the interrupt */
		//DDRD = 0x10; /* MOSI output, MISO input, SCK output */
		//while(PORTB & 0x80); /* wait for /DRDY to go low */
		
		//PORTB &= 0xDF ; /* /CS is low */
		//for(q=0;q<reglength;q++)
		//{
			//SPDR = 0;
			//while(!(SPSR & 0x80)); /* wait until port ready before reading */
			//*datapointer++=SPDR; /* read SPDR into store array via datapointer */
		//}
		//PORTB|= 0x20; /* /CS is high */
	//}

unsigned int Read(char reglength)
{	
	wdt_reset();
	int b;
	unsigned int k;
	
	while(PORTB & 0x80); /* wait for /DRDY to go low */
	
	//PORTB &= 0xDF ; /* /CS is low */
	CLEARBIT(PORTB,PINB5);
	
	for(b=0;b<reglength;b++)
	{	
		wdt_reset();
		SPDR = 0xFF;
		while(!(SPSR &(1<<SPIF))); // Wait for transmission complete
		if(b==0)
		{
			k=SPDR*256;
		}
		else
		k=k+SPDR;
	}
	SETBIT(PORTB,PINB5);
	//PORTB|=0x20; /* /CS is high */
	return k;
}

/************************************************
ADC INITIALISATION
*************************************************/

void ADCinit(void)
{
	wdt_reset();
	Writetoreg(0x20); //Active channel is Ain(+) Ain(-), next operation as write to clock register
	Writetoreg(0x0C); //(for clock register) master clock enabled, 2MHz Clock, set output rate to 20Hz*///0x08
	Writetoreg(0x10); //Active channel is Ain1(+) Ain1(-), next operation as write to the setup register
	Writetoreg(0x40); //(for setup register) gain = 1, bipolar mode, buffer off, clear FSYNC and perform a Self Calibration
	
}

//*****************************AUTO PROGRAMS******************************
 void prog_01(uint8_t the_step)
{			
		wdt_reset();
		dig_ip=0;
		//  READING PORT L FOR ERRORS
		dig_ip=PINL;
		
		dig_Err();	
				
		dly_routine(3);
		//Relay_set(ON,2,DOOR_LOCK);
		doorlck_ON();
		do 
		{
			 wdt_reset();
			 switch(the_step)
				{ 
                    
					case 1: set_hr=0;
							set_min=1;
							set_sec=0; 
							do 
							{
								wdt_reset();
								dig_ip=0;
								//  READING PORT L FOR ERRORS
								dig_ip=PINL;
								
								dig_Err();
								
								if (err_rst==1)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									err_rst=0;
									doorlck_OFF();		
								}
								
								ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
								_delay_us(80);
								wdt_reset();
								
								Pulse1S();
								
								LCDGotoXY(11,0);
								lcd_write_string_4d("0");
								_delay_us(80);
								lcd_number_write(set_min,10);
								_delay_us(80);
								lcd_write_character_4d(':');
								lcd_number_write(set_sec,10);
								_delay_us(80);
								lcd_write_character_4d(' ');
								
								menu_key=GetKeyPressed();
								wdt_reset();
								
								if(menu_key==31)
								{
									wdt_reset();
									rly_status=0;
									Relay_set(OFF,OFF,OFF);
									rly_status=1;
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									time_up=0;
									set_min=0;
									set_sec=0;
									//prog_no=0;
									loop_brk=1;
									doorlck_OFF();
									
								}
							
							if (rly_status!=1)
							{
								wdt_reset();
								Relay_set(ON,1,TANK1OUT);
								Relay_set(ON,1,DRUMIN);
								Relay_set(ON,2,PUMP);
								Relay_set(ON,2,DOOR_LOCK);	       // rly 
								//Relay_set(ON,2,DOOR_LOCK);
							}
						    rly_status=1;
							
							} while (!time_up);
							
							if(loop_brk!=1)
							the_step=2;
							
							rly_status=0;
							//Relay_set(OFF,OFF,OFF);
							time_up=0;
							_delay_ms(10);
							wdt_reset();
							break;
								
					case 2: wdt_reset();
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_Home);
							_delay_us(80);
							lcd_write_string_4d("P0");
							_delay_us(80);
							lcd_number_write(prog_no,10);
							_delay_us(80);
							LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							lcd_number_write(the_step,10);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
							_delay_us(80);
							lcd_write_string_4d("NORMAL WASH    ");
							
							set_hr=0;
							set_min=6;
							set_sec=0;
							do
							{
								wdt_reset();
								dig_ip=0;
								//  READING PORT L FOR ERRORS
								dig_ip=PINL;
						
								dig_Err();
						
								if (err_rst==1)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									time_up=0;
									err_rst=0;
									doorlck_OFF();
								}
						
								ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
								_delay_us(80);
						
								Pulse1S();
								wdt_reset();
								
								LCDGotoXY(11,0);
								lcd_write_string_4d("0");
								_delay_us(80);
								lcd_number_write(set_min,10);
								_delay_us(80);
								lcd_write_character_4d(':');
								lcd_number_write(set_sec,10);
								_delay_us(80);
								lcd_write_character_4d(' ');
						
								menu_key=GetKeyPressed();
								wdt_reset();
								
								if(menu_key==31)
								{
									wdt_reset();
									rly_status=0;
									Relay_set(OFF,OFF,OFF);
									rly_status=1;
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									doorlck_OFF();
									
								}
						
								if (rly_status!=1)
								{
									wdt_reset();
									Relay_set(OFF,1,TANK1OUT);
									Relay_set(ON,1,DRUMOUT);
									Relay_set(ON,1,DRUMIN);
									Relay_set(ON,2,PUMP);
									Relay_set(ON,2,DOOR_LOCK);	       // rly
									//Relay_set(ON,2,DOOR_LOCK);
								}
								rly_status=1;
						
							} while (!time_up);
					
							if(loop_brk!=1)
							the_step=3;

							rly_status=0;
							time_up=0;
							//Relay_set(OFF,OFF,OFF);
							wdt_reset();
							
							break;
							
					case 3: wdt_reset();
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_Home);
							_delay_us(80);
							lcd_write_string_4d("P0");
							_delay_us(80);
							lcd_number_write(prog_no,10);
							_delay_us(80);
							LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							lcd_number_write(the_step,10);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
							_delay_us(80);
							lcd_write_string_4d("SOLVENT IN TANK");
							
							set_hr=0;
							set_min=0;
							set_sec=30;
							do
							{
								wdt_reset();
								dig_ip=0;
								//  READING PORT L FOR ERRORS
								dig_ip=PINL;
						
								dig_Err();
						
								if (err_rst==1)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									err_rst=0;
									doorlck_OFF();
								}
						
								ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
								_delay_us(80);
						
								Pulse1S();
						
								LCDGotoXY(11,0);
								lcd_write_string_4d("0");
								_delay_us(80);
								lcd_number_write(set_min,10);
								_delay_us(80);
								lcd_write_character_4d(':');
								lcd_number_write(set_sec,10);
								_delay_us(80);
								lcd_write_character_4d(' ');
						
								menu_key=GetKeyPressed();
								wdt_reset();
								
								if(menu_key==31)
								{
									wdt_reset();
									rly_status=0;
									Relay_set(OFF,OFF,OFF);
									rly_status=1;
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									doorlck_OFF();
									
								}
						
								if (rly_status!=1)
								{
									wdt_reset();
									Relay_set(ON,1,DRUMOUT);
									Relay_set(OFF,1,DRUMIN);
									Relay_set(ON,1,TANK1IN);
									Relay_set(ON,2,PUMP);
									Relay_set(ON,2,DOOR_LOCK);	       // rly
									//Relay_set(ON,2,DOOR_LOCK);
								}
								rly_status=1;
						
							} while (!time_up);
					
							if(loop_brk!=1)
							the_step=4;
							
							rly_status=0;
							time_up=0;
							Relay_set(OFF,OFF,OFF);
							_delay_ms(30);
							//Relay_set(ON,2,DOOR_LOCK);
							
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_Home);
							_delay_us(80);
							lcd_write_string_4d("WAIT");
							_delay_us(80);
							
							wdt_reset();
							dly_routine(8);
							
							
							wdt_reset();
							break;				
							
					case 4: wdt_reset();
							
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_Home);
							_delay_us(80);
							lcd_write_string_4d("P0");
							_delay_us(80);
							lcd_number_write(prog_no,10);
							_delay_us(80);
							LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							lcd_number_write(the_step,10);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
							_delay_us(80);
							lcd_write_string_4d("EXTRACT        ");
							wdt_reset();

							set_hr=0;
							set_min=4;
							set_sec=0;
							do
							{
								wdt_reset();
								dig_ip=0;
								//  READING PORT L FOR ERRORS
								dig_ip=PINL;
								
								dig_Err();
								
								if (err_rst==1)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									err_rst=0;
									doorlck_OFF();
								}
								
								ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
								_delay_us(80);
								
								Pulse1S();
								
								LCDGotoXY(11,0);
								lcd_write_string_4d("0");
								_delay_us(80);
								lcd_number_write(set_min,10);
								_delay_us(80);
								lcd_write_character_4d(':');
								lcd_number_write(set_sec,10);
								_delay_us(80);
								lcd_write_character_4d(' ');
								
								menu_key=GetKeyPressed();
								wdt_reset();
								
								if(menu_key==31)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									rly_status=0;
									Relay_set(OFF,OFF,OFF);
									rly_status=1;
									doorlck_OFF();
								}
								
								if (rly_status!=1)
								{
									wdt_reset();
									Relay_set(ON,1,DRUMOUT);
									Relay_set(ON,1,TANK1IN);
									Relay_set(ON,2,PUMP);
									Relay_set(ON,2,EXTRACT);
									Relay_set(OFF,2,DOOR_LOCK);	       // rly
									//Relay_set(ON,4,DOOR_LOCK);
								}
								rly_status=1;
								
							} while (!time_up);
							
							if(loop_brk!=1)
							the_step=0;    // original step 5
							rly_status=0;
							Relay_set(OFF,OFF,OFF);
							
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_Home);
							_delay_us(80);
							lcd_write_string_4d("WAIT");
							_delay_us(80);
							
							wdt_reset();
							dly_routine(90);
							
							time_up=0;
							//*****************************TEMP*****************************
							
							//if(loop_brk!=1)
							//the_step=0;
							
							wdt_reset();
							//Relay_set(OFF,OFF,OFF);
							prog_run=0;
							main_menuflag=2;
							rly_status=0;
							the_step=0;
							step_no=0;
							//Relay_set(OFF,OFF,OFF);
							time_up=0;
							
							if(loop_brk!=1)
							{
								wdt_reset();
								lcd_write_instruction_4d(lcd_Clear);
								_delay_us(80);
								lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
								_delay_us(80);
								lcd_write_string_4d("--PROGRAM ENDS--");
								_delay_us(80);
								lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
								_delay_us(80);
								lcd_write_string_4d("WAIT:");
								_delay_us(80);
								
								dly=10;
								do
								{
									wdt_reset();
									dly--;
									LCDGotoXY(11,1);
									lcd_write_string_4d("00:");
									lcd_number_write(dly,10);
									lcd_write_character_4d(' ');
									_delay_ms(80);                     // LOADING COUNTDWN 10 SEC
									PORTJ ^= _BV(PJ7);
									
								} while (!dly==0);
								
								CLEARBIT(PORTJ,PINJ7);
								lcd_write_instruction_4d(lcd_Clear);
								_delay_us(80);
								lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
								_delay_us(80);
								lcd_write_string_4d("-SELECT PROGRAM-");
								_delay_us(80);
								lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
								_delay_us(80);
								lcd_write_string_4d("PROG NO:");
								_delay_us(80);
								wdt_reset();
								prog_no=1;
								LCDGotoXY(8,1);
								lcd_number_write(prog_no,10);
							}
							doorlck_OFF();
							
							
							//***************************************************************
							wdt_reset();
							break;
							
					case 5: wdt_reset();
					
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_Home);
							_delay_us(80);
							lcd_write_string_4d("P0");
							_delay_us(80);
							lcd_number_write(prog_no,10);
							_delay_us(80);
							LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							lcd_number_write(the_step,10);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
							_delay_us(80);
							lcd_write_string_4d("HEATING        ");

							set_hr=0;
							set_min=4;
							set_sec=0;
							do
							{
								wdt_reset();
								dig_ip=0;
								//  READING PORT L FOR ERRORS
								dig_ip=PINL;
						
								dig_Err();
						
								if (err_rst==1)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									err_rst=0;
									doorlck_OFF();
								}
						
								ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
								_delay_us(80);
						
								Pulse1S();
						
								LCDGotoXY(11,0);
								lcd_write_string_4d("0");
								_delay_us(80);
								lcd_number_write(set_min,10);
								_delay_us(80);
								lcd_write_character_4d(':');
								lcd_number_write(set_sec,10);
								_delay_us(80);
								lcd_write_character_4d(' ');
						
								menu_key=GetKeyPressed();
								wdt_reset();
								
								if(menu_key==31)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									rly_status=0;
									Relay_set(OFF,OFF,OFF);
									rly_status=1;
									doorlck_OFF();
								}
						
								if (rly_status!=1)
								{
									wdt_reset();
									Relay_set(OFF,1,TANK1IN);
									Relay_set(OFF,1,DRUMOUT);
									Relay_set(OFF,2,PUMP);
									Relay_set(OFF,2,EXTRACT);
									Relay_set(ON,2,BLOWER);
									Relay_set(ON,2,HEATER);
									Relay_set(ON,2,DRUM_ROT);
								}
								rly_status=1;
						
							} while (!time_up);
					
							if(loop_brk!=1)
							the_step=6;
							rly_status=0;
							time_up=0;
							wdt_reset();
							break;	
							
					case 6: wdt_reset();
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_Home);
							_delay_us(80);
							lcd_write_string_4d("P0");
							_delay_us(80);
							lcd_number_write(prog_no,10);
							_delay_us(80);
							LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							lcd_number_write(the_step,10);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
							_delay_us(80);
							lcd_write_string_4d("PERC RECOVERY  ");
					
							set_hr=0;
							set_min=12;
							set_sec=0;
							do
							{	
								wdt_reset();
								dig_ip=0;
								//  READING PORT L FOR ERRORS
								dig_ip=PINL;
						
								dig_Err();
						
								if (err_rst==1)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									err_rst=0;
									doorlck_OFF();
								}
						
								ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
								_delay_us(80);
						
								Pulse1S();
						
								LCDGotoXY(11,0);
								lcd_write_string_4d("0");
								_delay_us(80);
								lcd_number_write(set_min,10);
								_delay_us(80);
								lcd_write_character_4d(':');
								lcd_number_write(set_sec,10);
								_delay_us(80);
								lcd_write_character_4d(' ');
						
								menu_key=GetKeyPressed();
								wdt_reset();
								
								if(menu_key==31)
								{
									wdt_reset();
									rly_status=0;
									Relay_set(OFF,OFF,OFF);
									rly_status=1;
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									doorlck_OFF();
							
								}
						
								if (rly_status!=1)
								{
									wdt_reset();
									Relay_set(ON,2,BLOWER);
									Relay_set(ON,2,HEATER);
									Relay_set(ON,2,DRUM_ROT);
									Relay_set(ON,2,CHILLER);
								}
								rly_status=1;
						
							} while (!time_up);
					
							if(loop_brk!=1)
							the_step=7;
							rly_status=0;
							time_up=0;
							wdt_reset();
							break;
							
					case 7: lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_Home);
							_delay_us(80);
							lcd_write_string_4d("P0");
							_delay_us(80);
							lcd_number_write(prog_no,10);
							_delay_us(80);
							LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							lcd_number_write(the_step,10);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
							_delay_us(80);
							lcd_write_string_4d("COOLING        ");
					
							set_hr=0;
							set_min=3;
							set_sec=0;
							do
							{
								wdt_reset();
								dig_ip=0;
								//  READING PORT L FOR ERRORS
								dig_ip=PINL;
						
								dig_Err();
						
								if (err_rst==1)
								{
									wdt_reset();
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									err_rst=0;
									doorlck_OFF();
								}
						
								ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
								_delay_us(80);
						
								Pulse1S();
						
								LCDGotoXY(11,0);
								lcd_write_string_4d("0");
								_delay_us(80);
								lcd_number_write(set_min,10);
								_delay_us(80);
								lcd_write_character_4d(':');
								lcd_number_write(set_sec,10);
								_delay_us(80);
								lcd_write_character_4d(' ');
						
								menu_key=GetKeyPressed();
								wdt_reset();
						
								if(menu_key==31)
								{
									wdt_reset();
									rly_status=0;
									Relay_set(OFF,OFF,OFF);
									rly_status=1;
									prog_run=0;
									main_menuflag=2;
									the_step=0;
									step_no=1;
									set_sec=0;
									set_min=0;
									time_up=0;
									//prog_no=0;
									loop_brk=1;
									doorlck_OFF();
							
								}
						
								if (rly_status!=1)
								{
									wdt_reset();
									Relay_set(ON,2,BLOWER);
									Relay_set(OFF,2,HEATER);
									Relay_set(ON,2,DRUM_ROT);
									Relay_set(ON,2,CHILLER);
								}
								rly_status=1;
						
							} while (!time_up);
					
							if(loop_brk!=1)
							the_step=0;
							
							wdt_reset();
							//Relay_set(OFF,OFF,OFF);
							prog_run=0;
							main_menuflag=2;
							rly_status=0;
							the_step=0;
							step_no=0;
							Relay_set(OFF,OFF,OFF);
							time_up=0;
							
							if(loop_brk!=1)
							{
							wdt_reset();
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
							_delay_us(80);
							lcd_write_string_4d("--PROGRAM ENDS--");
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
							_delay_us(80);
							lcd_write_string_4d("WAIT:");
							_delay_us(80);
							
							dly=10;
							do
							{
								wdt_reset();
								dly--;
								LCDGotoXY(11,1);
								lcd_write_string_4d("00:");
								lcd_number_write(dly,10);
								lcd_write_character_4d(' ');
								_delay_ms(80);                     // LOADING COUNTDWN 10 SEC
								 PORTJ ^= _BV(PJ7);
								
							} while (!dly==0);
							
							CLEARBIT(PORTJ,PINJ7);
							lcd_write_instruction_4d(lcd_Clear);
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
							_delay_us(80);
							lcd_write_string_4d("-SELECT PROGRAM-");
							_delay_us(80);
							lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
							_delay_us(80);
							lcd_write_string_4d("PROG NO:");
							_delay_us(80);
							wdt_reset();
							prog_no=1;
							LCDGotoXY(8,1);
							lcd_number_write(prog_no,10);
							}
							doorlck_OFF();
							wdt_reset();
							break;
							
					default: break;							
											
				}
			
		} while (prog_run==1);
}// end of prog_01

 void prog_02(uint8_t the_step)
 {
	 	wdt_reset();
		 dig_ip=0;
	 	//  READING PORT L FOR ERRORS
	 	dig_ip=PINL;
	 	
	 	dig_Err();
	 	
		wdt_reset();
	 	dly_routine(3);
	 	
	 	do
	 	{
		 	wdt_reset();
			 switch(the_step)
		 	{
			 	
			 	case 1: wdt_reset();
						set_hr=0;
						set_min=1;
			 			set_sec=0;
			 			do
			 			{
				 			 wdt_reset();
							 dig_ip=0;
				 			//  READING PORT L FOR ERRORS
				 			dig_ip=PINL;
				 	
				 			dig_Err();
				 	
				 			if (err_rst==1)
				 			{
					 			wdt_reset();
								prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 			err_rst=0;
				 			}
				 	
				 			ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 			_delay_us(80);
				 	
				 			Pulse1S();
				 	
				 			LCDGotoXY(11,0);
				 			lcd_write_string_4d("0");
				 			_delay_us(80);
				 			lcd_number_write(set_min,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(':');
				 			lcd_number_write(set_sec,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(' ');
				 	
				 			menu_key=GetKeyPressed();
				 			wdt_reset();
							 
				 			if(menu_key==31)
				 			{
					 			wdt_reset();
								rly_status=0;
					 			Relay_set(OFF,OFF,OFF);
					 			rly_status=1;
					 			prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
								step_no=1;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 	
				 			}
				 	
				 			if (rly_status!=1)
				 			{
					 			wdt_reset();
								Relay_set(ON,1,TANK2OUT);
					 			Relay_set(ON,1,DRUMIN);
					 			Relay_set(ON,2,PUMP);
					 			Relay_set(ON,2,DRUM_ROT);	       // rly
				 			}
				 			rly_status=1;
				 	
			 			} while (!time_up);
			 	
			 			if(loop_brk!=1)
			 			the_step=2;
			 			rly_status=0;
						 time_up=0;
			 			//Relay_set(OFF,OFF,OFF);
			 			_delay_ms(10);
			 			wdt_reset();
			 			break;
			 	
			 	case 2: wdt_reset();
						lcd_write_instruction_4d(lcd_Clear);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_Home);
			 			_delay_us(80);
			 			lcd_write_string_4d("P0");
			 			_delay_us(80);
			 			lcd_number_write(prog_no,10);
			 			_delay_us(80);
			 			LCDGotoXY(3,0);
			 			_delay_us(80);
			 			lcd_write_string_4d("/S0");
			 			lcd_number_write(the_step,10);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 			_delay_us(80);
			 			lcd_write_string_4d("NORMAL WASH    ");
			 	
			 			set_hr=0;
						set_min=6;
			 			set_sec=0;
			 			do
			 			{
				 			wdt_reset();
							 dig_ip=0;
				 			//  READING PORT L FOR ERRORS
				 			dig_ip=PINL;
				 	
				 			dig_Err();
				 	
				 			if (err_rst==1)
				 			{
					 			wdt_reset();
								prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 			err_rst=0;
				 			}
				 	
				 			ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 			_delay_us(80);
				 	
				 			Pulse1S();
				 	
				 			LCDGotoXY(11,0);
				 			lcd_write_string_4d("0");
				 			_delay_us(80);
				 			lcd_number_write(set_min,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(':');
				 			lcd_number_write(set_sec,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(' ');
				 	
				 			menu_key=GetKeyPressed();
							wdt_reset(); 
				 	
				 			if(menu_key==31)
				 			{
					 			wdt_reset();
								rly_status=0;
					 			Relay_set(OFF,OFF,OFF);
					 			rly_status=1;
					 			prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
								step_no=1;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 	
				 			}
				 	
				 			if (rly_status!=1)
				 			{
					 			wdt_reset();
								Relay_set(OFF,1,TANK2OUT);
					 			Relay_set(ON,1,DRUMOUT);
					 			Relay_set(ON,1,DRUMIN);
					 			Relay_set(ON,2,PUMP);
					 			Relay_set(ON,2,DRUM_ROT);	       // rly
				 			}
				 			rly_status=1;
				 	
			 			} while (!time_up);
			 	
			 			if(loop_brk!=1)
			 			the_step=3;
			 			rly_status=0;
						 time_up=0;
			 			//Relay_set(OFF,OFF,OFF);
			 			wdt_reset();
			 			break;
			 	
			 	case 3: wdt_reset();
						lcd_write_instruction_4d(lcd_Clear);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_Home);
			 			_delay_us(80);
			 			lcd_write_string_4d("P0");
			 			_delay_us(80);
			 			lcd_number_write(prog_no,10);
			 			_delay_us(80);
			 			LCDGotoXY(3,0);
			 			_delay_us(80);
			 			lcd_write_string_4d("/S0");
			 			lcd_number_write(the_step,10);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 			_delay_us(80);
			 			lcd_write_string_4d("SOLVENT IN TANK");
			 	
			 			set_hr=0;
						set_min=0;
			 			set_sec=30;
			 			do
			 			{
				 			wdt_reset();
							dig_ip=0;
				 			//  READING PORT L FOR ERRORS
				 			dig_ip=PINL;
				 	
				 			dig_Err();
				 	
				 			if (err_rst==1)
				 			{
					 			wdt_reset();
								prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 			err_rst=0;
				 			}
				 	
				 			ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 			_delay_us(80);
				 	
				 			Pulse1S();
				 	
				 			LCDGotoXY(11,0);
				 			lcd_write_string_4d("0");
				 			_delay_us(80);
				 			lcd_number_write(set_min,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(':');
				 			lcd_number_write(set_sec,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(' ');
				 	
				 			menu_key=GetKeyPressed();
							wdt_reset(); 
				 	
				 			if(menu_key==31)
				 			{
					 			wdt_reset();
								 rly_status=0;
					 			Relay_set(OFF,OFF,OFF);
					 			rly_status=1;
					 			prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
								step_no=1;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 	
				 			}
				 	
				 			if (rly_status!=1)
				 			{
					 			wdt_reset();
								Relay_set(ON,1,DRUMOUT);
					 			Relay_set(OFF,1,DRUMIN);
					 			Relay_set(ON,1,TANK2IN);
					 			Relay_set(ON,2,PUMP);
					 			Relay_set(ON,2,DRUM_ROT);	       // rly
				 			}
				 			rly_status=1;
				 	
			 			} while (!time_up);
			 	
			 			if(loop_brk!=1)
			 			the_step=4;
			 			rly_status=0;
			 			Relay_set(OFF,OFF,OFF);
						 
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_Home);
						 _delay_us(80);
						 lcd_write_string_4d("WAIT");
						 _delay_us(80);
						 
						 wdt_reset();
						 dly_routine(8);
						time_up=0; 
			 			wdt_reset();
			 			break;
			 	
			 	case 4: wdt_reset();
						
			 			lcd_write_instruction_4d(lcd_Clear);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_Home);
			 			_delay_us(80);
			 			lcd_write_string_4d("P0");
			 			_delay_us(80);
			 			lcd_number_write(prog_no,10);
			 			_delay_us(80);
			 			LCDGotoXY(3,0);
			 			_delay_us(80);
			 			lcd_write_string_4d("/S0");
			 			lcd_number_write(the_step,10);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 			_delay_us(80);
			 			lcd_write_string_4d("EXTRACT        ");
						wdt_reset();
			 	
			 			set_hr=0;
						set_min=4;
			 			set_sec=0;
			 			do
			 			{
				 			wdt_reset();
							dig_ip=0;
				 			//  READING PORT L FOR ERRORS
				 			dig_ip=PINL;
				 	
				 			dig_Err();
				 	
				 			if (err_rst==1)
				 			{
					 			wdt_reset();
								prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 			err_rst=0;
				 			}
				 	
				 			ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 			_delay_us(80);
				 	
				 			Pulse1S();
				 	
				 			LCDGotoXY(11,0);
				 			lcd_write_string_4d("0");
				 			_delay_us(80);
				 			lcd_number_write(set_min,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(':');
				 			lcd_number_write(set_sec,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(' ');
				 	
				 			menu_key=GetKeyPressed();
							wdt_reset(); 
				 	
				 			if(menu_key==31)
				 			{
					 			wdt_reset();
								prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
								step_no=1;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 			rly_status=0;
					 			Relay_set(OFF,OFF,OFF);
					 			rly_status=1;
				 			}
				 	
				 			if (rly_status!=1)
				 			{
					 			wdt_reset();
								Relay_set(ON,1,DRUMOUT);
					 			Relay_set(ON,1,TANK2IN);
					 			Relay_set(ON,2,PUMP);
					 			Relay_set(ON,2,EXTRACT);
					 			Relay_set(OFF,2,DRUM_ROT);	       // rly
				 			}
				 			rly_status=1;
				 	
			 			} while (!time_up);
			 	
			 			if(loop_brk!=1)
			 			the_step=5;
			 			rly_status=0;
			 			Relay_set(OFF,OFF,OFF);
						 time_up=0;
						 
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_Home);
						 _delay_us(80);
						 lcd_write_string_4d("WAIT");
						 _delay_us(90);
						 
						 wdt_reset();
						 dly_routine(90);
						wdt_reset(); 
			 	
			 			break;
			 	
			 			case 5: wdt_reset(); 
								
			 					lcd_write_instruction_4d(lcd_Clear);
			 					_delay_us(80);
			 					lcd_write_instruction_4d(lcd_Home);
			 					_delay_us(80);
			 					lcd_write_string_4d("P0");
			 					_delay_us(80);
			 					lcd_number_write(prog_no,10);
			 					_delay_us(80);
			 					LCDGotoXY(3,0);
			 					_delay_us(80);
			 					lcd_write_string_4d("/S0");
			 					lcd_number_write(the_step,10);
			 					_delay_us(80);
			 					lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 					_delay_us(80);
			 					lcd_write_string_4d("HEATING        ");

			 	
			 					set_hr=0;
								set_min=4;
			 					set_sec=0;
			 					do
			 					{
				 					wdt_reset();
									 dig_ip=0;
				 					//  READING PORT L FOR ERRORS
				 					dig_ip=PINL;
				 	
				 					dig_Err();
				 	
				 					if (err_rst==1)
				 					{
					 					wdt_reset();
										 prog_run=0;
					 					main_menuflag=2;
					 					the_step=0;
					 					set_sec=0;
					 					//prog_no=0;
					 					loop_brk=1;
					 					err_rst=0;
				 					}
				 	
				 					ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 					_delay_us(80);
				 	
				 					Pulse1S();
				 	
				 					LCDGotoXY(11,0);
				 					lcd_write_string_4d("0");
				 					_delay_us(80);
				 					lcd_number_write(set_min,10);
				 					_delay_us(80);
				 					lcd_write_character_4d(':');
				 					lcd_number_write(set_sec,10);
				 					_delay_us(80);
				 					lcd_write_character_4d(' ');
				 	
				 					menu_key=GetKeyPressed();
									wdt_reset(); 
				 	
				 					if(menu_key==31)
				 					{
					 					wdt_reset();
										prog_run=0;
					 					main_menuflag=2;
					 					the_step=0;
										step_no=1;
					 					set_sec=0;
					 					//prog_no=0;
					 					loop_brk=1;
					 					rly_status=0;
					 					Relay_set(OFF,OFF,OFF);
					 					rly_status=1;
				 					}
				 	
				 					if (rly_status!=1)
				 					{
					 					wdt_reset();
										Relay_set(OFF,1,TANK2IN);
					 					Relay_set(OFF,1,DRUMOUT);
					 					Relay_set(OFF,2,PUMP);
					 					Relay_set(OFF,2,EXTRACT);
					 					Relay_set(ON,2,BLOWER);
					 					Relay_set(ON,2,HEATER);
					 					Relay_set(ON,2,DRUM_ROT);
				 					}
				 					rly_status=1;
				 	
			 					} while (!time_up);
			 	
			 					if(loop_brk!=1)
			 					the_step=6;
			 					rly_status=0;
								 time_up=0;
								 wdt_reset();
			 			break;
			 	
			 	case 6: wdt_reset();
						lcd_write_instruction_4d(lcd_Clear);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_Home);
			 			_delay_us(80);
			 			lcd_write_string_4d("P0");
			 			_delay_us(80);
			 			lcd_number_write(prog_no,10);
			 			_delay_us(80);
			 			LCDGotoXY(3,0);
			 			_delay_us(80);
			 			lcd_write_string_4d("/S0");
			 			lcd_number_write(the_step,10);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 			_delay_us(80);
			 			lcd_write_string_4d("PERC RECOVERY  ");
			 	
			 			set_hr=0;
						 set_min=12;
			 			set_sec=0;
			 			do
			 			{
				 			wdt_reset();
							 dig_ip=0;
				 			//  READING PORT L FOR ERRORS
				 			dig_ip=PINL;
				 	
				 			dig_Err();
				 	
				 			if (err_rst==1)
				 			{
					 			wdt_reset();
								 prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
					 			set_sec=0;
								set_min=0;
								time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 			err_rst=0;
				 			}
				 	
				 			ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 			_delay_us(80);
				 	
				 			Pulse1S();
				 	
				 			LCDGotoXY(11,0);
				 			lcd_write_string_4d("0");
				 			_delay_us(80);
				 			lcd_number_write(set_min,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(':');
				 			lcd_number_write(set_sec,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(' ');
				 	
				 			menu_key=GetKeyPressed();
				 			wdt_reset();
							 
				 			if(menu_key==31)
				 			{
					 			wdt_reset();
								rly_status=0;
					 			Relay_set(OFF,OFF,OFF);
					 			rly_status=1;
					 			prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
								step_no=1;
					 			set_sec=0;
								 set_min=0;
								 time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 	
				 			}
				 	
				 			if (rly_status!=1)
				 			{
					 			wdt_reset();
								Relay_set(ON,2,BLOWER);
					 			Relay_set(ON,2,HEATER);
					 			Relay_set(ON,2,DRUM_ROT);
					 			Relay_set(ON,2,CHILLER);
				 			}
				 			rly_status=1;
				 	
			 			} while (!time_up);
			 	
			 			if(loop_brk!=1)
			 			the_step=7;
			 			rly_status=0;
						 time_up=0;
						wdt_reset();
			 			break;
			 	
			 	case 7: wdt_reset();
						lcd_write_instruction_4d(lcd_Clear);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_Home);
			 			_delay_us(80);
			 			lcd_write_string_4d("P0");
			 			_delay_us(80);
			 			lcd_number_write(prog_no,10);
			 			_delay_us(80);
			 			LCDGotoXY(3,0);
			 			_delay_us(80);
			 			lcd_write_string_4d("/S0");
			 			lcd_number_write(the_step,10);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 			_delay_us(80);
			 			lcd_write_string_4d("PERC RECOVERY  ");
			 	
			 			set_hr=0;
						 set_min=3;
			 			set_sec=0;
			 			do
			 			{
				 			wdt_reset();
							 dig_ip=0;
				 			//  READING PORT L FOR ERRORS
				 			dig_ip=PINL;
				 	
				 			dig_Err();
				 	
				 			if (err_rst==1)
				 			{
					 			wdt_reset();
								 prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
					 			set_sec=0;
					 			set_min=0;
					 			time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 			err_rst=0;
				 			}
				 	
				 			ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 			_delay_us(80);
				 	
				 			Pulse1S();
				 	
				 			LCDGotoXY(11,0);
				 			lcd_write_string_4d("0");
				 			_delay_us(80);
				 			lcd_number_write(set_min,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(':');
				 			lcd_number_write(set_sec,10);
				 			_delay_us(80);
				 			lcd_write_character_4d(' ');
				 	
				 			menu_key=GetKeyPressed();
							wdt_reset(); 
				 	
				 			if(menu_key==31)
				 			{
					 			wdt_reset();
								 rly_status=0;
					 			Relay_set(OFF,OFF,OFF);
					 			rly_status=1;
					 			prog_run=0;
					 			main_menuflag=2;
					 			the_step=0;
								step_no=1;
					 			set_sec=0;
								 set_min=0;
								 time_up=0;
					 			//prog_no=0;
					 			loop_brk=1;
					 	
				 			}
				 	
				 			if (rly_status!=1)
				 			{
					 			wdt_reset();
								 Relay_set(ON,2,BLOWER);
					 			Relay_set(OFF,2,HEATER);
					 			Relay_set(ON,2,DRUM_ROT);
					 			Relay_set(ON,2,CHILLER);
				 			}
				 			rly_status=1;
				 	
			 			} while (!time_up);
			 	
			 			if(loop_brk!=1)
			 			the_step=0;
			 	
			 			Relay_set(OFF,OFF,OFF);
			 			prog_run=0;
			 			main_menuflag=2;
			 			rly_status=0;
			 			the_step=0;
						step_no=0;
			 			Relay_set(OFF,OFF,OFF);
						 time_up=0;
						wdt_reset(); 
			 	
			 			if(loop_brk!=1)
			 			{
						 wdt_reset();
						 lcd_write_instruction_4d(lcd_Clear);
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
			 			_delay_us(80);
						lcd_write_string_4d("-PROGRAM ENDS-");
						_delay_us(80);
						lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						_delay_us(80);
						lcd_write_string_4d("WAIT:");
						_delay_us(80);
						 
						 dly=10;
						 do
						 {
							
							 wdt_reset();
							 LCDGotoXY(11,1);
							 lcd_write_string_4d("00:");
							 lcd_number_write(dly,10);
							 lcd_write_character_4d(' ');
							 _delay_ms(80);                     // LOADING COUNTDWN 10 SEC
							  dly--;
							  PORTJ ^= _BV(PJ7);
							 //Buzz1S();
							 
						 } while (!dly==0);
						 
						CLEARBIT(PORTJ,PINJ7);
						lcd_write_instruction_4d(lcd_Clear);
						_delay_us(80);
						lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						_delay_us(80);
			 			lcd_write_string_4d("-SELECT PROGRAM-");
			 			_delay_us(80);
			 			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
			 			_delay_us(80);
			 			lcd_write_string_4d("PROG NO:");
			 			_delay_us(80);
						 wdt_reset();
			 			prog_no=1;
			 			LCDGotoXY(8,1);
			 			lcd_number_write(prog_no,10);
					}
			 	wdt_reset();
			 			break;
			 	
			 	default: wdt_reset();
						break;
			 	
		 	}
		 	
	 	} while (prog_run==1);
	 	

 }// end of program 2

 void prog_03(uint8_t the_step)
 {
	 wdt_reset();
	 dig_ip=0;
	 //  READING PORT L FOR ERRORS
	 dig_ip=PINL;
	 
	 dig_Err();
	 wdt_reset();
	 dly_routine(3);
	 
	 do
	 {
		 wdt_reset();
		 switch(the_step)
		 {
			 
			 case 1:wdt_reset(); 
			 set_hr=0;
			 set_min=1;
			 set_sec=0;
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(11,0);
				 lcd_write_string_4d("0");
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(ON,1,TANK2OUT);
					 Relay_set(ON,1,DRUMIN);
					 Relay_set(ON,2,PUMP);
					 Relay_set(ON,2,DRUM_ROT);	       // rly
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			 
			 if(loop_brk!=1)
			 the_step=2;
			 rly_status=0;
			 //Relay_set(OFF,OFF,OFF);
			 time_up=0;
			 _delay_ms(10);
			 wdt_reset();
			 
			 break;
			 
			 case 2:wdt_reset(); 
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("P0");
			 _delay_us(80);
			 lcd_number_write(prog_no,10);
			 _delay_us(80);
			 LCDGotoXY(3,0);
			 _delay_us(80);
			 lcd_write_string_4d("/S0");
			 lcd_number_write(the_step,10);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 _delay_us(80);
			 lcd_write_string_4d("NORMAL WASH    ");
			 
			 set_hr=0;
			 set_min=6;
			 set_sec=0;
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(11,0);
				 lcd_write_string_4d("0");
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(OFF,1,TANK1OUT);
					 Relay_set(ON,1,DRUMOUT);
					 Relay_set(ON,1,DRUMIN);
					 Relay_set(ON,2,PUMP);
					 Relay_set(ON,2,DRUM_ROT);	       // rly
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			 
			 if(loop_brk!=1)
			 the_step=3;
			 rly_status=0;
			 time_up=0;
			 wdt_reset();
			 //Relay_set(OFF,OFF,OFF);
			 
			 break;
			 
			 case 3: wdt_reset();
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("P0");
			 _delay_us(80);
			 lcd_number_write(prog_no,10);
			 _delay_us(80);
			 LCDGotoXY(3,0);
			 _delay_us(80);
			 lcd_write_string_4d("/S0");
			 lcd_number_write(the_step,10);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 _delay_us(80);
			 lcd_write_string_4d("SOLVENT IN TANK");
			 
			 set_hr=0;
			 set_min=0;
			 set_sec=30;
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(11,0);
				 lcd_write_string_4d("0");
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(ON,1,DRUMOUT);
					 Relay_set(OFF,1,DRUMIN);
					 Relay_set(ON,1,TANK1IN);
					 Relay_set(ON,2,PUMP);
					 Relay_set(ON,2,DRUM_ROT);	       // rly
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			 
			 if(loop_brk!=1)
			 the_step=4;
			 rly_status=0;
			 Relay_set(OFF,OFF,OFF);
			 time_up=0;
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("WAIT");
			 _delay_us(80);
			 
			 wdt_reset();
			 dly_routine(8);
			 wdt_reset();
			 
			 break;
			 
			 case 4: wdt_reset();
			 
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("P0");
			 _delay_us(80);
			 lcd_number_write(prog_no,10);
			 _delay_us(80);
			 LCDGotoXY(3,0);
			 _delay_us(80);
			 lcd_write_string_4d("/S0");
			 lcd_number_write(the_step,10);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 _delay_us(80);
			 lcd_write_string_4d("EXTRACT        ");

			 set_hr=0;
			 set_min=4;
			 set_sec=0;
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					set_sec=0;
					set_min=0;
					time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(11,0);
				 lcd_write_string_4d("0");
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(ON,1,DRUMOUT);
					 Relay_set(ON,1,TANK1IN);
					 Relay_set(ON,2,PUMP);
					 Relay_set(ON,2,EXTRACT);
					 Relay_set(OFF,2,DRUM_ROT);	       // rly
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			 
			 if(loop_brk!=1)
			 the_step=5;
			 rly_status=0;
			 Relay_set(OFF,OFF,OFF);
			 time_up=0;
			 
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("WAIT");
			 _delay_us(80);
			 
			 wdt_reset();
			 dly_routine(90);
			 
			 wdt_reset();
			 break;
			 
			 case 5: wdt_reset();
			 
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("P0");
			 _delay_us(80);
			 lcd_number_write(prog_no,10);
			 _delay_us(80);
			 LCDGotoXY(3,0);
			 _delay_us(80);
			 lcd_write_string_4d("/S0");
			 lcd_number_write(the_step,10);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 _delay_us(80);
			 lcd_write_string_4d("HEATING        ");

			 set_hr=0;
			 set_min=4;
			 set_sec=0;
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(11,0);
				 lcd_write_string_4d("0");
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(OFF,1,TANK1IN);
					 Relay_set(OFF,1,DRUMOUT);
					 Relay_set(OFF,2,PUMP);
					 Relay_set(OFF,2,EXTRACT);
					 Relay_set(ON,2,BLOWER);
					 Relay_set(ON,2,HEATER);
					 Relay_set(ON,2,DRUM_ROT);
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			 
			 if(loop_brk!=1)
			 the_step=6;
			 rly_status=0;
			 time_up=0;
			 wdt_reset();
			 break;
			 
			 case 6: wdt_reset();
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("P0");
			 _delay_us(80);
			 lcd_number_write(prog_no,10);
			 _delay_us(80);
			 LCDGotoXY(3,0);
			 _delay_us(80);
			 lcd_write_string_4d("/S0");
			 lcd_number_write(the_step,10);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 _delay_us(80);
			 lcd_write_string_4d("PERC RECOVERY  ");
			 
			 set_hr=0;
			 set_min=12;
			 set_sec=0;
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(11,0);
				 lcd_write_string_4d("0");
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(ON,2,BLOWER);
					 Relay_set(ON,2,HEATER);
					 Relay_set(ON,2,DRUM_ROT);
					 Relay_set(ON,2,CHILLER);
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			 
			 if(loop_brk!=1)
			 the_step=7;
			 rly_status=0;
			 time_up=0;
			 wdt_reset();
			 break;
			 
			 case 7: wdt_reset(); 
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("P0");
			 _delay_us(80);
			 lcd_number_write(prog_no,10);
			 _delay_us(80);
			 LCDGotoXY(3,0);
			 _delay_us(80);
			 lcd_write_string_4d("/S0");
			 lcd_number_write(the_step,10);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 _delay_us(80);
			 lcd_write_string_4d("PERC RECOVERY  ");
			 
			 set_hr=0;
			 set_min=3;
			 set_sec=0;
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(11,0);
				 lcd_write_string_4d("0");
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(ON,2,BLOWER);
					 Relay_set(OFF,2,HEATER);
					 Relay_set(ON,2,DRUM_ROT);
					 Relay_set(ON,2,CHILLER);
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			 
			 if(loop_brk!=1)
			 the_step=0;
			 
			 Relay_set(OFF,OFF,OFF);
			 prog_run=0;
			 main_menuflag=2;
			 rly_status=0;
			 the_step=0;
			 step_no=0;
			 Relay_set(OFF,OFF,OFF);
			 time_up=0;
			 wdt_reset();
			 
			 if(loop_brk!=1)
			 {
				 wdt_reset();
				 lcd_write_instruction_4d(lcd_Clear);
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
				 _delay_us(80);
				 lcd_write_string_4d("--PROGRAM ENDS--");
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("WAIT:");
				 _delay_us(80);
			 
				 dly=10;
				 do
				 {
					 wdt_reset();
					 dly--;
					 LCDGotoXY(11,1);
					 lcd_write_string_4d("00:");
					 lcd_number_write(dly,10);
					 lcd_write_character_4d(' ');
					 _delay_ms(80);                     // LOADING COUNTDWN 10 SEC
					 PORTJ ^= _BV(PJ7);
				 
				 } while (!dly==0);
			 
				 CLEARBIT(PORTJ,PINJ7);
				 lcd_write_instruction_4d(lcd_Clear);
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
				 _delay_us(80);
				 lcd_write_string_4d("-SELECT PROGRAM-");
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("PROG NO:");
				 _delay_us(80);
				 wdt_reset();
				 prog_no=1;
				 LCDGotoXY(8,1);
				 lcd_number_write(prog_no,10); 
			}
			wdt_reset();
			 break;
			 
			 default: wdt_reset();
						break;
			 
		 }
		 
	 } while (prog_run==1);
 } // program 3 ends
 
 void prog_04()
 {
	 wdt_reset();
	 dig_ip=0;
	 //  READING PORT L FOR ERRORS
	 dig_ip=PINL;
	 
	 dig_Err();
	 
	 lcd_write_instruction_4d(lcd_Clear);
	 _delay_us(80);
	 lcd_write_instruction_4d(lcd_Home);
	 _delay_us(80);
	 lcd_write_string_4d("P0");
	 _delay_us(80);
	 lcd_number_write(prog_no,10);
	 _delay_us(80);
	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
	 _delay_us(80);
	 lcd_write_string_4d("PERC TRANSFER");
	 
	 wdt_reset();
	 dly_routine(3);
	 
	 set_hr=0;
	 set_min=2;
	 set_sec=0;
	 do
	 {
		 wdt_reset();
		 dig_ip=0;
		 //  READING PORT L FOR ERRORS
		 dig_ip=PINL;
		 
		 dig_Err();
		 
		 if (err_rst==1)
		 {
			 wdt_reset();
			 prog_run=0;
			 main_menuflag=2;
			 set_sec=0;
			 set_min=0;
			 time_up=0;
			 //prog_no=0;
			 loop_brk=1;
			 err_rst=0;
		 }
		 
		 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
		 _delay_us(80);
		 
		 Pulse1S();
		 
		 LCDGotoXY(11,0);
		 lcd_write_string_4d("0");
		 _delay_us(80);
		 lcd_number_write(set_min,10);
		 _delay_us(80);
		 lcd_write_character_4d(':');
		 lcd_number_write(set_sec,10);
		 _delay_us(80);
		 lcd_write_character_4d(' ');
		 
		 menu_key=GetKeyPressed();
		 wdt_reset();
		 
		 if(menu_key==31)
		 {
			 wdt_reset();
			 rly_status=0;
			 Relay_set(OFF,OFF,OFF);
			 rly_status=1;
			 prog_run=0;
			 main_menuflag=2;
			 set_sec=0;
			 set_min=0;
			 time_up=0;
			 //prog_no=0;
			 loop_brk=1;
			 
		 }
		 
		 if (rly_status!=1)
		 {
			 wdt_reset();
			 Relay_set(ON,1,TANK2OUT);
			 Relay_set(ON,1,TANK1IN);
			 Relay_set(ON,2,PUMP);
		 }
		 rly_status=1;
		 
	 } while (!time_up);
	 
	 prog_run=0;
	 main_menuflag=2;
	 rly_status=0;
	 Relay_set(OFF,OFF,OFF);
	 time_up=0;
	 wdt_reset();

	 if(loop_brk!=1)
	 {
		 wdt_reset();
		 lcd_write_instruction_4d(lcd_Clear);
		 _delay_us(80);
		 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
		 _delay_us(80);
		 lcd_write_string_4d("--PROGRAM ENDS--");
		 _delay_us(80);
		 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		 _delay_us(80);
		 lcd_write_string_4d("WAIT:");
		 _delay_us(80);
		 
		 dly=10;
		 do
		 {
			 wdt_reset();
			 dly--;
			 LCDGotoXY(11,1);
			 lcd_write_string_4d("00:");
			 lcd_number_write(dly,10);
			 lcd_write_character_4d(' ');
			 _delay_ms(80);                     // LOADING COUNTDWN 10 SEC
			 PORTJ ^= _BV(PJ7);
			 
		 } while (!dly==0);
		 
		 CLEARBIT(PORTJ,PINJ7);
		 lcd_write_instruction_4d(lcd_Clear);
		 _delay_us(80);
		 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
		 _delay_us(80);
		 lcd_write_string_4d("-SELECT PROGRAM-");
		 _delay_us(80);
		 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		 _delay_us(80);
		 lcd_write_string_4d("PROG NO:");
		 _delay_us(80);
		 wdt_reset();
		 prog_no=1;
		 LCDGotoXY(8,1);
		 lcd_number_write(prog_no,10); 
	 }
 }
 
 
  void prog_05()
  {
	  wdt_reset();
	  dig_ip=0;
	  //  READING PORT L FOR ERRORS
	  dig_ip=PINL;
	  
	  dig_Err();

	  lcd_write_instruction_4d(lcd_Clear);
	  _delay_us(80);
	  lcd_write_instruction_4d(lcd_Home);
	  _delay_us(80);
	  lcd_write_string_4d("P0");
	  _delay_us(80);
	  lcd_number_write(prog_no,10);
	  _delay_us(80);
	  lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
	  _delay_us(80);
	  lcd_write_string_4d("PERC IN TANK1");
	  
	  wdt_reset();
	  dly_routine(3);
	  
	  set_hr=0;
	  set_min=2;
	  set_sec=0;
	  do
	  {
		  wdt_reset();
		  dig_ip=0;
		  //  READING PORT L FOR ERRORS
		  dig_ip=PINL;
		  
		  dig_Err();
		  
		  if (err_rst==1)
		  {
			  wdt_reset();
			  prog_run=0;
			  main_menuflag=2;
			  set_sec=0;
			  set_min=0;
			  time_up=0;
			  //prog_no=0;
			  loop_brk=1;
			  err_rst=0;
		  }
		  
		  ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
		  _delay_us(80);
		  
		  Pulse1S();
		  
		  LCDGotoXY(11,0);
		  lcd_write_string_4d("0");
		  _delay_us(80);
		  lcd_number_write(set_min,10);
		  _delay_us(80);
		  lcd_write_character_4d(':');
		  lcd_number_write(set_sec,10);
		  _delay_us(80);
		  lcd_write_character_4d(' ');
		  
		  menu_key=GetKeyPressed();
		  wdt_reset();
		  
		  if(menu_key==31)
		  {
			  wdt_reset();
			  rly_status=0;
			  Relay_set(OFF,OFF,OFF);
			  rly_status=1;
			  prog_run=0;
			  main_menuflag=2;
			  set_sec=0;
			  set_min=0;
			  time_up=0;
			  //prog_no=0;
			  loop_brk=1;
			  
		  }
		  
		  if (rly_status!=1)
		  {
			  wdt_reset();
			  Relay_set(ON,1,TANK1IN);
			  Relay_set(ON,2,PUMP);
		  }
		  rly_status=1;
		  
	  } while (!time_up);
	  
	  prog_run=0;
	  main_menuflag=2;
	  rly_status=0;
	  Relay_set(OFF,OFF,OFF);
	  time_up=0;
	  wdt_reset();

	  if(loop_brk!=1)
	  {
		  lcd_write_instruction_4d(lcd_Clear);
		  _delay_us(80);
		  lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
		  _delay_us(80);
		  lcd_write_string_4d("--PROGRAM ENDS--");
		  _delay_us(80);
		  lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		  _delay_us(80);
		  lcd_write_string_4d("WAIT:");
		  _delay_us(80);
		  
		  dly=10;
		  do
		  {
			  wdt_reset();
			  dly--;
			  LCDGotoXY(11,1);
			  lcd_write_string_4d("00:");
			  lcd_number_write(dly,10);
			  lcd_write_character_4d(' ');
			  _delay_ms(80);                     // LOADING COUNTDWN 10 SEC
			  PORTJ ^= _BV(PJ7);
			  
		  } while (!dly==0);
		  
		  CLEARBIT(PORTJ,PINJ7);
		  lcd_write_instruction_4d(lcd_Clear);
		  _delay_us(80);
		  lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
		  _delay_us(80);
		  lcd_write_string_4d("-SELECT PROGRAM-");
		  _delay_us(80);
		  lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		  _delay_us(80);
		  lcd_write_string_4d("PROG NO:");
		  _delay_us(80);
		  wdt_reset();
		  prog_no=1;
		  LCDGotoXY(8,1);
		  lcd_number_write(prog_no,10);
	  }
  }

 
 
 void prog_06()
 {
	 wdt_reset();
	 dig_ip=0;
	 //  READING PORT L FOR ERRORS
	 dig_ip=PINL;
	 
	 dig_Err();

	 lcd_write_instruction_4d(lcd_Clear);
	 _delay_us(80);
	 lcd_write_instruction_4d(lcd_Home);
	 _delay_us(80);
	 lcd_write_string_4d("P0");
	 _delay_us(80);
	 lcd_number_write(prog_no,10);
	 _delay_us(80);
	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
	 _delay_us(80);
	 lcd_write_string_4d("DISTL TRANSFER");
	 
	 wdt_reset();
	 dly_routine(3);
	 
	 set_hr=0;
	 set_min=2;
	 set_sec=0;
	 do
	 {
		 wdt_reset();
		 dig_ip=0;
		 //  READING PORT L FOR ERRORS
		 dig_ip=PINL;
		 
		 dig_Err();
		 
		 if (err_rst==1)
		 {
			 wdt_reset();
			 prog_run=0;
			 main_menuflag=2;
			 set_sec=0;
			 set_min=0;
			 time_up=0;
			 //prog_no=0;
			 loop_brk=1;
			 err_rst=0;
		 }
		 
		 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
		 _delay_us(80);
		 
		 Pulse1S();
		 
		 LCDGotoXY(11,0);
		 lcd_write_string_4d("0");
		 _delay_us(80);
		 lcd_number_write(set_min,10);
		 _delay_us(80);
		 lcd_write_character_4d(':');
		 lcd_number_write(set_sec,10);
		 _delay_us(80);
		 lcd_write_character_4d(' ');
		 
		 menu_key=GetKeyPressed();
		 wdt_reset();
		 
		 if(menu_key==31)
		 {
			 wdt_reset();
			 rly_status=0;
			 Relay_set(OFF,OFF,OFF);
			 rly_status=1;
			 prog_run=0;
			 main_menuflag=2;
			 set_sec=0;
			 set_min=0;
			 time_up=0;
			 //prog_no=0;
			 loop_brk=1;
			 
		 }
		 
		 if (rly_status!=1)
		 {
			 wdt_reset();
			 Relay_set(ON,1,TANK1OUT);
			 Relay_set(ON,1,DIST_IN);
			 Relay_set(ON,2,PUMP);
		 }
		 rly_status=1;
		 
	 } while (!time_up);
	 
	 prog_run=0;
	 main_menuflag=2;
	 rly_status=0;
	 Relay_set(OFF,OFF,OFF);
	 time_up=0;
	 wdt_reset();

	 if(loop_brk!=1)
	 {
		 lcd_write_instruction_4d(lcd_Clear);
		 _delay_us(80);
		 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
		 _delay_us(80);
		 lcd_write_string_4d("--PROGRAM ENDS--");
		 _delay_us(80);
		 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		 _delay_us(80);
		 lcd_write_string_4d("WAIT:");
		 _delay_us(80);
		 
		 dly=10;
		 do
		 {
			 wdt_reset();
			 dly--;
			 LCDGotoXY(11,1);
			 lcd_write_string_4d("00:");
			 lcd_number_write(dly,10);
			 lcd_write_character_4d(' ');
			 _delay_ms(80);                     // LOADING COUNTDWN 10 SEC
			 PORTJ ^= _BV(PJ7);
			 
		 } while (!dly==0);
		 
		 CLEARBIT(PORTJ,PINJ7);
		 lcd_write_instruction_4d(lcd_Clear);
		 _delay_us(80);
		 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
		 _delay_us(80);
		 lcd_write_string_4d("-SELECT PROGRAM-");
		 _delay_us(80);
		 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		 _delay_us(80);
		 lcd_write_string_4d("PROG NO:");
		 _delay_us(80);
		 wdt_reset();
		 prog_no=1;
		 LCDGotoXY(8,1);
		 lcd_number_write(prog_no,10);
	 }
 }
  
 
 void prog_07(uint8_t the_step)
 {
	 wdt_reset();
	 dig_ip=0;
	 //  READING PORT L FOR ERRORS
	 dig_ip=PINL;
	 
	 dig_Err();
	 
	 wdt_reset();
	 dly_routine(3);
	 
	 do
	 {
		 wdt_reset();
		 switch(the_step)
		 {
			 case 1: wdt_reset();
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("P0");
			 _delay_us(80);
			 lcd_number_write(prog_no,10);
			 _delay_us(80);
			 LCDGotoXY(3,0);
			 _delay_us(80);
			 lcd_write_string_4d("/S0");
			 lcd_number_write(the_step,10);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 _delay_us(80);
			 lcd_write_string_4d("DISTILLATION");
			 
			 set_hr=3;
			 set_min=10;
			 set_sec=0;
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 set_hr=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(9,0);
				 lcd_number_write(set_hr,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_hr=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(ON,2,DIST);
					 //Relay_set(ON,2,CHILLER);
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			
			 if(loop_brk!=1)
			 the_step=0;
			 
			 //Relay_set(OFF,OFF,OFF);
			 prog_run=0;
			 main_menuflag=2;
			 rly_status=0;
			 the_step=0;
			 Relay_set(OFF,OFF,OFF);
			 time_up=0;
			 wdt_reset();
			 
			 if(loop_brk!=1)
			 {
				 wdt_reset();
				 lcd_write_instruction_4d(lcd_Clear);
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
				 _delay_us(80);
				 lcd_write_string_4d("--PROGRAM ENDS--");
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("WAIT:");
				 _delay_us(80);
				 
				 dly=10;
				 do
				 {
					 wdt_reset();
					 dly--;
					 LCDGotoXY(11,1);
					 lcd_write_string_4d("00:");
					 lcd_number_write(dly,10);
					 lcd_write_character_4d(' ');
					 _delay_ms(80);                     // LOADING COUNTDWN 10 SEC
					 PORTJ ^= _BV(PJ7);
					 
				 } while (!dly==0);
				 
				 CLEARBIT(PORTJ,PINJ7);
				 lcd_write_instruction_4d(lcd_Clear);
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
				 _delay_us(80);
				 lcd_write_string_4d("-SELECT PROGRAM-");
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("PROG NO:");
				 _delay_us(80);
				 wdt_reset();
				 prog_no=1;
				 LCDGotoXY(8,1);
				 lcd_number_write(prog_no,10);
				}
			 wdt_reset();
			 break;
			 
			/* case 2:wdt_reset(); 
			 lcd_write_instruction_4d(lcd_Clear);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_Home);
			 _delay_us(80);
			 lcd_write_string_4d("P0");
			 _delay_us(80);
			 lcd_number_write(prog_no,10);
			 _delay_us(80);
			 LCDGotoXY(3,0);
			 _delay_us(80);
			 lcd_write_string_4d("/S0");
			 lcd_number_write(the_step,10);
			 _delay_us(80);
			 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
			 _delay_us(80);
			 lcd_write_string_4d("COOLING     ");
			 
			 set_hr=0;
			 set_min=10;
			 set_sec=0;
			 
			 do
			 {
				 wdt_reset();
				 dig_ip=0;
				 //  READING PORT L FOR ERRORS
				 dig_ip=PINL;
				 
				 dig_Err();
				 
				 if (err_rst==1)
				 {
					 wdt_reset();
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 err_rst=0;
				 }
				 
				 ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
				 _delay_us(80);
				 
				 Pulse1S();
				 
				 LCDGotoXY(11,0);
				 lcd_write_string_4d("0");
				 _delay_us(80);
				 lcd_number_write(set_min,10);
				 _delay_us(80);
				 lcd_write_character_4d(':');
				 lcd_number_write(set_sec,10);
				 _delay_us(80);
				 lcd_write_character_4d(' ');
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==31)
				 {
					 wdt_reset();
					 rly_status=0;
					 Relay_set(OFF,OFF,OFF);
					 rly_status=1;
					 prog_run=0;
					 main_menuflag=2;
					 the_step=0;
					 step_no=1;
					 set_sec=0;
					 set_min=0;
					 time_up=0;
					 //prog_no=0;
					 loop_brk=1;
					 
				 }
				 
				 if (rly_status!=1)
				 {
					 wdt_reset();
					 Relay_set(OFF,2,DIST);
					 Relay_set(ON,2,CHILLER);
				 }
				 rly_status=1;
				 
			 } while (!time_up);
			 
			 if(loop_brk!=1)
			 the_step=0;
			 
			 Relay_set(OFF,OFF,OFF);
			 prog_run=0;
			 main_menuflag=2;
			 rly_status=0;
			 the_step=0;
			 Relay_set(OFF,OFF,OFF);
			 time_up=0;
			 wdt_reset();
			 
			 if(loop_brk!=1)
			 {
				 wdt_reset();
				 lcd_write_instruction_4d(lcd_Clear);
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
				 _delay_us(80);
				 lcd_write_string_4d("--PROGRAM ENDS--");
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("WAIT:");
				 _delay_us(80);
				 
				 dly=10;
				 do
				 {
					 wdt_reset();
					 dly--;
					 LCDGotoXY(11,1);
					 lcd_write_string_4d("00:");
					 lcd_number_write(dly,10);
					 lcd_write_character_4d(' ');
					 _delay_ms(80);                     // LOADING COUNTDWN 10 SEC
					 PORTJ ^= _BV(PJ7);
					 
				 } while (!dly==0);
				 
				 CLEARBIT(PORTJ,PINJ7);
				 lcd_write_instruction_4d(lcd_Clear);
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
				 _delay_us(80);
				 lcd_write_string_4d("-SELECT PROGRAM-");
				 _delay_us(80);
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("PROG NO:");
				 _delay_us(80);
				 wdt_reset();
				 prog_no=1;
				 LCDGotoXY(8,1);
				 lcd_number_write(prog_no,10);
			 }
			 wdt_reset();
			 break;*/
			 
			 default: wdt_reset();
						break;
			 
		 }
	}while (prog_run==1);
 }
/******************************* Main Program Code *************************/
int main(void)
{   
	cli();
	wdt_reset();
	MCUSR=0;
	wdt_disable();
	
	//int i = 999;		// MAX7219 TEST

	/**********************DDR INTIALIZATION***********************************
	DATA DIRECTION REGISTERS - DDR
	0 - INPUT
	1 - OUTPUT
	
	/****************************************************************************/
	DDRA=0xFF;
	_delay_us(10);														// OUTPUT - ADDRESS BUS
	DDRB= (1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<0);						// SLAVE SELECT OUTPUT
	_delay_us(10);
	DDRC=0xFF;
	_delay_us(10);														// OUTPUT - ADDRESS BUS
	DDRD=(1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3);
    DDRE=(1<<1)|(0<<0);
	_delay_us(10);
	DDRF=0xFF;
	_delay_us(10);
	DDRH=0xFF;                                                      // TEMP LEDS
	_delay_us(10);
	DDRJ=(1<<PINJ7);													//BUZZ 7, COL 6~0
	_delay_us(10);
	DDRK=0xFF;														// DNC
	_delay_us(10);
	DDRL=0x00;														//ANALOG I/P'S
	_delay_us(10);
	
	//**************************PORT INTIALIZATION************************************
	PORTA=0x00;
	_delay_us(10);
	//PORTB=0x00;
	PORTC&=0xF0;
	_delay_us(10);
	//PORTD=0x00;
	//PORTE=0xFC;
	PORTF=0x00;
	PORTG=0x1F;
	PORTH=0x00;
	//PORTJ=0x7F;
	PORTK=0x00;
	//************************************ALL RELAYS OFF*********************************************
	rly_status=0;
	Relay_set(OFF,OFF,OFF);
	//********************************************************************************
	
	//init uart
	//uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
    //uart_puts("startup...");

	//**********************************RTC**********************************************

	//init ds1307
	ds1307_init();
		
	sei();

	//check set date
	//ds1307_setdate(16, 21, 16, 1, 16, 00);
	
/**********************************LCD INTIALIZATION*******************************************/	
	
// configure the microprocessor pins for the data lines
    lcd_D7_ddr |= (1<<lcd_D7_bit);                  // 4 data lines - output
    lcd_D6_ddr |= (1<<lcd_D6_bit);
    lcd_D5_ddr |= (1<<lcd_D5_bit);
    lcd_D4_ddr |= (1<<lcd_D4_bit);

// configure the microprocessor pins for the control lines
    lcd_E_ddr |= (1<<lcd_E_bit);                    // E line - output
    lcd_RS_ddr |= (1<<lcd_RS_bit);                  // RS line - output

// initialize the LCD controller as determined by the defines (LCD instructions)
    lcd_init_4d();                                  // initialize the LCD display for a 4-bit interface

// display the first line of information
    lcd_write_instruction_4d(lcd_Clear);
    _delay_us(80);
    lcd_write_instruction_4d(lcd_Home);
    _delay_us(80);
    lcd_write_string_4d(program_author);

// set cursor to start of second line
    lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
    _delay_us(80);                                  // 40 uS delay (min)

// display the second line of information
    lcd_write_string_4d(program_version);
	_delay_ms(150);
	
//**************************************************PAGE 2***************************************	
	lcd_write_instruction_4d(lcd_Clear);
	_delay_us(80);
	lcd_write_instruction_4d(lcd_Home);
	_delay_us(80);
	lcd_write_string_4d(program_message);                     //PLEASE WAIT MESSAGE
	_delay_ms(20);
	LCDGotoXY(2,1);
	lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
	_delay_us(80);
	LCDGotoXY(1,1);
	lcd_write_string_4d(program_status);                     // LOADING COUNTDWN 10 SEC
	_delay_ms(20);

/**************************************SPI INTIALIZATION******************************************************/

   SPI_init();
 
   // Decode mode to "Font Code-B"
   MAX7219_writeData(MAX7219_MODE_DECODE, 0X0F);
   _delay_us(10);

   // Scan limit runs from 0.
   MAX7219_writeData(MAX7219_MODE_SCAN_LIMIT, digitsInUse - 1);
   _delay_us(10);
   MAX7219_writeData(MAX7219_MODE_INTENSITY, 2);
   _delay_us(10);
   MAX7219_writeData(MAX7219_MODE_POWER, ON);
   _delay_us(10);
      
   for (uint8_t i=0;i<5;i++)
   {
	   MAX7219_writeData(i, MAX7219_CHAR_NEGATIVE);
   }


//*********************************LOADING SCREEN 10 sec********************************************	
	dly=5;
	
	do
	{   
		dly--;
		LCDGotoXY(14,1);
	    lcd_number_write(dly,10);
		_delay_ms(80);                     // LOADING COUNTDWN 10 SEC
		
	} while (!dly==0);
	
	//lcd_write_instruction_4d(lcd_Clear);
	//_delay_us(80);
	//lcd_write_instruction_4d(lcd_Home);
	//_delay_us(80);
//******************************************WATCH DOG INIT*************************************
//wdt_enable(WDTO_4S);
	WDT_Init();
/**********************************************************************************************/

// endless loop
    while(1)
	{   wdt_reset();
		
		dig_ip=0;
		//  READING PORT L FOR ERRORS
		dig_ip=PINL;
		dig_Err();                       //check error continuously 
				
		//rly_status=0;						//safety added to assure all relays OFF when in main loop
		//Relay_set(OFF,OFF,OFF);
		
		
			
	if (prog_run==0)								//if no program running
	{	
		wdt_reset();
		lcd_write_instruction_4d(lcd_Home);
		_delay_us(80);
		
		if (main_menuflag==0)
		{	
			lcd_write_instruction_4d(lcd_Clear);
			_delay_us(80);
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
			_delay_us(80);
			lcd_write_string_4d("---MAIN  MENU---");
			_delay_us(80);
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
			_delay_us(80);
			lcd_write_string_4d("MODE: 1. AUTO  ");
			_delay_us(80);
			mode_auto=1;
			mode_manu=0;
			main_menuflag=1;
			wdt_reset();
		}    // Choose mode AUTO/MANUAL
	     
		 
		menu_key=GetKeyPressed();
		wdt_reset(); 
		
		if(main_menuflag==1)
		{	
			wdt_reset();	
			switch(menu_key)
			{
				case 34: LCDGotoXY(6,1);
						 lcd_write_string_4d("2. MANUAL");
				         _delay_us(80);
						 mode_manu=1;
						 mode_auto=0;
						 wdt_reset();
						 break;
						 
				case 35: _delay_us(80);
				         LCDGotoXY(6,1);
			             lcd_write_string_4d("1. AUTO  ");
			             _delay_us(80);
						 mode_auto=1;
						 mode_manu=0;
						 wdt_reset(); 
						 break;
				
				default: wdt_reset();
							break;
			}
		}
			
		menu_key=GetKeyPressed();
		wdt_reset();
		
		if ((mode_manu==1)&&(menu_key==41))
		{
			wdt_reset();
			lcd_write_instruction_4d(lcd_Clear);
			_delay_us(80);
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
			_delay_us(80);
			lcd_write_string_4d("--MANUAL  MODE--");
			_delay_us(80);
			lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
			_delay_us(80);
			lcd_write_string_4d("****DISABLED****");
			dly=3;
			do
			{
				dly--;
				_delay_ms(80);
				wdt_reset();                     // LOADING COUNTDWN 10 SEC
				
			} while (!dly==0);
			main_menuflag=0;
		} //manual mode : DISABLED POPUP
		
		if (((mode_auto==1) && (menu_key==41))|| (bck_key==1)) 
		{   
			wdt_reset();
			bck_key=0;
			lcd_write_instruction_4d(lcd_Home);
			_delay_us(80);
			
			
			if (main_menuflag==1)
			{
				wdt_reset();
				lcd_write_instruction_4d(lcd_Clear);
				_delay_us(80);
				lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
				_delay_us(80);
				lcd_write_string_4d("-SELECT PROGRAM-");
				_delay_us(80);
				lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				_delay_us(80);
				lcd_write_string_4d("PROG NO:");
				_delay_us(80);
				prog_no=1;
				LCDGotoXY(8,1);
				lcd_number_write(prog_no,10);
				
				main_menuflag=2;
			}
		}    //if auto mode & enter is pressed --> go to main screen 2 i.e choose program no.
			
			menu_key=GetKeyPressed();
			wdt_reset();
			
			if(main_menuflag==2)                // SELECT PROG NO: 
			{   
				wdt_reset();				 
				if(menu_key==34)
				{
					prog_no++;
					_delay_ms(15);
				}
				
				if(menu_key==35)
				{
					prog_no--;
					_delay_ms(15);
				}
				
				if(prog_no>7)
				{
					prog_no=1;
					_delay_ms(15);
				}
				   
				if(prog_no<1)
				{
					prog_no=7;
					_delay_ms(15);
				}
				   
				LCDGotoXY(8,1);
				lcd_number_write(prog_no,10); 
				
				if(menu_key==38)                     // ESC to main screen
				 {
					main_menuflag=0;
				 }
				 
				 if(menu_key==32)
				 {
					_delay_ms(10);
					prog_run=1;
				 }
				 
				 /*if(menu_key==33)
				 {  
					_delay_ms(10); 
			        if(door_status==0)
					{
						doorlck_ON();
						door_status=1;	
					}
					else if(door_status==1)
					{
						doorlck_OFF();
						door_status=0;
					}
					
				 }*/
				 	  
			 }      // main_menuflag=2  //choose prog no window
		
	}  // if no program running loop
	
	wdt_reset();
	if (prog_run==1)
	{
	 	wdt_reset();
		lcd_write_instruction_4d(lcd_Clear);
	 	_delay_us(80);
		lcd_write_instruction_4d(lcd_Home);
		_delay_us(80);
		lcd_write_string_4d("P0");
		_delay_us(80);
		lcd_number_write(prog_no,10);
		_delay_us(80);

 
		 while(prog_run==1)
		 {   
			 wdt_reset();
			 dig_ip=0;
			 //  READING PORT L FOR ERRORS
			 dig_ip=PINL;
			 wdt_reset();
			 dig_Err();
			 
			 if (prog_no==1)                         // if program 1 selected
			 {	   
				   wdt_reset();
				   menu_key=GetKeyPressed();
					
					if(menu_key==34)
					{
						step_no++;
						_delay_ms(20);
					}
					
					if(menu_key==35)
					{
						step_no--;
						_delay_ms(20);
					}
					
					if(step_no>7)
					{
						step_no=1;
						_delay_ms(20);
					}
					
					if(step_no<1)
					{
						step_no=7;
						_delay_ms(20);
					}
				
					LCDGotoXY(3,0);
					_delay_us(80);
					lcd_write_string_4d("/S0");
					lcd_number_write(step_no,10);
					_delay_us(80);
					wdt_reset();
				
				if(step_no==1)
				 {
					wdt_reset();
					LCDGotoXY(11,0);
					lcd_write_string_4d("01:00");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("SOLVENT IN DRUM");
				 }
				
				if(step_no==2)
				{   
					wdt_reset();
					LCDGotoXY(11,0);
					lcd_write_string_4d("06:00");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("NORMAL WASH    ");
				}
				
				if(step_no==3)
				{
					wdt_reset();
					LCDGotoXY(11,0);
					lcd_write_string_4d("00:30");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("SOLVENT IN TANK");
				}
				
				if(step_no==4)
				{   
					wdt_reset();
					LCDGotoXY(11,0);
					lcd_write_string_4d("04:00");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("EXTRACT        ");
				}
				
				if(step_no==5)
				{
					wdt_reset();
					LCDGotoXY(11,0);
					lcd_write_string_4d("04:00");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("HEATING        ");
				}
				
				if(step_no==6)
				{
					wdt_reset();
					LCDGotoXY(11,0);
					lcd_write_string_4d("12:00");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("PERC RECOVERY  ");
				}
				
				if(step_no==7)
				{
					wdt_reset();
					LCDGotoXY(11,0);
					lcd_write_string_4d("03:00");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("COOLING        ");
				}
				
			_delay_ms(10);	
			menu_key=GetKeyPressed();
			wdt_reset();
				
			if (menu_key==32)
			{
				prog_01(step_no);
				
				if (loop_brk==1)
				{   
					wdt_reset();
					lcd_write_instruction_4d(lcd_Clear);
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
					_delay_us(80);
					lcd_write_string_4d("-PROGRAM  ABORT-");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("WAIT");
				
					dly=5;
					
					do 
					{
						wdt_reset();
						LCDGotoXY(11,1);
						wdt_reset();
						lcd_write_string_4d("00:");
						lcd_number_write(dly,10);
						lcd_write_character_4d(' ');
						_delay_ms(80);                     // LOADING COUNTDWN 3 SEC
						dly--;
						
					} while (!dly==0);
					
					
					lcd_write_instruction_4d(lcd_Clear);
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
					_delay_us(80);
					lcd_write_string_4d("-SELECT PROGRAM-");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("PROG NO:");
					_delay_us(80);
					wdt_reset();
					prog_no=1;
					LCDGotoXY(8,1);
					lcd_number_write(prog_no,10);
					loop_brk=0;
				}

			}
				
			if (menu_key==38)
			{	
				wdt_reset();
				_delay_ms(10);
				
				lcd_write_instruction_4d(lcd_Clear);
				_delay_us(80);
				lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
				_delay_us(80);
				lcd_write_string_4d("-SELECT PROGRAM-");
				_delay_us(80);
				lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				_delay_us(80);
				lcd_write_string_4d("PROG NO:");
				_delay_us(80);
				wdt_reset();
				prog_no=1;
				LCDGotoXY(8,1);
				lcd_number_write(prog_no,10);
				
				main_menuflag=2;
				prog_run=0;
				bck_key=1;
			}

			 } // end of program 1
		 
			 if(prog_no==2)
			 {
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if(menu_key==34)
				 {
					 step_no++;
					 _delay_ms(20);
				 }
				 
				 if(menu_key==35)
				 {
					 step_no--;
					 _delay_ms(20);
				 }
				 
				 if(step_no>7)
				 {
					 step_no=1;
					 _delay_ms(20);
				 }
				 
				 if(step_no<1)
				 {
					 step_no=7;
					 _delay_ms(20);
				 }
				 
				 LCDGotoXY(3,0);
				 _delay_us(80);
				 lcd_write_string_4d("/S0");
				 lcd_number_write(step_no,10);
				 _delay_us(80);
				 wdt_reset();
				 
				 if(step_no==1)
				 {
					 wdt_reset();
					 LCDGotoXY(11,0);
					 lcd_write_string_4d("01:00");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("SOLVENT IN DRUM");
				 }
				 
				 if(step_no==2)
				 {
					 wdt_reset();
					 LCDGotoXY(11,0);
					 lcd_write_string_4d("06:00");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("NORMAL WASH    ");
				 }
				 
				 if(step_no==3)
				 {
					 wdt_reset();
					 LCDGotoXY(11,0);
					 lcd_write_string_4d("00:30");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("SOLVENT IN TANK");
				 }
				 
				 if(step_no==4)
				 {
					 wdt_reset();
					 LCDGotoXY(11,0);
					 lcd_write_string_4d("04:00");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("EXTRACT        ");
				 }
				 
				 if(step_no==5)
				 {
					 wdt_reset();
					 LCDGotoXY(11,0);
					 lcd_write_string_4d("04:00");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("HEATING        ");
				 }
				 
				 if(step_no==6)
				 {
					 wdt_reset();
					 LCDGotoXY(11,0);
					 lcd_write_string_4d("12:00");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("PERC RECOVERY  ");
				 }
				 
				 if(step_no==7)
				 {
					 wdt_reset();
					 LCDGotoXY(11,0);
					 lcd_write_string_4d("03:00");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("COOLING        ");
				 }
				 
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if (menu_key==32)
				 {
					 wdt_reset();
					 //prog_02(step_no);
					 
					 if (loop_brk==1)
					 {
						 wdt_reset();
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 _delay_us(80);
						 lcd_write_string_4d("-PROGRAM  ABORT-");
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 _delay_us(80);
						 lcd_write_string_4d("WAIT");
						 
						 dly=5;
						 
						 do
						 {
							 wdt_reset();
							 LCDGotoXY(11,1);
							 lcd_write_string_4d("00:");
							 lcd_number_write(dly,10);
							 lcd_write_character_4d(' ');
							 _delay_ms(80);                     // LOADING COUNTDWN 3 SEC
							 dly--;
							 
						 } while (!dly==0);
						 
						 
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 _delay_us(80);
						 lcd_write_string_4d("-SELECT PROGRAM-");
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 _delay_us(80);
						 lcd_write_string_4d("PROG NO:");
						 _delay_us(80);
						 wdt_reset();
						 prog_no=1;
						 LCDGotoXY(8,1);
						 lcd_number_write(prog_no,10);
						 loop_brk=0;
					 }

				 }
				 
				 if (menu_key==38)
				 {
					 wdt_reset();
					 _delay_ms(10);
					 
					 lcd_write_instruction_4d(lcd_Clear);
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
					 _delay_us(80);
					 lcd_write_string_4d("-SELECT PROGRAM-");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("PROG NO:");
					 _delay_us(80);
					 prog_no=1;
					 LCDGotoXY(8,1);
					 lcd_number_write(prog_no,10);
					 
					 main_menuflag=2;
					 prog_run=0;
					 bck_key=1;
				 }

			 
			 }
			 
			 if(prog_no==3)
			 {
				 	 menu_key=GetKeyPressed();
					 wdt_reset(); 
				 	 
				 	 if(menu_key==34)
				 	 {
					 	 step_no++;
					 	 _delay_ms(20);
				 	 }
				 	 
				 	 if(menu_key==35)
				 	 {
					 	 step_no--;
					 	 _delay_ms(20);
				 	 }
				 	 
				 	 if(step_no>7)
				 	 {
					 	 step_no=1;
					 	 _delay_ms(20);
				 	 }
				 	 
				 	 if(step_no<1)
				 	 {
					 	 step_no=7;
					 	 _delay_ms(20);
				 	 }
				 	 
				 	 LCDGotoXY(3,0);
				 	 _delay_us(80);
				 	 lcd_write_string_4d("/S0");
				 	 lcd_number_write(step_no,10);
				 	 _delay_us(80);
					  wdt_reset();
				 	 
				 	 if(step_no==1)
				 	 {
					 	 wdt_reset();
						  LCDGotoXY(11,0);
					 	 lcd_write_string_4d("01:00");
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("SOLVENT IN DRUM");
				 	 }
				 	 
				 	 if(step_no==2)
				 	 {
					 	 wdt_reset();
						  LCDGotoXY(11,0);
					 	 lcd_write_string_4d("06:00");
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("NORMAL WASH    ");
				 	 }
				 	 
				 	 if(step_no==3)
				 	 {
					 	 wdt_reset();
						  LCDGotoXY(11,0);
					 	 lcd_write_string_4d("00:30");
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("SOLVENT IN TANK");
				 	 }
				 	 
				 	 if(step_no==4)
				 	 {
					 	 wdt_reset();
						  LCDGotoXY(11,0);
					 	 lcd_write_string_4d("04:00");
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("EXTRACT        ");
				 	 }
				 	 
				 	 if(step_no==5)
				 	 {
					 	 wdt_reset();
						  LCDGotoXY(11,0);
					 	 lcd_write_string_4d("04:00");
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("HEATING        ");
				 	 }
				 	 
				 	 if(step_no==6)
				 	 {
					 	 wdt_reset();
						  LCDGotoXY(11,0);
					 	 lcd_write_string_4d("12:00");
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("PERC RECOVERY  ");
				 	 }
				 	 
				 	 if(step_no==7)
				 	 {
					 	 wdt_reset();
						  LCDGotoXY(11,0);
					 	 lcd_write_string_4d("03:00");
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor|lcd_LineTwo);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("COOLING        ");
				 	 }
				 	 
				 	 
				 	 menu_key=GetKeyPressed();
					  wdt_reset();
				 	 
				 	 if (menu_key==32)
				 	 {
					 	 //prog_03(step_no);
					 	 
					 	 if (loop_brk==1)
					 	 {
						 	 wdt_reset();
							  lcd_write_instruction_4d(lcd_Clear);
						 	 _delay_us(80);
						 	 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 	 _delay_us(80);
						 	 lcd_write_string_4d("-PROGRAM  ABORT-");
						 	 _delay_us(80);
						 	 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 	 _delay_us(80);
						 	 lcd_write_string_4d("WAIT");
						 	 
						 	 dly=5;
						 	 
						 	 do
						 	 {
							 	 wdt_reset();
								  LCDGotoXY(11,1);
							 	 lcd_write_string_4d("00:");
							 	 lcd_number_write(dly,10);
							 	 lcd_write_character_4d(' ');
							 	 _delay_ms(80);                     // LOADING COUNTDWN 3 SEC
							 	 dly--;
							 	 
						 	 } while (!dly==0);
						 	 
						 	 
						 	 lcd_write_instruction_4d(lcd_Clear);
						 	 _delay_us(80);
						 	 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 	 _delay_us(80);
						 	 lcd_write_string_4d("-SELECT PROGRAM-");
						 	 _delay_us(80);
						 	 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 	 _delay_us(80);
						 	 lcd_write_string_4d("PROG NO:");
						 	 _delay_us(80);
							  wdt_reset();
						 	 prog_no=1;
						 	 LCDGotoXY(8,1);
						 	 lcd_number_write(prog_no,10);
						 	 loop_brk=0;
					 	 }

				 	 }
				 	 
				 	 if (menu_key==38)
				 	 {
					 	wdt_reset();
						  _delay_ms(10);
					 	 
					 	 lcd_write_instruction_4d(lcd_Clear);
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("-SELECT PROGRAM-");
					 	 _delay_us(80);
					 	 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
					 	 _delay_us(80);
					 	 lcd_write_string_4d("PROG NO:");
					 	 _delay_us(80);
					 	 prog_no=1;
					 	 LCDGotoXY(8,1);
					 	 lcd_number_write(prog_no,10);
					 	 
					 	 main_menuflag=2;
					 	 prog_run=0;
					 	 bck_key=1;
				 	 }
			 } // program 3 ends
			 
			 if (prog_no==4)
			 {  
				wdt_reset();
				lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				_delay_us(80);
				lcd_write_string_4d("PERC TRANSFER");
				 
				menu_key=GetKeyPressed();
				wdt_reset();
				
				if (menu_key==32)
				{
					//prog_04();
					
					if (loop_brk==1)
					{
						wdt_reset();
						lcd_write_instruction_4d(lcd_Clear);
						_delay_us(80);
						lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						_delay_us(80);
						lcd_write_string_4d("-PROGRAM  ABORT-");
						_delay_us(80);
						lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						_delay_us(80);
						lcd_write_string_4d("WAIT");
						
						dly=5;
						
						do
						{
							wdt_reset();
							LCDGotoXY(11,1);
							lcd_write_string_4d("00:");
							lcd_number_write(dly,10);
							lcd_write_character_4d(' ');
							_delay_ms(80);                     // LOADING COUNTDWN 3 SEC
							dly--;
							
						} while (!dly==0);
						
						
						lcd_write_instruction_4d(lcd_Clear);
						_delay_us(80);
						lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						_delay_us(80);
						lcd_write_string_4d("-SELECT PROGRAM-");
						_delay_us(80);
						lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						_delay_us(80);
						lcd_write_string_4d("PROG NO:");
						_delay_us(80);
						wdt_reset();
						prog_no=1;
						LCDGotoXY(8,1);
						lcd_number_write(prog_no,10);
						loop_brk=0;
					}

				}
				
				if (menu_key==38)
				{
					wdt_reset();
					_delay_ms(10);
					
					lcd_write_instruction_4d(lcd_Clear);
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
					_delay_us(80);
					lcd_write_string_4d("-SELECT PROGRAM-");
					_delay_us(80);
					lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
					_delay_us(80);
					lcd_write_string_4d("PROG NO:");
					_delay_us(80);
					wdt_reset();
					prog_no=1;
					LCDGotoXY(8,1);
					lcd_number_write(prog_no,10);
					
					main_menuflag=2;
					prog_run=0;
					bck_key=1;
				}

			 }
			 
			 if (prog_no==5)
			 {
				 wdt_reset();
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("PERC IN TANK1  ");
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if (menu_key==32)
				 {
					 //prog_05();
					 
					 if (loop_brk==1)
					 {
						 wdt_reset();
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 _delay_us(80);
						 lcd_write_string_4d("-PROGRAM  ABORT-");
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 _delay_us(80);
						 lcd_write_string_4d("WAIT");
						 
						 dly=5;
						 
						 do
						 {
							 wdt_reset();
							 LCDGotoXY(11,1);
							 lcd_write_string_4d("00:");
							 lcd_number_write(dly,10);
							 lcd_write_character_4d(' ');
							 _delay_ms(80);                     // LOADING COUNTDWN 3 SEC
							 dly--;
							 
						 } while (!dly==0);
						 
						 
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 _delay_us(80);
						 lcd_write_string_4d("-SELECT PROGRAM-");
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 _delay_us(80);
						 lcd_write_string_4d("PROG NO:");
						 _delay_us(80);
						 wdt_reset();
						 prog_no=1;
						 LCDGotoXY(8,1);
						 lcd_number_write(prog_no,10);
						 loop_brk=0;
					 }

				 }
				 
				 if (menu_key==38)
				 {
					 wdt_reset();
					 _delay_ms(10);
					 
					 lcd_write_instruction_4d(lcd_Clear);
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
					 _delay_us(80);
					 lcd_write_string_4d("-SELECT PROGRAM-");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("PROG NO:");
					 _delay_us(80);
					 prog_no=1;
					 LCDGotoXY(8,1);
					 lcd_number_write(prog_no,10);
					 
					 main_menuflag=2;
					 prog_run=0;
					 bck_key=1;
				 }
				wdt_reset();
			 }// prog 5 ends
			 
			 wdt_reset();
			 
			 if (prog_no==6)
			 {
				 wdt_reset();
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("DISTL TRANSFER");
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if (menu_key==32)
				 {
					 //prog_06();
					 
					 if (loop_brk==1)
					 {
						 wdt_reset();
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 _delay_us(80);
						 lcd_write_string_4d("-PROGRAM  ABORT-");
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 _delay_us(80);
						 lcd_write_string_4d("WAIT");
						 
						 dly=5;
						 
						 do
						 {
							 wdt_reset();
							 LCDGotoXY(11,1);
							 lcd_write_string_4d("00:");
							 lcd_number_write(dly,10);
							 lcd_write_character_4d(' ');
							 _delay_ms(80);                     // LOADING COUNTDWN 3 SEC
							 dly--;
							 
						 } while (!dly==0);
						 
						 
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 _delay_us(80);
						 lcd_write_string_4d("-SELECT PROGRAM-");
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 _delay_us(80);
						 lcd_write_string_4d("PROG NO:");
						 _delay_us(80);
						 wdt_reset();
						 prog_no=1;
						 LCDGotoXY(8,1);
						 lcd_number_write(prog_no,10);
						 loop_brk=0;
					 }

				 }
				 
				 if (menu_key==38)
				 {
					 wdt_reset();
					 _delay_ms(10);
					 
					 lcd_write_instruction_4d(lcd_Clear);
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
					 _delay_us(80);
					 lcd_write_string_4d("-SELECT PROGRAM-");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("PROG NO:");
					 _delay_us(80);
					 prog_no=1;
					 LCDGotoXY(8,1);
					 lcd_number_write(prog_no,10);
					 
					 main_menuflag=2;
					 prog_run=0;
					 bck_key=1;
				 }
				 wdt_reset();
			 }// prog 6 ends
			 
			 wdt_reset();
			 
			 if (prog_no==7)
			 {
				 wdt_reset();
				 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
				 _delay_us(80);
				 lcd_write_string_4d("DISTILLATION");
				 
				 menu_key=GetKeyPressed();
				 wdt_reset();
				 
				 if (menu_key==32)
				 {
					 //prog_07(1);
					 
					 if (loop_brk==1)
					 {
						 wdt_reset();
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 _delay_us(80);
						 lcd_write_string_4d("-PROGRAM  ABORT-");
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 _delay_us(80);
						 lcd_write_string_4d("WAIT");
						 
						 dly=5;
						 
						 do
						 {
							 wdt_reset();
							 LCDGotoXY(11,1);
							 lcd_write_string_4d("00:");
							 lcd_number_write(dly,10);
							 lcd_write_character_4d(' ');
							 _delay_ms(80);                     // LOADING COUNTDWN 3 SEC
							 dly--;
							 
						 } while (!dly==0);
						 
						 
						 lcd_write_instruction_4d(lcd_Clear);
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
						 _delay_us(80);
						 lcd_write_string_4d("-SELECT PROGRAM-");
						 _delay_us(80);
						 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
						 _delay_us(80);
						 lcd_write_string_4d("PROG NO:");
						 _delay_us(80);
						 wdt_reset();
						 prog_no=1;
						 LCDGotoXY(8,1);
						 lcd_number_write(prog_no,10);
						 loop_brk=0;
					 }

				 }
				 
				 if (menu_key==38)
				 {
					 wdt_reset();
					 _delay_ms(10);
					 
					 lcd_write_instruction_4d(lcd_Clear);
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
					 _delay_us(80);
					 lcd_write_string_4d("-SELECT PROGRAM-");
					 _delay_us(80);
					 lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
					 _delay_us(80);
					 lcd_write_string_4d("PROG NO:");
					 _delay_us(80);
					 prog_no=1;
					 LCDGotoXY(8,1);
					 lcd_number_write(prog_no,10);
					 
					 main_menuflag=2;
					 prog_run=0;
					 bck_key=1;
				 }
				 wdt_reset();
			 }
			 wdt_reset();
		 } // while prog_run ends
		 wdt_reset();
	}  //if --- program running loop
	wdt_reset();
	}    //while(1) 
    return 0;
}
/******************************* End of Main Program Code ******************/







//********************************************TRIAL RUN**************************************	
		/*lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
		_delay_us(80);
		lcd_write_string_4d("SELECT MODE");
		 _delay_us(80);
		lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		_delay_us(80);
		LCDGotoXY(4,1);
		lcd_write_string_4d("1. AUTO");
		_delay_us(80);
		
		key=GetKeyPressed();
		LCDGotoXY(0,0);
		lcd_write_string_4d("KEY:");
		lcd_number_write(key,10);
		
		
		
		/*ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
		_delay_us(80);
		
		lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		_delay_us(80);
		lcd_write_string_4d("TIME:");
		_delay_us(80);
		lcd_number_write(hour,10);
		_delay_us(80);
		lcd_write_character_4d(':');
		_delay_us(80);
		lcd_number_write(minute,10);
		_delay_us(80);
		lcd_write_character_4d(':');
		_delay_us(80);
		lcd_number_write(second,10);
		_delay_us(80);*/
		
		
		
		
		   
		
		/*lcd_write_instruction_4d(lcd_SetCursor | lcd_LineOne);
		_delay_us(80);
		
        ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
		
        lcd_number_write(hour,10);
		lcd_write_character_4d(':');
		lcd_number_write(minute,10);
		lcd_write_character_4d(':');
		lcd_number_write(second,10);
		
		key=GetKeyPressed();							//Get the keycode of pressed key
		LCDGotoXY(8,0);
		lcd_write_string_4d("KEY:");
		lcd_number_write(key,10);
	    
	    lcd_write_instruction_4d(lcd_SetCursor | lcd_LineTwo);
		_delay_us(80);
	    lcd_write_string_4d(program_date);
		_delay_ms(20);
		
		
		MAX7219_writeData(MAX7219_MODE_TEST,0x00);
		_delay_us(10);
		MAX7219_displayNumber(--i);
		lcd_write_string_4d("i:");
		lcd_number_write(i,10);
		_delay_ms(10);

		if (i == 0) {
			i = 999;
		}*/
	