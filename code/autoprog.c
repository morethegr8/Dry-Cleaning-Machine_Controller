/*
 * autoprog.c
 *
 * Created: 10/25/2016 11:48:14 PM
 *  Author: Shailesh
 */ 

#include <avr/io.h>
#include "peri/rtc/ds1307.h"
#include "peri/rtc/i2c/i2cmaster.h"
#include "peri/lcd/lcd_NAPL.h"
#include <util/delay.h>

uint8_t dig_ip1=0,prog=1,dly1=0;

uint8_t year = 0;
uint8_t month = 0;
uint8_t day = 0;
uint8_t hour = 0;
uint8_t minute = 0,set_min=0,temp1,temp2;
uint8_t second = 0,set_sec=0;





void prog_01(uint8_t the_step)
{
	
		dig_ip1=0;
		//  READING PORT L FOR ERRORS
		dig_ip1=PINL;
		
		dig_Err();	
				
				dly1=3;
				do
				{
					dly1--;
					LCDGotoXY(11,0);
					lcd_write_string_4d("00:0");
					lcd_number_write(dly1,10);
					_delay_ms(1000);                     // LOADING COUNTDWN 10 SEC
					
				} while (!dly1==0);
		
		do 
		{
				switch(the_step)
				{ 
					case 1: ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
							_delay_us(80);
					        set_min=1;
							set_sec=0;
				            set_min=minute+set_min;
							set_sec=second+set_sec;
							 
                            do 
							{
								dig_ip1=0;
								//  READING PORT L FOR ERRORS
								dig_ip1=PINL;
								
								dig_Err();
								
								ds1307_getdate(&year, &month, &day, &hour, &minute, &second);
								_delay_us(80);
								
								LCDGotoXY(9,0);
								lcd_number_write(hour,10);
								lcd_write_character_4d(':');
								lcd_number_write(minute,10);
								lcd_write_character_4d(':');
								lcd_number_write(second,10);
								
								
								
								/*temp1=set_min-minute;
								temp2=set_sec-second;
								
								//lcd_write_instruction_4d(lcd_Home);
								LCDGotoXY(11,0);
								_delay_us(80);
								lcd_number_write(minute,10);
								_delay_us(80);
								lcd_write_character_4d(":");
								_delay_us(80);
								lcd_number_write(second,10);*/
								
								
							} while ((set_min==minute)&&(set_sec==second));
					        //LCDGotoXY(11,0);
							//lcd_write_string_4d("00:0");
							
					        _delay_ms(80);
							break;
								
					case 2: LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							//lcd_number_write(step_no,10);
							_delay_ms(80);
							break;
							
					case 3: LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							//lcd_number_write(step_no,10);
							_delay_ms(80);
							break;				
							
					case 4: LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							//lcd_number_write(step_no,10);
							_delay_ms(80);
							break;
							
					case 5: LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							//lcd_number_write(step_no,10);
							_delay_ms(80);
							break;	
							
					case 6: LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							//lcd_number_write(step_no,10);
							_delay_ms(80);
							break;
							
					case 7: LCDGotoXY(3,0);
							_delay_us(80);
							lcd_write_string_4d("/S0");
							//lcd_number_write(step_no,10);
							_delay_ms(80);
							
							break;
							
					default: break;							
											
				}
			
		} while (prog==1);
		

}
