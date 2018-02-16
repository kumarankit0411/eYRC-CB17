/********************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates simple motion control.

 Concepts covered: Simple motion control

 There are two components to the motion control:
 1. Direction control using pins PORTA0 to PORTA3
 2. Velocity control by PWM on pins PL3 and PL4 using OC5A and OC5B of timer 5.

 In this experiment for the simplicity PL3 and PL4 are kept at logic 1.
 
 Pins for PWM are kept at logic 1.
  
 Connection Details:    L-1---->PA0;    L-2---->PA1;
              R-1---->PA2;    R-2---->PA3;
              PL3 (OC5A) ----> Logic 1;   PL4 (OC5B) ----> Logic 1; 


 Note: 
 
 1. Make sure that in the configuration options following settings are 
  done for proper operation of the code

  Microcontroller: atmega2560
    Frequency: 14745600
  Optimization: -Os (For more information read section: Selecting proper optimization 
          options below figure 2.22 in the Software Manual)

 2. Auxiliary power can supply current up to 1 Ampere while Battery can supply current up to 
  2 Ampere. When both motors of the robot changes direction suddenly without stopping, 
  it produces large current surge. When robot is powered by Auxiliary power which can supply
  only 1 Ampere of current, sudden direction change in both the motors will cause current 
  surge which can reset the microcontroller because of sudden fall in voltage. 
  It is a good practice to stop the motors for at least 0.5seconds before changing 
  the direction. This will also increase the useable time of the fully charged battery.
  the life of the motor.
*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
   For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode


********************************************************************************/

#include <UART.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Function to initialize ports
void port_init()
{
 motion_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
  TCCR5B = 0x00;  //Stop
  TCNT5H = 0xFF;  //Counter higher 8-bit value to which OCR5xH value is compared with
  TCNT5L = 0x01;  //Counter lower 8-bit value to which OCR5xH value is compared with
  OCR5AH = 0x00;  //Output compare register high value for Left Motor
  OCR5AL = 0xFF;  //Output compare register low value for Left Motor
  OCR5BH = 0x00;  //Output compare register high value for Right Motor
  OCR5BL = 0xFF;  //Output compare register low value for Right Motor
  OCR5CH = 0x00;  //Output compare register high value for Motor C1
  OCR5CL = 0xFF;  //Output compare register low value for Motor C1
  TCCR5A = 0xA9;  /*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
            For Overriding normal port functionality to OCRnA outputs.
              {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
  
  TCCR5B = 0x0B;  //WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
  OCR5AL = (unsigned char)left_motor;
  OCR5BL = (unsigned char)right_motor;
} 

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F;       // removing upper nibbel as it is not needed
 PortARestore = PORTA;      // reading the PORTA's original status
 PortARestore &= 0xF0;      // setting lower direction nibbel to 0
 PortARestore |= Direction;   // adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore;      // setting the command to the port
}

//ISR(INT7_vect)
//{
  //boot_key_press=1;
//}


void forward (void) //both wheels forward
{
  motion_set(0x06);
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void stop (void) //hard stop
{
  motion_set(0x00);
}

void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 timer5_init();
 uart0_init();
 sei(); //Enables the global interrupts
}

int main()
{
  init_devices();
  int rec;
  int vel[2], cur=0,v=0, neg=0;
  while(true)
  {
    while(true)
    {
      rec = (int)uart_rx();
      uart_tx(rec);
      if (rec == 0x2D)
      {
        neg=1;
      }
      if (rec >= 0x30 && rec <= 0x39)  
      {
        v = (v*10)+(rec-48);
      }  
      else if (rec == 0x2C)
      {
        cur=1;
        if (neg==1)
          vel[0]=-v;
        else
          vel[0] = v;  
        v=0;
        neg=0;
      }
      else if (rec == 0x5D)
      {
        if (neg==1)
          vel[1]=-v;
        else
          vel[1] = v;
        cur=0;
        v=0;
        neg=0;
        break;
      } 
    }
    velocity(abs(vel[0]), abs(vel[1]));
    if (vel[0]==0 and vel[1]==0)
      stop();
    else if (vel[0]>=0 && vel[1]>=0)
    {
      forward();
      _delay_ms(1);
    }  
    else if (vel[0]<0 && vel[1]>=0)
    {
      left();
      _delay_ms(1);
    }
    else if (vel[0]>=0 && vel[1]<0)
    { 
      right();
      _delay_ms(1);
    }
    else if (vel[0]<0 && vel[1]<0)
    { 
      back(); 
      _delay_ms(1);
    }  
  }
}
  
/*
//Main Function
int main()
{
  init_devices();
  char rec;
  velocity (150, 180);
  //while(boot_key_press==0);
  while(1)
  {
    rec = uart_rx();
    uart_tx(rec);
    if (rec == 's')
    {
      stop();
      _delay_ms(500);
      forward(); //both wheels forward
    }  
    else if (rec == '4')
    {
      stop();           
    } 
    else if (rec == '8')
    {
      stop();
      _delay_ms(500); 
      back(); //bpth wheels backward            
    } 
    else if (rec == 'd')
    {
      stop();
      _delay_ms(500);
      left(); //Left wheel backward, Right wheel forward
    }  
    else if (rec == 'a')
    { 
      stop();
      _delay_ms(500);
      right(); //Left wheel forward, Right wheel backward
    }   
  }
}*/

