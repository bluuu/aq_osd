/*

 Copyright (c) 2011.  All rights reserved.
 An Open Source Arduino based OSD and Camera Control project.
 
 Program  : ArduCAM-OSD (MinimOSD [and variants] Firmware)
 Version  : V2.2, May 8th 2014
 Author(s): Sandro Benigno
 Coauthor(s):
 Jani Hirvinen   (All the EEPROM routines)
 Michael Oborne  (OSD Configutator)
 Zóltan Gábor, Pedro Santos and MinimOSD-Extra Team (Extra OSD Tools/Panels)
 Mike Smith      (BetterStream and Fast Serial libraries)
 Special Contribuitor:
 Andrew Tridgell by all the support on MAVLink
 Doug Weibel by his great orientation since the start of this project
 Contributors: James Goppert, Max Levine
 and all other members of DIY Drones Dev team
 Thanks to: Chris Anderson and Jordi Munoz
 
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 
 */

/* ************************************************************ */
/* **************** MAIN PROGRAM - MODULES ******************** */
/* ************************************************************ */

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 


/* **********************************************/
/* ***************** INCLUDES *******************/
// AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
//#include <AP_Math.h>
//#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "wiring.h"
#endif
#include <EEPROM.h>
#include <SimpleTimer.h>

#include <GCS_MAVLink.h>

#ifdef membug
#include <MemoryFree.h>
#endif

// Configurations
#include "ArduCam_Max7456.h"
#include "config_hw.h"
#include "eeprom_vars.h"

// experimental new libs
#define DIGITALIO_NO_INTERRUPT_SAFETY
#define DIGITALIO_NO_MIX_ANALOGWRITE
#include "digitalIOPerformance.h"

extern uint8_t osd_fix_type;
extern uint32_t custom_mode;
extern int16_t osd_rssi;
extern double osd_vbat_A;
extern uint16_t osd_chan7_raw;
extern boolean rtcSet;

extern uint16_t timeOfFlightFrozen; 
extern uint64_t osd_time;
extern uint64_t timestampArm;
uint8_t countdownCounter = 10;
extern char osd_text[51];

// Objects and Serial definitions
FastSerialPort0(Serial);
OSD osd; //OSD object 

SimpleTimer timer;
SimpleTimer timer2;

uint8_t screenFlag = 3;
boolean firstTimeArmed = false; //czy od wlaczenia/resetu uzbrojono choc raz silniki?
boolean motor_armed_flag = false;
uint8_t cArmingSeq = 0;
uint8_t cDisarmingSeq = 10;

boolean somethingWasReadFromMavlink = false;

void mainScreenEnable()
{
  screenFlag = 0;
  osd.clear();
}

void statisticsScreenEnable()
{
  screenFlag = 2;
  osd.clear();
  timeOfFlightFrozen = (osd_time-timestampArm);
}

void updateCountdownScreen()
{
  osd.setPanel(23,10);
  osd.openPanel();
  osd.printf("%d", --countdownCounter);
  osd.closePanel();
}
 
void ledUpdate()
{
  uint8_t value = 0;
 
  if(!motor_armed_flag)
    {             
      if(cDisarmingSeq == 10)
      {
        value = 1;        
      }
      else
      {
        value = 14; //disarming
        cDisarmingSeq++;
      }
      cArmingSeq = 0;
    }
    else
    {
      if(cArmingSeq == 10)
      {
        value = 3;      
        
        if(custom_mode & (1<<1)) //manual
        {
          value = 4;
        }        
        else if(custom_mode & (1<<2)) //althold
        {  
          value = 5;  
        }
        else if(custom_mode & (1<<3)) //poshold
        {  
            value = 6;  
        }
        else if(custom_mode & (1<<5)) //mission
        {  
          value = 7;  
        }
        
        if(osd_chan7_raw < 675) //rth
        {
          value = 12;
        }       
        
        if(osd_rssi < 20) //lower than 20%
            value = 10;
            
        if(osd_vbat_A < 11) 
            value = 9;  
      }        
      else
      {
        value = 13; //arming
        cArmingSeq++; 
      }
       
      cDisarmingSeq = 0;     
    }  

    signalingWriteLeds(value, osd_fix_type);     
}

void setup()
{
//wylaczenie dla malego OSD//  io_init(); //zainicjowanie wejsc i wyjsc 
//  analogReference(EXTERNAL);
//wylaczenie dla malego OSD  digitalWriteFast(debug_led,HIGH);
  Serial.begin(uart_speed); //start UART
  Serial.flush();
  mavlink_comm_0_port = &Serial;
  digitalWriteFast(MAX7456_SELECT, HIGH);    
  osd.init(); //inicjalizacja ekranu
  
//wylaczenie dla malego OSD  timer.Set(ledUpdate, 100);
//wylaczenie dla malego OSD  timer.Enable();
  
  timer2.Set(updateCountdownScreen, 1000);
  timer2.Enable();
   
  detect_screen();
  countdown_screen();  
  
  getCoordsFromEeprom();
//wylaczenie dla malego OSD  digitalWriteFast(debug_led,LOW);

}
 
void loop()
{  
  if(read_mavlink())
    somethingWasReadFromMavlink = true;
  
  timer2.Run();
  
  if(screenFlag == 1)
  {   
  }
  else if(screenFlag == 0)
  {  
    timer.Run();
    main_screen(); //ekran startowy
    if(!motor_armed_flag) 
    {
      if(firstTimeArmed) 
      {
        statisticsScreenEnable();
      }
    }
    else
    {
      firstTimeArmed = true;    
    }
    delay(100);
  }
  else if(screenFlag == 2)
  {   
//wylaczenie dla malego OSD    timer.Run();
    stat_screen();
    
    if(motor_armed_flag)
    {
      mainScreenEnable();
    }
    delay(100);
  }
  else if(screenFlag == 3)
  {      
    if(!countdownCounter)
    {
       timer2.Disable();       
  
       if(!somethingWasReadFromMavlink)
       { 
         error_screen();
         delay(500);          
       }     
       else
       {
         screenFlag = 0;    
       }     
    }
  }  
}

