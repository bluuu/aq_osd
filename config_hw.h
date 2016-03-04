//Inicjalizacja I/O
#define debug_led 4
#define l1_led A3
#define l2_led A4
#define gps_led A5
#define aux_led 9
#define buzzer 8
#define MAX7456_SELECT 6

#define DIGITALIO_NO_INTERRUPT_SAFETY
#define DIGITALIO_NO_MIX_ANALOGWRITE
#include "digitalIOPerformance.h"

#define uart_speed 115200

void io_init () {
  pinModeFast(debug_led, OUTPUT);
  pinModeFast(l1_led, OUTPUT);
  pinModeFast(l2_led, OUTPUT);
  pinModeFast(gps_led, OUTPUT);
  pinModeFast(aux_led, OUTPUT);
  pinModeFast(buzzer, OUTPUT);
  
//  pinModeFast(MAX7456_SELECT, OUTPUT);
  
  pinModeFast(A0, INPUT_PULLUP);
  pinModeFast(A2, INPUT);
}


