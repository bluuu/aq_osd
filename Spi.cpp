// Get the common arduino functions
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif
#include "Spi.h"

#define DIGITALIO_NO_INTERRUPT_SAFETY
#define DIGITALIO_NO_MIX_ANALOGWRITE
#include "digitalIOPerformance.h"

//---------- constructor ----------------------------------------------------

SPI::SPI()
{
  // initialize the SPI pins
  pinModeFast(SCK_PIN, OUTPUT);
  pinModeFast(MOSI_PIN, OUTPUT);
  pinModeFast(MISO_PIN, INPUT);
  pinModeFast(SS_PIN, OUTPUT); // <------- !!! (Remember! This pin will select USB host chip Max3421)

  // enable SPI Master, MSB, SPI mode 0, FOSC/4
  mode(0);
}

//------------------ mode ---------------------------------------------------

void SPI::mode(byte config)
{
  byte tmp;

  // enable SPI master with configuration byte specified
  SPCR = 0;
  SPCR = (config & 0x7F) | (1<<SPE) | (1<<MSTR);
  tmp = SPSR;
  tmp = SPDR;
}

//------------------ transfer -----------------------------------------------

byte SPI::transfer(byte value)
{
  SPDR = value;
  while (!(SPSR & (1<<SPIF))) ;
  return SPDR;
}

byte SPI::transfer(byte value, byte period)
{
  SPDR = value;
  if (period > 0) delayMicroseconds(period);
  while (!(SPSR & (1<<SPIF))) ;
  return SPDR;
}


//---------- preinstantiate SPI object --------------------------------------

SPI Spi = SPI();
