#include "eeprom_vars.h"

byte readEEPROM(int address)
{
  return EEPROM.read(address);   
}

void writeEEPROM(byte value, int address)
{
  EEPROM.write(address, value);
}





