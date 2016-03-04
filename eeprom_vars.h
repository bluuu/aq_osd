//Adresy
#define PAL_NTSC_ADDR 2

//pomiar wartosci analogowych?
#define ANALOG_MEASURES 0xA

//uklad ekranu (zmienne):
#define BATTERY_VOLTAGE_POS_VIS 0xC
#define VIDEO_VOLTAGE_POS_VIS 0xF
#define RSSI_POS_VIS 0x12
#define GPS_POS_VIS 0x15
#define TIME_POS_VIS 0x18 //utc
#define FLIGHT_TIME_POS_VIS 0x1B
#define ALTITUDE_POS_VIS 0x1E
#define HEIGHT_POS_VIS 0x21
#define ARTIF_HORIZON_POS_VIS 0x24
#define HOME_POS_VIS 0x27
#define SPEED_POS_VIS 0x2A

#define GPS_LL_SCALE 0x37

#define CALI_VIDEO_VALUE 0x38 //float
#define CALI_RSSI_VALUE_MAX 0x42 //float
#define CALI_RSSI_VALUE_MIN 0x46 //float
#define RSSI_MODE 0x50
#define MOD 0x51
#define TIMESHIFT 0x55
//exp
#define COMP 0x58


byte readEEPROM(int address);
void writeEEPROM(byte value, int address);


