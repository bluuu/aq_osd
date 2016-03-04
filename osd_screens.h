//osd_screens.h

#include "Spi.h"

#define MAX_VID_VOLTAGE 12.6
//#define ADC_RES 1023 //external aref
#define ADC_RES 867 //default aref

typedef struct //screenCoords
{
  uint8_t col;
  uint8_t row;
  boolean isVisible;
} 
screenCoords;

typedef union
{
  double f;
  char c[4];
} 
chardouble;

typedef struct gPSPosS
{
  chardouble lat;
  chardouble lon;  
} 
gPSPos;

static screenCoords batteryVoltagePosVis; 
static screenCoords videoVoltagePosVis;
static screenCoords rssiPosVis;
static screenCoords gpsPosVis;
static screenCoords timePosVis;
static screenCoords flightTimePosVis;
static screenCoords altitudePosVis; //bezwzgledna
static screenCoords heightPosVis; //wzgledna
static screenCoords artificialHorizonPosVis;
static screenCoords homePosVis;
static screenCoords speedPosVis;
static screenCoords modePosVis;
static screenCoords compPosVis;

uint8_t gPSLLscale;
double startAlt; //wysokosc startowa (do liczenia wys. wzglednej)

chardouble cal_video_val;
chardouble cal_rssi_val_max;
chardouble cal_rssi_val_min;

static float osd_video_v = 12.6; //max

//stats
uint16_t maxHomeDistance;
uint16_t maxAltM;
uint32_t traveledDistanceM;
uint8_t minRSSI = 255;
double minVoltage = 999999.0;
double minVideoVoltage = 999999.0;

gPSPos homePosLatch = {{ 0 }, { 0 }}; //zapisanie pozycji home z GPS
boolean homePosLatched = false;
uint64_t timestampArm = 0;
boolean motorArmedLatch = false;
boolean motorArmedLatch2 = false;
boolean rssi_mode = 1; //default !=0 -> from mavlink
uint16_t failsafeCounter = 0;

int8_t timeShift;
uint16_t timeOfFlightFrozen;

