const uint16_t sig_pattern[15][4] = {
  //    LED1         LED2       LED3(unused)      BUZZ
  { 0b0000000000, 0b0000000000, 0b0000000000, 0b0000000000}, 	// 0 (All-Off)
  { 0b1100110011, 0b0000000000, 0b0000000000, 0b0000000000}, 	// 1 (Disarmed)
  { 0b1111111111, 0b0000000000, 0b0000000000, 0b0000000000}, 	// 2 (Init) <not used>	
  { 0b1111111111, 0b0000000000, 0b0000000000, 0b0000000000}, 	// 3 (Armed)
  { 0b1111111111, 0b0000000000, 0b0000000000, 0b0000000000}, 	// 4 (Manual)
  { 0b1111111111, 0b1000010000, 0b0000000000, 0b0000000000}, 	// 5 (Alt hold)		
  { 0b1111111111, 0b1111111111, 0b0000000000, 0b0000000000}, 	// 6 (Pos hold)		
  { 0b1111111111, 0b0000010000, 0b0000000000, 0b1000000000}, 	// 7 (Mission)
  { 0b1111111111, 0b0101010101, 0b0000000000, 0b1000000000}, 	// 8 (DVH) <not used>
  { 0b1100000000, 0b0000000011, 0b0000000000, 0b1001000000}, 	// 9 (Low batt 1)
  { 0b0100110010, 0b1001001000, 0b0000000000, 0b1100001100},	// 10 (Radio loss 1/Low batt 2)
  { 0b0100110010, 0b1001001000, 0b0000000000, 0b1101011010},	// 11 (Radio loss 2) <not used>
  { 0b1100000000, 0b0011000000, 0b0000000000, 0b0000000110},	// 12 (RTH)
  { 0b1111111111, 0b1111100000, 0b0000000000, 0b0010000000}, 	// 13 (Arming)		
  { 0b1111111111, 0b0000011111, 0b0000000000, 0b0010100000}, 	// 14 (Disarming)	  
};

//GPS LED
const uint16_t gps_sig_pattern[4] = {0b0000000000, 0b1100000000, 0b1100110011, 0b1111111111};

const uint8_t Out[] = {l1_led,l2_led,gps_led,aux_led,buzzer};
static uint8_t ioCounter = 0;
static uint8_t ioCounterGps = 0;

#define LED1 0
#define LED2 1
#define LED3 2
#define LED4 3
#define BUZZ 4


void signalingWriteLeds(uint8_t statusId, uint8_t gpsState) 
{
  static uint8_t previousStatusId = statusId;
  
  if(previousStatusId != statusId)
  	ioCounter = 0;  	
  
  digitalWriteFast (Out[LED1], (sig_pattern[statusId][0] >> ioCounter) & 1);       
  digitalWriteFast (Out[LED2], (sig_pattern[statusId][1] >> ioCounter) & 1);
  digitalWriteFast (Out[LED3], (gps_sig_pattern[gpsState] >> ioCounterGps) & 1); //GPS  
  //digitalWrite (Out[LED4], (sig_pattern[statusId][2] >> ioCounter) & 1);
  digitalWriteFast (Out[BUZZ], (sig_pattern[statusId][3] >> ioCounter) & 1);
  
  if(ioCounter++ == 9)
    ioCounter = 0;

  if(ioCounterGps++ == 9)
  	ioCounterGps = 0;
    
  previousStatusId = statusId;
}
