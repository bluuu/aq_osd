//osd_screens.ino

#include "osd_screens.h"

void countdown_screen(void)
{
  osd.setPanel(5,7);
  osd.openPanel();
  osd.printf_P(PSTR("\x88\x89\x8a\x8b|\x8c\x8d\x8e\x8f by bluuu v1.1 beta|\xb0\xb1\xb2\xb3"));
  osd.closePanel();
  
  osd.setPanel(5,5);
  osd.openPanel();
  osd.printf_P(PSTR("AQ OSD Generator"));
  osd.closePanel();
  
  osd.setPanel(5,10);
  osd.openPanel();
  osd.printf_P(PSTR("Initializing ..."));
  osd.closePanel();
}

void error_screen(void)
{
  osd.clear();
  osd.setPanel(2,6);
  osd.openPanel();
  osd.printf_P(PSTR("No telemetry data from AQ!"));  
  osd.closePanel();
}

void detect_screen(void)
{
  osd.setPanel(5,3);
  osd.openPanel();

  if (osd.getMode() == 1)
  { //PAL
    osd.printf_P(PSTR("System mode: PAL"));
  }
  else 
  { //NTSC
    osd.printf_P(PSTR("System mode: NTSC"));
  }
  osd.closePanel();
}

void getCoordsFromEeprom(void)
{
  batteryVoltagePosVis.col = readEEPROM(BATTERY_VOLTAGE_POS_VIS);
  batteryVoltagePosVis.row = readEEPROM(BATTERY_VOLTAGE_POS_VIS+1);
  batteryVoltagePosVis.isVisible = readEEPROM(BATTERY_VOLTAGE_POS_VIS+2);
  videoVoltagePosVis.col = readEEPROM(VIDEO_VOLTAGE_POS_VIS);
  videoVoltagePosVis.row = readEEPROM(VIDEO_VOLTAGE_POS_VIS+1);
  videoVoltagePosVis.isVisible = readEEPROM(VIDEO_VOLTAGE_POS_VIS+2);  
  rssiPosVis.col = readEEPROM(RSSI_POS_VIS);
  rssiPosVis.row = readEEPROM(RSSI_POS_VIS+1);
  rssiPosVis.isVisible = readEEPROM(RSSI_POS_VIS+2);  
  gpsPosVis.col = readEEPROM(GPS_POS_VIS);
  gpsPosVis.row = readEEPROM(GPS_POS_VIS+1);
  gpsPosVis.isVisible = readEEPROM(GPS_POS_VIS+2);  
  timePosVis.col = readEEPROM(TIME_POS_VIS);
  timePosVis.row = readEEPROM(TIME_POS_VIS+1);
  timePosVis.isVisible = readEEPROM(TIME_POS_VIS+2);  
  flightTimePosVis.col = readEEPROM(FLIGHT_TIME_POS_VIS);
  flightTimePosVis.row = readEEPROM(FLIGHT_TIME_POS_VIS+1);
  flightTimePosVis.isVisible = readEEPROM(FLIGHT_TIME_POS_VIS+2);  
  altitudePosVis.col = readEEPROM(ALTITUDE_POS_VIS);
  altitudePosVis.row = readEEPROM(ALTITUDE_POS_VIS+1);
  altitudePosVis.isVisible = readEEPROM(ALTITUDE_POS_VIS+2);  
  heightPosVis.col = readEEPROM(HEIGHT_POS_VIS);
  heightPosVis.row = readEEPROM(HEIGHT_POS_VIS+1);
  heightPosVis.isVisible = readEEPROM(HEIGHT_POS_VIS+2);  
  artificialHorizonPosVis.col = readEEPROM(ARTIF_HORIZON_POS_VIS);
  artificialHorizonPosVis.row = readEEPROM(ARTIF_HORIZON_POS_VIS+1);
  artificialHorizonPosVis.isVisible = readEEPROM(ARTIF_HORIZON_POS_VIS+2);  
  homePosVis.col = readEEPROM(HOME_POS_VIS);
  homePosVis.row = readEEPROM(HOME_POS_VIS+1);
  homePosVis.isVisible = readEEPROM(HOME_POS_VIS+2);  
  speedPosVis.col = readEEPROM(SPEED_POS_VIS);
  speedPosVis.row = readEEPROM(SPEED_POS_VIS+1);
  speedPosVis.isVisible = readEEPROM(SPEED_POS_VIS+2);  
  modePosVis.col = readEEPROM(MOD);
  modePosVis.row = readEEPROM(MOD+1);
  modePosVis.isVisible = readEEPROM(MOD+2);
  
  compPosVis.col = readEEPROM(COMP);
  compPosVis.row = readEEPROM(COMP+1);
  compPosVis.isVisible = readEEPROM(COMP+2);
  
  gPSLLscale = readEEPROM(GPS_LL_SCALE); 
  
  cal_video_val.c[0] = readEEPROM(CALI_VIDEO_VALUE);
  cal_video_val.c[1] = readEEPROM(CALI_VIDEO_VALUE+1);
  cal_video_val.c[2] = readEEPROM(CALI_VIDEO_VALUE+2);
  cal_video_val.c[3] = readEEPROM(CALI_VIDEO_VALUE+3);
  
  cal_rssi_val_max.c[0] = readEEPROM(CALI_RSSI_VALUE_MAX);
  cal_rssi_val_max.c[1] = readEEPROM(CALI_RSSI_VALUE_MAX+1);
  cal_rssi_val_max.c[2] = readEEPROM(CALI_RSSI_VALUE_MAX+2);
  cal_rssi_val_max.c[3] = readEEPROM(CALI_RSSI_VALUE_MAX+3);
  
  cal_rssi_val_min.c[0] = readEEPROM(CALI_RSSI_VALUE_MIN);
  cal_rssi_val_min.c[1] = readEEPROM(CALI_RSSI_VALUE_MIN+1);
  cal_rssi_val_min.c[2] = readEEPROM(CALI_RSSI_VALUE_MIN+2);
  cal_rssi_val_min.c[3] = readEEPROM(CALI_RSSI_VALUE_MIN+3);
  
  rssi_mode = readEEPROM(RSSI_MODE); 
  
  timeShift = readEEPROM(TIMESHIFT);
}

void setPaneOpenPanel(uint8_t x, uint8_t y)
{
  osd.setPanel(x,y);
  osd.openPanel();
}

void stat_screen(void)
{
  int iter = 2;
  
  osd.clear();
   
  setPaneOpenPanel(9,0);
  osd.printf_P(PSTR("Statistics"));
  osd.closePanel();
  
  if(flightTimePosVis.isVisible)
  {
    
    setPaneOpenPanel(0,1);
    osd.printf("Time of flight: %u sec", (unsigned)timeOfFlightFrozen);
    osd.closePanel();
  }
  if(homePosVis.isVisible)
  {
    setPaneOpenPanel(0,2);
    osd.printf("Max dist from HOME: %ukm", maxHomeDistance);
    osd.closePanel(); 
 
    iter++;
  }
  
  setPaneOpenPanel(0,iter++);
  osd.printf("Max altitude: %um", maxAltM);
  osd.closePanel();
  
  setPaneOpenPanel(0,iter++);
  osd.printf("Traveled distance: %um", (unsigned)traveledDistanceM);
  osd.closePanel();

  setPaneOpenPanel(0,iter++);
  osd.printf("Min. RSSI: %u%%", minRSSI);
  osd.closePanel();
  
  setPaneOpenPanel(0,iter++);
  osd.printf("Min. voltage: %.2fv", minVoltage);
  osd.closePanel();
  
  setPaneOpenPanel(0,iter++);
  osd.printf("Min. video voltage: %.2fv", minVideoVoltage);
  osd.closePanel();
  
}

void main_screen(void)
{
  uint64_t h, m, s;
  double prevDistance = 0;

  char string2display[20] = {}; 
  osd.clear();
  
//  if(compPosVis.isVisible)
//    showYaw(compPosVis.col,compPosVis.row);
    showYaw(12,14);
  
  if(artificialHorizonPosVis.isVisible)
    showHorizon(artificialHorizonPosVis.col, artificialHorizonPosVis.row);
  
  if(batteryVoltagePosVis.isVisible)
  {
    osd.setPanel(batteryVoltagePosVis.col, batteryVoltagePosVis.row);
    osd.openPanel();
    osd.printf_P(PSTR("\x18"));
    osd.printf("%2.2fv", osd_vbat_A);
    osd.closePanel();
    if(osd_vbat_A < minVoltage)
      minVoltage = osd_vbat_A;
  }
  
  if(videoVoltagePosVis.isVisible)
  {
    osd.setPanel(videoVoltagePosVis.col, videoVoltagePosVis.row);
    osd.openPanel();
    uint16_t tmp = analogRead(A2);
//    osd_video_v = ((double)tmp*MAX_VID_VOLTAGE*cal_video_val.f)/ADC_RES; //FIXME    
    osd_video_v = (tmp*MAX_VID_VOLTAGE*cal_video_val.f)/ADC_RES; //FIXME    
//    osd_video_v = tmp; //FIXME    
    delay(1);//aby kolejny pomiar adc ponizej nie byl zaklocony
    osd.printf_P(PSTR("\x17"));
    osd.printf("%2.2fv", osd_video_v);
    osd.closePanel(); 
    if(osd_video_v < minVideoVoltage)
      minVideoVoltage = osd_video_v;
  }

  if(rssiPosVis.isVisible)
  {
          osd.setPanel(rssiPosVis.col, rssiPosVis.row);
      osd.openPanel();
    //wariant    
    if(!rssi_mode)
    {
      osd_rssi = analogRead(A0);
      osd_rssi = ((osd_rssi*100ul)-((uint32_t)(ADC_RES*cal_rssi_val_min.f)))/(uint32_t)(ADC_RES*cal_rssi_val_max.f);//*cal_rssi_val; //FIXME     
    }
    if(osd_rssi > 100)
    {
      osd.printf("\x09 invalid");    
    }
    else
    {
      osd.printf("\x09 %d%%", osd_rssi);  
    }
    osd.closePanel(); 
        
    if(osd_rssi < minRSSI)
      minRSSI = osd_rssi;  
  }

  if(gpsPosVis.isVisible)
  {
    gPSPos tmp;
    tmp.lat.f = osd_lat;
    tmp.lon.f = osd_lon;
    double tmpCurrDistance = 0;
    
    osd.setPanel(gpsPosVis.col, gpsPosVis.row);
    osd.openPanel();
    osd.printf_P(PSTR("\x86 "));
    
    if(osd_fix_type < 2)
      osd.printf("no fix");
    else if(osd_fix_type == 2)
      osd.printf("2d fix");
    else if(osd_fix_type == 3)
      osd.printf("3d fix");     
      
    if((osd_fix_type >= 2) && !homePosLatched)  
    {
      homePosLatched = true;
      homePosLatch.lat.f = osd_lat;
      homePosLatch.lon.f = osd_lon;
    }

    displayCoordsDegOrMin(osd_lat, osd_lon);
    
    osd.closePanel();     
    
    tmpCurrDistance = calculateDistanceBeetwCoords(homePosLatch, tmp);
    
    if(tmpCurrDistance != prevDistance)    
      traveledDistanceM += abs(tmpCurrDistance - prevDistance);
    prevDistance = tmpCurrDistance;
  }
  
  if(timePosVis.isVisible)
  {
    osd.setPanel(timePosVis.col, timePosVis.row);
    osd.openPanel();   
    
    if(rtcSet)
    {
      h = (baseTime.hour + timeShift + (osd_time-frozenRelativeTime)/3600ull)%24ull;      
      m = (baseTime.minute + (osd_time-frozenRelativeTime)/60ull)%60ull;

      osd.printf("\x08%02u:%02u", (unsigned)h, (unsigned)m);
    }
    else
      osd.printf("\x08No rtc");
    osd.closePanel();
  }
  
  if(flightTimePosVis.isVisible)
  {
    if(motor_armed_flag && !motorArmedLatch)
    {
      timestampArm = osd_time;
      motorArmedLatch = true;
    }
    else if(!motor_armed_flag && motorArmedLatch)
    {
      motorArmedLatch = false;
    }
    osd.setPanel(flightTimePosVis.col, flightTimePosVis.row);
    osd.openPanel();
    if(timestampArm == 0)
      osd.printf_P(PSTR("\x02""00:00"));
    else
    {     
      s = (osd_time-timestampArm)%60ull;       
      m = (osd_time-timestampArm)/60ull;

      osd.printf("\x02""%02u:%02u", (unsigned)m, (unsigned)s);
    }    
    osd.closePanel();     
  }

  if(altitudePosVis.isVisible) //bezwzgledna
  {
    osd.setPanel(altitudePosVis.col, altitudePosVis.row);
    osd.openPanel();
    osd.printf_P(PSTR("\x12"));
    osd.printf("%4.1f", osd_alt);
    osd.closePanel();
    if(maxAltM < osd_alt)
      maxAltM = osd_alt;  
  }
    
  if(heightPosVis.isVisible); //wzgledna
  {
    if(motor_armed_flag && !motorArmedLatch2)
    {
      startAlt = osd_alt;
      motorArmedLatch2 = true;
    }
    else if(!motor_armed_flag && motorArmedLatch2)
    {
      motorArmedLatch2 = false;
    }
    osd.setPanel(heightPosVis.col, heightPosVis.row);
    osd.openPanel();
    osd.printf_P(PSTR("\x14"));
    osd.printf("%4.1f", osd_alt-startAlt);
    osd.closePanel();
  }

  if(homePosVis.isVisible)
  {
    double distance;
    gPSPos tmp;
    tmp.lat.f = osd_lat;
    tmp.lon.f = osd_lon;
    
    distance = calculateDistanceBeetwCoords(homePosLatch, tmp);

    osd.setPanel(homePosVis.col, homePosVis.row);
    osd.openPanel();
    osd.printf("\x0b%5.2f\x1b", distance);
  
    osd.closePanel();
    
    if(maxHomeDistance < distance)
      maxHomeDistance = distance;
  }
  if(modePosVis.isVisible)
  {    
    osd.setPanel(modePosVis.col, modePosVis.row);
    osd.openPanel();
    
    if(!motor_armed_flag)
    {
      strcpy(string2display, "\x7f disarmed");
    }
    else
    {

      if(!(custom_mode & (1<<0)))
      {
        strcpy(string2display, "\x7f init");
     
      }
      
      if(custom_mode & (1<<0))
      {
        strcpy(string2display, "\x7f armed");
      }
      
      if(custom_mode & (1<<1))
      {
          strcpy(string2display, "\x7f manual");
      
      }
      
      if(custom_mode & (1<<2))
      {
          strcpy(string2display, "\x7f althold");
      }
      else if(custom_mode & (1<<3))
      {
        strcpy(string2display, "\x7f poshold");
      }
      else if(custom_mode & (1<<5))
      {
        strcpy(string2display, "\x7f mission");
      }
      
     //co odswiezenie ekranu tu bedziemy
     if(osd_chan7_raw < 675)
     {
       strcat(string2display, "+rth");
     }
      
      if(custom_mode & 0x80000000)
      {
        if(failsafeCounter > 5)
        {
          strcpy(string2display, "\x7f");
          failsafeCounter = -1;
        }
        else
        {
          strcpy(string2display, "\x7f failsafe");
        }
        ++failsafeCounter;        
      }
    }
      osd.printf(string2display);
      osd.closePanel();   
  }


  if(speedPosVis.isVisible)
  {
    osd.setPanel(speedPosVis.col, speedPosVis.row);
    osd.openPanel();
    osd.printf("v:%.2f\x10", (osd_vel*36)/10);
    osd.closePanel();
  }
}

void showHorizon(int start_col, int start_row) 
{
    int x, nose, row, minval, hit, subval = 0;
    const int cols = 12;
    const int rows = 5;
    int col_hit[cols];
    double  pitch, roll;

    (abs(osd_pitch) == 90)?pitch = 89.99 * (90/osd_pitch) * -0.017453293:pitch = osd_pitch * -0.017453293;
    (abs(osd_roll) == 90)?roll = 89.99 * (90/osd_roll) * 0.017453293:roll = osd_roll * 0.017453293;

    nose = round(tan(pitch) * (rows*9));
    for(int col=1;col <= cols;col++){
        x = (col * 12) - (cols * 6) - 6;
        col_hit[col-1] = (tan(roll) * x) + nose + (rows*9) - 1;
    }

    for(int col=0;col < cols; col++){
        hit = col_hit[col];
        if(hit > 0 && hit < (rows * 18)){
            row = rows - ((hit-1)/18);
            minval = rows*18 - row*18 + 1;
            subval = hit - minval;
            subval = round((subval*9)/18);
            if(subval == 0) subval = 1;
            printHit(start_col + col, start_row + row - 1, subval);
        }
    }
}

void showYaw(byte col, byte row)
{
  osd.setPanel(col, row);
    osd.openPanel();
//    osd.printf_P(PSTR("\x14"));
    osd.printf("%4.0f", ((double)osd_yaw*100)/100);
    osd.printf_P(PSTR("\x05"));
    osd.closePanel();  
}


void printHit(byte col, byte row, byte subval)
{
    osd.openSingle(col, row);
    char subval_char = 0;
        switch (subval){
        case 1:
            subval_char = 0xC7;
            break;
        case 2:
            subval_char = 0xC8; 
            break;
        case 3:
            subval_char = 0xC9;
            break;
        case 4:
            subval_char = 0xCA;
            break;
        case 5:
            subval_char = 0xCB; 
            break;
        case 6:
            subval_char = 0xCC;
            break;
        case 7:
            subval_char = 0xCD;
            break;
        case 8:
            subval_char = 0xCE;
            break;
        case 9:
            subval_char = 0xCF;
            break;
        }
        osd.printf("%c", subval_char);
}

double calculateDistanceBeetwCoords(struct gPSPosS first, struct gPSPosS second)
{
  const uint16_t R = 6371; //promien Ziemii w km
  //konwersja na radiany
  double lat1 = (first.lat.f*M_PI)/180.0;
  double lat2 = (second.lat.f*M_PI)/180.0;
  double deltaLat = ((second.lat.f-first.lat.f)*M_PI)/180.0;
  double deltaLon = ((second.lon.f-first.lon.f)*M_PI)/180.0;
  
  double tmp = sin(deltaLat/2)*sin(deltaLat/2)+cos(lat1)*cos(lat2)*sin(deltaLon/2)*sin(deltaLon/2);
  
  return (2.0*atan2(sqrt(tmp), sqrt(1-tmp))*R);
}

void displayCoordsDegOrMin(double lat, double lon)
{
  boolean latNeg = false;
  boolean lonNeg = false;  
  uint8_t degs;
  uint8_t mins; 
  double secs;
  
  if(gPSLLscale == 0)
      {
        if(lat < 0)
        {
          latNeg = true;
          lat = -lat;
        }
        if(lon < 0)
        {
          lonNeg = true;
          lon = -lon;
        }
        
        degs = lat;
        mins = ((uint16_t)((lat-degs)*60.0));
        secs = (((lat-degs)*60.0)-mins)*60.0;
        
        if(latNeg)
        {
          osd.printf("|s %d\x5%d\x27%.1f\x4", degs, mins, secs);
          lat = -lat;
        }
        else if(lat == 0)
          osd.printf("|- 0\x5""0\x27""0\x4");
        else
          osd.printf("|n %d\x5%d\x27%.1f\x4", degs, mins, secs);
      
        degs = lon;
        mins = ((uint16_t)((lon-degs)*60.0));
        secs = (((lon-degs)*60.0)-mins)*60.0;
          
        if(lonNeg)
        {
          osd.printf("|w %d\x5%d\x27%.1f\x4", degs, mins, secs);
          lon = -lon;
        }
        else if(lon == 0)
          osd.printf("|- 0\x5""0\x27""0\x4");
        else
          osd.printf("|e %d\x5%d\x27%.1f\x4", degs, mins, secs);      
      }
      else
      {
        if(lat < 0)
          osd.printf("|s %3.5f\x5", lat*(-1));
        else if(lat == 0)
          osd.printf("|- 0");
        else
          osd.printf("|n %3.5f\x5", lat);
          
        if(lon < 0)
          osd.printf("|w %3.5f\x5", lon*(-1));
        else if(lon == 0)
          osd.printf("|- 0");
        else
          osd.printf("|e %3.5f\x5", lon);      
      }
 }
