#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

//Zmienne
/* MAVLink session control */
static bool     mavbeat = 0;
static uint8_t  aq_mav_type;
static uint8_t  aq_mav_system; 
static uint8_t  aq_mav_component;
static uint16_t messageCounter;
static bool     mavlink_active;
static uint8_t crlf_count = 0;

//zmienne niestatyczne
double    osd_vbat_A = 999;
double    osd_lat;
double    osd_lon;
double  osd_alt;
uint8_t  osd_fix_type;
//uint8_t  osd_airspeed;
//uint8_t  osd_climb;
int8_t   osd_pitch;
int8_t   osd_roll;
int16_t   osd_yaw;
//int8_t   osd_heading;
uint64_t osd_time;
//uint16_t osd_flight_time;
int16_t osd_rssi = 100;
double osd_vel;
uint32_t  custom_mode;
uint8_t  base_mode;
uint16_t osd_chan7_raw;
char osd_text[51]; //+1
boolean rtcSet = false;

typedef struct
{
  uint8_t hour;
  uint8_t minute;
} baseTimeS;
baseTimeS baseTime = {0, 0};  
uint32_t frozenRelativeTime;  

uint8_t ascii2number(uint8_t ascii)
{
  return ascii-0x30;
}

boolean read_mavlink()
{
  mavlink_message_t msg; 
  mavlink_status_t status;
  boolean returnValue = false;

  // grabing data 
  while(Serial.available() > 0) 
  { 
    uint8_t c = Serial.read();
   
        /* allow CLI to be started by hitting enter 3 times, if no
        heartbeat packets have been received */
        if (mavlink_active == 0 ){//&& millis() < 20000 && millis() > 5000) {
            if (c == '\n' || c == '\r') {
                crlf_count++;
            } else {
                crlf_count = 0;
            }
            if (crlf_count == 3) {
                //noInterrupts();
                uploadFont();
                //interrupts();
            }
        } 
    

    // trying to grab msg  
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
    {
       messageCounter = 0; 
       mavlink_active = 1;
       returnValue = true;
       switch(msg.msgid)           /* Handle msg */
       {
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
            mavbeat = 1;
	    aq_mav_system    = msg.sysid;
	    aq_mav_component = msg.compid;
            aq_mav_type      = mavlink_msg_heartbeat_get_type(&msg);
            base_mode = mavlink_msg_heartbeat_get_base_mode(&msg);
            custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
            if(base_mode&(1<<7))
            {
              motor_armed_flag = true;
            }
            else 
            {
              motor_armed_flag = false;
            }
          }
          break;
          
          case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    osd_vbat_A = (mavlink_msg_sys_status_get_voltage_battery(&msg) / 1000.0f); //Battery voltage, in millivolts (1 = 1 millivolt)
                }
                break;

            case MAVLINK_MSG_ID_GPS_RAW_INT:
                {
                    osd_lat = mavlink_msg_gps_raw_int_get_lat(&msg) / 10000000.0f;
                    osd_lon = mavlink_msg_gps_raw_int_get_lon(&msg) / 10000000.0f;
                    osd_alt = mavlink_msg_gps_raw_int_get_alt(&msg) / 1000.0f;
                    osd_vel = mavlink_msg_gps_raw_int_get_vel(&msg) / 100.0f;
                    osd_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);
                    osd_time = mavlink_msg_gps_raw_int_get_time_usec(&msg) / 1000000ull;
                    //osd_satellites_visible = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
                }
                break; 
            case MAVLINK_MSG_ID_ATTITUDE:
                {
                    osd_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
                    osd_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
                    osd_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg)); //experimental
                }
                break;                
            case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                {
                    osd_rssi = mavlink_msg_rc_channels_raw_get_rssi(&msg);
                    osd_chan7_raw = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
                }
                break;
//            case MAVLINK_MSG_ID_VFR_HUD:
//                {
                //    osd_airspeed = mavlink_msg_vfr_hud_get_airspeed(&msg);
                //    osd_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
//                    osd_heading = mavlink_msg_vfr_hud_get_heading(&msg); // 0..360 deg, 0=north
                //    osd_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
                    //if(osd_throttle > 100 && osd_throttle < 150) osd_throttle = 100;//Temporary fix for ArduPlane 2.28
                    //if(osd_throttle < 0 || osd_throttle > 150) osd_throttle = 0;//Temporary fix for ArduPlane 2.28
                //    osd_alt = mavlink_msg_vfr_hud_get_alt(&msg);
                //    osd_climb = mavlink_msg_vfr_hud_get_climb(&msg);
//                }
//                break;
            case MAVLINK_MSG_ID_STATUSTEXT:
                {             
                  mavlink_msg_statustext_get_text(&msg, osd_text);
                  if(osd_text[0] == 'R' &&
                     osd_text[1] == 'T' &&
                     osd_text[2] == 'C' &&
                     osd_text[4] == 's' &&
                     osd_text[5] == 'e' &&
                     osd_text[6] == 't' && !rtcSet)
                    {
                      baseTime.hour = 10*(osd_text[20]-0x30) + osd_text[21]-0x30;
                      baseTime.minute = 10*(osd_text[23]-0x30) + osd_text[24]-0x30;       
                      frozenRelativeTime = (uint32_t)osd_time;
                      rtcSet = true;
                    }
                }
                break;	
         }    
      }
    }
    return returnValue;
}
