#ifndef MAVLINK_TELEMETRY_H
#define MAVLINK_TELEMETRY_H

#include <GCS_MAVLink.h>
#include <inttypes.h>
#include <Arduino.h>

class MavLinkTelemetry
{
public:

    MavLinkTelemetry(void);
    void begin();
    void worker();
    bool is_connected;
    bool new_data_trigger;
    typedef struct
    {
        //Sensor
        uint16_t sens_voltage_batt;
        uint16_t sens_current;

        //system
        uint8_t sys_type;
        uint8_t sys_autopilot;
        uint8_t sys_base_mode;
        uint32_t sys_custom_mode;
        uint8_t sys_system_status;

        //GPS
        uint16_t gps_fix_type;          //0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix
        uint16_t gps_sats;              //numbers of visible satelites
        uint16_t gps_status;            //(gps_sats*10) + gps_fix_type; 
        int32_t  gps_latitude;          // 585522540;
        int32_t  gps_longitude;         // 162344467;
        int32_t  gps_altitude;          // 1000 = 1m

        //ACC
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;

        //HUD        
        uint32_t hud_groundspeed;       //100 = 1m/s
        uint32_t hud_heading;           //100 = 100 deg
        uint16_t hud_throttle;          //100 = 100%
        int32_t hud_altitude = 0;       //1 = 1m
        int32_t hud_climb_rate = 0;     //1= 1m/s


    }MavlinkData;
    MavlinkData *data;

private:
    uint32_t get_time();
    bool is_busy();
    uint32_t connected_timer;
    uint32_t last_process_time;
    uint16_t heartbeat_cnt;
    uint16_t connection_changed_trigger;

    uint8_t     buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg;

    void mavlink_recive();
    MavlinkData _data;

    
};
#endif