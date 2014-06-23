#include "MavLinkTelemetry.h"
#include <GCS_MAVLink.h>

#define DataPort            Serial3
#define DataBaud            57600
#define MSG_RATE            5       //data rate in Hz
#define DATA_START_STOP     1       //Start
#define POLLING_TIME        2       //2ms polling rate
#define CONNECTION_TIMEOUT  2500    //2.5 sek timeout to connection lost

MavLinkTelemetry::MavLinkTelemetry()
{
    data = &_data;
    last_process_time = 0;
    connected_timer = 0;
    heartbeat_cnt = 0;
    connection_changed_trigger = false;
    new_data_trigger = false;
    is_connected = false;
}

/// <summary>
/// Start usage of FrSky Telemtry
/// </summary>
void MavLinkTelemetry::begin()
{
    DataPort.begin(DataBaud);
    is_connected = false;
}

/// <summary>
/// get current time, encapsulate arduino function
/// </summary>
/// <returns>current time</returns>
uint32_t MavLinkTelemetry::get_time()
{
    return millis();
}

/// <summary>
/// Check if the main worker is busy
/// </summary>
/// <returns>true when worker is not ready yet</returns>
bool MavLinkTelemetry::is_busy()
{
    if (get_time() - last_process_time >= POLLING_TIME)
        return false;
    else
        return true;
}

/// <summary>
/// FrSky Telemetry Main loop worker, this function handle the communication
/// with the SPort and sends telemtry data to FrSky Receiver via serial port
/// </summary>
void MavLinkTelemetry::worker()
{
    mavlink_message_t msg;
    mavlink_status_t status;

    if (is_busy())
        return;
    mavlink_recive();

    if (connection_changed_trigger)
    {
        if (is_connected)
        {
            uint16_t len;   
            switch (connection_changed_trigger)
            {
            case 1:
                mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_EXTENDED_STATUS, MSG_RATE, DATA_START_STOP);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                DataPort.write(buf, len);                
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7: //empty case condition to generate a delay between messages
                connection_changed_trigger++;
                break;                
            
            case 8:
                mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_EXTRA2, MSG_RATE, DATA_START_STOP);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                DataPort.write(buf, len);                
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
                break;

            default:
                mavlink_msg_request_data_stream_pack(0xFF, 0xBE, &msg, 1, 1, MAV_DATA_STREAM_RAW_SENSORS, MSG_RATE, DATA_START_STOP);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                DataPort.write(buf, len);
                connection_changed_trigger = 0;
                break;
            }
        }
    }

    //check connection timeout
    if (is_connected && (get_time() - connected_timer) > CONNECTION_TIMEOUT)
    {
        //connection is lost, reset all triggers
        is_connected = false;
        connection_changed_trigger = 0;
        connected_timer = 0;
        heartbeat_cnt = 0;
    }

    last_process_time = get_time();
}

void MavLinkTelemetry::mavlink_recive()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint16_t byte_count = 0;

    if (DataPort.available() < 8)
        return;

    while (DataPort.available() && byte_count < 64)
    {
        uint8_t c = DataPort.read();
        byte_count++;
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
            switch (msg.msgid)
            {
            case MAVLINK_MSG_ID_HEARTBEAT:  // 0                
                data->sys_type = mavlink_msg_heartbeat_get_type(&msg);
                data->sys_autopilot = mavlink_msg_heartbeat_get_autopilot(&msg);
                data->sys_base_mode = (mavlink_msg_heartbeat_get_base_mode(&msg) & 0x80) > 7;
                data->sys_custom_mode = mavlink_msg_heartbeat_get_custom_mode(&msg);
                data->sys_system_status = mavlink_msg_heartbeat_get_system_status(&msg);
                
                connected_timer = get_time();
                if (!is_connected)
                {
                    heartbeat_cnt++;
                    if (heartbeat_cnt > 8) // If  received > 10 heartbeats from MavLink then we are connected
                    {
                        is_connected = true;
                        connection_changed_trigger = 1;
                    }
                }

                if (is_connected)
                    new_data_trigger = true;
                break;

            case MAVLINK_MSG_ID_SYS_STATUS:   // 1                                						  
                data->sens_voltage_batt = mavlink_msg_sys_status_get_voltage_battery(&msg);  // 1 = 1mV
                data->sens_current = mavlink_msg_sys_status_get_current_battery(&msg);     // 1=10mA
                break;

            case MAVLINK_MSG_ID_GPS_RAW_INT:   // 24
                data->gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg);                               // 0 = No GPS, 1 =No Fix, 2 = 2D Fix, 3 = 3D Fix                
                data->gps_sats =  mavlink_msg_gps_raw_int_get_satellites_visible(&msg);          // numbers of visible satelites
                data->gps_status = (data->gps_sats * 10) + data->gps_fix_type;
                data->gps_altitude = mavlink_msg_gps_raw_int_get_alt(&msg);
                data->gps_latitude = mavlink_msg_gps_raw_int_get_lat(&msg);
                data->gps_longitude = mavlink_msg_gps_raw_int_get_lon(&msg);                
                break;

            case MAVLINK_MSG_ID_RAW_IMU:   // 27
                data->acc_x = mavlink_msg_raw_imu_get_xacc(&msg);
                data->acc_y = mavlink_msg_raw_imu_get_yacc(&msg);
                data->acc_z = mavlink_msg_raw_imu_get_zacc(&msg);
                break;

            case MAVLINK_MSG_ID_VFR_HUD:   //  74
                data->hud_groundspeed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
                data->hud_heading = mavlink_msg_vfr_hud_get_heading(&msg);
                data->hud_throttle = mavlink_msg_vfr_hud_get_throttle(&msg);
                data->hud_altitude = mavlink_msg_vfr_hud_get_alt(&msg);
                data->hud_climb_rate = mavlink_msg_vfr_hud_get_climb(&msg);                
                break;

            default:
                break;
            }
        }
    }
}