#include "FrSkySPortTelemetry.h"
#include "FrSkyData.h"

#define FrSkySPort                  Serial1
#define FrSkySPort_BAUD             57600
#define FrSkySPort_C1               UART0_C1
#define FrSkySPort_C3               UART0_C3
#define FrSkySPort_S2               UART0_S2
#define SetSendDirection()          FrSkySPort_C3 |= 0x20
#define SetReciveDirection()        FrSkySPort_C3 &= ~0x20
#define FrSkySendPackageTimeout     2                           //timeout in ms to send a data packet

//Frsky Sensor-ID to use. Must be something that is polled by FrSky RX.
//Vario Sensor ID 01, Voltage Sensor ID 02, Current Sensor ID 03, GPS Sensor ID 04
#define SENSOR_ID1                0x1B     
#define SENSOR_ID2                0x0D       
#define SENSOR_ID3                0x34      
#define SENSOR_ID4                0x67      

//frsky protocol specific
#define START_STOP                0x7e
#define DATA_FRAME                0x10

FrSkySPortTelemetry::TelemetryData _data_local_frsky;
FrSkySPortTelemetry::DataPackage data_link[] = {
    
    { FR08_GPS_ALT, &_data_local_frsky.fr08_gps_alt },
    { FR08_GPS_ALT_X, &_data_local_frsky.fr08_gps_alt_x },
    { FR08_GPS_SPEED, &_data_local_frsky.fr08_gps_speed },
    { FR08_GPS_SPEED_X, &_data_local_frsky.fr08_gps_speed_x },
    { FR08_GPS_LON, &_data_local_frsky.fr08_gps_lon },
    { FR08_GPS_LON_X, &_data_local_frsky.fr08_gps_lon_x },
    { FR08_GPS_LON_EW, &_data_local_frsky.fr08_gps_lon_ew },
    { FR08_GPS_LAT, &_data_local_frsky.fr08_gps_lat },
    { FR08_GPS_LAT_X, &_data_local_frsky.fr08_gps_lat_x },
    { FR08_GPS_LAT_NS, &_data_local_frsky.fr08_gps_lat_ns },

    //{ FR08_VOLATGE, &_data_local_frsky.fr08_volatge },
    //{ FR08_VOLATGE_X, &_data_local_frsky.fr08_volatge_x },
    //{ FR08_ALT, &_data_local_frsky.fr08_alt },
    //{ FR08_ALT_X, &_data_local_frsky.fr08_alt_x },
    { FR08_COURSE, &_data_local_frsky.fr08_course },
    { FR08_COURSE_X, &_data_local_frsky.fr08_course_x },
    { FR08_TEMPERATURE_01, &_data_local_frsky.fr08_temperature_01 },
    { FR08_TEMPERATURE_02, &_data_local_frsky.fr08_temperature_02 },
    { FR08_RPM, &_data_local_frsky.fr08_rpm },
    { FR08_FUEL, &_data_local_frsky.fr08_fuel },
    { FR08_CELL_VOLT, &_data_local_frsky.fr08_cell_volt },
    { FR08_ACC_X, &_data_local_frsky.fr08_acc_x },
    { FR08_ACC_Y, &_data_local_frsky.fr08_acc_y },
    { FR08_ACC_Z, &_data_local_frsky.fr08_acc_z },
    { FR08_CURRENT, &_data_local_frsky.fr08_current },
    //{ FR08_VERT_SPEED, &_data_local_frsky.fr08_vert_speed },    
    //{ FR08_DATE, &_data_local_frsky.fr08_date },    
    //{ FR08_YEAR, &_data_local_frsky.fr08_year },
    //{ FR08_TIME, &_data_local_frsky.fr08_time },
    //{ FR08_TIME_SECOND, &_data_local_frsky.fr08_time_second },

    //{ FR16_ALTITUDE, &_data_local_frsky.fr16_altitude },
    //{ FR16_VARIO, &_data_local_frsky.fr16_vario },
    //{ FR16_CURRENT, &_data_local_frsky.fr16_current },
    { FR16_VFAS, &_data_local_frsky.fr16_vfas },
    //{ FR16_CELLS, &_data_local_frsky.fr16_cells_12 },
    //{ FR16_CELLS, &_data_local_frsky.fr16_cells_34 },
    //{ FR16_CELLS, &_data_local_frsky.fr16_cells_56 },
    //{ FR16_T1, &_data_local_frsky.fr16_t1 },
    //{ FR16_T2, &_data_local_frsky.fr16_t2 },
    //{ FR16_RPM, &_data_local_frsky.fr16_rpm },
    //{ FR16_FUEL, &_data_local_frsky.fr16_fuel },    
    //{ FR16_ACCX, &_data_local_frsky.fr16_accx },
    //{ FR16_ACCY, &_data_local_frsky.fr16_accy },
    //{ FR16_ACCZ, &_data_local_frsky.fr16_accz },
    //{ FR16_LATLONG, &_data_local_frsky.fr16_latlong_lat },
    //{ FR16_LATLONG, &_data_local_frsky.fr16_latlong_lon },

    //{ FR16_GPS_ALT, &_data_local_frsky.fr16_gps_alt },
    //{ FR16_SPEED, &_data_local_frsky.fr16_speed },
    //{ FR16_HEADING, &_data_local_frsky.fr16_heading },
    //{ FR16_ADC1, &_data_local_frsky.fr16_adc1 },
    //{ FR16_ADC2, &_data_local_frsky.fr16_adc2 },
};

#define MESSAGE_COUNT (sizeof(data_link) / sizeof(FrSkySPortTelemetry::DataPackage))

FrSkySPortTelemetry::FrSkySPortTelemetry()
{
    message_loop_count = 0;
    last_process_time = 0;
    data = &_data_local_frsky;
}

/// <summary>
/// get current time, encapsulate arduino function
/// </summary>
/// <returns>current time</returns>
uint32_t FrSkySPortTelemetry::get_time()
{
    return millis();
}

/// <summary>
/// Check if the main worker is busy
/// </summary>
/// <returns>true when worker is not ready yet</returns>
bool FrSkySPortTelemetry::is_busy()
{
    if (get_time() - last_process_time >= FrSkySendPackageTimeout)
        return false;
    else
        return true;
}

/// <summary>
/// Start usage of FrSky Telemtry
/// </summary>
void FrSkySPortTelemetry::begin()
{
    FrSkySPort.begin(FrSkySPort_BAUD);
    FrSkySPort_C3 = 0x10;		//Tx invert
    FrSkySPort_C1 = 0xA0;		//Single wire mode
    FrSkySPort_S2 = 0x10;		//Rx Invert
    last_process_time = get_time();
    send_buffer[0] = 0;
    last_rx = 0;
    message_loop_count = 0;
}

/// <summary>
/// FrSky Telemetry Main loop worker, this function handle the communication
/// with the SPort and sends telemtry data to FrSky Receiver via serial port
/// </summary>
void FrSkySPortTelemetry::worker()
{
    uint8_t read_data;
    uint16_t id = 0xffff;
    uint32_t value;

    if (is_busy())
        return;
    
    if (send_buffer[0] == DATA_FRAME)
        send_buffer[0] = 0;
    SetReciveDirection();      //S.Port(recive) 

    if (!FrSkySPort.available())
    {
        last_process_time = get_time();
        return;
    }

    read_data = FrSkySPort.read();
    if (last_rx == START_STOP && ((read_data == SENSOR_ID1) || (read_data == SENSOR_ID2) || (read_data == SENSOR_ID3) || (read_data == SENSOR_ID4)))
    {
        message_loop_count++;
        if (message_loop_count >= MESSAGE_COUNT){
            message_loop_count = 0;
        }
        id = data_link[message_loop_count].id;
        value = *data_link[message_loop_count].pData;
    }

    if (id != 0xffff)
        send_package(id, value);
    last_rx = read_data;
    last_process_time = get_time();
}

/// <summary>
/// Calculate frsky crc
/// </summary>
/// <param name="data">data byte to send</param>
/// <returns>frsky crc value</returns>
uint8_t FrSkySPortTelemetry::get_crc(uint8_t *data)
{
    uint16_t crc;
    uint8_t i;

    crc = 0;
    for (i = 0; i < 7; i++)
    {
        crc += data[i];
        crc += crc >> 8;
        crc &= 0x00ff;
    }

    crc = 0xff - crc;
    return (uint8_t)crc;
}

void FrSkySPortTelemetry::send_package(uint16_t id, uint32_t value)
{
    uint8_t *data;

    //fill send buff with data
    send_buffer[0] = DATA_FRAME; //start frame    
    data = (uint8_t*)&id;
    send_buffer[1] = data[0];   //id
    send_buffer[2] = data[1];
    data = (uint8_t*)&value;
    send_buffer[3] = data[0];   //value
    send_buffer[4] = data[1];
    send_buffer[5] = data[2];
    send_buffer[6] = data[3];
    send_buffer[7] = get_crc(send_buffer);

    //send new frsky package
    SetSendDirection();      //S.Port(send) 
    FrSkySPort.write(send_buffer, 8);    
}