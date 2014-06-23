#ifndef FRSKY_SPORT_TELEMETRY_H
#define FRSKY_SPORT_TELEMETRY_H

#include <inttypes.h>
#include <Arduino.h>

class FrSkySPortTelemetry
{
public:
    FrSkySPortTelemetry(void);
    void begin();
    void worker();

    typedef struct
    {
        uint16_t id;
        uint32_t *pData;
    }DataPackage;

    typedef struct
    {
        uint32_t  fr08_gps_alt;
        uint32_t  fr08_gps_alt_x;
        uint32_t  fr08_gps_speed;
        uint32_t  fr08_gps_speed_x;
        uint32_t  fr08_gps_lon;
        uint32_t  fr08_gps_lon_x;
        uint32_t  fr08_gps_lon_ew;
        uint32_t  fr08_gps_lat;
        uint32_t  fr08_gps_lat_x;
        uint32_t  fr08_gps_lat_ns;

        uint32_t  fr08_volatge;
        uint32_t  fr08_volatge_x;
        uint32_t  fr08_alt;
        uint32_t  fr08_alt_x;
        uint32_t  fr08_course;
        uint32_t  fr08_course_x;
        uint32_t  fr08_temperature_01;
        uint32_t  fr08_temperature_02;
        uint32_t  fr08_rpm;
        uint32_t  fr08_fuel;
        uint32_t  fr08_cell_volt;
        uint32_t  fr08_acc_x;
        uint32_t  fr08_acc_y;
        uint32_t  fr08_acc_z;
        uint32_t  fr08_current;
        uint32_t  fr08_vert_speed;
        uint32_t  fr08_date;
        uint32_t  fr08_year;
        uint32_t  fr08_time;
        uint32_t  fr08_time_second;

        uint32_t fr16_altitude;
        uint32_t fr16_vario;
        uint32_t fr16_current;
        uint32_t fr16_vfas;
        uint32_t fr16_cells_12;
        uint32_t fr16_cells_34;
        uint32_t fr16_cells_56;
        uint32_t fr16_t1;
        uint32_t fr16_t2;
        uint32_t fr16_rpm;
        uint32_t fr16_fuel;
        uint32_t fr16_accx;
        uint32_t fr16_accy;
        uint32_t fr16_accz;
        uint32_t fr16_latlong_lat;
        uint32_t fr16_latlong_lon;
        uint32_t fr16_gps_alt;
        uint32_t fr16_speed;
        uint32_t fr16_heading;
        uint32_t fr16_adc1;
        uint32_t fr16_adc2;
    }TelemetryData;
    TelemetryData *data;
    

private:
    uint8_t send_buffer[8];

    uint32_t get_time();
    bool is_busy();
    
    uint8_t get_crc(uint8_t *data);
    void send_package(uint16_t id, uint32_t value);

    uint32_t FR_ID_count = 0;
    uint8_t latlong_flag = 0;
    uint32_t latlong = 0;
    uint8_t first = 0;

    uint16_t message_loop_count;
    uint32_t last_process_time;
    uint8_t last_rx;    
};
#endif