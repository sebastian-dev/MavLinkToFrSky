#include "MavLinkToFrSkyConverter.h"
#include "MavLinkTelemetry.h"
#include "FrSkySPortTelemetry.h"

void MavLinkToFrSkyConverter::MavLinkToFrSky(MavLinkTelemetry::MavlinkData *mav_data, FrSkySPortTelemetry::TelemetryData *frsky_data)
{
    int32_t tmp;

    frsky_data->fr16_vfas = mav_data->sens_voltage_batt / 10;
    frsky_data->fr08_current = mav_data->sens_current / 10;

    if (mav_data->gps_fix_type != 3)
    {
        tmp = mav_data->hud_altitude;
    }
    else
    {
        tmp = mav_data->gps_altitude;
        tmp /= 1000;
    }
    frsky_data->fr08_alt = tmp;
    frsky_data->fr08_alt_x = 0;
    frsky_data->fr08_gps_alt = frsky_data->fr08_alt;
    frsky_data->fr08_gps_alt_x = frsky_data->fr08_alt_x;
    frsky_data->fr16_altitude = tmp;

    frsky_data->fr08_gps_speed = mav_data->hud_groundspeed;
    frsky_data->fr08_gps_speed_x = 0;

    frsky_data->fr08_fuel = mav_data->hud_throttle;
    frsky_data->fr08_temperature_01 = mav_data->gps_status;  
    frsky_data->fr08_temperature_02 = mav_data->hud_heading;

    //gps:
    if (mav_data->gps_fix_type != 3)
    {
        frsky_data->fr08_gps_lon = 0;
        frsky_data->fr08_gps_lon_x = 0;
        frsky_data->fr08_gps_lon_ew = 0;
        frsky_data->fr08_gps_lat = 0;
        frsky_data->fr08_gps_lat_x = 0;
        frsky_data->fr08_gps_lat_ns = 0;
    }
    else
    {
        uint32_t loc, grad, min, sek;

        //1byte id
        if (mav_data->gps_longitude < 0)
            frsky_data->fr08_gps_lon_ew = 1;
        else
            frsky_data->fr08_gps_lon_ew = 0;

        if (mav_data->gps_latitude < 0)
            frsky_data->fr08_gps_lat_ns = 1;
        else
            frsky_data->fr08_gps_lat_ns = 0;

        //degree format to 9° 57' 18.17"
        loc = abs(mav_data->gps_longitude);
        grad = loc / 10000000;
        min = (loc - (grad * 10000000)) * 60;
        sek = min;
        min /= 10000000;
        sek = ((loc - (grad * 10000000)) * 60 - (min * 10000000)) * 60;
        sek /= 100000;
        frsky_data->fr08_gps_lon = grad * 100 + min;
        frsky_data->fr08_gps_lon_x = sek;

        //degree format to 9° 57' 18.17"
        loc = abs(mav_data->gps_latitude);
        grad = loc / 10000000;
        min = (loc - (grad * 10000000)) * 60;
        sek = min;
        min /= 10000000;
        sek = ((loc - (grad * 10000000)) * 60 - (min * 10000000)) * 60;
        sek /= 100000;
        frsky_data->fr08_gps_lat = grad * 100 + min;
        frsky_data->fr08_gps_lat_x = sek;

        //2byte id GPS position
        if (mav_data->gps_longitude < 0)
            frsky_data->fr16_latlong_lon = ((abs(mav_data->gps_longitude) / 100) * 6) | 0xC0000000;
        else
            frsky_data->fr16_latlong_lon = ((abs(mav_data->gps_longitude) / 100) * 6) | 0x80000000;

        if (mav_data->gps_latitude < 0)
            frsky_data->fr16_latlong_lat = ((abs(mav_data->gps_latitude) / 100) * 6) | 0x40000000;
        else
            frsky_data->fr16_latlong_lat = ((abs(mav_data->gps_latitude) / 100) * 6);

        



    }

}