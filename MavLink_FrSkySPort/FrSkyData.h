#ifndef FRSKY_DATA_H
#define FRSKY_DATA_H

//                                  DataID      Meaning             Unit    Range        Note
#define FR08_GPS_ALT                0x01        //GPS altitude      m                   Before.
#define FR08_TEMPERATURE_01         0x02        //Temperature1      °C      -30-250
#define FR08_RPM                    0x03        //RPM               BPS     0-60000
#define FR08_FUEL                   0x04        //Fuel Level        %       0, 25, 50, 75, 100
#define FR08_TEMPERATURE_02         0x05        //Temperature2      °C      -30-250
#define FR08_CELL_VOLT              0x06        //Cell Volt         1/500v  0-4.2v,     top 4 bits are cell #
#define FR08_GPS_ALT_X              0x09        //GPS altitude      m                   After.
#define FR08_ALT                    0x10        //Altitude          m       0-9999      Before .
#define FR08_GPS_SPEED              0x11        //GPS speed         Knots               Before .
#define FR08_GPS_LON                0x12        //Longitude         dddmm.mmmm          Before .
#define FR08_GPS_LAT                0x13        //Latitude          ddmm.mmmm           Before .
#define FR08_COURSE                 0x14        //Course            degree 0-360        Before .
#define FR08_DATE                   0x15        //Date/Month
#define FR08_YEAR                   0x16        //Year
#define FR08_TIME                   0x17        //Hour /Minute
#define FR08_TIME_SECOND            0x18        //Second
#define FR08_GPS_SPEED_X            0x19        //GPS speed         Knots               After .
#define FR08_GPS_LON_X              0x1A        //Longitude         dddmm.mmmm          After .
#define FR08_GPS_LAT_X              0x1B        //Latitude          ddmm.mmmm           After .
#define FR08_COURSE_X               0x1C        //Course            degree 0-360        After .
#define FR08_ALT_X                  0x21        //Altitude          m       0-9999      After .
#define FR08_GPS_LON_EW             0x22        //Long - E/W
#define FR08_GPS_LAT_NS             0x23        //Lat. N/S
#define FR08_ACC_X                  0x24        //Acc-x             1/256g  -8g ~ +8g
#define FR08_ACC_Y                  0x25        //Acc-y             1/256g  -8g ~ +8g
#define FR08_ACC_Z                  0x26        //Acc-z             1/256g  -8g ~ +8g
#define FR08_CURRENT                0x28        //Current           1A      0-100A
#define FR08_VERT_SPEED             0x29        //VerticalSpeed
#define FR08_VOLATGE                0x3A        //Voltage(amp sensor) 0.5v  0-48V       Before .
#define FR08_VOLATGE_X              0x3B        //Voltage(amp sensor)                   After .

#define FR16_ALTITUDE               0x0100
#define FR16_ALTITUDE_LAST          0x010F
#define FR16_VARIO                  0x0110
#define FR16_VARIO_LAST             0x011F
#define FR16_CURRENT                0x0200
#define FR16_CURRENT_LAST           0x020F
#define FR16_VFAS                   0x0210
#define FR16_VFAS_LAST              0x021F
#define FR16_CELLS                  0x0300     
#define FR16_CELLS_LAST             0x030F
#define FR16_T1                     0x0400
#define FR16_T1_LAST                0x040F
#define FR16_T2                     0x0410
#define FR16_T2_LAST                0x041F
#define FR16_RPM                    0x0500
#define FR16_RPM_LAST               0x050F
#define FR16_FUEL                   0x0600 
#define FR16_FUEL_LAST              0x060F
#define FR16_ACCX                   0x0700
#define FR16_ACCX_LAST              0x070F
#define FR16_ACCY                   0x0710
#define FR16_ACCY_LAST              0x071F
#define FR16_ACCZ                   0x0720
#define FR16_ACCZ_LAST              0x072F
#define FR16_LATLONG                0x0800
#define FR16_LATLONG_LAST           0x080F
#define FR16_GPS_ALT                0x0820
#define FR16_GPS_ALT_LAST           0x082F
#define FR16_SPEED                  0x0830
#define FR16_SPEED_LAST             0x083F
#define FR16_HEADING                0x0840
#define FR16_HEADING_LAST           0x084F

#define FR16_RSSI                   0xF101  
#define FR16_ADC1                   0xF102 
#define FR16_ADC2                   0xF103
#define FR16_BATT                   0xF104
#define FR16_SWR                    0xF105

#endif