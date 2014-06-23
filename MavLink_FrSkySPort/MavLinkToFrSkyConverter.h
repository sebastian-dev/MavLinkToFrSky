#ifndef MAVLINK_TO_FRSKY_CONVERTER_H
#define MAVLINK_TO_FRSKY_CONVERTER_H

#include <inttypes.h>
#include <Arduino.h>
#include "MavLinkTelemetry.h"
#include "FrSkySPortTelemetry.h"

class MavLinkToFrSkyConverter
{
public:
    static void MavLinkToFrSky(MavLinkTelemetry::MavlinkData *mav_data, FrSkySPortTelemetry::TelemetryData *frsky_data);
private:
    
};
#endif