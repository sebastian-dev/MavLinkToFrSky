/*
APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1  http://www.pjrc.com/teensy/index.html
based on ideas found here http://code.google.com/p/telemetry-convert/
******************************************************
Cut board on the backside to separate Vin from VUSB
Connection on Teensy 3.1:
SPort S --> TX1
SPort + --> Vin
SPort  - --> GND
APM Telemetry DF13-5  Pin 2 --> RX2
APM Telemetry DF13-5  Pin 3 --> TX2
APM Telemetry DF13-5  Pin 5 --> GND
*/

#include <GCS_MAVLink.h>
#include "FrSkySPortTelemetry.h"
#include "MavLinkTelemetry.h"
#include "MavLinkToFrSkyConverter.h"
#include "PulsePosition.h"
#include "FUTABA_SBUS.h"

FrSkySPortTelemetry frsky;
MavLinkTelemetry mavlink;
FUTABA_SBUS sbus;
PulsePositionOutput ppm_out(FALLING);

int led = 13;
uint32_t sbus_timer;
uint32_t telemetry_timer;
uint32_t performance1_timer;
uint32_t performance2_timer;

// ******************************************
void setup()
{
    int i;
    analogReference(DEFAULT);

    frsky.begin();//MavLink
    mavlink.begin();//FrSky

    //debug LED
    pinMode(led, OUTPUT);

    //ppm output at pin 23
    ppm_out.begin(23);
    for (int i = 0; i < 16; i++)
        ppm_out.write(i+1, 1000);

    //sbus servo signal reading
    sbus.begin();
    digitalWrite(led, LOW);   
}

// ******************************************
void loop()  {
  uint16_t len;
  uint32_t current_time;
  current_time = millis();

  if (telemetry_timer != current_time)
  {
      telemetry_timer = current_time;
      mavlink.worker();
      frsky.worker();
      if (mavlink.new_data_trigger)
      {
          mavlink.new_data_trigger = false;
          MavLinkToFrSkyConverter::MavLinkToFrSky(mavlink.data, frsky.data);
      }
  }
  
  //do every ms check the servo channel s-bus
  if (current_time != sbus_timer)
  {
      sbus_timer = current_time;    //reset current time
      sbus.FeedLine();
      if (sbus.toChannels)
      {
          sbus.UpdateChannels();    //read out all channels
          for (int i = 0; i < 16; i++)
          {
              float servo = sbus.channels[i];
              float max = (2100.0 - 900.0) / 2048.0;
              servo *= max;
              servo += 900;
              ppm_out.write(i + 1, servo);
          }
          sbus.toChannels = 0;
      }
  }

  //main loop runtime test
  if ((millis() - performance1_timer) > 2)
  {
      digitalWrite(led, HIGH);
      performance2_timer = millis();
  }
  performance1_timer = millis();
  if ((millis() - performance2_timer) > 2000)
  {
      performance2_timer = millis();
      digitalWrite(led, LOW);
  }
  
}
