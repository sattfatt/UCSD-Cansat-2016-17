#include <Wire.h>
#include <LPS.h> 
#include <LSM6.h>
#include <LIS3MDL.h>
#include "cannerdata.h"

LPS PTS; 
LIS3MDL MAG;
data_s candata; 
HardwareSerial gpsSerial = Serial1; //read and print from serial port 1 on the GPS
Adafruit_GPS GPS(&gpsSerial); //need to use the address for the GPS serial
HardwareSerial Xbee = Serial2; //calls class to send data to serial 2 for the xbee



boolean usingInterrupt = true;
void useInterrupt(boolean); 


void setup() {
  // put your setup code here, to run once:

Serial.begin(9600);
Wire.begin();

if(! PTS.init()){
  Serial.println("Pressure and temperature sensor failed to initialize");
}
if(! MAG.init()){
  Serial.println("Magnetometer sensor failed to initialize");
}
PTS.enableDefault();
MAG.enableDefault();
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 

}

void getData(){
  candata.pres = PTS.readPressureMillibars();
  candata.temp = PTS.readTemperatureC();
  candata.alt = PTS.pressureToAltitudeMeters(candata.pres);
  MAG.read();
  candata.magx = MAG.m.x;
  candata.magy = MAG.m.y;
  candata.magz = MAG.m.z;
  candata.gpsalt = GPS.altitude;
  candata.latitude = GPS.lat;
  candata.longitude = GPS.lon;
  candata.satnum = (int)GPS.satellites;

  if 
  
   

}
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}


//create a function that sends data (the struct) into a serial port on the arduino
void sendData(){
  Xbee.print(candata.state);
  Xbee.print(",");
  Xbee.print(candata.pres); //print uses ASCII format (Serial.Write uses values instead of ASCII)
  Xbee.print(",");
  Xbee.print(candata.temp);
  Xbee.print(",");
  Xbee.print(candata.alt);Xbee.print(",");
  Xbee.print(candata.airspeed);Xbee.print(",");
  Xbee.print(candata.magx);Xbee.print(",");
  Xbee.print(candata.magy);Xbee.print(",");
  Xbee.print(candata.magz);Xbee.print(",");
  Xbee.print(candata.lat);Xbee.print(",");
  Xbee.print(candata.lon);Xbee.print(",");
  Xbee.print(candata.gpsalt);Xbee.print(",");
  Xbee.print(candata.piccount);
  Xbee.println();
}
}

