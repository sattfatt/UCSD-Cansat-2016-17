#include <Wire.h>
#include <LPS.h> 
#include <LSM6.h>
#include <LIS3MDL.h>
#include "cannerdata.h"
#include <SD.h>

LPS PTS; 
LIS3MDL MAG;
data_s candata; 
HardwareSerial gpsSerial = Serial1; //read and print from serial port 1 on the GPS
Adafruit_GPS GPS(&gpsSerial); //need to use the address for the GPS serial
HardwareSerial Xbee = Serial2; //calls class to send data to serial 2 for the xbee
const int chipSelect = 4;


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
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
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

//create a function that logs the data (the struct)
void logdata(){
  File dataFile = SD.open("canlog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(candata.alt); dataFile.print(",");
    dataFile.print(candata.temp); dataFile.print(",");
    dataFile.print(candata.state);dataFile.print(",");
    dataFile.print(candata.pres); dataFile.print(",");
    dataFile.print(candata.airspeed); dataFile.print(",");
    dataFile.print(candata.magx); dataFile.print(",");
    dataFile.print(candata.magy); dataFile.print(",");
    dataFile.print(candata.magz); dataFile.print(",");
    dataFile.print(candata.lat); dataFile.print(",");
    dataFile.print(candata.lon); dataFile.print(",");
    dataFile.print(candata.gpsalt); dataFile.print(",");
    dataFile.print(candata.piccount); dataFile.println();
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}

