#include <Adafruit_GPS.h>
#include <Adafruit_SSD1306.h>

#include <Wire.h>
#include <LPS.h> 
#include <LSM6.h>
#include <LIS3MDL.h>
#include "cannerdata.h"
#include <SD.h>
#include <Servo.h>
#include <math.h>

//------------MACROS--------------
#define BUZZER_PIN 10
#define SERVO_PIN 9
#define SERVO_CLOSED 0
#define LAUNCH_ACCEL_THRESH 5 //in g's
//--------------------------------
//-----------GLOBALS--------------

LPS PTS; 
LIS3MDL MAG;
data_s candata; 
HardwareSerial gpsSerial = Serial1; //read and print from serial port 1 on the GPS
Adafruit_GPS GPS(&gpsSerial); //need to use the address for the GPS serial
HardwareSerial Xbee = Serial2; //calls class to send data to serial 2 for the xbee
const int chipSelect = 4;

Servo releaseServo;

boolean usingInterrupt = true;
void useInterrupt(boolean); 

//--------------------------------


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

releaseServo.attach(SERVO_PIN);

}

void getData(){
  candata.pres = PTS.readPressureMillibars();
  candata.temp = PTS.readTemperatureC();
  candata.alt = PTS.pressureToAltitudeMeters(candata.pres);
  MAG.read();
  candata.magx = MAG.m.x;
  candata.magy = MAG.m.y;
  candata.magz = MAG.m.z;
  candata.accx = MAG.a.x;
  candata.accy = MAG.a.y;
  candata.accz = MAG.a.z;
  candata.gpsalt = GPS.altitude;
  candata.lat = GPS.lat;
  candata.lon = GPS.lon;
  candata.satnum = (int)GPS.satellites;
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

void buzzerOn(){
  static bool ran = false;
  if(ran == false){
    digitalWrite(BUZZER_PIN,HIGH);
    ran = true;
  }
}

void buzzerOff(){
  digitalWrite(BUZZER_PIN,LOW);
}

int servoControl(bool servostate){
  if(servostate == false){
    releaseServo.write(SERVO_CLOSED);
  }
  else{
    releaseServo.write(SERVO_OPEN);
  }
  return releaseServo.read();
}

void state(){
  if(candata.state == 0){
    float netforce = sqrt(candata.accx^2+candata.accy^2+candata.accz^2)/9.8;
    if(netforce >= LAUNCH_ACCEL_THRESH){
      candata.state = 1; 
    }
  }
  else if(candata.state == 1){
    if
  }
}






