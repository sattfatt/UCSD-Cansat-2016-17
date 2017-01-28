/******************************************************************************************
 * glider.ino code
 * 
 * Cansat 2016-2017 competition for glider flight
 ******************************************************************************************/

//-----------INCLUDES--------------
#include <Adafruit_GPS.h>
#include <Adafruit_VC0706.h>
#include <Wire.h>
#include <LPS.h> 
#include <LSM6.h>
#include <LIS3MDL.h>
#include "gliderdata.h"               // Header file with constants and parameters       
#include <SD.h>
#include <math.h>

//------------MACROS------------------
#define BUZZER_PIN 10                 // Pin for turning buzzer on and off  
#define DATA_BUFFER_LENGTH 10         // 
#define ALT_THRESH 5                  // Threshold altitude before turning buzzer on (5m)
#define PITOT_PIN 20                  // Pin for reading pitot pressure data
#define PITOT_CAL 32                  // Calibration for the pitot tube
#define VOLTAGE_PIN 18                // Pin for reading voltage values from solar panels  

//-------------GLOBALS---------------
LPS PTS; 
LIS3MDL MAG;
LSM6 ACC;                       
data_s gliderdata[DATA_BUFFER_LENGTH];  //
HardwareSerial gpsSerial = Serial1;     // Read and print from serial port 1 on the GPS
Adafruit_GPS GPS(&gpsSerial);           // Need to use the address for the GPS serial
HardwareSerial Xbee = Serial2;          // Calls class to send data to serial 2 for the xbee
const int chipSelect = 4;                
int packet_count = 0;                   // Initialize packet count as zero 
boolean usingInterrupt = true;          // Initialize the interupt function as true
void useInterrupt(boolean);             // Define useInterrrupt function with boolean input
int pos = 0;                            // Initialize pos vector at 0  

/********************************************************************************************
 * Setup function()
 * 
 * Initialize PTS, MAG, ACC, and SD card
 *******************************************************************************************/

void setup() {

// Intialize Serial monitor at 9600 bps 
Serial.begin(9600); 

Wire.begin();

// Checck if PTS, MAG, ACC, and SD cards are initialized
if(! PTS.init()){
  Serial.println("Pressure and temperature sensor failed to initialize");
}
if(! MAG.init()){
  Serial.println("Magnetometer sensor failed to initialize");
}
if(! ACC.init()){
  Serial.println("Accelerometer failed to initialize");
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

// Enable default settings for PTS, MAG, and GPS
PTS.enableDefault();
MAG.enableDefault();
GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 

// Set PITOT_PIN as an input
pinMode(PITOT_PIN,INPUT);
}

/********************************************************************************************
 * getData()
 * 
 * Fill gliderdata[pos] with glider mission data to prepare for sending
 * Calculate airspeed (using pitot tube), voltage, and piccount 
 *******************************************************************************************/

void getData(int pos){
  gliderdata[pos].pres = PTS.readPressureMillibars();
  gliderdata[pos].temp = PTS.readTemperatureC();
  gliderdata[pos].alt = PTS.pressureToAltitudeMeters(candata[pos].pres);
  MAG.read();
  gliderdata[pos].magx = MAG.m.x;
  gliderdata[pos].magy = MAG.m.y;
  gliderdata[pos].magz = MAG.m.z;
  gliderdata[pos].accx = ACC.a.x;
  gliderdata[pos].accy = ACC.a.y;
  gliderdata[pos].accz = ACC.a.z;
  gliderdata[pos].gpsalt = GPS.altitude;
  gliderdata[pos].lat = GPS.lat;
  gliderdata[pos].lon = GPS.lon;
  gliderdata[pos].satnum = (int)GPS.satellites;
   
//---------RTC----------------
//  static bool ran == false;
//  if(GPS.satellites>=5 && ran == false){
//    gpsTime = GPS.hour*60*60*1000+GPS.minute*60*1000+GPS.seconds*1000;
//    unsigned long timenow = millis();
//    ran == true;
//  }
//  else gpsTime = 0;
//  candata[pos].missiontime = gpsTime + millis()-timenow;
// ---------------------------  

/*
//-----Velocity calc------------ 
  unsigned long sum1;
  long sum2;
  long num;
  long den;
  for(int i = 0; i<10;i++){
    sum1 = 0;
    sum2 = 0;
    sum1+=candata[i].missiontime;
    sum2+=candata[i].alt;
  }
  unsigned long avgtime = sum1/DATA_BUFFER_LENGTH;
  int avgdist = sum2/DATA_BUFFER_LENGTH;
  for (int i = 0; i<10; i++){
    num += (candata[i].missiontime - avgtime)*(candata[i].alt - avgdist);
    den += pow(candata[i].missiontime - avgtime,2);
  }
  candata[pos].vel = num/den; 
//-------net force-------------
  gliderdata[pos].netforce = sqrt(pow(candata[pos].accx,2)+pow(candata[pos].accy,2)+pow(candata[pos].accz,2))/9.8;
//-----------------------------  
}*/

//----------Pitot----------------

// Read pitot values from the pin
pitotRead = analogRead(PITOT_PIN);

// Calculate airspeed from conastants and equations
gliderdata[pos].airspeed=sqrt(2000.*((((float)pitotRead-(float)PITOT_CAL/(0.2*1024.0))-2.5)/1.225);

// Check to see if airspeed is being read
if(gliderdata[pos].airspeed != gliderdata[pos].airspeed){
  glider[pos].airspeed = 0.0;
  //Serial.println("NAN"); 
}

//----------Voltage--------------

// 0-614.4 assuming dividing to half 6V * Ratio -> ADC
// R1 = 10k, R2 = 1k + 1k

// Define ratio
float ratio = 5/6;

// Read raw data from voltage pin
float rawVoltage = analogRead(VOLTAGE_PIN);

// Map raw voltage data to 6V scale
float voltage = map(rawVoltage, rawLow, rawHigh, voltageLow, voltageHigh);

gliderdata[pos].volt = voltage;

//---------PicCount-------------


//gliderdata[pos].piccount=
}


/********************************************************************************************
 * userInterrupt()
 * 
 * Interrupt function
 *******************************************************************************************/

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

/*********************************************************************************************
 * loop()
 * 
 * Run freqLimiterGet and freqLimiterSend functions
 ********************************************************************************************/

void loop() {
  freqLimiterGet(pos,.01,1);
  state(pos);
  freqLimiterSend(pos,1,1);
  pos++;
  if(pos == DATA_BUFFER_LENGTH){
    pos = 0;
  }
}

/*********************************************************************************************
 * sendData()
 * 
 * Create a function that sends data (the struct) into serial port on the arduino
 ********************************************************************************************/

void sendData(int pos){
  Xbee.print(gliderdata[pos].state); Xbee.print(",");
  Xbee.print(gliderdata[pos].pres); //print uses ASCII format (Serial.Write uses values instead of ASCII)
  Xbee.print(",");
  Xbee.print(gliderdata[pos].temp);Xbee.print(",");
  Xbee.print(gliderdata[pos].alt);Xbee.print(",");
  Xbee.print(gliderdata[pos].airspeed);Xbee.print(",");
  Xbee.print(gliderdata[pos].magx);Xbee.print(",");
  Xbee.print(gliderdata[pos].magy);Xbee.print(",");
  Xbee.print(gliderdata[pos].magz);Xbee.print(",");
  Xbee.print(gliderdata[pos].lat);Xbee.print(",");
  Xbee.print(gliderdata[pos].lon);Xbee.print(",");
  Xbee.print(gliderdata[pos].volt);Xbee.print(",");
  Xbee.print(gliderdata[pos].gpsalt);Xbee.print(",");
  Xbee.print(gliderdata[pos].piccount);Xbee.println();
}

/*********************************************************************************************
 * logData()
 * 
 * Create a function that logs the data (the struct)
 *********************************************************************************************/

void logdata(int pos){
  File dataFile = SD.open("gliderlog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(gliderdata[pos].alt); dataFile.print(",");
    dataFile.print(gliderdata[pos].temp); dataFile.print(",");
    dataFile.print(gliderdata[pos].state);dataFile.print(",");
    dataFile.print(gliderdata[pos].pres); dataFile.print(",");
    dataFile.print(gliderdata[pos].airspeed); dataFile.print(",");
    dataFile.print(gliderdata[pos].magx); dataFile.print(",");
    dataFile.print(gliderdata[pos].magy); dataFile.print(",");
    dataFile.print(gliderdata[pos].magz); dataFile.print(",");
    dataFile.print(gliderdata[pos].lat); dataFile.print(",");
    dataFile.print(gliderdata[pos].lon); dataFile.print(",");
    dataFile.print(gliderdata[pos].gpsalt); dataFile.print(",");
    dataFile.print(gliderdata[pos].volt); dataFile.print(",");
    dataFile.print(gliderdata[pos].piccount); dataFile.println();
    dataFile.close();
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}

/*********************************************************************************************
 * buzzerOn()
 * 
 * Create function to turn on buzzer when ran equal to faluse
 ********************************************************************************************/

void buzzerOn(){
  static bool ran = false;

  // If ran is faluse, turn buzzer on
  if(ran == false){
    digitalWrite(BUZZER_PIN,HIGH);
    ran = true;
  }
}

/*********************************************************************************************
 * buzzerOff()
 * 
 * Create function to turn buzzer off possibly
 ********************************************************************************************/

void buzzerOff(){
  digitalWrite(BUZZER_PIN,LOW);
}

/*********************************************************************************************
 * state()
 * 
 * Change to initial state and initialize buzzer
 ********************************************************************************************/

void state(int pos){
  if(gliderdata[pos].state == 0 && gliderdata[pos].alt =< ALT_THRESH){
     buzzerOn();
  }
}

/*********************************************************************************************
 * freqLimiterGet()
 * 
 * Limit the frequency with which getData is called and sampled
 ********************************************************************************************/

void freqLimiterGet(int pos, int per, int tol){
  static unsigned long last_send = 0;
  int this_send = millis();
  if (this_send - last_send > .5*per)  {
    if (this_send%1000 < tol*per/100. || this_send%1000 > (100-tol)*per/100)  {
      last_send = this_send;
      getData(pos);
    }
  }
}

/*********************************************************************************************
 * freqLimiterSend()
 * 
 * Limit the frequency with which data is sent and the sampling rate
 ********************************************************************************************/

void freqLimiterSend(int pos, int per, int tol){
  static unsigned long last_send = 0;
  unsigned long this_send = millis();
  if (this_send - last_send > .5*per)  {
    if (this_send%1000 < tol*per/100. || this_send%1000 > (100-tol)*per/100)  {
      last_send = this_send;
      //packet_count++;
      sendData(pos);
    }
  }
}
