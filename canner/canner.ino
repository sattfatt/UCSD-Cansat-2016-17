#include <Wire.h>
#include <LPS.h> 
#include <LSM6.h>
#include <LIS3MDL.h>
#include "cannerdata.h"

LPS PTS; 
LIS3MDL MAG;
data_s candata; 


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

}

void loop() {
  // put your main code here, to run repeatedly:

}
