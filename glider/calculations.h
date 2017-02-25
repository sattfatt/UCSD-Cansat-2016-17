/********************************************************************************************
 * calculations.h
 * 
 * contains algorithms and equations 
 * 
 *******************************************************************************************/

 float headingCalc(float ax, float ay, float az, float mx, float my, float mz,float del){
  //calculates heading angle 
  float phi = (atan2(-ay,-az))*180/PI;
  float theta = (atan2(ax, sqrt(ay*ay + az*az)))*180/PI;
  float psi = del + (atan2(mz*sin(phi) - my*cos(phi),mx*cos(theta) + my*sin(theta)*sin(phi) + mz*sin(theta)*cos(phi)))*180/PI;
  
  return psi;
  
 }

