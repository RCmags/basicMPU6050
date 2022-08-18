/*
 Get raw output of MPU6050
 */

#include <basicMPU6050.h> 

// Create instance
basicMPU6050<> imu;

void setup() {
  // Set registers - Always required
  imu.setup();
  
  // Start console
  Serial.begin(38400);
}

void loop() {  
  //-- Raw output:
  // Accel
  Serial.print( imu.rawAx() );
  Serial.print( " " );
  Serial.print( imu.rawAy() );
  Serial.print( " " );
  Serial.print( imu.rawAz() );
  Serial.print( "    " );
  
  // Gyro
  Serial.print( imu.rawGx() );
  Serial.print( " " );
  Serial.print( imu.rawGy() );
  Serial.print( " " );
  Serial.print( imu.rawGz() );
  Serial.print( "    " ); 
  
  // Temp
  Serial.print( imu.rawTemp() );
  Serial.println();
}
