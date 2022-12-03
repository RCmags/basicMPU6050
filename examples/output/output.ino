/*
 Get scaled and calibrated output of MPU6050
 */

#include <basicMPU6050.h> 

// Create instance
basicMPU6050<> imu;

void setup() {
  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  // Start console
  Serial.begin(38400);
}

void loop() { 
  // Update gyro calibration 
  imu.updateBias();
 
  //-- Scaled and calibrated output:
  // Accel
  Serial.print( imu.ax() );
  Serial.print( " " );
  Serial.print( imu.ay() );
  Serial.print( " " );
  Serial.print( imu.az() );
  Serial.print( "    " );
  
  // Gyro
  Serial.print( imu.gx() );
  Serial.print( " " );
  Serial.print( imu.gy() );
  Serial.print( " " );
  Serial.print( imu.gz() );
  Serial.print( "    " );  
  
  // Temp
  Serial.print( imu.temp() );
  Serial.println(); 
}
