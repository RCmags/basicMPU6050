/*
 This sketch shows how to configure the output
 */

#include <basicMPU6050.h> 

//-- Input parameters:

// Gyro settings:
#define         LP_FILTER   3           // Low pass filter.                    Value from 0 to 6
#define         GYRO_SENS   0           // Gyro sensitivity.                   Value from 0 to 3
#define         ACCEL_SENS  0           // Accelerometer sensitivity.          Value from 0 to 3
#define         ADDRESS_A0  LOW         // I2C address from state of A0 pin.   A0 -> GND : ADDRESS_A0 = LOW
                                        //                                     A0 -> 5v  : ADDRESS_A0 = HIGH
// Accelerometer offset:
constexpr int   AX_OFFSET =  552;       // Use these values to calibrate the accelerometer. The sensor should output 1.0g if held level. 
constexpr int   AY_OFFSET = -241;       // These values are unlikely to be zero.
constexpr int   AZ_OFFSET = -3185;

// Output scale: 
constexpr float AX_SCALE = 1.00457;     // Multiplier for accelerometer outputs. Use this to calibrate the sensor. If unknown set to 1.
constexpr float AY_SCALE = 0.99170;
constexpr float AZ_SCALE = 0.98317;

constexpr float GX_SCALE = 0.99764;     // Multiplier to gyro outputs. Use this to calibrate the sensor. If unknown set to 1.
constexpr float GY_SCALE = 1.0;
constexpr float GZ_SCALE = 1.01037;

// Bias estimate:
#define         GYRO_BAND   35          // Standard deviation of the gyro signal. Gyro signals within this band (relative to the mean) are suppresed.   
#define         BIAS_COUNT  5000        // Samples of the mean of the gyro signal. Larger values provide better calibration but delay suppression response. 

//-- Set the template parameters:

basicMPU6050<LP_FILTER,  GYRO_SENS,  ACCEL_SENS, ADDRESS_A0,
             AX_OFFSET,  AY_OFFSET,  AZ_OFFSET, 
             &AX_SCALE,  &AY_SCALE,  &AZ_SCALE,
             &GX_SCALE,  &GY_SCALE,  &GZ_SCALE,
             GYRO_BAND,  BIAS_COUNT 
            >imu;

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
  Serial.println();
}
