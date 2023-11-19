//--------------------- Constants --------------------

template<TEMPLATE_TYPE>
const uint8_t basicMPU6050<TEMPLATE_INPUTS>
::MPU_ADDRESS = ADDRESS_A0 == HIGH ? MPU_ADDRESS_HIGH : MPU_ADDRESS_LOW;        

template<TEMPLATE_TYPE>
const float basicMPU6050<TEMPLATE_INPUTS>
::MEAN = 1.0/float(N_BIAS);                                 // Inverse of sample count 

template<TEMPLATE_TYPE>
const float basicMPU6050<TEMPLATE_INPUTS>
::ACCEL_LSB = AFS_SEL < 0 || AFS_SEL >  3 ? 1         :     // Scaling factor for accelerometer. Depends on sensitivity setting.
                       1.0/( AFS_SEL == 0 ? 16384.0   :     // Output is in: g 
                             AFS_SEL == 1 ? 8192.0    :
                             AFS_SEL == 2 ? 4096.0    :
                                            2048.0    );
template<TEMPLATE_TYPE>
const float basicMPU6050<TEMPLATE_INPUTS>
::GYRO_LSB = FS_SEL < 0 || FS_SEL >  3 ? 1      :           // Scaling factor for gyro. Depends on sensitivity setting. 
              (PI/180.0)/( FS_SEL == 0 ? 131.0  :           // Output is in: rad/s
                           FS_SEL == 1 ? 65.5   :
                           FS_SEL == 2 ? 32.8   :
                                         16.4   );

//----------------- I2C communication -----------------
                                         
template<TEMPLATE_TYPE>
void basicMPU6050<TEMPLATE_INPUTS>
::setRegister( uint8_t reg, uint8_t mode ) {
  Wire.beginTransmission(MPU_ADDRESS);                                        
  Wire.write(reg);                                                   
  Wire.write(mode);                                                    
  Wire.endTransmission();
}

template<TEMPLATE_TYPE>
void basicMPU6050<TEMPLATE_INPUTS>
::readRegister( uint8_t reg ) {
  Wire.beginTransmission(MPU_ADDRESS);        
  Wire.write(reg);                                                   
  Wire.endTransmission();                                               
  Wire.requestFrom( MPU_ADDRESS, uint8_t(2), uint8_t(true) );  
}

template<TEMPLATE_TYPE>
int16_t basicMPU6050<TEMPLATE_INPUTS>
::readWire() {
  return int( Wire.read()<<8|Wire.read() );
}
 
template<TEMPLATE_TYPE>
void basicMPU6050<TEMPLATE_INPUTS>
::setup() {
  // Turn on communication
  Wire.begin();
    
  // Enable sensor  
  setRegister( 0x6B, 0x00 );
  
  // Low pass filter
  setRegister( 0x1A, DLPF_CFG <= 6 ? DLPF_CFG : 0x00 ); 
  
  // Gyro sensitivity
  setRegister( 0x1B, FS_SEL == 1 ? 0x08   :
                     FS_SEL == 2 ? 0x10   :
                     FS_SEL == 3 ? 0x18   :
                                   0x00   );
  // Accel sensitivity
  setRegister( 0x1C, AFS_SEL == 1 ? 0x08  :
                     AFS_SEL == 2 ? 0x10  :
                     AFS_SEL == 3 ? 0x18  :
                                    0x00  );
}

//---------------- Raw measurements ------------------
 
// Acceleration
template<TEMPLATE_TYPE>
int16_t basicMPU6050<TEMPLATE_INPUTS>
::rawAx() {
  readRegister( 0x3B );
  return readWire();       
}

template<TEMPLATE_TYPE>
int16_t basicMPU6050<TEMPLATE_INPUTS>
::rawAy() {
  readRegister( 0x3D );
  return readWire();
}

template<TEMPLATE_TYPE>
int16_t basicMPU6050<TEMPLATE_INPUTS>
::rawAz() {
  readRegister( 0x3F );
  return readWire();
}

// Temperature
template<TEMPLATE_TYPE>
int16_t basicMPU6050<TEMPLATE_INPUTS>
::rawTemp() {
  readRegister( 0x41 );                                                                                                           
  return readWire();                                 
}

// Gyro
template<TEMPLATE_TYPE>
int16_t basicMPU6050<TEMPLATE_INPUTS>
::rawGx() {
  readRegister( 0x43 );   
  return readWire();    
}

template<TEMPLATE_TYPE>
int16_t basicMPU6050<TEMPLATE_INPUTS>
::rawGy() {
  readRegister( 0x45 );
  return readWire();
}

template<TEMPLATE_TYPE>
int16_t basicMPU6050<TEMPLATE_INPUTS>
::rawGz() {
  readRegister( 0x47 );
  return readWire();
}

//--------------- Scaled measurements ----------------

// Acceleration
template<TEMPLATE_TYPE>
float basicMPU6050<TEMPLATE_INPUTS>
::ax() {
  const float SCALE  = (*AX_S) * ACCEL_LSB;
  const float OFFSET = (*AX_S) * float(AX_OFS)/ACCEL_LSB_0;   
  
  return float( rawAx() )*SCALE - OFFSET;       
}

template<TEMPLATE_TYPE>
float basicMPU6050<TEMPLATE_INPUTS>
::ay() {
  const float SCALE  = (*AY_S) * ACCEL_LSB;
  const float OFFSET = (*AY_S) * float(AY_OFS)/ACCEL_LSB_0;  
  
  return float( rawAy() )*SCALE - OFFSET;         
}

template<TEMPLATE_TYPE>
float basicMPU6050<TEMPLATE_INPUTS>
::az() {  
  const float SCALE  = (*AZ_S) * ACCEL_LSB;
  const float OFFSET = (*AZ_S) * float(AZ_OFS)/ACCEL_LSB_0; 
  
  return float( rawAz() )*SCALE - OFFSET;         
}

// Temperature
template<TEMPLATE_TYPE>
float basicMPU6050<TEMPLATE_INPUTS>
::temp() {
  const float TEMP_MUL = 1.0/340.0;                                  
  return rawTemp()*TEMP_MUL + 36.53;
}

// Gyro
template<TEMPLATE_TYPE>
float basicMPU6050<TEMPLATE_INPUTS>
::gx() {
  const float SCALE = (*GX_S) * GYRO_LSB;
  return ( float( rawGx() ) - mean[0] )*SCALE;  
}
template<TEMPLATE_TYPE>
float basicMPU6050<TEMPLATE_INPUTS>
::gy() {
  const float SCALE = (*GY_S) * GYRO_LSB;
  return ( float( rawGy() ) - mean[1] )*SCALE;  
}

template<TEMPLATE_TYPE>
float basicMPU6050<TEMPLATE_INPUTS>
::gz() {
  const float SCALE = (*GZ_S) * GYRO_LSB;
  return ( float( rawGz() ) - mean[2] )*SCALE;  
}

//--------------- Gyro bias estimate -----------------
        
template<TEMPLATE_TYPE>
void basicMPU6050<TEMPLATE_INPUTS>
::setBias() {
  for( int count = 0; count < N_BIAS; count += 1 ) { 
    int gyro[] = { rawGx(), rawGy(), rawGz() };
    delayMicroseconds(250);
    
    // Sum all samples
    for( int index = 0; index < N_AXIS; index += 1 ) {
      mean[index] += float( gyro[index] ); 
    } 
  }
  
  // Divide sums by sample count
  for( int index = 0; index < N_AXIS; index += 1 ) {
    mean[index] *= MEAN; 
  }    
}

template<TEMPLATE_TYPE>
void basicMPU6050<TEMPLATE_INPUTS>
::updateBias() {
  constexpr float BAND_SQ = GYRO_BAND*GYRO_BAND;
  
  // error in measurement
  float dw[N_AXIS] = { rawGx() - mean[0] ,
                       rawGy() - mean[1] , 
                       rawGz() - mean[2] };
  
  // update mean with kalman gain
  for( int index = 0; index < N_AXIS; index += 1 ) {  
    float error_sq = dw[index] * dw[index] / BAND_SQ;
    
    float gain = MEAN/( 1 + var[index]*var[index] );
    
    var[index] = error_sq + var[index]*gain;
    
    mean[index] += dw[index] * gain;
  }
}
