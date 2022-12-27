# basicMPU6050
The purpose of this library is to make a basic and lightweight interface for the MPU6050. It can do the following:
- Configure the inbuilt low pass filter 
- Configure the sensitivity of the accelerometer and gyro
- Retrieve the raw output of the sensor
- Output scaled accelerometer and gyro values

# Calibration
The library includes two functions to calibrate the gyro and remove bias. The first is intended to be called when the sensor is turned on and is not moving. It takes a long term average of the output of each axis and subtracts these values from the raw signals.

The second function is designed to update the averages. It updates the values with a moving average and the gain is controlled by something akin to a kalman filter. By polling this function one can correct for drift in the gyro bias. By combining these two functions one can obtain stable and consistent gyro outputs. However, The accelerometer needs to calibrated manually by correcting the bias and scale. 

# References
See this link for the information on the registers of the MPU6050: 
[MPU 6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

<img src = "https://www.prometec.net/wp-content/uploads/2015/10/MPU-6050-Board-GY-521-MEGA_bb.png" width = "60%"></img>   
