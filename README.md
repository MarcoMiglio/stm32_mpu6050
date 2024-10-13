# **MPU6050 (GY-521) Driver for STM32 with Motion and Zero-Motion Detection**

An optimized STM32 library for interfacing with the MPU6050 (GY-521), featuring motion and zero-motion detection capabilities.  

The following code is intended for STM32 boards. (Tested using STM32L476-RG NUCELO Board)  
The library is adapted from [Jeff Rowberg's](https://github.com/ElectronicCats/mpu6050/blob/master/src/MPU6050.cpp) existing library for the MPU6050 and Arduino boards.

## Features

This driver is specifically designed to optimize power consumption, making it ideal for battery-powered devices. It exploits the MPU6050's motion and zero-motion detection features keeping the MCU in low-power mode until motion is detected. Burst read of the mpu6050 internal buffer is also supported to minimize active processing time, further enhancing the device's energy efficiency.

- Motion and zero-motion detection support
- Efficient communication using DMA mechanism
- Low-power integration with STM32 wake-up functionality (Standby mode and Stop mode 2)
- FIFO burst read functionality: MCU remains into low power mode most of the time

## Getting started

The Core/Src and Core/Inc directories, contain the code responsible for communicating with the MPU6050:  

- Core/Src/mpu6050.c and Core/Inc/mpu6050.h provide the functions for interacting with the sensor
- The files Core/Src/main.c and Core/Inc/main.h contain a complete example that demonstrates all the features available in this driver

**Setup the project** using STM32CubeIDE:
- Start and initialize the I2C bus in fast mode (400 kHz), in the example I2C1 was used and managed through `&hi2c1`.
