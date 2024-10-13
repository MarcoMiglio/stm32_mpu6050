# **MPU6050 (GY-521) Driver for STM32 with Motion and Zero-Motion Detection**

An optimized STM32 library for interfacing with the MPU6050 (GY-521), featuring motion and zero-motion detection capabilities.  

The following code is intended for STM32 boards. (Tested using STM32L476-RG NUCELO Board)  
The library is adapted from [Jeff Rowberg](https://github.com/ElectronicCats/mpu6050/blob/master/src/MPU6050.cpp)'s existing library for the MPU6050 and Arduino boards 

## Features

- Motion and zero-motion detection support
- Efficient communication using DMA mechanism
- Low-power integration with STM32 wake-up functionality (Standby mode and Stop mode 2)
- FIFO burst read functionality: MCU remains into low power mode most of the time

