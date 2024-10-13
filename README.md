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
- The files Core/Src/main.c and Core/Inc/main.h contain a complete example that demonstrates a possible solution to implement all the features available in this driver

**Setup the project** using STM32CubeIDE:
- Start and initialize the I2C bus in fast mode (400 kHz), in the example I2C1 was used and managed through `&hi2c1`.
- Configure the I2C DMA settings: enable DMA reception by selecting I2C1_RX, DMA1 channel 7, peripheral to memory. In the NVIC settings tab enable "I2C1 event interrupt" and "I2C1 error handler".
- In the GPIO tab select pin PA9 and configure it as "EXTI" (External interrupt mode with rising edge trigger detection), then enable "EXTI line[9:5] interrupts" in the NVIC interrupt table. Enable also the internal pull-down under GPIO Pull-up/Pull-down. Please note that in the example the user label was modified, therefore this pin is referenced as "IMU_Int1".
- Select pin PC5 and configure it as "SYS_WKUP5".
- Now select Timers -> RTC and enable "Activate clock source", "Activate calendar". Make also sure that the RTC is sourced through the LSE (Low speed external) oscillator. This can be done in the clock configuration window, by selecting LSE under the RTC/LCD Source Mux.
- In the same window (Timers -> RTC) select "Internal WakeUp" under the WakeUp menu, and enable the RTC wake-up interrupt trough EXTI line 20 in the NVIC settings.
- Add these lines to your "_FLASH.id" file (in the example STM32L476RGTX_FLASH.id), needed to allocate a memory region that can be powered through the internal low power regulator, allowing this RAM region to be maintained across sleep cycles in standby mode.
```
/*Allocate space in RAM2 to store variables between sleep cycles*/
.ram2 (NOLOAD):
{
  _sram2 = .;
  *(.ram2*)
  . = ALIGN(4);
  _eram2 = .;
} >RAM2
```
