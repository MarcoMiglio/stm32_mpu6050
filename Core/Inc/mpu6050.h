/*
 * mpu6050.h
 *
 *  Created on: Apr 16, 2024
 *      Author: marcomiglio
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#endif /* INC_MPU6050_H_ */

#include "stm32l4xx_hal.h"
#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

//#include "i2c.h"

// **************  MPU6050 registers: *********************

#define MPU6050_ADDR     0x68 << 1  // MPU6050 address in I2C bus

#define WHO_AM_I_REG     0x75		// Test i2c connection

#define PWR_MGMT_1_REG   0x6B		// Power management register 1
#define PWR_MGMT_2_REG   0x6C		// Power management register 2
#define SMPLRT_DIV_REG   0x19		// Adjust sample rate
#define USER_CTRL_REG    0x6A		// Bit 7 used to enable/disable DMP
#define SMPLRT_DIV_REG   0x19       // Set sampling rate

#define INT_ENABLE_REG   0x38		// Interrupt enable register
#define INT_PIN_CFG      0x37		// Interrupt pin - configurations
#define MOT_DUR_TH_REG   0x20		// Motion duration threshold register
#define MOT_DET_TH_REG   0x1F		// Motion detection threshold register
#define INT_STATUS_REG   0x3A   // Interrupt status register

#define MOT_DETECT_STATUS_REG    0x61

#define ZRMOT_THR_REG    0x21   // zero motion threshold register
#define ZRMOT_DUR_REG    0x22   // zero motion duration register

#define ACCEL_CONFIG_REG 0x1C		// Accelerometer configurations - High Pass Filter
#define ACCEL_XOUT_H_REG 0x3B		// Accelerometer output register (1st byte of x-axis)

#define TEMP_OUT_H_REG   0x41		// Temperature output register (1st byte of x-axis)

#define GYRO_CONFIG_REG  0x1B		// Gyroscope configurations
#define GYRO_XOUT_H_REG  0x43		// Gyroscope output register (1st byte of x-axis)
#define DLPF_CFG_REG     0x1A       // DLPF configuration

#define XA_OFFS_H        0x06       //[15:0] XA_OFFS --> Warning these are 16-bit registers!!!
#define YA_OFFS_H        0x08       //[15:0] YA_OFFS
#define ZA_OFFS_H        0x0A       //[15:0] ZA_OFFS

#define XG_OFFS_USRH     0x13       //[15:0] XG_OFFS_USR  --> Warning these are 16-bit registers!!!
#define YG_OFFS_USRH     0x15       //[15:0] YG_OFFS_USR
#define ZG_OFFS_USRH     0x17       //[15:0] ZG_OFFS_USR



// **************  MPU6050 registers internal bits: *********************

/** PWR_MGMT_1 REGISTER **/
#define MPU6050_PWR1_DEVICE_RESET_BIT       7   // Reset all registers bit
#define MPU6050_PWR1_SLEEP_BIT 				      6	// Enable/disable power mode
#define MPU6050_PWR1_CYCLE_BIT              5   // cycle mode bit
#define MPU6050_PWR1_TEMP_DIS_BIT           3   // Enable/disable temperature sensor

/** PWR_MGMT_2 REGISTER **/
#define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU6050_PWR2_STBY_XA_BIT            5   // Accelerometer axes standby mode
#define MPU6050_PWR2_STBY_YA_BIT            4
#define MPU6050_PWR2_STBY_ZA_BIT            3
#define MPU6050_PWR2_STBY_XG_BIT            2   // Gyroscope axes standby mode
#define MPU6050_PWR2_STBY_YG_BIT            1
#define MPU6050_PWR2_STBY_ZG_BIT            0

/** ACCEL_CONFIG_REG **/
#define MPU6050_ACONFIG_AFS_SEL_BIT         4   // Select Accelerometer FS range
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2   // Select HPF configurations
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

/** ACCEL_HPF_CONFIGS **/
#define MPU6050_ACONFIG_ACCEL_HPF_BIT       2   // Select HPF configurations
#define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

/** INTERNAL DLPF CONFIGS **/
#define MPU6050_CFG_DLPF_CFG_BIT            2   // Select LPF configurations
#define MPU6050_CFG_DLPF_CFG_LENGTH         3

/** GYRO_CONFIG_REG **/
#define MPU6050_GCONFIG_FS_SEL_BIT          4	// Select Gyroscope FS range
#define MPU6050_GCONFIG_FS_SEL_LENGTH   	  2

/** INTERRUPT_ENABLE_REG **/
#define MPU6050_INTERRUPT_MOT_BIT           6   // Motion detection interrupt bit
#define MPU6050_INTERRUPT_ZMOT_BIT          5   // Zero motion detection interrupt bit

/** MOTION INTERRUPT STATUS **/
#define MPU6050_MOTION_MOT_ZRMOT_BIT        0   // Zero motion detection interrupt bit

/** INTERRUPT_CFG_REG **/
#define MPU6050_INTCFG_INT_LEVEL_BIT        7   // 0 = high, 1 = low
#define MPU6050_INTCFG_INT_OPEN_BIT         6   // 0 = push-pull, 1 = openDrain
#define MPU6050_INTCFG_LATCH_INT_EN_BIT     5   // 0 = 50us pulse, 1 = high until cleared
#define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4   // 0 = INT status cleared by reading 0x3A, 1 = INT cleared by reading any reg

/** DMP internal bits **/
#define MPU6050_USERCTRL_DMP_EN_BIT         7   // bit 7 used to disable DMP (save ~100 uA)

/** CYCLING MODE WAKE FREQUENCY **/
#define MPU6050_WAKE_FREQ_1P25              0x0
#define MPU6050_WAKE_FREQ_5                 0x1
#define MPU6050_WAKE_FREQ_20                0x2
#define MPU6050_WAKE_FREQ_40                0x3


/** OTHER GYROSCOPE CONFIGS **/

// Gyroscope Full scale values:
typedef enum {
    G250DPS,	// 0 = 250  °/s --> allows maximum resolution
    G500DPS,	// 1 = 500  °/s
    G1000DPS,	// 2 = 1000 °/s
    G2000DPS	// 3 = 2000 °/s
} GYRO_FS;

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03



/** OTHER ACCELEROMETER CONFIGS **/

// Accelerometer Full scale values:
typedef enum {
    A2G,				// 0 = 2g --> maximum possible resolution
    A4G,				// 1 = 4g
    A8G,				// 2 = 8g
    A16G				// 3 = 16g
} ACCEL_FS;

#define MPU6050_ACCEL_FS_2         	0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

// Accelerometer DHPF values:
#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07

// Accelerometer & Gyroscope DLPF values:
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

// Save accelerometer and gyroscope resolution:
extern float accelerationResolution;
extern float gyroscopeResolution;



// **************  I2C support functions: *********************

HAL_StatusTypeDef I2Cdev_readBit(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

HAL_StatusTypeDef I2Cdev_writeBit(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);

HAL_StatusTypeDef I2Cdev_readBits(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);

HAL_StatusTypeDef I2Cdev_writeBits(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

HAL_StatusTypeDef I2Cdev_readByte(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t *data);

HAL_StatusTypeDef I2Cdev_writeByte(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t data);

HAL_StatusTypeDef I2Cdev_readBytes(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

HAL_StatusTypeDef I2Cdev_writeWord(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint16_t value);

// **************  Actual MPU6050 functions: *********************

uint8_t MPU6050_Initialize(I2C_HandleTypeDef *I2Cx, ACCEL_FS accelRange, GYRO_FS gyroRange, int16_t* offsets);


void MPU6050_reset(I2C_HandleTypeDef *I2Cx);


uint8_t MPU6050_getRate(I2C_HandleTypeDef *I2Cx);

void MPU6050_setRate(I2C_HandleTypeDef *I2Cx, uint8_t rate);


bool MPU6050_getSleepEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setSleepEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


uint8_t MPU6050_getIntEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setIntEnabled(I2C_HandleTypeDef *I2Cx, uint8_t enabled);


bool MPU6050_getIntMotionEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setIntMotionEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


bool MPU6050_getIntZeroMotionEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setIntZeroMotionEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


uint8_t MPU6050_getMotionDetectionDuration(I2C_HandleTypeDef *I2Cx);

void MPU6050_setMotionDetectionDuration(I2C_HandleTypeDef *I2Cx, uint8_t duration);


uint8_t MPU6050_getZeroMotionDetectionDuration(I2C_HandleTypeDef *I2Cx);

void MPU6050_setZeroMotionDetectionDuration(I2C_HandleTypeDef *I2Cx, uint8_t duration);


uint8_t MPU6050_getMotionDetectionThreshold(I2C_HandleTypeDef *I2Cx);

void MPU6050_setMotionDetectionThreshold(I2C_HandleTypeDef *I2Cx, uint8_t threshold);


uint8_t MPU6050_getZeroMotionDetectionThreshold(I2C_HandleTypeDef *I2Cx);

void MPU6050_setZeroMotionDetectionThreshold(I2C_HandleTypeDef *I2Cx, uint8_t threshold);


void MPU6050_setWakeCycleEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);

bool MPU6050_getWakeCycleEnabled(I2C_HandleTypeDef *I2Cx);


bool MPU6050_getInterruptLatch(I2C_HandleTypeDef *I2Cx);

void MPU6050_setInterruptLatch(I2C_HandleTypeDef *I2Cx, bool latch);


bool MPU6050_getInterruptLatchClear(I2C_HandleTypeDef *I2Cx);

void MPU6050_setInterruptLatchClear(I2C_HandleTypeDef *I2Cx, bool clear);


bool MPU6050_getInterruptDrive(I2C_HandleTypeDef *I2Cx);

void MPU6050_setInterruptDrive(I2C_HandleTypeDef *I2Cx, bool drive);


bool MPU6050_getInterruptMode(I2C_HandleTypeDef *I2Cx);

void MPU6050_setInterruptMode(I2C_HandleTypeDef *I2Cx, bool mode);


uint8_t MPU6050_getIntStatus(I2C_HandleTypeDef *I2Cx);

bool MPU6050_getIntMotionStatus(I2C_HandleTypeDef *I2Cx);

bool MPU6050_getIntZeroMotionStatus(I2C_HandleTypeDef *I2Cx);

bool MPU6050_getZeroMotionDetected(I2C_HandleTypeDef *I2Cx);


uint8_t MPU6050_getFullScaleAccelRange(I2C_HandleTypeDef *I2Cx);

void MPU6050_setFullScaleAccelRange(I2C_HandleTypeDef *I2Cx, uint8_t range);


void MPU6050_setFullScaleGyroRange(I2C_HandleTypeDef *I2Cx, uint8_t range);

uint8_t MPU6050_getFullScaleGyroRange(I2C_HandleTypeDef *I2Cx);


void MPU6050_setTempSensorEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);

bool MPU6050__getTempSensorEnabled(I2C_HandleTypeDef *I2Cx);


uint8_t MPU6050_getWakeFrequency(I2C_HandleTypeDef *I2Cx);

void MPU6050_setWakeFrequency(I2C_HandleTypeDef *I2Cx, uint8_t frequency);


bool MPU6050_getStandbyXAccelEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setStandbyXAccelEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


bool MPU6050_getStandbyYAccelEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setStandbyYAccelEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


bool MPU6050_getStandbyZAccelEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setStandbyZAccelEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


bool MPU6050_getStandbyXGyroEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setStandbyXGyroEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


bool MPU6050_getStandbyYGyroEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setStandbyYGyroEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


bool MPU6050_getStandbyZGyroEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setStandbyZGyroEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


uint8_t MPU6050_getDHPFMode(I2C_HandleTypeDef *I2Cx);

void MPU6050_setDHPFMode(I2C_HandleTypeDef *I2Cx, uint8_t bandwidth);


uint8_t MPU6050_getDLPFMode(I2C_HandleTypeDef *I2Cx);

void MPU6050_setDLPFMode(I2C_HandleTypeDef *I2Cx, uint8_t mode);


bool MPU6050_getDMPEnabled(I2C_HandleTypeDef *I2Cx);

void MPU6050_setDMPEnabled(I2C_HandleTypeDef *I2Cx, bool enabled);


void MPU6050_getAcceleration(I2C_HandleTypeDef *I2Cx, int16_t* x, int16_t* y, int16_t* z);

int16_t MPU6050_getTemperature(I2C_HandleTypeDef *I2Cx);

void MPU6050_getRotation(I2C_HandleTypeDef *I2Cx, int16_t* x, int16_t* y, int16_t* z);

void MPU6050_getMotion6(I2C_HandleTypeDef *I2Cx, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);


void MPU6050_setupMotionInt(I2C_HandleTypeDef *I2Cx, uint8_t duration, uint8_t threshold, int16_t* offsets);

void MPU6050_setupZeroMotionInt(I2C_HandleTypeDef *I2Cx, uint8_t duration, uint8_t threshold, int16_t* offsets);


int16_t MPU6050_getXAccelOffset(I2C_HandleTypeDef *I2Cx);

void MPU6050_setXAccelOffset(I2C_HandleTypeDef *I2Cx, int16_t offset);

int16_t MPU6050_getYAccelOffset(I2C_HandleTypeDef *I2Cx);

void MPU6050_setYAccelOffset(I2C_HandleTypeDef *I2Cx, int16_t offset);

int16_t MPU6050_getZAccelOffset(I2C_HandleTypeDef *I2Cx);

void MPU6050_setZAccelOffset(I2C_HandleTypeDef *I2Cx, int16_t offset);


int16_t MPU6050_getXGyroOffset(I2C_HandleTypeDef *I2Cx);

void MPU6050_setXGyroOffset(I2C_HandleTypeDef *I2Cx, int16_t offset);

int16_t MPU6050_getYGyroOffset(I2C_HandleTypeDef *I2Cx);

void MPU6050_setYGyroOffset(I2C_HandleTypeDef *I2Cx, int16_t offset);

int16_t MPU6050_getZGyroOffset(I2C_HandleTypeDef *I2Cx);

void MPU6050_setZGyroOffset(I2C_HandleTypeDef *I2Cx, int16_t offset);


void MPU6050_selfCalibration(I2C_HandleTypeDef *I2Cx, ACCEL_FS accelRange, GYRO_FS gyroRange, uint16_t readings, uint8_t acel_deadzone, uint8_t gyro_deadzone);

void MPU6050_meanSensors(I2C_HandleTypeDef *I2Cx, uint16_t buffersize, int16_t* meanVals);

void MPU6050_calibration(I2C_HandleTypeDef *I2Cx, int16_t* meanVals, uint16_t readings, uint8_t acel_deadzone, uint8_t gyro_deadzone);


