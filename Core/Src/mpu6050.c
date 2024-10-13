/*
 * mpu6050.c
 *
 *  Created on: Apr 16, 2024
 *  Author: Marco Migliorini
 *
 *  The following code is intended for STM32 boards.
 *  The library is adapted from Jeff Rowberg's existing library for the MPU6050 and Arduino boards
 *  (see his GitHub page: https://github.com/ElectronicCats/mpu6050/blob/master/src/MPU6050.cpp).
 *
 *  The following code only contains the functions necessary to setup motion interrupt, zero motion interrupt, perform
 *  simple reading on accelerometer and gyroscope axes and implement buffering overflow using the MPU internal FIFO.
 */

#include <math.h>

#include "mpu6050.h"



// Global variables definition:
float accelerationResolution;
float gyroscopeResolution;



/**
 ****************************************************************************************************
 ********************************* I2C SUPPORT FUNCTIONS: *******************************************
 ****************************************************************************************************
*/


/** Read a single bit from an 8-bit device register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
HAL_StatusTypeDef I2Cdev_readBit(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
  uint8_t b;
  HAL_StatusTypeDef status =  HAL_I2C_Mem_Read(I2Cx, devAddr, regAddr, 1, &b, 1, HAL_MAX_DELAY);
  if (status == HAL_OK){
    //*data = b & (1 << bitNum);
    *data = (b >> bitNum) & 0x01;
  }
  return status;
}


/** write a single bit in an 8-bit device register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
HAL_StatusTypeDef I2Cdev_writeBit(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
  uint8_t b;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, devAddr, regAddr, 1, &b, 1, HAL_MAX_DELAY);
  if (status == HAL_OK) {
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    status = HAL_I2C_Mem_Write(I2Cx, devAddr, regAddr, 1, &b, 1, HAL_MAX_DELAY);
  }
  return status;
}


/** Read multiple bits from an 8-bit device register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
HAL_StatusTypeDef I2Cdev_readBits(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  uint8_t b;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2Cx, devAddr, regAddr, 1, &b, 1, HAL_MAX_DELAY);
  if (status == HAL_OK) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);		// (1 << length)-1: creates length-subsequent ones
                                    // Then the mask is shifted into correct position
    b &= mask;						  // Set to zero non important bits
    b >>= (bitStart - length + 1);    // Shift significant bits at the beginning of the sequence
    *data = b;
  }
  return status;
}


/** Write multiple bits in an 8-bit device register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
HAL_StatusTypeDef I2Cdev_writeBits(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  uint8_t b;

  if (HAL_I2C_Mem_Read(I2Cx, devAddr, regAddr, 1, &b, 1, HAL_MAX_DELAY) == HAL_OK) {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1); // (1 << length)-1: creates length-subsequent ones
                                   // Then the mask is shifted into correct position
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    return HAL_I2C_Mem_Write(I2Cx, devAddr, regAddr, 1, &b, 1, HAL_MAX_DELAY);
  } else {
    return HAL_ERROR;
  }
}


/** Read single byte from an 8-bit device register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
HAL_StatusTypeDef I2Cdev_readByte(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t *data) {
  return HAL_I2C_Mem_Read(I2Cx, devAddr, regAddr, 1, data, 1, HAL_MAX_DELAY);
}


/** Write single byte to an 8-bit device register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
HAL_StatusTypeDef I2Cdev_writeByte(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t data) {
  return HAL_I2C_Mem_Write(I2Cx, devAddr, regAddr, 1, &data, 1, HAL_MAX_DELAY);
}


/** Read multiple bytes from an 8-bit device register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return Status of operation (true = success)
 */
HAL_StatusTypeDef I2Cdev_readBytes(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint16_t length, uint8_t *data) {
  return HAL_I2C_Mem_Read(I2Cx, devAddr, regAddr, 1, data, length, HAL_MAX_DELAY);
}


/** Write single word to a 16-bit device register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
HAL_StatusTypeDef I2Cdev_writeWord(I2C_HandleTypeDef *I2Cx, uint8_t devAddr, uint8_t regAddr, uint16_t value) {
  uint8_t data[2];
  data[0] = (uint8_t)(value >> 8);    // Most significant byte;
  data[1] = (uint8_t)(value & 0xFF);  // Least significant byte;
  return HAL_I2C_Mem_Write(I2Cx, devAddr, regAddr, 1, data, 2, HAL_MAX_DELAY);
}


/**
 *******************************************************************************************************
 ********************************* MPU6050 SUPPORT FUNCTIONS *******************************************
 *******************************************************************************************************
*/



/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets the clock source to use the default internal clock source.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param accelRange accelerometer FS range, see ACCEL_FS in mpu6050.h
 * @param gyroRange gyroscope FS range, see GYRO_FS in mpu6050.h
 * @param offsets pointer to buffer containing offsets for both accelerometer and gyroscope
 * @return initialization status (true if correctly done)
 */
uint8_t MPU6050_Initialize(I2C_HandleTypeDef *I2Cx, ACCEL_FS accelRange, GYRO_FS gyroRange, int16_t* offsets)
{
  uint8_t check;
  uint8_t Data;

  // check device ID WHO_AM_I

  HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);

  if (check == 104) // 0x68 will be returned by the sensor if everything goes well
  {
    // Power management register 0X6B we should write all 0's to wake the sensor up
    // Also reset all the registers status
    Data = 0x00;
    HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 100);

    // Set accelerometer configuration in ACCEL_CONFIG Register
    switch (accelRange)
    {
      case A2G:
      MPU6050_setFullScaleAccelRange(I2Cx, MPU6050_ACCEL_FS_2);
      accelerationResolution = 2.0 / 32768.0;
      break;

      case A4G:
      MPU6050_setFullScaleAccelRange(I2Cx, MPU6050_ACCEL_FS_4);
      accelerationResolution = 4.0 / 32768.0;
      break;

      case A8G:
      MPU6050_setFullScaleAccelRange(I2Cx, MPU6050_ACCEL_FS_8);
      accelerationResolution = 8.0 / 32768.0;
      break;

      case A16G:
      MPU6050_setFullScaleAccelRange(I2Cx, MPU6050_ACCEL_FS_16);
      accelerationResolution = 16.0 / 32768.0;
      break;

      default:
      return 1;
    }

    // Set Gyroscope configuration in GYRO_CONFIG Register
    switch (gyroRange)
    {
      case G250DPS:
      MPU6050_setFullScaleGyroRange(I2Cx, MPU6050_GYRO_FS_250);
      gyroscopeResolution = 250.0 / 32768.0;
      break;

      case G500DPS:
      MPU6050_setFullScaleGyroRange(I2Cx, MPU6050_GYRO_FS_500);
      gyroscopeResolution = 500.0 / 32768.0;
      break;

      case G1000DPS:
      MPU6050_setFullScaleGyroRange(I2Cx, MPU6050_GYRO_FS_1000);
      gyroscopeResolution = 1000.0 / 32768.0;
      break;

      case G2000DPS:
      MPU6050_setFullScaleGyroRange(I2Cx, MPU6050_GYRO_FS_2000);
      gyroscopeResolution = 2000.0 / 32768.0;
      break;

      default:
      return 1;
    }

    // make sure everything is running:
    MPU6050_setSleepEnabled(I2Cx, false);

    // set correct offset values:
    MPU6050_setXAccelOffset(I2Cx, offsets[0]);
    MPU6050_setYAccelOffset(I2Cx, offsets[1]);
    MPU6050_setZAccelOffset(I2Cx, offsets[2]);

    MPU6050_setXGyroOffset(I2Cx, offsets[3]);
    MPU6050_setYGyroOffset(I2Cx, offsets[4]);
    MPU6050_setZGyroOffset(I2Cx, offsets[5]);

    // turn off temperature sensor, saves ~ 60 uA in active mode
    MPU6050_setTempSensorEnabled(I2Cx, 0);

    // turn off internal DMP, saves ~ 100 uA in active mode
    MPU6050_setDMPEnabled(I2Cx, 0);
  }
  return 0;
}


/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050_reset(I2C_HandleTypeDef *I2Cx) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}


/** Get gyroscope output rate divider.
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6050 Product Specification document.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current sample rate
 * @see MPU6050_RA_SMPLRT_DIV
 */
uint8_t MPU6050_getRate(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readByte(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, &data);
  return data;
}


/** Set gyroscope sample rate divider.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void MPU6050_setRate(I2C_HandleTypeDef *I2Cx, uint8_t rate) {
  I2Cdev_writeByte(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, rate);
}


/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_getSleepEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, MPU6050_PWR1_SLEEP_BIT, &data);
  return data != 0;
}


/** Set sleep mode status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see PWR_MGMT_1_REG
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_setSleepEnabled(I2C_HandleTypeDef *I2Cx, bool enabled){
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, MPU6050_PWR1_SLEEP_BIT, enabled);
}


/** Get full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit will be
 * set 0 for disabled, 1 for enabled.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
uint8_t MPU6050_getIntEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readByte(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, &data);
  return data;
}


/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
void MPU6050_setIntEnabled(I2C_HandleTypeDef *I2Cx, uint8_t enabled) {
  I2Cdev_writeByte(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, enabled);
}


/** Get Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_MOT_BIT
 **/
bool MPU6050_getIntMotionEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, MPU6050_INTERRUPT_MOT_BIT, &data);
  return data != 0;
}


/** Set Motion Detection interrupt enabled status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New interrupt enabled status
 * @see getIntMotionEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_MOT_BIT
 **/
void MPU6050_setIntMotionEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, MPU6050_INTERRUPT_MOT_BIT, enabled);
}


/** Get Zero Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_ZMOT_BIT
 **/
bool MPU6050_getIntZeroMotionEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, MPU6050_INTERRUPT_ZMOT_BIT, &data);
  return data != 0;
}


/** Set Zero Motion Detection interrupt enabled status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New interrupt enabled status
 * @see getIntZeroMotionEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_ZMOT_BIT
 **/
void MPU6050_setIntZeroMotionEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
    I2Cdev_writeBit(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, MPU6050_INTERRUPT_ZMOT_BIT, enabled);
}


/** Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
 * of 1LSB = 1ms. The Motion detection duration counter increments when the
 * absolute value of any of the accelerometer measurements exceeds the Motion
 * detection threshold (Register 31). The Motion detection interrupt is
 * triggered when the Motion detection counter reaches the time count specified
 * in this register.
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current motion detection duration threshold value (LSB = 1ms)
 * @see MPU6050_RA_MOT_DUR
 */
uint8_t MPU6050_getMotionDetectionDuration(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readByte(I2Cx, MPU6050_ADDR, MOT_DUR_TH_REG, &data);
  return data;
}


/** Set motion detection event duration threshold.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void MPU6050_setMotionDetectionDuration(I2C_HandleTypeDef *I2Cx, uint8_t duration) {
  I2Cdev_writeByte(I2Cx, MPU6050_ADDR, MOT_DUR_TH_REG, duration);
}


/** Get zero motion detection event duration threshold.
 * This register configures the duration counter threshold for Zero Motion
 * interrupt generation. The duration counter ticks at 16 Hz, therefore
 * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
 * increments while the absolute value of the accelerometer measurements are
 * each less than the detection threshold (Register 33). The Zero Motion
 * interrupt is triggered when the Zero Motion duration counter reaches the time
 * count specified in this register.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56
 * and 58 of this document.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current zero motion detection duration threshold value (LSB = 64ms)
 * @see MPU6050_RA_ZRMOT_DUR
 */
uint8_t MPU6050_getZeroMotionDetectionDuration(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readByte(I2Cx, MPU6050_ADDR, ZRMOT_DUR_REG, &data);
  return data;
}


/** Set zero motion detection event duration threshold.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void MPU6050_setZeroMotionDetectionDuration(I2C_HandleTypeDef *I2Cx, uint8_t duration) {
    I2Cdev_writeByte(I2Cx, MPU6050_ADDR, ZRMOT_DUR_REG, duration);
}


/** Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
 * absolute value of any of the accelerometer measurements exceeds this Motion
 * detection threshold. This condition increments the Motion detection duration
 * counter (Register 32). The Motion detection interrupt is triggered when the
 * Motion Detection counter reaches the time count specified in MOT_DUR
 * (Register 32).
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in MOT_DETECT_STATUS (Register 97).
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_MOT_THR
 */
uint8_t MPU6050_getMotionDetectionThreshold(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readByte(I2Cx, MPU6050_ADDR, MOT_DET_TH_REG, &data);
  return data;
}


/** Set motion detection event acceleration threshold.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void MPU6050_setMotionDetectionThreshold(I2C_HandleTypeDef *I2Cx, uint8_t threshold) {
  I2Cdev_writeByte(I2Cx, MPU6050_ADDR, MOT_DET_TH_REG, threshold);
}


/** Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
 * the absolute value of the accelerometer measurements for the 3 axes are each
 * less than the detection threshold. This condition increments the Zero Motion
 * duration counter (Register 34). The Zero Motion interrupt is triggered when
 * the Zero Motion duration counter reaches the time count specified in
 * ZRMOT_DUR (Register 34).
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
 * interrupt both when Zero Motion is first detected and when Zero Motion is no
 * longer detected.
 *
 * When a zero motion event is detected, a Zero Motion Status will be indicated
 * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
 * condition is detected, the status bit is set to 1. When a zero-motion-to-
 * motion condition is detected, the status bit is set to 0.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current zero motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_ZRMOT_THR
 */
uint8_t MPU6050_getZeroMotionDetectionThreshold(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readByte(I2Cx, MPU6050_ADDR, ZRMOT_THR_REG, &data);
  return data;
}


/** Set zero motion detection event acceleration threshold.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void MPU6050_setZeroMotionDetectionThreshold(I2C_HandleTypeDef *I2Cx, uint8_t threshold) {
    I2Cdev_writeByte(I2Cx, MPU6050_ADDR, ZRMOT_THR_REG, threshold);
}


/** Get wake cycle enabled status.
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
bool MPU6050_getWakeCycleEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, MPU6050_PWR1_CYCLE_BIT, &data);
  return data != 0;
}


/** Set wake cycle enabled status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New sleep mode enabled status
 * @see getWakeCycleEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
void MPU6050_setWakeCycleEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, MPU6050_PWR1_CYCLE_BIT, enabled);
}


/** Get interrupt latch mode.
 * Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_LATCH_INT_EN_BIT
 */
bool MPU6050_getInterruptLatch(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, &data);
  return data != 0;
}


/** Set interrupt latch mode.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_LATCH_INT_EN_BIT
 */
void MPU6050_setInterruptLatch(I2C_HandleTypeDef *I2Cx, bool latch) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}


/** Get interrupt latch clear mode.
 * Will be set 0 for status-read-only, 1 for any-register-read.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current latch clear mode (0=status-read-only, 1=any-register-read)
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_RD_CLEAR_BIT
 */
bool MPU6050_getInterruptLatchClear(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, &data);
  return data != 0;
}


/** Set interrupt latch clear mode.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_RD_CLEAR_BIT
 */
void MPU6050_setInterruptLatchClear(I2C_HandleTypeDef *I2Cx, bool clear) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}


/** Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt drive mode (0=push-pull, 1=open-drain)
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_OPEN_BIT
 */
bool MPU6050_getInterruptDrive(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, &data);
  return data != 0;
}


/** Set interrupt drive mode.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_OPEN_BIT
 */
void MPU6050_setInterruptDrive(I2C_HandleTypeDef *I2Cx, bool drive) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive);
}


/** Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_LEVEL_BIT
 */
bool MPU6050_getInterruptMode(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, &data);
  return data != 0;
}


/** Set interrupt logic level mode.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_LEVEL_BIT
 */
void MPU6050_setInterruptMode(I2C_HandleTypeDef *I2Cx, bool mode) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}


/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT status, since each single bit-read operation clears
 * the entire register.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t MPU6050_getIntStatus(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readByte(I2Cx, MPU6050_ADDR, INT_STATUS_REG, &data);
  return data;
}


/** Get Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Motion Detection interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_MOT_BIT
 */
bool MPU6050_getIntMotionStatus(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_STATUS_REG, MPU6050_INTERRUPT_MOT_BIT, &data);
  return data != 0;
}


/** Get Zero Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Zero Motion Detection interrupt has
 * been generated. The bit clears to 0 after the register has been read.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_ZMOT_BIT
 */
bool MPU6050_getIntZeroMotionStatus(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_STATUS_REG, MPU6050_INTERRUPT_ZMOT_BIT, &data);
  return data != 0;
}


/** Get zero motion detection interrupt status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZRMOT_BIT
 */
bool MPU6050_getZeroMotionDetected(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MOT_DETECT_STATUS_REG, MPU6050_MOTION_MOT_ZRMOT_BIT, &data);
  return data != 0;
}


/** Get temperature sensor enabled status.
 * Control the usage of the internal temperature sensor.
 *
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current temperature sensor enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
bool MPU6050__getTempSensorEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, MPU6050_PWR1_TEMP_DIS_BIT, &data);
  return data == 0; // 1 is actually disabled here
}


/** Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New temperature sensor enabled status
 * @see getTempSensorEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
void MPU6050_setTempSensorEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  // 1 is actually disabled here
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}


/** Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * <pre>
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 20 Hz
 * 3            | 40 Hz
 * </pre>
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current wake frequency
 * @see MPU6050_RA_PWR_MGMT_2
 */
uint8_t MPU6050_getWakeFrequency(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBits(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, &data);
  return data;
}


/** Set wake frequency in Accel-Only Low Power Mode.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param frequency New wake frequency
 * @see MPU6050_RA_PWR_MGMT_2
 */
void MPU6050_setWakeFrequency(I2C_HandleTypeDef *I2Cx, uint8_t frequency) {
  I2Cdev_writeBits(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}


/** Get X-axis accelerometer standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current X-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XA_BIT
 */
bool MPU6050_getStandbyXAccelEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_XA_BIT, &data);
  return data != 0;
}


/** Set X-axis accelerometer standby enabled status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param New X-axis standby enabled status
 * @see getStandbyXAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XA_BIT
 */
void MPU6050_setStandbyXAccelEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_XA_BIT, enabled);
}


/** Get Y-axis accelerometer standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current Y-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YA_BIT
 */
bool MPU6050_getStandbyYAccelEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_YA_BIT, &data);
  return data != 0;
}


/** Set Y-axis accelerometer standby enabled status.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param New Y-axis standby enabled status
 * @see getStandbyYAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YA_BIT
 */
void MPU6050_setStandbyYAccelEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_YA_BIT, enabled);
}


/** Get Z-axis accelerometer standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current Z-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZA_BIT
 */
bool MPU6050_getStandbyZAccelEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_ZA_BIT, &data);
  return data != 0;
}


/** Set Z-axis accelerometer standby enabled status.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param New Z-axis standby enabled status
 * @see getStandbyZAccelEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZA_BIT
 */
void MPU6050_setStandbyZAccelEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_ZA_BIT, enabled);
}


/** Get X-axis gyroscope standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current X-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XG_BIT
 */
bool MPU6050_getStandbyXGyroEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_XG_BIT, &data);
  return data != 0;
}


/** Set X-axis gyroscope standby enabled status.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param New X-axis standby enabled status
 * @see getStandbyXGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_XG_BIT
 */
void MPU6050_setStandbyXGyroEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_XG_BIT, enabled);
}


/** Set full-scale accelerometer range.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void MPU6050_setFullScaleAccelRange(I2C_HandleTypeDef *I2Cx, uint8_t range) {
  I2Cdev_writeBits(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}


/** Get Y-axis gyroscope standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current Y-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YG_BIT
 */
bool MPU6050_getStandbyYGyroEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_YG_BIT, &data);
  return data != 0;
}


/** Set Y-axis gyroscope standby enabled status.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param New Y-axis standby enabled status
 * @see getStandbyYGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_YG_BIT
 */
void MPU6050_setStandbyYGyroEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_YG_BIT, enabled);
}


/** Get Z-axis gyroscope standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current Z-axis standby enabled status
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZG_BIT
 */
bool MPU6050_getStandbyZGyroEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_ZG_BIT, &data);
  return data != 0;
}


/** Set Z-axis gyroscope standby enabled status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param New Z-axis standby enabled status
 * @see getStandbyZGyroEnabled()
 * @see MPU6050_RA_PWR_MGMT_2
 * @see MPU6050_PWR2_STBY_ZG_BIT
 */
void MPU6050_setStandbyZGyroEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, MPU6050_PWR2_STBY_ZG_BIT, enabled);
}


/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_getFullScaleAccelRange(I2C_HandleTypeDef *I2Cx) {
	uint8_t data;
  I2Cdev_readBits(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &data);
  return data;
}


/** Set full-scale gyroscope range.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(I2C_HandleTypeDef *I2Cx, uint8_t range) {
  I2Cdev_writeBits(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}


/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_getFullScaleGyroRange(I2C_HandleTypeDef *I2Cx) {
	uint8_t data;
  I2Cdev_readBits(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &data);
  return data;
}


/** Get the high-pass filter configuration.
 * The DHPF is a filter module in the path leading to motion detectors (Free
 * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
 * available to the data registers (see Figure in Section 8 of the MPU-6000/
 * MPU-6050 Product Specification document).
 *
 * The high pass filter has three modes:
 *
 * <pre>
 *    Reset: The filter output settles to zero within one sample. This
 *           effectively disables the high pass filter. This mode may be toggled
 *           to quickly settle the filter.
 *
 *    On:    The high pass filter will pass signals above the cut off frequency.
 *
 *    Hold:  When triggered, the filter holds the present sample. The filter
 *           output will be the difference between the input sample and the held
 *           sample.
 * </pre>
 *
 * <pre>
 * ACCEL_HPF | Filter Mode | Cut-off Frequency
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 * </pre>
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current high-pass filter configuration
 * @see MPU6050_DHPF_RESET
 * @see MPU6050_RA_ACCEL_CONFIG
 */
uint8_t MPU6050_getDHPFMode(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBits(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, &data);
  return data;
}


/** Set the high-pass filter configuration.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param bandwidth New high-pass filter configuration
 * @see setDHPFMode()
 * @see MPU6050_DHPF_RESET
 * @see MPU6050_RA_ACCEL_CONFIG
 */
void MPU6050_setDHPFMode(I2C_HandleTypeDef *I2Cx, uint8_t bandwidth) {
  I2Cdev_writeBits(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}


/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return DLFP configuration
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
uint8_t MPU6050_getDLPFMode(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBits(I2Cx, MPU6050_ADDR, DLPF_CFG_REG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, &data);
  return data;
}


/** Set digital low-pass filter configuration.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void MPU6050_setDLPFMode(I2C_HandleTypeDef *I2Cx, uint8_t mode) {
  I2Cdev_writeBits(I2Cx, MPU6050_ADDR, DLPF_CFG_REG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}


/** Get DMP enabled status
 * Disable to save ~100 uA during active mode of the device
 * During cycling mode DMP has no effect on power consumption
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current DMP enabled status
 * @see Online gitHub page, no documentation is provided about DMP
 */
bool MPU6050_getDMPEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, USER_CTRL_REG, MPU6050_USERCTRL_DMP_EN_BIT, &data);
  return data != 0;
}


/** Set DMP enabled status.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled updated DMP status
 * @see gitHub page, no documentation provided about this register
 */
void MPU6050_setDMPEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, USER_CTRL_REG, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}


/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 16384 LSB/mg
 * 1       | +/- 4g           | 8192 LSB/mg
 * 2       | +/- 8g           | 4096 LSB/mg
 * 3       | +/- 16g          | 2048 LSB/mg
 * </pre>
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPU6050_getAcceleration(I2C_HandleTypeDef *I2Cx, int16_t* x, int16_t* y, int16_t* z) {
  uint8_t data[6];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 6, &data[0]);
  *x = (((int16_t)data[0]) << 8) | data[1];
  *y = (((int16_t)data[2]) << 8) | data[3];
  *z = (((int16_t)data[4]) << 8) | data[5];
}


/** Get current internal temperature.
 * Get actual temperature by doing: (float)temp_raw / 340.0f + 36.53f;
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Temperature reading in 16-bit 2's complement format
 * @see MPU6050_RA_TEMP_OUT_H
 */
int16_t MPU6050_getTemperature(I2C_HandleTypeDef *I2Cx) {
  uint8_t data[2];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 2, &data[0]);
  return (((int16_t)data[0]) << 8) | data[1];
}


/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPU6050_getRotation(I2C_HandleTypeDef *I2Cx, int16_t* x, int16_t* y, int16_t* z) {
  uint8_t data[6];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 6, &data[0]);
  *x = (((int16_t)data[0]) << 8) | data[1];
  *y = (((int16_t)data[2]) << 8) | data[3];
  *z = (((int16_t)data[4]) << 8) | data[5];
}


/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050_getMotion6(I2C_HandleTypeDef *I2Cx, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  uint8_t data[14];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 14, &data[0]);
  *ax = (((int16_t)data[0]) << 8) | data[1];
  *ay = (((int16_t)data[2]) << 8) | data[3];
  *az = (((int16_t)data[4]) << 8) | data[5];
  // skip bytes 6 and 7 reserved for temperature readings
  *gx = (((int16_t)data[8]) << 8) | data[9];
  *gy = (((int16_t)data[10]) << 8) | data[11];
  *gz = (((int16_t)data[12]) << 8) | data[13];
}


/** Procedure to setup motion interrupt
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param duration 8-bit unsigned integer container for motion duration threshold in ms
 * @param threshold 8-bit unsigned integer container for motion detection threshold in mg
 * @param offsets data buffer containing gyroscope and accelerometer offsets to be used during initialization
 */
void MPU6050_setupMotionInt(I2C_HandleTypeDef *I2Cx, uint8_t duration, uint8_t threshold, int16_t* offsets) {

  // make sure standard settings are used
  MPU6050_Initialize(I2Cx, A2G, G250DPS, offsets);

  //set trigger event: Active high until interrupt status register is cleared, push-pull configuration
  MPU6050_setInterruptLatch(I2Cx, 1);
  MPU6050_setInterruptLatchClear(I2Cx, 0);
  MPU6050_setInterruptDrive(I2Cx, 0);
  MPU6050_setInterruptMode(I2Cx, 0);

  // Ensure the accelerometer is running
  MPU6050_setSleepEnabled(I2Cx, false);
  MPU6050_setWakeCycleEnabled(I2Cx, false);
  MPU6050_setStandbyXAccelEnabled(I2Cx, false);
  MPU6050_setStandbyYAccelEnabled(I2Cx, false);
  MPU6050_setStandbyZAccelEnabled(I2Cx, false);

  //Set the accelerometer HPF to reset settings
  MPU6050_setDHPFMode(I2Cx, MPU6050_DHPF_RESET);

  //Set the accelerometer LPF to 256Hz Bandwidth
  MPU6050_setDLPFMode(I2Cx, MPU6050_DLPF_BW_256);

  //Enable the motion interrupt
  MPU6050_setIntEnabled(I2Cx, 0b00000000);
  MPU6050_setIntMotionEnabled(I2Cx, true);

  //Set the motion detection duration
  MPU6050_setMotionDetectionDuration(I2Cx, duration); //Duration in ms

  //Set the motion detection threshold
  MPU6050_setMotionDetectionThreshold(I2Cx, threshold); // Threshold in 2mg

  //1 ms delay
  HAL_Delay(1);

  //Set the accelerometer HPF to HOLD settings
  MPU6050_setDHPFMode(I2Cx, MPU6050_DHPF_HOLD);

  // Set the wakeup frequency
  MPU6050_setWakeFrequency(I2Cx, MPU6050_WAKE_FREQ_1P25);
  MPU6050_setStandbyXGyroEnabled(I2Cx, true);
  MPU6050_setStandbyYGyroEnabled(I2Cx, true);
  MPU6050_setStandbyZGyroEnabled(I2Cx, true);

  // Enable cycle mode
  MPU6050_setWakeCycleEnabled(I2Cx, true);

}



/** Procedure to setup zero motion interrupt
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param duration 8-bit unsigned integer container for zero motion duration threshold in ms
 * @param threshold 8-bit unsigned integer container for zero motion detection threshold in mg
 * @param offsets data buffer containing gyroscope and accelerometer offsets to be used during initialization
 */
void MPU6050_setupZeroMotionInt(I2C_HandleTypeDef *I2Cx, uint8_t duration, uint8_t threshold, int16_t* offsets) {

  // make sure standard settings are used
  MPU6050_Initialize(I2Cx, A2G, G250DPS, offsets);

  //set trigger event: Active high until interrupt status register is cleared, push-pull configuration
  MPU6050_setInterruptLatch(I2Cx, 1);
  MPU6050_setInterruptLatchClear(I2Cx, 0);
  MPU6050_setInterruptDrive(I2Cx, 0);
  MPU6050_setInterruptMode(I2Cx, 0);

  // Ensure the accelerometer is running
  MPU6050_setSleepEnabled(I2Cx, false);
  MPU6050_setWakeCycleEnabled(I2Cx, false);
  MPU6050_setStandbyXAccelEnabled(I2Cx, false);
  MPU6050_setStandbyYAccelEnabled(I2Cx, false);
  MPU6050_setStandbyZAccelEnabled(I2Cx, false);

  //Set the accelerometer HPF to reset settings
  MPU6050_setDHPFMode(I2Cx, MPU6050_DHPF_RESET);

  //Set the accelerometer LPF to 256Hz Bandwidth
  /*
   * This one is commented since in this application the DLPF affects the output rate.
   * The same parameter is controlled while configuring the MPU6050 internal FIFO.
   */
  //
  //MPU6050_setDLPFMode(I2Cx, MPU6050_DLPF_BW_256);


  //Enable the motion interrupt
  MPU6050_setIntEnabled(I2Cx, 0b00000000);
  MPU6050_setIntZeroMotionEnabled(I2Cx, true);

  //Set the motion detection duration
  MPU6050_setZeroMotionDetectionDuration(I2Cx, duration);   //Duration in ms

  //Set the motion detection threshold
  MPU6050_setZeroMotionDetectionThreshold(I2Cx, threshold); // Threshold in 2mg

  //1 ms delay
  HAL_Delay(1);

  //Set the accelerometer HPF to HOLD settings
  MPU6050_setDHPFMode(I2Cx, MPU6050_DHPF_5);

}


/** Procedure to configure MPU6050 internal FIFO buffer
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param dlpfMode 8-bit unsigned integer variable to specify the lowpass filter frequency, which affects sensors output rate
 * @param freqDivider 8-bit unsigned integer that specifies the sensors update frequency (actual sampFrequency = gyroFreq/freqDivider)
 * @param overflowEnabled bool used to activate the fifo buffer overflow interrupt line
 */
void MPU6050_setupFifoBuffer(I2C_HandleTypeDef *I2Cx, uint8_t dlpfMode, uint8_t freqDivider, bool overflowEnabled) {
  // force gyro output rate to 1kHz
   MPU6050_setDLPFMode(I2Cx, dlpfMode);

   // frequency divider:
   MPU6050_setRate(I2Cx, freqDivider);

   // enable sensors writing to FIFO:
   MPU6050_setXGyroFIFOEnabled(I2Cx, true);
   MPU6050_setYGyroFIFOEnabled(I2Cx, true);
   MPU6050_setZGyroFIFOEnabled(I2Cx, true);
   MPU6050_setAccelFIFOEnabled(I2Cx, true);

   // enable interrupt generation when the FIFO overflows:
   MPU6050_setIntFIFOBufferOverflowEnabled(I2Cx, overflowEnabled);

   //set trigger event: Active high until interrupt status register is cleared, push-pull configuration
   MPU6050_setInterruptLatch(I2Cx, 1);
   MPU6050_setInterruptLatchClear(I2Cx, 0);
   MPU6050_setInterruptDrive(I2Cx, 0);
   MPU6050_setInterruptMode(I2Cx, 0);

   // Enable the FIFO here:
   MPU6050_setFIFOEnabled(I2Cx, true);

}







/*
 * The subsequent functions are not well documented in the product manual
 * however, they have been tested and seem to properly work
 */









/** Return current accelerometer x-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return int16_t value corresponding to the stored offset
 */
int16_t MPU6050_getXAccelOffset(I2C_HandleTypeDef *I2Cx) {
  uint8_t data[2];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, XA_OFFS_H, 2, data);
  return (((int16_t)data[0]) << 8) | data[1];
}


/** Update accelerometer x-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
void MPU6050_setXAccelOffset(I2C_HandleTypeDef *I2Cx, int16_t offset) {
  I2Cdev_writeWord(I2Cx, MPU6050_ADDR, XA_OFFS_H, offset);
}


/** Return current accelerometer y-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return int16_t value corresponding to the stored offset
 */
int16_t MPU6050_getYAccelOffset(I2C_HandleTypeDef *I2Cx) {
  uint8_t data[2];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, YA_OFFS_H, 2, data);
  return (((int16_t)data[0]) << 8) | data[1];
}


/** Update accelerometer y-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
void MPU6050_setYAccelOffset(I2C_HandleTypeDef *I2Cx, int16_t offset) {
  I2Cdev_writeWord(I2Cx, MPU6050_ADDR, YA_OFFS_H, offset);
}


/** Return current accelerometer z-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return int16_t value corresponding to the stored offset
 */
int16_t MPU6050_getZAccelOffset(I2C_HandleTypeDef *I2Cx) {
  uint8_t data[2];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, ZA_OFFS_H, 2, data);
  return (((int16_t)data[0]) << 8) | data[1];
}


/** Update accelerometer z-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
void MPU6050_setZAccelOffset(I2C_HandleTypeDef *I2Cx, int16_t offset) {
  I2Cdev_writeWord(I2Cx, MPU6050_ADDR, ZA_OFFS_H, offset);
}


/** Update gyroscope x-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
int16_t MPU6050_getXGyroOffset(I2C_HandleTypeDef *I2Cx) {
  uint8_t data[2];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, XG_OFFS_USRH, 2, data);
  return (((int16_t)data[0]) << 8) | data[1];
}


/** Update gyroscope x-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
void MPU6050_setXGyroOffset(I2C_HandleTypeDef *I2Cx, int16_t offset) {
  I2Cdev_writeWord(I2Cx, MPU6050_ADDR, XG_OFFS_USRH, offset);
}


/** Update gyroscope y-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
int16_t MPU6050_getYGyroOffset(I2C_HandleTypeDef *I2Cx) {
  uint8_t data[2];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, YG_OFFS_USRH, 2, data);
  return (((int16_t)data[0]) << 8) | data[1];
}


/** Update gyroscope y-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
void MPU6050_setYGyroOffset(I2C_HandleTypeDef *I2Cx, int16_t offset) {
  I2Cdev_writeWord(I2Cx, MPU6050_ADDR, YG_OFFS_USRH, offset);
}


/** Update gyroscope z-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
int16_t MPU6050_getZGyroOffset(I2C_HandleTypeDef *I2Cx) {
  uint8_t data[2];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, ZG_OFFS_USRH, 2, data);
  return (((int16_t)data[0]) << 8) | data[1];
}


/** Update gyroscope z-axis offset
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param offset 16-bit signed integer container for new offset value
 */
void MPU6050_setZGyroOffset(I2C_HandleTypeDef *I2Cx, int16_t offset) {
  I2Cdev_writeWord(I2Cx, MPU6050_ADDR, ZG_OFFS_USRH, offset);
}


//TODO: study alternative calibration process based on PID controllers:
//https://forum.arduino.cc/t/new-mpu6050_calibration-faster-and-exact-using-pid-pi-actually/594989/10

/* This function allows to calibrate the acceleromenter and the gyrsocope by running multiple cycles until the average error
 * falls within a user defined range. Based on: https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param accelRange accelerometer FS range, see ACCEL_FS in mpu6050.h
 * @param gyroRange gyroscope FS range, see GYRO_FS in mpu6050.h
 * @param readings 16-bit unsigned integer container for the readings used to compute the average
 * @param acel_deadzone 8-bit unsigned integer: allowed error on accelerometer axis(|accelReading| <= acel_deadzone)
 * @param gyro_deadzone 8-bit unsigned integer: allowed error on gyroscope axis (|gyroReading| <= gyro_deadzone)
 */
void MPU6050_selfCalibration(I2C_HandleTypeDef *I2Cx, ACCEL_FS accelRange, GYRO_FS gyroRange, uint16_t readings, uint8_t acel_deadzone, uint8_t gyro_deadzone) {

  int16_t meanValues[6];

  // Reinitialize the device with standard configurations:
  MPU6050_reset(I2Cx);
  HAL_Delay(50);

  // Remove factory offset values
  int16_t clear_offsets[6] = {0,0,0,0,0,0};

  MPU6050_Initialize(I2Cx, accelRange, gyroRange, clear_offsets);

  // Start calibration process:
  MPU6050_meanSensors(I2Cx, readings, meanValues);

  MPU6050_calibration(I2Cx, meanValues, readings, acel_deadzone, gyro_deadzone);

  int16_t AXoffs = MPU6050_getXAccelOffset(I2Cx);
  int16_t AYoffs = MPU6050_getYAccelOffset(I2Cx);
  int16_t AZoffs = MPU6050_getZAccelOffset(I2Cx);
  int16_t GXoffs = MPU6050_getXGyroOffset(I2Cx);
  int16_t GYoffs = MPU6050_getYGyroOffset(I2Cx);
  int16_t GZoffs = MPU6050_getZGyroOffset(I2Cx);

  printf("AX-offset: %d \r\n", AXoffs);
  printf("AY-offset: %d \r\n", AYoffs);
  printf("AZ-offset: %d \r\n", AZoffs);
  printf("GX-offset: %d \r\n", GXoffs);
  printf("GY-offset: %d \r\n", GYoffs);
  printf("GZ-offset: %d \r\n", GZoffs);

  // let enough time to copy updated offsets:
  HAL_Delay(20000);

}

/** Auxiliary function used during calibration process: computes x,y,z averages on both accelerometer and gyroscope.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param buffersize 16-bit unsigned integer container for the readings used to compute the average
 * @param meanVals pointer to 16-bit signed integer container used to store averages along all axes:
 *          meanVals[0] = Accele-X, meanVals[1] = Accele-Y, meanVals[2] = Accele-Z
 *          meanVals[3] = Gyro-X, meanVals[2] = Gyro-Y, meanVals[3] = Gyro-Z
 *
 * See MPU6050_selfCalibration.
 */
void MPU6050_meanSensors(I2C_HandleTypeDef *I2Cx, uint16_t buffersize, int16_t* meanVals) {
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  int16_t ax,ay,az,gx,gy,gz;

  while (i<(buffersize+101)){

      // read raw accel/gyro measurements from device
      MPU6050_getMotion6(I2Cx, &ax, &ay, &az, &gx, &gy, &gz);

      if (i > 100) { //First 100 measures are discarded
          buff_ax+=ax;
          buff_ay+=ay;
          buff_az+=az;
          buff_gx+=gx;
          buff_gy+=gy;
          buff_gz+=gz;
      }

      i++;
      HAL_Delay(2); //Needed so we don't get repeated measures
  }

  meanVals[0] = buff_ax/buffersize;   // Ax
  meanVals[1] = buff_ay/buffersize;   // Ay
  meanVals[2] = buff_az/buffersize;   // Az
  meanVals[3] = buff_gx/buffersize;   // Gx
  meanVals[4] = buff_gy/buffersize;   // Gy
  meanVals[5] = buff_gz/buffersize;   // Gz
}


/* Auxiliary function used during calibration process.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param meanVals pointer to 16-bit signed integer container used to store averages along all axes:
 *          meanVals[0] = Accele-X, meanVals[1] = Accele-Y, meanVals[2] = Accele-Z
 *          meanVals[3] = Gyro-X, meanVals[2] = Gyro-Y, meanVals[3] = Gyro-Z
 * @param readings 16-bit unsigned integer container for the readings used to compute the average
 * @param acel_deadzone 8-bit unsigned integer: allowed error on accelerometer axis(|accelReading| <= acel_deadzone)
 * @param gyro_deadzone 8-bit unsigned integer: allowed error on gyroscope axis (|gyroReading| <= gyro_deadzone)
 *
 * See MPU6050_selfCalibration.
 */
void MPU6050_calibration(I2C_HandleTypeDef *I2Cx, int16_t* meanVals, uint16_t readings, uint8_t acel_deadzone, uint8_t gyro_deadzone) {
  int16_t mean_ax = meanVals[0];
  int16_t mean_ay = meanVals[1];
  int16_t mean_az = meanVals[2];

  int16_t mean_gx = meanVals[3];
  int16_t mean_gy = meanVals[4];
  int16_t mean_gz = meanVals[5];

  int16_t ax_offset=-mean_ax/8;
  int16_t ay_offset=-mean_ay/8;
  int16_t az_offset=(int16_t)(1/accelerationResolution-mean_az)/8;

  int16_t gx_offset=-mean_gx/4;
  int16_t gy_offset=-mean_gy/4;
  int16_t gz_offset=-mean_gz/4;

  while (1){
    int ready=0;

    // Set updated offset values:
    MPU6050_setXAccelOffset(I2Cx, ax_offset);
    MPU6050_setYAccelOffset(I2Cx, ay_offset);
    MPU6050_setZAccelOffset(I2Cx, az_offset);

    MPU6050_setXGyroOffset(I2Cx, gx_offset);
    MPU6050_setYGyroOffset(I2Cx, gy_offset);
    MPU6050_setZGyroOffset(I2Cx, gz_offset);

    // Recompute average values with updated offsets:
    MPU6050_meanSensors(I2Cx, readings, meanVals);
    mean_ax = meanVals[0];
    mean_ay = meanVals[1];
    mean_az = meanVals[2];

    mean_gx = meanVals[3];
    mean_gy = meanVals[4];
    mean_gz = meanVals[5];

    // Calibrate offset values:
    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(1/accelerationResolution-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=gyro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(gyro_deadzone+1);

    if (abs(mean_gy)<=gyro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(gyro_deadzone+1);

    if (abs(mean_gz)<=gyro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(gyro_deadzone+1);

    if (ready==6) break;
  }
}




// FIFO_EN register

/** Get temperature FIFO enabled value.
 * When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and
 * 66) to be written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current temperature FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool MPU6050_getTempFIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, &data);
  return data;
}


/** Set temperature FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New temperature FIFO enabled value
 * @see getTempFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setTempFIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_TEMP_FIFO_EN_BIT, enabled);
}


/** Get gyroscope X-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and
 * 68) to be written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current gyroscope X-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool MPU6050_getXGyroFIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, &data);
  return data;
}


/** Set gyroscope X-axis FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New gyroscope X-axis FIFO enabled value
 * @see getXGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setXGyroFIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_XG_FIFO_EN_BIT, enabled);
}


/** Get gyroscope Y-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and
 * 70) to be written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current gyroscope Y-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool MPU6050_getYGyroFIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, &data);
  return data;
}


/** Set gyroscope Y-axis FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New gyroscope Y-axis FIFO enabled value
 * @see getYGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setYGyroFIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
    I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_YG_FIFO_EN_BIT, enabled);
}


/** Get gyroscope Z-axis FIFO enabled value.
 * When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and
 * 72) to be written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current gyroscope Z-axis FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool MPU6050_getZGyroFIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, &data);
  return data;
}


/** Set gyroscope Z-axis FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New gyroscope Z-axis FIFO enabled value
 * @see getZGyroFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setZGyroFIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
    I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_ZG_FIFO_EN_BIT, enabled);
}


/** Get accelerometer FIFO enabled value.
 * When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H,
 * ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be
 * written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current accelerometer FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool MPU6050_getAccelFIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, &data);
  return data;
}


/** Set accelerometer FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New accelerometer FIFO enabled value
 * @see getAccelFIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setAccelFIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_ACCEL_FIFO_EN_BIT, enabled);
}


/** Get Slave 2 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 2 to be written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current Slave 2 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool MPU6050_getSlave2FIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, &data);
  return data;
}


/** Set Slave 2 FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New Slave 2 FIFO enabled value
 * @see getSlave2FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setSlave2FIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_SLV2_FIFO_EN_BIT, enabled);
}


/** Get Slave 1 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 1 to be written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current Slave 1 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool MPU6050_getSlave1FIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, &data);
  return data;
}


/** Set Slave 1 FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New Slave 1 FIFO enabled value
 * @see getSlave1FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setSlave1FIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_SLV1_FIFO_EN_BIT, enabled);
}


/** Get Slave 0 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 0 to be written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current Slave 0 FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
bool MPU6050_getSlave0FIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, &data);
  return data;
}


/** Set Slave 0 FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New Slave 0 FIFO enabled value
 * @see getSlave0FIFOEnabled()
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setSlave0FIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_EN, MPU6050_SLV0_FIFO_EN_BIT, enabled);
}


/** Get Slave 3 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 3 to be written into the FIFO buffer.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current Slave 3 FIFO enabled value
 * @see MPU6050_RA_MST_CTRL
 */
bool MPU6050_getSlave3FIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, &data);
  return data;
}


/** Set Slave 3 FIFO enabled value.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New Slave 3 FIFO enabled value
 * @see getSlave3FIFOEnabled()
 * @see MPU6050_RA_MST_CTRL
 */
void MPU6050_setSlave3FIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, MPU6050_RA_I2C_MST_CTRL, MPU6050_SLV_3_FIFO_EN_BIT, enabled);
}





// FIFO OVerlfow Registers:


/** Get FIFO Buffer Overflow interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 **/
bool MPU6050_getIntFIFOBufferOverflowEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, &data);
  return data;
}


/** Set FIFO Buffer Overflow interrupt enabled status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New interrupt enabled status
 * @see getIntFIFOBufferOverflowEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 **/
void MPU6050_setIntFIFOBufferOverflowEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}


/** Get FIFO Buffer Overflow interrupt status.
 * This bit automatically sets to 1 when a FIFO overflow interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 */
bool MPU6050_getIntFIFOBufferOverflowStatus(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, INT_STATUS_REG, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, &data);
  return data;
}





// USER_CTRL register


/** Get FIFO enabled status.
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current FIFO enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
bool MPU6050_getFIFOEnabled(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readBit(I2Cx, MPU6050_ADDR, USER_CTRL_REG, MPU6050_USERCTRL_FIFO_EN_BIT, &data);
  return data;
}


/** Set FIFO enabled status.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void MPU6050_setFIFOEnabled(I2C_HandleTypeDef *I2Cx, bool enabled) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, USER_CTRL_REG, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}


/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPU6050_resetFIFO(I2C_HandleTypeDef *I2Cx) {
  I2Cdev_writeBit(I2Cx, MPU6050_ADDR, USER_CTRL_REG, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}


// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Current FIFO buffer size
 */
uint16_t MPU6050_getFIFOCount(I2C_HandleTypeDef *I2Cx) {
  uint8_t data[2];
  I2Cdev_readBytes(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_COUNTH, 2, data);
  return (((uint16_t)data[0]) << 8) | data[1];
}


// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @param I2C_HandleTypeDef pointer to I2C HAL handler
 * @return Byte from FIFO buffer
 */
uint8_t MPU6050_getFIFOByte(I2C_HandleTypeDef *I2Cx) {
  uint8_t data;
  I2Cdev_readByte(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_R_W, &data);
  return data;
}


void MPU6050_getFIFOBytes(I2C_HandleTypeDef *I2Cx, uint8_t *data, uint16_t length) {
  if(length > 0){
    I2Cdev_readBytes(I2Cx, MPU6050_ADDR, MPU6050_RA_FIFO_R_W, length, data);
  } else {
    *data = 0;
  }
}










