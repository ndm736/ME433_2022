#ifndef MPU6050__H__
#define MPU6050__H__
// Header file for mpu6050.c.
// Enables use of InvenSense MPU-6050 IMU
// Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
// Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#define IMU_ADDR 0x68  // I2C hardware address of MPU-6050
#define IMU_ARRAY_LEN 14 // 14 contiguous registers from ACCEL_XOUT_H to GYRO_ZOUT_L

// register addresses
// TODO: config addresses
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
// TODO: any other config registers I need to use?

// sensor data registers:
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define WHO_AM_I     0x75

// NOTE: the reset value is 0x00 for all registers other than:
//  register 0x6B (107): 0x40 (device resets into sleep mode)
//  register 0x75 (117): 0x68 (WHOAMI contents, should be 0x68)

void init_mpu6050(void);

// Verify the identity of the MPU-6050.
// If MPU-6050 is function and communications are working,
// this function will return 0x68
uint8_t whoami(void);

// Burst read from MPU-6050, from ACCEL_XOUT_H reg through GYRO_ZOUT_L reg.
// Raw data from IMU registers is stored in "data" - an array passed to this
// function by reference.
// The number of registers this function reads from is defined by IMU_ARRAY_LEN,
// and this function does not check to see if the length of "data" equals
// IMU_ARRAY_LEN or not.
// This function is basically just a wrapper for burst_read_I2C1(), defined in
// i2c_noint.{c,h}.
void burst_read_mpu6050(uint8_t * data);

/*******************************************************************************
 Functions to combine 8-bit register pairs (each uint8_t) into int16_t's:
 ******************************************************************************/
int16_t get_xXL(uint8_t * data); // convert x-acceleration LSB and MSB to int16_t
int16_t get_yXL(uint8_t * data); // convert y-acceleration LSB and MSB to int16_t
int16_t get_zXL(uint8_t * data); // convert z-acceleration LSB and MSB to int16_t
int16_t get_temp(uint8_t * data); // convert temperature LSB and MSB to int16_t
int16_t get_xG(uint8_t * data);  // convert x-gyro LSB and MSB to int16_t
int16_t get_yG(uint8_t * data); // convert y-gyro LSB and MSB to int16_t
int16_t get_zG(uint8_t * data); // convert z-gyro LSB and MSB to int16_t

/*******************************************************************************
 * Functions to convert int16_t representation of 16-bit IMU data (acceleration
 * in x, y, and z; temperature; gyro rates) to float representation
 ******************************************************************************/
float conv_xXL(uint8_t * data); // convert x-acceleration to float (g's)

float conv_yXL(uint8_t * data); // convert y-acceleration to float (g's)

float conv_zXL(uint8_t * data); // convert z-acceleration to float (g's)

float conv_xG(uint8_t * data); // convert x-gyro rate to dps

float conv_yG(uint8_t * data); // convert y-gyro rate to dps

float conv_zG(uint8_t * data); // convert z-gyro rate to dps

float conv_temp(uint8_t * data); // convert int16_t temperature signed short to float (Celsius)

// i2c functions
// read one byte from a register (reg_addr) of a device (dev_addr):
uint8_t read_byte_I2C1(uint8_t dev_addr,
                       uint8_t reg_addr);

// burst read from device (dev_addr), beginning at specified register by
// start_reg_addr:
void burst_read_I2C1(uint8_t dev_addr,
                     uint8_t start_reg_addr,
                     uint8_t * data,
                     uint8_t data_len);

// write one byte (data) from a register (reg_addr) of a device (dev_addr):
void write_byte_I2C1(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t data);


#endif