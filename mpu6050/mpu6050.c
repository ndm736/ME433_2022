#include "i2c_master_noint.h"
#include "mpu6050.h"

// Source file for mpu6050.h.
// Enables use of InvenSense MPU-6050 IMU
// Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
// Register map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

// Initialize MPU6050 IMU:
void init_mpu6050(void) {
    i2c_master_setup(); // initialize I2C1 (used for comm w/ IMU)
    //write_UART1("Initialized I2C1.\r\n");
    
    // 1. Configure IMU to get clock signal from internal 8 MHz oscillator:
    //  write to MPU-6050 PWR_MGMT_1 register,
    //  clear bit 6 (defaults to 1) to wake IMU from sleep (logic is awake but
    //  MEMS sensor stuff is asleep)
    write_byte_I2C1(IMU_ADDR,PWR_MGMT_1,0x00);
    //delay(); // delay a bit just to be safe. TODO: remove?
    
    // 2. TODO: initialize CONFIG register
    //  bits [2:0]: configure digital low pass filter
    //  bits [5:3]: configure external sync (leave disabled)
    //  bits [7:6]: reserved
    
    // 3. Initialize ACCEL_CONFIG (accelerometer) register
    //  bits [2:0]: reserved
    //  bits [4:3]: configure full scale range; TODO: update
    //      0b00: +/- 2g
    //      0b01: +/- 4g
    //      0b10: +/- 8g
    //      0b11: +/- 16g -- use this for now
    //  bits [7:5]: configure accelerometer self-test (leave disabled)
    write_byte_I2C1(IMU_ADDR,ACCEL_CONFIG,0x00); // 0x0: +/- 2g, no self-tests

    // 4. Initialize MPU-6050 GYRO_CONFIG (gyroscope) register
    //  bits [2:0]: reserved
    //  bits [4:3]: configure full scale range; TODO: update
    //      0b00: +/-  250 dps
    //      0b01: +/-  500 dps
    //      0b10: +/- 1000 dps
    //      0b11: +/- 2000 dps -- use this for now
    //  bits [7:5]: configure gyroscope self-test (leave disabled)
    write_byte_I2C1(IMU_ADDR,GYRO_CONFIG,0x0C); // 0x0C: +/- 2000 dps, no self-tests

    // TODO: configure for sequential reading
}

// Verify the identity of the MPU-6050.
// If MPU-6050 is functioning and communications are working,
// this function will return 0x68
uint8_t whoami(void) {
    return read_byte_I2C1(IMU_ADDR, WHO_AM_I);
}

// Burst read from MPU-6050, from ACCEL_XOUT_H reg through GYRO_ZOUT_L reg.
// Raw data from IMU registers is stored in "data" - an array passed to this
// function by reference.
// The number of registers this function reads from is defined by IMU_ARRAY_LEN,
// and this function does not check to see if the length of "data" equals
// IMU_ARRAY_LEN or not.
// This function is basically just a wrapper for burst_read_I2C1(), defined in
// i2c_noint.{c,h}.
void burst_read_mpu6050(uint8_t * data) {
    burst_read_I2C1(IMU_ADDR,ACCEL_XOUT_H,data,IMU_ARRAY_LEN);
}

/*******************************************************************************
 Functions to combine 8-bit register pairs (each uint8_t) into int16_t's:
 ******************************************************************************/
int16_t get_xXL(uint8_t * data) { // convert x-acceleration LSB and MSB to int16_t
    return data[0]<<8 | data[1];
}

int16_t get_yXL(uint8_t * data) { // convert y-acceleration LSB and MSB to int16_t
    return data[2]<<8 | data[3];
}

int16_t get_zXL(uint8_t * data) { // convert z-acceleration LSB and MSB to int16_t
    return data[4]<<8 | data[5];
}

int16_t get_temp(uint8_t * data) { // convert temperature LSB and MSB to int16_t
    return data[6]<<8 | data[7];
}

int16_t get_xG(uint8_t * data) { // convert x-gyro LSB and MSB to int16_t
    return data[8]<<8 | data[9];
}

int16_t get_yG(uint8_t * data) { // convert y-gyro LSB and MSB to int16_t
    return data[10]<<8 | data[11];
}

int16_t get_zG(uint8_t * data) { // convert z-gyro LSB and MSB to int16_t
    return data[12]<<8 | data[13];
}

// TODO: test these functions!!!
/*
 * Functions to convert int16_t representation of 16-bit IMU data (acceleration
 * in x, y, and z; temperature; gyro rates) to float representation
 */
float conv_xXL(uint8_t * data) { // convert x-acceleration to float (g's)
    return (get_xXL(data))*0.000061;
}

float conv_yXL(uint8_t * data) { // convert y-acceleration to float (g's)
    return (get_yXL(data))*0.000061;
}

float conv_zXL(uint8_t * data) { // convert z-acceleration to float (g's)
    return (get_zXL(data))*0.000061;
}

float conv_xG(uint8_t * data) { // convert x-gyro rate to dps
    return (get_xG(data))*0.007630;
}

float conv_yG(uint8_t * data) { // convert y-gyro rate to dps
    return (get_yG(data))*0.007630;
}

float conv_zG(uint8_t * data) { // convert z-gyro rate to dps
    return (get_zG(data))*0.007630;
}

float conv_temp(uint8_t * data) { // convert int16_t temperature signed short to float (Celsius)
    return (get_temp(data)/340.00) + 36.53;
}

// i2c functions
// read one byte from a register (reg_addr) of a device (dev_addr):
uint8_t read_byte_I2C1(uint8_t dev_addr,
                       uint8_t reg_addr) {
    uint8_t answer;
    i2c_master_start();
    i2c_master_send(dev_addr << 1); // hardware address and write bit
    i2c_master_send(reg_addr);  // WHO_AM_I register: 0x0F
    i2c_master_restart(); // this line is REALLY important!
    i2c_master_send((dev_addr << 1) | 1); // hardware address and read bit
    answer = i2c_master_recv(); // receive a byte from the slave. Should be 0x69 = 105d
    i2c_master_ack(1); // send NACK to slave
    i2c_master_stop();
    return answer;
}

// burst read from device (dev_addr), beginning at specified register by
// start_reg_addr:
void burst_read_I2C1(uint8_t dev_addr,
                     uint8_t start_reg_addr,
                     uint8_t * data,
                     uint8_t data_len) {
    char i;
    i2c_master_start();
    i2c_master_send((dev_addr << 1)); // hardware address and write bit
    i2c_master_send(start_reg_addr);  // starting register. "register" has some other meaning in C, so I called it "registerrr" instead.
    i2c_master_restart(); // this line is REALLY important!
    i2c_master_send((dev_addr << 1) | 1); // hardware address and read bit
    for (i = 0; i < data_len; i++) {
        data[i] = i2c_master_recv(); // receive a byte from the slave. Should be 0x69 = 105d
        if (i==(data_len-1)) {
            i2c_master_ack(1); // send NACK to slave to stop reading
        }
        else {
            i2c_master_ack(0); // send ACK to slave to continue reading
        }
    }
    i2c_master_stop();
}

// write one byte (data) from a register (reg_addr) of a device (dev_addr):
void write_byte_I2C1(uint8_t dev_addr,
                     uint8_t reg_addr,
                     uint8_t data) {
    i2c_master_start();             // START bit
    i2c_master_send(dev_addr << 1); // hardware address; RW (lsb) = 0, indicates write
    i2c_master_send(reg_addr);      // specify register to write to
    i2c_master_send(data);          // specify data to write to reg_addr
    i2c_master_stop();              // STOP bit
}

