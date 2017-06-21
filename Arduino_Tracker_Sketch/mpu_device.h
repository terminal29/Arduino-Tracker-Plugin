#ifndef _MPU_DEVICE_H_
#define _MPU_DEVICE_H_

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class mpu_device{
  public:
    mpu_device(VectorFloat gyro_offset, VectorFloat accel_offset);
    void calibrate(int accuracy);
    void update();
    void close();
    Quaternion get_quaternion();
    int is_available();
    String get_error_desc(int code);
    void dmp_data_callback();

    static VectorFloat get_VectorFloat_eeprom(int offset);

    volatile bool mpu_interrupt = false; 
    const int interrupt_pin = 2;
    const int gyro_epprom_offset = 0;
    const int accel_eeprom_offset = sizeof(VectorFloat);
  
  private:
    mpu_device();

    MPU6050* mpu;
    Quaternion last_quaternion;
    VectorFloat _gyro_offset, _accel_offset;
    
    bool dmp_ready = false;  // set true if DMP init was successful
    uint8_t mpu_int_status;   // holds actual interrupt status byte from MPU
    uint8_t dev_status = -1;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifo_count;     // count of all bytes currently in FIFO
    uint8_t fifo_buffer[64]; // FIFO storage buffer

    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aa_real;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aa_world;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    int error_codes[2] = { 1, 2 };
    String error_code_desc[2] = { "Unable to communicate with MPU6050. Please check connections!", "DMP init failed. Either this sketch or your board is broken." };
    int last_error = 0;

    void mean_sensors(int buffersize, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int* mean_ax, int*mean_ay, int*mean_az, int*mean_gx, int*mean_gy, int*mean_gz);

    void write_VectorFloat_eeprom(VectorFloat data, int offset);
    
};

#endif
