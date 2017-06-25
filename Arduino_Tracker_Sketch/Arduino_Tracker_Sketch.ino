#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"
#include "mpu_device.h"
#include "print_helper.h"

VectorFloat gyro_offset;
VectorFloat accel_offset;

mpu_device* device;

void dmp_data_callback(){
  device->dmp_data_callback();
}

void setup() {
    Serial.begin(38400);
    gyro_offset = mpu_device::get_VectorFloat_eeprom(mpu_device::gyro_offset_addr);
    accel_offset = mpu_device::get_VectorFloat_eeprom(mpu_device::accel_offset_addr);
    Serial << endl;
    Serial << "D:" << "Read offsets from EEPROM: <" << gyro_offset.x << ',' << gyro_offset.y << ',' << gyro_offset.z << ">,<" << accel_offset.x << ',' << accel_offset.y << ',' << accel_offset.z << '>' << endl;
    device = new mpu_device(gyro_offset, accel_offset);
    attachInterrupt(digitalPinToInterrupt(device->interrupt_pin), dmp_data_callback, RISING);
}

void loop() {
  device->update();
  Quaternion q = device->get_quaternion();
  Serial << "Q:" << q.w << ',' << q.x << ',' << q.y << ',' << q.z << endl;
  if(int j = device->get_last_error() != 0){
    Serial << "E:" << device->get_error_desc(j) << endl;
  }
  if(Serial.available()){
    String comm = Serial.readString();
    if(comm.charAt(0) == 'C'){
      Serial.println("C:Begin");
      device->calibrate(500);
      Serial.println("C:End");
    }
  }
}


