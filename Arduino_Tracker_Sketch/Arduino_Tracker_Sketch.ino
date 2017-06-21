#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"
#include "mpu_device.h"

// <3 http://forum.arduino.cc/index.php?topic=46290.0
template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
char endl = '\n';
//

VectorFloat gyro_offset;
VectorFloat accel_offset;

mpu_device* device;

#define LED_PIN 13
bool blinkState = false;

void dmp_data_callback(){
  device->dmp_data_callback();
}

void setup() {
    Serial.begin(38400);
//    gyro_offset = mpu_device::read_eeprom_gyro_offset();
//    accel_offset = mpu_device::read_eeprom_accel_offset();
    device = new mpu_device(gyro_offset, accel_offset);
    attachInterrupt(digitalPinToInterrupt(device->interrupt_pin), dmp_data_callback, RISING);
}

void loop() {
  device->update();
  Quaternion q = device->get_quaternion();
  Serial << "Q:" << q.w << ',' << q.x << ',' << q.y << ',' << q.z << endl;
  if(Serial.available()){
    String comm = Serial.readString();
    if(comm.charAt(0) == 'C'){
      Serial.println("C:Begin");
      device->calibrate(10);
      Serial.println("C:Done");
    }
  }
}


