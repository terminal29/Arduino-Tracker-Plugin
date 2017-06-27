#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"
#include "mpu_device.h"
#include "print_helper.h"

void setup(){
  Serial.begin(115200);
  while(!Serial){}
  int res = mpu_device::init_mpu();
  if(res != 0){
    Serial << "E:" << mpu_device::get_error_desc(res) << endl;
  }
}

void loop(){
  if(mpu_device::mpu_interrupt){
    mpu_device::update();
  }
  Quaternion q = mpu_device::last_quat;
  Serial << "Q:" << q.w << ',' << q.x << ',' << q.y << ',' << q.z << endl;

  if(Serial.available()){
    String comm = Serial.readString();
    if(comm.charAt(0) == 'C'){
      Serial.println("C:Begin");
      mpu_device::calibrate(500);
      Serial.println("C:End");
    }
  }
}


