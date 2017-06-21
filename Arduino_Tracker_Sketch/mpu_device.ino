#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include <EEPROM.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

mpu_device::mpu_device(VectorFloat gyro_offset, VectorFloat accel_offset):_gyro_offset(gyro_offset),_accel_offset(accel_offset){
  mpu_device::mpu = new MPU6050();
  Wire.begin();
  Wire.setClock(400000); 
  mpu_device::mpu->initialize();
  pinMode(mpu_device::interrupt_pin, INPUT);
  mpu_device::mpu->testConnection() ? : mpu_device::last_error = 1;
  dev_status = mpu_device::mpu->dmpInitialize();
  if(dev_status != 0){
    mpu_device::last_error = 2;
    return;
  }
  
  mpu_device::mpu->setXGyroOffset(_gyro_offset.x);
  mpu_device::mpu->setYGyroOffset(_gyro_offset.y);
  mpu_device::mpu->setZGyroOffset(_gyro_offset.z);
  mpu_device::mpu->setXAccelOffset(_accel_offset.x); 
  mpu_device::mpu->setYAccelOffset(_accel_offset.y); 
  mpu_device::mpu->setZAccelOffset(_accel_offset.z); 

  mpu_device::mpu->setDMPEnabled(true);

  //attachInterrupt(digitalPinToInterrupt(mpu_device::interrupt_pin), mpu_device::dmp_data_callback, RISING);
  //mpu_int_status = mpu_device::mpu->getIntStatus();
  dmp_ready = true;
  packet_size = mpu->dmpGetFIFOPacketSize();
}


void mpu_device::calibrate(int accuracy){
  if(!dmp_ready) return;
  int buffersize=accuracy;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
  int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
  int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
  int16_t ax, ay, az,gx, gy, gz;
  
  int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;
  int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
  
  mpu->setXAccelOffset(0);
  mpu->setYAccelOffset(0);
  mpu->setZAccelOffset(0);
  mpu->setXGyroOffset(0);
  mpu->setYGyroOffset(0);
  mpu->setZGyroOffset(0);
  
  mean_sensors(accuracy,&ax, &ay, &az, &gx, &gy, &gz, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);

  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;
 
  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    mpu->setXAccelOffset(ax_offset);
    mpu->setYAccelOffset(ay_offset);
    mpu->setZAccelOffset(az_offset);
 
    mpu->setXGyroOffset(gx_offset);
    mpu->setYGyroOffset(gy_offset);
    mpu->setZGyroOffset(gz_offset);
 
    mean_sensors(accuracy,&ax, &ay, &az, &gx, &gy, &gz, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;
 
    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;
 
    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;
 
    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);
 
    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);
 
    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);
 
    if (ready==6) break;
  }
  mean_sensors(accuracy,&ax, &ay, &az, &gx, &gy, &gz, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
  
  mpu->setXAccelOffset(ax_offset);
  mpu->setYAccelOffset(ay_offset);
  mpu->setZAccelOffset(az_offset);
  mpu->setXGyroOffset(gx_offset);
  mpu->setYGyroOffset(gy_offset);
  mpu->setZGyroOffset(gz_offset);

 // write_VectorFloat_eeprom(
}

inline void mpu_device::mean_sensors(int buffersize, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int* mean_ax, int*mean_ay, int*mean_az, int*mean_gx, int*mean_gy, int*mean_gz){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
 
  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    mpu->getMotion6(ax, ay, az, gx, gy, gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      *mean_ax=buff_ax/buffersize;
      *mean_ay=buff_ay/buffersize;
      *mean_az=buff_az/buffersize;
      *mean_gx=buff_gx/buffersize;
      *mean_gy=buff_gy/buffersize;
      *mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}


void mpu_device::update(){
  if(!dmp_ready) return;
  if (mpu_interrupt) {
    mpu_interrupt = false;
    mpu_int_status = mpu->getIntStatus();
    fifo_count = mpu->getFIFOCount();
    if ((mpu_int_status & 0x10) || fifo_count == 1024) {
      mpu->resetFIFO();
    }else if(mpu_int_status & 0x02) {
      while (fifo_count < packet_size) fifo_count = mpu->getFIFOCount();
      mpu->getFIFOBytes(fifo_buffer, packet_size);
      fifo_count -= packet_size;
      mpu->dmpGetQuaternion(&last_quaternion, fifo_buffer);
    }
  }
}

void mpu_device::close(){
  //TODO
  
}

Quaternion mpu_device::get_quaternion(){
  return mpu_device::last_quaternion;
}

int mpu_device::is_available(){
  return mpu_device::last_error;
}

String mpu_device::get_error_desc(int code){
  if(code - 1 < sizeof(error_code_desc)/sizeof(String) && code >= 0){
    return error_code_desc[code - 1];
  }
  return "Unknown Error";
}

void mpu_device::dmp_data_callback(){
  mpu_device::mpu_interrupt = true;
}

