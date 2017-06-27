#ifndef _MPU_DEVICE_H_
#define _MPU_DEVICE_H_

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include <EEPROM.h>
#include "print_helper.h"
#include "Wire.h"

typedef struct vec3f { float x = 0,y = 0, z = 0; } vec3f;

namespace mpu_device{
  namespace internal{
    MPU6050 mpu;
    
    const int interrupt_pin = 2;
    
    bool dmp_ready = false;
    uint8_t mpu_int_status;
    uint8_t dev_status;
    uint16_t packet_size;
    uint16_t fifo_count;
    uint8_t fifo_buffer[64];

    const int gyro_offset_addr = 0;
    const int accel_offset_addr = sizeof(vec3f);

    String error_code_desc[2] = { "Unable to communicate with MPU6050. Please check connections!", "DMP init failed. Either this sketch or your board is broken." };
    
  }
  
  Quaternion last_quat;
  volatile bool mpu_interrupt = false;    
   
  void dmp_data_ready() {
      mpu_interrupt = true;
  }

  VectorFloat get_VectorFloat_eeprom(int offset){
    vec3f in;
    EEPROM.get(offset, in);
    if(isnan(in.x) || isnan(in.y) || isnan(in.z)){
      return VectorFloat(0,0,0);
    }
    return VectorFloat(in.x, in.y, in.z);
  }
  
  void set_VectorFloat_eeprom(int offset, VectorFloat in_vec){
    vec3f in;
    in.x = in_vec.x;
    in.y = in_vec.y;
    in.z = in_vec.z;
    EEPROM.put(offset, in);
  }

  int init_mpu(){
    int last_error = 0;
    Wire.begin();
    Wire.setClock(400000); 
    internal::mpu.initialize();
    pinMode(internal::interrupt_pin, INPUT);
    
    internal::mpu.testConnection() ? /*success*/ : last_error = 1;
    internal::dev_status = internal::mpu.dmpInitialize();
    internal::dev_status == 0 ? /*success*/ : last_error = 2;
    
    VectorFloat g_offsets = get_VectorFloat_eeprom(internal::gyro_offset_addr);
    VectorFloat a_offsets = get_VectorFloat_eeprom(internal::accel_offset_addr);

    internal::mpu.setXGyroOffset(g_offsets.x);
    internal::mpu.setYGyroOffset(g_offsets.y);
    internal::mpu.setZGyroOffset(g_offsets.z);
    internal::mpu.setXAccelOffset(a_offsets.x);
    internal::mpu.setYAccelOffset(a_offsets.y);
    internal::mpu.setZAccelOffset(a_offsets.z);

    Serial << "D:Read offsets: <" << g_offsets.x << ',' << g_offsets.y << ','<< g_offsets.z << "> <" << a_offsets.x << ','<< a_offsets.y << ',' << a_offsets.z << '>' << endl;

    if(internal::dev_status == 0){
        internal::mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(internal::interrupt_pin), dmp_data_ready, RISING);
        internal::mpu_int_status = internal::mpu.getIntStatus();
        internal::dmp_ready = true;
        internal::packet_size = internal::mpu.dmpGetFIFOPacketSize();
    }
    return last_error;
  }

  void update(){
    if (!internal::dmp_ready) return;
    while (!mpu_interrupt && internal::fifo_count < internal::packet_size) {}
    mpu_interrupt = false;
    internal::mpu_int_status = internal::mpu.getIntStatus();
    internal::fifo_count = internal::mpu.getFIFOCount();
    if ((internal::mpu_int_status & 0x10) || internal::fifo_count == 1024) {
        internal::mpu.resetFIFO();
    } else if (internal::mpu_int_status & 0x02) {
        while (internal::fifo_count < internal::packet_size){
          internal::fifo_count = internal::mpu.getFIFOCount();
        }
        internal::mpu.getFIFOBytes(internal::fifo_buffer, internal::packet_size);
        internal::fifo_count -= internal::packet_size;
        internal::mpu.dmpGetQuaternion(&last_quat, internal::fifo_buffer);
    }
  }
  
  void mean_sensors(int buffersize, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int* mean_ax, int* mean_ay, int* mean_az, int* mean_gx, int* mean_gy, int* mean_gz){
    long long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
   
    while (i<(buffersize+101)){
      // read raw accel/gyro measurements from device
      internal::mpu.getMotion6(ax, ay, az, gx, gy, gz);
  
      if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
        buff_ax = buff_ax + (*ax);
        buff_ay = buff_ay + (*ay);
        buff_az = buff_az + (*az);
        buff_gx = buff_gx + (*gx);
        buff_gy = buff_gy + (*gy);
        buff_gz = buff_gz + (*gz);
      }
      if (i==(buffersize+100)){
        (*mean_ax) = buff_ax / buffersize;
        (*mean_ay) = buff_ay / buffersize;
        (*mean_az) = buff_az / buffersize;
        (*mean_gx) = buff_gx / buffersize;
        (*mean_gy) = buff_gy / buffersize;
        (*mean_gz) = buff_gz / buffersize;
      }
      i++;
      delay(2); //Needed so we don't get repeated measures
    }
  }

  void calibrate(int accuracy){
    if(!internal::dmp_ready) return;
  
    int buffersize = accuracy;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
    int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
    int16_t ax, ay, az,gx, gy, gz;
    
    int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;
    int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
  
    internal::mpu.setXAccelOffset(0);
    internal::mpu.setYAccelOffset(0);
    internal::mpu.setZAccelOffset(0);
    internal::mpu.setXGyroOffset(0);
    internal::mpu.setYGyroOffset(0);
    internal::mpu.setZGyroOffset(0);
    mean_sensors(buffersize, &ax, &ay, &az, &gx, &gy, &gz, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
    delay(5);
  
    ax_offset=-mean_ax/8;
    ay_offset=-mean_ay/8;
    az_offset=(16384-mean_az)/8;
   
    gx_offset=-mean_gx/4;
    gy_offset=-mean_gy/4;
    gz_offset=-mean_gz/4;
    while (1){
      
      int ready=0;
      internal::mpu.setXAccelOffset(ax_offset);
      internal::mpu.setYAccelOffset(ay_offset);
      internal::mpu.setZAccelOffset(az_offset);
   
      internal::mpu.setXGyroOffset(gx_offset);
      internal::mpu.setYGyroOffset(gy_offset);
      internal::mpu.setZGyroOffset(gz_offset);
   
      mean_sensors(buffersize, &ax, &ay, &az, &gx, &gy, &gz, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
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
      
      Serial << "D:Stage " << ready << "/6" << endl;
    }
    delay(5);
    mean_sensors(buffersize, &ax, &ay, &az, &gx, &gy, &gz, &mean_ax, &mean_ay, &mean_az, &mean_gx, &mean_gy, &mean_gz);
    internal::mpu.setXAccelOffset(ax_offset);
    internal::mpu.setYAccelOffset(ay_offset);
    internal::mpu.setZAccelOffset(az_offset);
    internal::mpu.setXGyroOffset(gx_offset);
    internal::mpu.setYGyroOffset(gy_offset);
    internal::mpu.setZGyroOffset(gz_offset);
    
    set_VectorFloat_eeprom( internal::gyro_offset_addr, VectorFloat(gx_offset, gy_offset, gz_offset));
    set_VectorFloat_eeprom( internal::accel_offset_addr, VectorFloat(ax_offset, ay_offset, az_offset));
    
    Serial << "D:Set offsets to EEPROM: <" << gx_offset << ',' << gy_offset << ',' << gz_offset << "> <" << ax_offset << ',' << ay_offset << ',' << az_offset << '>' << endl;
  }

  String get_error_desc(int code){
    if(code - 1 < sizeof(internal::error_code_desc)/sizeof(String) && code >= 0){
      return internal::error_code_desc[code - 1];
    }
    return "Unknown Error";
  }
}




/*
class mpu_device{
  public:
    mpu_device(VectorFloat gyro_offset, VectorFloat accel_offset);
    void calibrate(int accuracy);
    void update();
    void close();
    Quaternion get_quaternion();
    int get_last_error();
    String get_error_desc(int code);
    void dmp_data_callback();

    static VectorFloat get_VectorFloat_eeprom(int offset);

    volatile bool mpu_interrupt = false; 
    static const int interrupt_pin = 2;
    static const int gyro_offset_addr = 0;
    static const int accel_offset_addr = sizeof(vec3f);
    
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

    void set_VectorFloat_eeprom(int offset, VectorFloat data);
    
};
*/
#endif
