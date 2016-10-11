#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool dmpReady = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
int M1_P = 5;
int M1_N = 6;
int M2_P = 10;
int M2_N = 11;
int led = 13;
int spd = 255;
int Trig1 = 7;
int echo1 = 8;
int Trig2 = 12;
int echo2 = 4;
int del = 20;
unsigned long echo = 0;
unsigned long dist = 0;
unsigned long distance = 0;
void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  pinMode(M1_P,OUTPUT);
  pinMode(M1_N,OUTPUT);
  pinMode(M2_P,OUTPUT);
  pinMode(M2_N,OUTPUT);
  pinMode(Trig1,OUTPUT);
  pinMode(echo1,INPUT);
  pinMode(Trig2,OUTPUT);
  pinMode(echo2,INPUT);
  pinMode(led,OUTPUT);
}

unsigned long ping(int x, int y){
  digitalWrite(led,HIGH);
  digitalWrite(x,LOW);
  delayMicroseconds(5);
  digitalWrite(x,HIGH);
  delayMicroseconds(10);
  digitalWrite(x,LOW);
  echo = pulseIn(y,HIGH);
  dist = echo/58.138;
  Serial.println(dist);
  digitalWrite(led,LOW);
  return dist;
}

void mov_direction(int M1_P_spd,int M1_N_spd,int M2_P_spd,int M2_N_spd){
  analogWrite(M1_P,M1_P_spd);
  analogWrite(M1_N,M1_N_spd);
  analogWrite(M2_P,M2_P_spd);
  analogWrite(M2_N,M2_N_spd);
  digitalWrite(led,HIGH);
  Serial.println("Forward");
  delay(del);
  analogWrite(M1_P,0);
  analogWrite(M1_N,0);
  analogWrite(M2_P,0);
  analogWrite(M2_N,0);
  digitalWrite(led,LOW);
}
void loop() {
  for(int kl = 1; kl<3; kl++){
  digitalWrite(led,HIGH);
  delay(1000);
  digitalWrite(led,LOW);
  delay(1000);
  }
  while(1){
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.println(ypr[0] * 180/M_PI);
        #endif
        #ifdef OUTPUT_TEAPOT
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
    }
  int incomingByte;
  
  if(Serial.available() > 0) {
    incomingByte = Serial.read();
    switch(incomingByte){
      case 65:
      mov_direction(spd,0,spd,0);
      break;
      case 66:
      mov_direction(0,spd,0,spd);
      break;
      case 67:
      mov_direction(spd,0,0,spd);
      break;
      case 68:
      mov_direction(0,spd,spd,0);
      break;
      case 69:
      ping(Trig1,echo1);
      break;
      case 70:
      ping(Trig2,echo2);
      break;
      case 71:
      Serial.println(ypr[0] * 180/M_PI);  
      break;
  }
}
  }
}
