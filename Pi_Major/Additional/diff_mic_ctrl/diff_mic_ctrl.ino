#include "Timer.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
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
bool flg = LOW;
int cnt;
int comp;
float gyr;
float gyr1;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
int M1_Pspd = 0;
int M1_Nspd = 255;
int M2_Pspd = 0;
int M2_Nspd = 255;
void dmpDataReady() {
    mpuInterrupt = true;
}
int w_enc = 0;
int M1_P = 5;
int M1_N = 6;
int M2_P = 10;
int M2_N = 11;
int led = 13;
int spd1 = 255;//right
int spd2 = 255;//left
int Trig1 = 7;
int echo1 = 8;
int Trig2 = 12;
int echo2 = 4;
int del = 20;
unsigned long echo = 0;
unsigned long dist = 0;
unsigned long distance = 0;
Timer t;
void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    if (devStatus == 0) {
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
    //pinMode(LED_PIN, OUTPUT);
    attachInterrupt(1, print, CHANGE);
  pinMode(M1_P,OUTPUT);
  pinMode(M1_N,OUTPUT);
  pinMode(M2_P,OUTPUT);
  pinMode(M2_N,OUTPUT);
  pinMode(Trig1,OUTPUT);
  pinMode(echo1,INPUT);
  pinMode(Trig2,OUTPUT);
  pinMode(echo2,INPUT);
  pinMode(led,OUTPUT);
  t.every(5000, donot);
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
  digitalWrite(led,LOW);
  return dist;
}

void mov_direction(int M1_P_spd,int M1_N_spd,int M2_P_spd,int M2_N_spd){
  analogWrite(M1_P,M1_P_spd);
  analogWrite(M1_N,M1_N_spd);
  analogWrite(M2_P,M2_P_spd);
  analogWrite(M2_N,M2_N_spd);
  digitalWrite(led,HIGH);
  //Serial.print("spd1=");
  //Serial.print(spd1);
  //Serial.print("spd2=");
  //Serial.println(spd2);
  //delay(del);
  //analogWrite(M1_P,0);
  //analogWrite(M1_N,0);
  //analogWrite(M2_P,0);
  //analogWrite(M2_N,0);
  //digitalWrite(led,LOW);
}
void stopmotors(){
  analogWrite(M1_P,0);
  analogWrite(M1_N,0);
  analogWrite(M2_P,0);
  analogWrite(M2_N,0);
  digitalWrite(led,LOW);
}
void runmotors(){
  while(1){
  analogWrite(M1_P,0);
  analogWrite(M1_N,255);
  analogWrite(M2_P,0);
  analogWrite(M2_N,255);
  digitalWrite(led,HIGH);
  delay(100);
  digitalWrite(led,HIGH);
  delay(100);
  }
}
void movestr(float gyrref,float gyract){
  comp = (int) (gyract-gyrref)*0.5;
  if(gyract<gyrref-2){
    M1_Nspd = M1_Nspd - comp;
    M2_Nspd = M2_Nspd + comp;
  }
  else if(gyract>gyrref+2){
    M1_Nspd = M1_Nspd - comp;
    M2_Nspd = M2_Nspd + comp;
  }
  if(M1_Nspd > 255){
   M1_Nspd = 255; 
  }
  if(M2_Nspd > 255){
   M2_Nspd = 255; 
  }
  analogWrite(M1_P,M1_Pspd);
  analogWrite(M1_N,M1_Nspd);
  analogWrite(M2_P,M2_Pspd);
  analogWrite(M2_N,M2_Nspd);
  digitalWrite(led,HIGH);
}
void print() {
  w_enc = w_enc + 1;
  //Serial.println(x/256);
if(w_enc == 40)
{
 w_enc = 0; 
}
}
void donot(){
  
}
void loop() {
  for(int kl = 1; kl<2; kl++){
  digitalWrite(led,HIGH);
  delay(100);
  digitalWrite(led,LOW);
  delay(100);
  t.update();
  }
  while(1){
  int incomingByte;
  t.update();
  if(Serial.available() > 0) {
    incomingByte = Serial.read();
    switch(incomingByte){
      case 65:
      mov_direction(spd1,0,spd2,0);
      digitalWrite(led,HIGH);
      break;
      case 66:
      spd1 = Serial.parseInt();
      spd2 = Serial.parseInt();
      mov_direction(0,spd1,0,spd2);
      digitalWrite(led,HIGH);
      break;
      case 67:
      mov_direction(spd1,0,0,spd2);
      digitalWrite(led,HIGH);
      break;
      case 68:
      mov_direction(0,spd1,spd2,0);
      digitalWrite(led,HIGH);
      break;
      case 69:
      dist = ping(Trig1,echo1);
      Serial.println(dist);
      digitalWrite(led,HIGH);
      break;
      case 70:
      dist = ping(Trig2,echo2);
      Serial.println(dist);
      digitalWrite(led,HIGH);
      break;
      case 71:
      Serial.println(ypr[0] * 180/M_PI);  //M_PI  
      digitalWrite(led,HIGH);
      break;
      case 72:
      Serial.println(w_enc);
      digitalWrite(led,HIGH);
      break;
      case 73:
      digitalWrite(led,HIGH);
      gyr = ypr[0] * 180/M_PI;
      flg = HIGH;
      break;
      case 74:
      stopmotors();
      break;
      case 75:
      runmotors();
      break;
  }
}
if(flg){
dist = ping(Trig1,echo1); 
gyr1 = ypr[0] * 180/M_PI;
movestr(gyr,gyr1);
if(dist<30){
  stopmotors();
  cnt = 0;
  flg = LOW;
}
}
    //if (!dmpReady) return;
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
        #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            //Serial.print("quat\t");
            //Serial.print(q.w);
            //Serial.print("\t");
            //Serial.print(q.x);
            //Serial.print("\t");
            //Serial.print(q.y);
            //Serial.print("\t");
            //Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            //Serial.print("euler\t");
            //Serial.print(euler[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(euler[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.println(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            //Serial.print("areal\t");
            //Serial.print(aaReal.x);
            //Serial.print("\t");
            //Serial.print(aaReal.y);
            //Serial.print("\t");
            //Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            //Serial.print("aworld\t");
            //Serial.print(aaWorld.x);
            //Serial.print("\t");
            //Serial.print(aaWorld.y);
            //Serial.print("\t");
            //Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
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

        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);
    }
  }
}
