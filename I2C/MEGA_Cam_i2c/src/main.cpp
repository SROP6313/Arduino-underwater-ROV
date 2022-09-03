#include <Arduino.h>

#include <Wire.h> 
#include <Servo.h> 

#include "I2Cdev.h"  
#include "MPU6050_6Axis_MotionApps20.h"
#include "SoftI2CMaster.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
Servo myServo1, myServo2, myServo3, myServo4, myServo5, myServo6; // 創建舵機對象來控制電調

#define OUTPUT_READABLE_YAWPITCHROLL

SoftI2CMaster i2c = SoftI2CMaster( 12, 13, 0 );  //第二個 i2c bus (SCL, SDA, 0)

#define dirPin1 2
#define stepPin1 3
#define dirPin2 4
#define stepPin2 5

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

bool checkStringIsNumerical(String myString)  //檢查是數字還是字母
{
  uint16_t Numbers = 0;

  for(uint16_t i = 0; i < myString.length(); i++)
  {
    if (myString[i] == '0' || myString[i] == '1' || myString[i] == '2' || myString[i] == '3' || myString[i] == '4' || myString[i] == '5' || myString[i] == '6' || myString[i] == '7' || myString[i] == '8' || myString[i] == '9')
    {
      Numbers ++;
    }
  }
  if(Numbers == myString.length()) return true;
  else return false;
}

int temperature; 
byte x = 0;
char command;

// void requestEvent()   //送出資料
// {   
//   i2c.write(x); // respond with message of 1 bytes as expected by master
//   x++; 
// }

void receiveEvent(int howMany)  //接收命令
{ 
  command = i2c.read();
}

void setup() {
  myServo1.attach(6);
  myServo2.attach(7);
  myServo3.attach(8);
  myServo4.attach(9);
  myServo5.attach(10);
  myServo6.attach(11);
  myServo1.writeMicroseconds(1475);  //推進器停止  (後退)1000--1475--2000(前進)
  myServo2.writeMicroseconds(1475);
  myServo3.writeMicroseconds(1475);
  myServo4.writeMicroseconds(1475);
  myServo5.writeMicroseconds(1475);
  myServo6.writeMicroseconds(1475);

  pinMode(dirPin1,OUTPUT);   // 步進馬達腳位
  pinMode(stepPin1,OUTPUT);
  pinMode(dirPin2,OUTPUT);   // 步進馬達腳位
  pinMode(stepPin2,OUTPUT);

  Wire.begin(2); // join i2c bus with address #2
  Wire.setClock(400000);
  //i2c.onRequest(requestEvent); // register event
  i2c.onReceive(receiveEvent);
  Serial.begin(115200);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  //pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));
    // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

int actioncomplete = 1;   //1 = 動作完成；0 = 動作未完成
int stableStart = 0;
int speed1, speed2, speed3, speed4, speed5, speed6 = 1475;
int originSpeed1, originSpeed2;
int control = 0;
int speednum = 0;
//int interval;
//int spdifference1, spdifference2, spdifference3, spdifference4, spdifference5, spdifference6; 
int steppernum = 0;
int speedlimit = 0;

void loop() {
  if (actioncomplete == 1)   // 接收命令 (此區只執行一次)
  {
    if(command == 102)  //在字符串中查找指定字符 因為 f 的 ASCII value 是 102
    {
      //工作區
      Serial.println("收到f命令");
      command = 0;
    }
    if(command == 104)  //在字符串中查找指定字符 因為 h 的 ASCII value 是 104
    {
      //工作區
      Serial.println("收到h命令");
      command = 0;
    }
  }
}