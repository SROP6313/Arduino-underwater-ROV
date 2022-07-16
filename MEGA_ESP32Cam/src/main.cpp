#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <Servo.h> 

#include "I2Cdev.h"  
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
Servo myServo; // 創建舵機對象來控制電調

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

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
String command;

void setup() {  
  Serial.begin(115200);
  Serial2.begin(38400);

  myServo.attach(9);
  myServo.writeMicroseconds(1475);  //推進器停止  (後退)1000--1475--2000(前進)

  Wire.begin(2);
  Wire.setClock(400000);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

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
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
  pinMode(LED_PIN, OUTPUT);
  temperature = 0;
}

int stableStart = 0;
int speed = 1475;
int control = 0;
int speednum = 0;
int interval;

void loop(){    //重要：MPU6050 INT腳位拔掉才會無限循環
    command = "";
    if (Serial2.available())
    {
      command = char(Serial2.read());
    }    
    //delay(10);
    //command.trim();   //修剪字串頭尾 
  
    if (command.length() > 0)   // 接收命令
    {      
      if(!checkStringIsNumerical(command))
      {
        if (command == "f")  // forward
        {
          Serial.print("收到f命令");
          //myServo.writeMicroseconds(1600);
          control = 1;
          stableStart = 0;
        }
        if (command == "l")  // left
        {
          Serial.print("收到l命令");
          stableStart = 0;
        }
        if (command == "r")  // right
        {
          Serial.print("收到r命令");
          stableStart = 0;
        }
        if (command == "b")  // backward
        {
          Serial.print("收到b命令");
          control = 2;
          stableStart = 0;
        }
        if (command == "t")  // stop
        {
          Serial.print("收到t命令");
          control = 3;
          stableStart = 0;
        }
        if (command == "s")  // stable
        {
          Serial.print("收到s命令");
          stableStart = 1;
        }
        if (command == "h")  // hand
        {
          Serial.println("收到h命令");
          stableStart = 0;
        }
        if (command == "a")  // arm
        {
          Serial.println("收到a命令");
          stableStart = 0;
        }
        if (command == "d")  // dive
        {
          Serial.print("收到d命令");
          stableStart = 0;
        }
        if (command == "e")  // rise
        {
          Serial.println("收到e命令");
          stableStart = 0;
        }
      }
    }

    switch (control)
    {
      case 0:
        speednum = 0;
        speed = speed;
        break;
    
      case 1:
        speednum++;
        speed++;
        if(speednum >= 100) 
        {
          speednum = 0;
          control = 0;
        }
        break;
    
      case 2:
        speednum++;
        speed--;
        if(speednum >= 100) 
        {
          speednum = 0;
          control = 0;
        }
        break;
      case 3:
        speednum = 0;
        speednum++;
        if(speed > 1475)
        {
          interval = speed - 1475;
          speed--;
        }
        if(speed < 1475)
        {
          interval = 1475 - speed;
          speed++;
        }
        if(speednum >= interval) 
        {
          speednum = 0;
          control = 0;
        }
        break;
    }

    myServo.writeMicroseconds(speed);
    delay(10);

    //if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
        if(stableStart == 1)
        {
          if((ypr[1] * 180/M_PI)>30 || (ypr[1] * 180/M_PI)<-30)
          {
            Serial.println("馬達轉動!");
            speed = speed;
          }
          if((ypr[2] * 180/M_PI)>30 || (ypr[2] * 180/M_PI)<-30)
          {
            Serial.println("馬達轉動!");
          }
        }
      #endif

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    
    }
}