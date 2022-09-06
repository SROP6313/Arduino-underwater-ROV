#include <Arduino.h>
#include <SPI.h>
#include <string.h>
#include <Wire.h>
#include <Servo.h>

#include "I2Cdev.h"  
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
Servo myServo1, myServo2, myServo3, myServo4, myServo5, myServo6; // 創建舵機對象來控制電調

#define OUTPUT_READABLE_YAWPITCHROLL

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

int temperature; 
String command;
volatile boolean process;

void setup () {
  Serial.begin (115200);
  pinMode(MISO, OUTPUT); // have to send on master in so it set as output
  SPCR |= _BV(SPE); // turn on SPI in slave mode
  process = false;
  SPI.attachInterrupt(); // turn on interrupt
  // Serial.print("MOSI: ");
  //Serial.println(MOSI);
  // Serial.print("MISO: ");
  //Serial.println(MISO);
  // Serial.print("SCK: ");
  //Serial.println(SCK);
  // Serial.print("SS: ");
  //Serial.println(SS);
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
  pinMode(dirPin2,OUTPUT);
  pinMode(stepPin2,OUTPUT);

  Wire.begin(2);
  Wire.setClock(400000);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

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
    // ERROR!a
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

char buff [50];
volatile byte indx;
volatile byte s_received;
byte c;

ISR (SPI_STC_vect) {// SPI interrupt routine { 
  c = SPDR; // read byte from SPI Data Register
  process = true;
}

int actioncomplete = 1;   //1 = 動作完成；0 = 動作未完成
int stableStart = 0;
int speed1 = 1475, speed2 = 1475, speed3 = 1475, speed4 = 1475, speed5 = 1475, speed6 = 1475;
int originSpeed1, originSpeed2;
int control = 0;
int speednum = 0; 
int steppernum = 0;
int speedlimit = 0;
byte speedstatus = 5;  // 5 = stop

void loop () {
  if (process && actioncomplete == 1) {
    process = false; //reset the process
    //Serial.println (buff); //print the array on serial monitor
    s_received = c;
    c = 0;
    if(s_received == 'f')  // forward
    {
      Serial.println("收到f命令");
      if(speedlimit < 4)
      {
        speedlimit++;
        control = 1;
        actioncomplete = 0;
      }
      else
      {
        speedlimit = 4;
      }
      stableStart = 0;          
      speednum = 0;
      speedstatus++;
    }
    else if(s_received == 'l')  // left
    {
      Serial.println("收到l命令");
      control = 2;
      originSpeed2 = speed2;
      stableStart = 0;
      actioncomplete = 0;
      speednum = 0;
    }
    else if(s_received== 'r')  // right
    {
      Serial.print("收到r命令");
      control = 3;
      originSpeed1 = speed1;
      stableStart = 0;
      actioncomplete = 0;
      speednum = 0;
    }
    else if(s_received == 'b')  // backward
    {
      Serial.println("收到b命令");
      if(speedlimit > (-2))
      {
        speedlimit--;
        control = 4;
        actioncomplete = 0;
      }
      else
      {
        speedlimit = -2;
      }
      stableStart = 0;
      speednum = 0;
      speedstatus--;
    }
    else if (s_received == 't')  // stop
    {
      Serial.print("收到t命令");
      control = 5;
      stableStart = 0;
      actioncomplete = 0;
      speednum = 0;
    }
    else if (s_received == 's')  // stable
    {
      Serial.print("收到s命令");
      stableStart = 1;
    }
    else if (s_received == 'h')  // hand
    {
      Serial.println("收到h命令");
      //stableStart = 0;
      actioncomplete = 0;
      if(steppernum == 0)
      {
        digitalWrite(dirPin1,HIGH);
        for(int i=0; i<500; i++)    // i=500 步進馬達轉 0.5s
        {
          digitalWrite(stepPin1,HIGH);
          delayMicroseconds(1000);
          digitalWrite(stepPin1,LOW);
          delayMicroseconds(1000);
        }
        steppernum = 1;
      }
      else
      {
        digitalWrite(dirPin1,LOW);
        for(int i=0; i<500; i++)    // i=500 步進馬達轉 0.5s
        {
          digitalWrite(stepPin1,HIGH);
          delayMicroseconds(1000);
          digitalWrite(stepPin1,LOW);
          delayMicroseconds(1000);
        }
        steppernum = 0;
      } 
      actioncomplete = 1;  
    }
    else if (s_received == 'a')  // armup
    {
      Serial.println("收到a命令");
      stableStart = 0;
      actioncomplete = 0;
      digitalWrite(dirPin1,HIGH);
      for(int i=0; i<500; i++)    // i=500 步進馬達轉 0.5s
      {
        digitalWrite(stepPin1,HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin1,LOW);
        delayMicroseconds(1000);
      }
      actioncomplete = 1;
    }
    else if (s_received == 'q')  // armdown
    {
      Serial.println("收到q命令");
      stableStart = 0;
      actioncomplete = 0;
      digitalWrite(dirPin1,LOW);
      for(int i=0; i<500; i++)    // i=500 步進馬達轉 0.5s
      {
        digitalWrite(stepPin1,HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin1,LOW);
        delayMicroseconds(1000);
      }
      actioncomplete = 1;
    }
    else if (s_received == 'd')  // dive
    {
      Serial.print("收到d命令");
      control = 6;
      stableStart = 0;
      actioncomplete = 0;
      speednum = 0;
    }
    else if (s_received == 'e')  // rise
    {
      Serial.println("收到e命令");
      control = 7;
      stableStart = 0;
      actioncomplete = 0;
      speednum = 0;
    }
    else
    {
      SPDR = speedstatus;
    }
    indx = 0; //reset button to zero
  }

  switch (control)
  {
    case 0:
      speednum = 0;
      speed1 = speed1;
      speed2 = speed2;
      break;
    
    case 1:      //前進
      speednum++;
      speed1++;
      speed2++;
      if(speednum >= 100) 
      {
        speednum = 0;
        control = 0;
        actioncomplete = 1;
      }
      break;

    case 2:      //左轉
      speednum++;
      if(speednum < 100)
      {
        speed2--;
      }        
      else
      {
        speed2++;
        if(speed2 >= originSpeed2)
        {
          speed2 = originSpeed2;
          speednum = 0;
          control = 0;
          actioncomplete = 1;
        }
      }
      break;
    
    case 3:      //右轉
      speednum++;
      if(speednum < 100)
      {
        speed1--;
      }        
      else
      {
        speed1++;
        if(speed1 >= originSpeed1)
        {
          speed1 = originSpeed1;
          speednum = 0;
          control = 0;
          actioncomplete = 1;
        }
      }
      break;
      
    case 4:      //後退
      speednum++;
      speed1--;
      speed2--;
      if(speednum >= 100) 
      {
        speednum = 0;
        control = 0;
        actioncomplete = 1;
      }
      break;

    case 5:      //停止
      speednum = 0;
        
      if(speed1 > 1475) speed1--;
      if(speed1 < 1475) speed1++;
      if(speed2 > 1475) speed2--;
      if(speed2 < 1475) speed2++;
      if(speed3 > 1475) speed3--;
      if(speed3 < 1475) speed3++;
      if(speed4 > 1475) speed4--;
      if(speed4 < 1475) speed4++;
      if(speed5 > 1475) speed5--;
      if(speed5 < 1475) speed5++;
      if(speed6 > 1475) speed6--;
      if(speed6 < 1475) speed6++;

      if(speed1==1475 && speed2==1475 && speed3==1475 && speed4==1475 && speed5==1475 && speed6==1475) 
      {          
        control = 0;
        actioncomplete = 1;
        speedstatus = 5;
      }
      break;

    case 6:      //下潛
      speednum++;
      speed3++;
      speed4++;
      speed5++;
      speed6++;
      if(speednum >= 100) 
      {
        speednum = 0;
        control = 0;
        actioncomplete = 1;
      }
      break;

    case 7:      //上升
      speednum++;
      speed3--;
      speed4--;
      speed5--;
      speed6--;
      if(speednum >= 100) 
      {
        speednum = 0;
        control = 0;
        actioncomplete = 1;
      }
      break;
  }
  myServo1.writeMicroseconds(speed1);
  myServo2.writeMicroseconds(speed2);
  myServo3.writeMicroseconds(speed3);
  myServo4.writeMicroseconds(speed4);
  myServo5.writeMicroseconds(speed5);
  myServo6.writeMicroseconds(speed6);
  delay(10);

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
        }
        if((ypr[2] * 180/M_PI)>30 || (ypr[2] * 180/M_PI)<-30)
        {
          Serial.println("馬達轉動!");
        }
      }
    #endif    
  }
}
