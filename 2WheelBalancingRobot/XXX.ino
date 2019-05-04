#include <SoftwareSerial.h>

#include <Wire.h>

#include <I2Cdev.h>

#include <PID_v1.h>

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

#endif

SoftwareSerial mySerial(0,1);

int _status = 315;


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 175;//173
double setpoint = originalSetpoint;
double input, output;

//adjust these values to fit your own design
double Kp = 4; //2.5;   //60    50  60
double Kd = 0.15;  //0.1; //1.4        2 
double Ki = 25;  //16;   // 70       70    
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
//MOTOR CONTROLLER
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int _currentSpeed;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}
//++++++++++++++++++++++++++++++++++++++++++++++
//________________________________________________
void Move(int _speed)
{
    if (_speed == _currentSpeed) return;
    
    if (_speed > 255) _speed= 255;
    else if (_speed < -255) _speed = -255;
    
    digitalWrite(IN1, _speed > 0 ? HIGH : LOW);
    digitalWrite(IN2, _speed > 0 ? LOW : HIGH);
    digitalWrite(IN3, _speed > 0 ? HIGH : LOW);
    digitalWrite(IN4, _speed > 0 ? LOW : HIGH);
    
    _currentSpeed = _speed;
}


//++++++++++++++++++++++++++++++++++++++++++++
//_____________________________________________________
void setup()

{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  mySerial.begin(9600);
  Serial.begin(9600); 
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
   TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)  nodeMCU = 8MHz, arduino = 16MHz
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  mpu.initialize();
  
  devStatus = mpu.dmpInitialize();
    //330  326 321

  mpu.setXGyroOffset(330); //220 
  mpu.setYGyroOffset(340);  //76       430  v1-335
  mpu.setZGyroOffset(320); //-85
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);   ///10
    pid.SetOutputLimits(-255, 255); 
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++
//____________________________________________________
void loop()
{

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    Move(output);
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180/M_PI + 180;
    serialEvent();
  }
}

//-----------------------------------------------------------------------------------
void serialEvent() {
  if(mySerial.available())
  {
    char inChar = mySerial.read();
    mySerial.println(inChar);
    if (inChar == 't'){
       mpu.setYGyroOffset(_status+10);
    }
    if (inChar == 'l'){
       mpu.setYGyroOffset(_status-10);
    }
    if (inChar == 'd'){
       mpu.setYGyroOffset(_status);
    }
    Serial.print(mpu.getYGyroOffset());
    mySerial.print(mpu.getYGyroOffset());
  }
}
