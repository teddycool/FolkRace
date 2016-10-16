#include <Servo.h>
#include <SharpIR.h>
#include<Wire.h>

const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int HMC_addr=0x1E;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int IN1=11;
int IN2=10;
int ENA=9;
int SERVO = 2;
int START=4;  //start race via the remote
int KILL=5;   //cut power to the motor
int timer;
// analog inputs from ir-sensors
// analog input from US sensor
// i2c bus, accelerometer and compass

Servo myservo;  // create servo object to control the servo
int pos = 90;    // variable to store the servo position, start in the middle = straigth forward
SharpIR fdist(0,25,90,1080);
SharpIR fldist(1,25,90,1080);
SharpIR frdist(2,25,90,1080);
SharpIR rdist(3,25,90,1080);
//
//    ir: the pin where your sensor is attached.
//    25: the number of readings the library will make before calculating an average distance.
//    93: the difference between two consecutive measurements to be taken as valid (in %)
//    model: is an int that determines your sensor:  1080 for GP2Y0A21Y, 20150 for GP2Y0A02Y  
//


void setup()
{
 pinMode(IN1,OUTPUT);
 pinMode(IN2,OUTPUT);  
 myservo.attach(SERVO);
 myservo.write(pos);              // tell servo to go to position in variable 'pos'
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);  // PWR_MGMT_1 register
Wire.write(0);     // set to zero (wakes up the MPU-6050)
Wire.endTransmission(true);
Wire.beginTransmission(HMC_addr); //start talking to compass
Wire.write(0x02); // Set the Register
Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
Wire.endTransmission(true);
 delay(15);                       // waits 15ms for the servo to reach the position
//Debugmode?
 Serial.begin(9600);
 
}

void loop()
{  
  //Wait for startsignal...
 //Read inputs
 //Forward looking IR sensors
 int fdis = fdist.distance();
 int fldis = fldist.distance();
 int frdis = frdist.distance();
 int rdis = rdist.distance();
//Accelerometer
//Compass

readI2C();



 //


 Serial.print("Left: ");
 Serial.print(fldis); 
 Serial.print(", Center: ");
 Serial.print(fdis); 
 Serial.print(", Right: ");
 Serial.print(frdis); 
 Serial.print(", Rear: ");
 Serial.println(rdis); 
 //
 
 if (timer == 1){
  setSpeed(255);
  forward();
  }

if (timer == 10){
  setSpeed(50);
  }

if (timer == 20){
reverse();
}

if (timer == 30){
  setSpeed(255);
  }
  
if (timer == 40){
  setSpeed(0);
  } 
  

if (timer == 50){
  timer=0;
  }
  
 timer = timer+1;
 delay(10);
} 



void setSpeed(int speed)
{
  analogWrite(ENA, speed); // motor speed  
   Serial.print("Speed set to: ");
    Serial.println(speed);
}

void forward()
{
 digitalWrite(IN1,LOW);// rotate forward
 digitalWrite(IN2,HIGH);
 Serial.println("Forward!");
}

void reverse()
{
  digitalWrite(IN1,HIGH);// rotate reverse
  digitalWrite(IN2,LOW);
   Serial.println("Reverse!");
}

void setServo(int POS){
  myservo.write(POS); 
  }

void readI2C(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" C");
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  
  int x,y,z; //triple axis data

  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(HMC_addr);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();
  
 
 //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(HMC_addr, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //MSB  x 
    x |= Wire.read(); //LSB  x
    z = Wire.read()<<8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read()<<8; //MSB y
    y |= Wire.read(); //LSB y
  }
  
  // Show Values
  Serial.print("X Value: ");
  Serial.print(x);
  Serial.print(" | Y Value: ");
  Serial.print(y);
  Serial.print(" | Z Value: ");
  Serial.print(z);
  Serial.println();
  }
