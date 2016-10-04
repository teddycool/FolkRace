#include <Servo.h>
#include <SharpIR.h>

int IN1=11;
int IN2=10;
int ENA=9;
int SERVO = 2;
int START=4;  //start race via the remote
int KILL=5;   //cut power to the motor
// analog inputs from ir-sensors
// analog input from US sensor
// i2c bus, accelerometer and compass

Servo myservo;  // create servo object to control the servo
int pos = 0;    // variable to store the servo position
SharpIR fdist(0,25,90,1080);
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
 Serial.begin(9600);
 
}

void loop()
{  
 int dis = fdist.distance();
 Serial.println(dis); 
 setSpeed(255);
 forward();
 for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
 setSpeed(150);
 reverse();
for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}



void setSpeed(int speed)
{
  analogWrite(ENA, speed); // motor speed  
}

void forward()
{
 digitalWrite(IN1,LOW);// rotate forward
 digitalWrite(IN2,HIGH);
}

void reverse()
{
  digitalWrite(IN1,HIGH);// rotate reverse
  digitalWrite(IN2,LOW);
}

void setServo(int POS){
  myservo.write(POS); 
  }


