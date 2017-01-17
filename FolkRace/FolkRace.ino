/*
 Name:		FolkRace.ino
 Created:	1/16/2017 7:37:53 PM
 Author:	psk
*/

#include <Wire.h>
#include <Servo.h>
#include <SharpIR.h>

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
const int HMC_addr = 0x1E;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int x, y, z; //triple axis data

const int servoCenter = 90; //Depends on servo mechanical installation
const int servoMax = 30; //Depends on frontaxle 
int currentServoPos = 90; //Where is servo now...

int intLonAxis; //Compass axis data fwd/revers
int intLatAxis; //Compass axis data left/right
int intUpAxis;  //Compass axis datat up/down

const int motorMin = 50; //Min pwm for motor to run at all (lowest possible speed). Max is 255, allways.

const int FORWARD = 1;
const int STOP = 0;
const int REVERSE = -1;

int state = STOP;


//int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int IN1 = 11;
int IN2 = 10;
int ENA = 9;
int SERVO = 2;
int START = 4;  //start race via the remote
int KILL = 5;   //cut power to the motor
int timer;
// analog inputs from ir-sensors
// analog input from US sensor
// i2c bus, accelerometer and compass

Servo myservo;  // create servo object to control the servo
SharpIR fdist(1,0);
SharpIR fldist(1,1);
SharpIR frdist(1,2);
SharpIR rdist(1,3);
//
//    model: is an int that determines your sensor:  1 for GP2Y0A21Y,  
//    ir: the pin where your sensor is attached.
//


void setup()
{

	//At start... Calibrate the sensors and set the compass relative to startposition
	//calibrate accelerometer. X and Y values should be zero. Up should be 1.
	//calibrate compass and set 

	//Set servo in mid-position



	pinMode(IN1, OUTPUT); 
	pinMode(IN2, OUTPUT);
	myservo.attach(SERVO);
	myservo.write(servoCenter);              // tell servo to go to position in variable 'pos'
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
	int fdis = fdist.getDistance();
	int fldis = fldist.getDistance();
	int frdis = frdist.getDistance();
	int rdis = rdist.getDistance();
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
	Serial.print("STATE: ");
	Serial.println(state);
	//

	switch (state) {

	case STOP:

		if (fdis > 50) {
			setSpeed(255);
			setServo(servoCenter);
			forward();
			state = FORWARD;
		}
		else {
			setSpeed(255);
			setServo(servoCenter - servoMax);
			reverse();
			state = REVERSE;
		}
		break;

	case FORWARD:
		if (fdis < 50) {
			setSpeed(0);
			setServo(servoCenter);
			state = STOP;
		}

		break;

	case REVERSE:
		if (fdis > 50) {
			setSpeed(0);
			setServo(servoCenter);
			state = STOP;
		}
		break;

	default:
		setSpeed(0);
		setServo(servoCenter);
		state = STOP;
		break;
	}
}



void setSpeed(int speed)
{
	analogWrite(ENA, speed); // motor speed  
	Serial.print("Speed set to: ");
	Serial.println(speed);
}

void forward()
{
	digitalWrite(IN1, LOW);// rotate forward
	digitalWrite(IN2, HIGH);
	Serial.println("Forward!");
}

void reverse()
{
	digitalWrite(IN1, HIGH);// rotate reverse
	digitalWrite(IN2, LOW);
	Serial.println("Reverse!");
}

void setServo(int POS) {
	myservo.write(POS);
}

void readI2C() {
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
	AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
	AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	Serial.print("AcX = "); Serial.print(AcX);
	Serial.print(" | AcY = "); Serial.print(AcY);
	Serial.print(" | AcZ = "); Serial.print(AcZ);
	Serial.print(" | Tmp = "); Serial.print(Tmp / 340.00 + 36.53);  //equation for temperature in degrees C from datasheet
	Serial.print(" C");
	Serial.print(" | GyX = "); Serial.print(GyX);
	Serial.print(" | GyY = "); Serial.print(GyY);
	Serial.print(" | GyZ = "); Serial.println(GyZ);

	int x, y, z; //triple axis data

				 //Tell the HMC what regist to begin writing data into
	Wire.beginTransmission(HMC_addr);
	Wire.write(0x03); //start with register 3.
	Wire.endTransmission();


	//Read the data.. 2 bytes for each axis.. 6 total bytes
	Wire.requestFrom(HMC_addr, 6);
	if (6 <= Wire.available()) {
		x = Wire.read() << 8; //MSB  x 
		x |= Wire.read(); //LSB  x
		z = Wire.read() << 8; //MSB  z
		z |= Wire.read(); //LSB z
		y = Wire.read() << 8; //MSB y
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