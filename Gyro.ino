/*
    Name:       Gyro.ino
    Created:	2/24/2019 10:56:34 AM
    Author:     James L
*/
#include "I2Cdev.h"
#include <SPI.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <RF24_config.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif



MPU6050 mpu;



bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q, tmp;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

bool equat(Quaternion&a, Quaternion& b) {
	return ((a.x == b.x)
		&& (a.y == b.y)
		&& (a.z == b.z)
		&& (a.w == b.w));
}

void setequate(Quaternion&a, const Quaternion& b) {
	a.x = b.x;
	a.y = b.y;
	a.z = b.z;
	a.w = b.w;
}


volatile bool mpuInterrupt = false; 
void dmpDataReady() {
	mpuInterrupt = true;
}

RF24 radio(A2, A3); // CE, CSN
const byte address[6] =  "00010" ;

char start[] = { '$' };
char stop[] = { '!' };
bool changed = false;

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	radio.begin();
	radio.openWritingPipe(address);
	radio.setPALevel(RF24_PA_MIN);
	radio.stopListening();
	mpu.initialize();

	devStatus = mpu.dmpInitialize();
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	if (devStatus == 0)
	{
		mpu.setDMPEnabled(true);
		mpuIntStatus = mpu.getIntStatus();
		dmpReady = true;

		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		Serial.print("DMP init failed: ");
		Serial.print(devStatus);
		Serial.println();
	}

}

void loop()
{
	if (!dmpReady) return;

	mpuInterrupt = true;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
	}
	else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;
#define OUTPUT_READABLE_QUATERNION

#ifdef OUTPUT_READABLE_QUATERNION
		// display quaternion values in easy matrix form: w x y z
		mpu.dmpGetQuaternion(&tmp, fifoBuffer);
		if (!equat(tmp, q)) {
			setequate(q, tmp);
			changed = true;
		}
		String s = String(q.x) + " " + String(q.y) + " " + String(q.z) + " " + String(q.w);
		//radio.write(s.c_str(), sizeof s);
		Serial.print("quat\t");
		Serial.print(q.w);
		Serial.print("\t");
		Serial.print(q.x);
		Serial.print("\t");
		Serial.print(q.y);
		Serial.print("\t");
		Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
		Serial.print("euler\t");
		Serial.print(euler[0] * 180 / M_PI);
		Serial.print("\t");
		Serial.print(euler[1] * 180 / M_PI);
		Serial.print("\t");
		Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180 / M_PI);
		Serial.print("\t");
		Serial.print(ypr[1] * 180 / M_PI);
		Serial.print("\t");
		Serial.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
		// display real acceleration, adjusted to remove gravity
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		Serial.print(aaReal.y);
		Serial.print("\t");
		Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
		// display initial world-frame acceleration, adjusted to remove gravity
		// and rotated based on known orientation from quaternion
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		Serial.print("aworld\t");
		Serial.print(aaWorld.x);
		Serial.print("\t");
		Serial.print(aaWorld.y);
		Serial.print("\t");
		Serial.println(aaWorld.z);
#endif

		radio.stopListening();
		if (changed) {
			String A = "A " + String(q.x) + " " + String(q.y);
			String G = "G " + String(q.z) + " " + String(q.w);

			//radio.write(res.c_str(), sizeof res);
			radio.write(start, sizeof start);
			delay(2);
			radio.write(A.c_str(), sizeof A);
			delay(2);
			radio.write(G.c_str(), sizeof G);
			delay(2);
			radio.write(stop, sizeof stop);
			delay(10);
			changed = false;
		}
	}
}


//#include <Servo.h>

//int16_t ax, ay, az;
//int16_t gx, gy, gz;

//Servo myservo1;
//Servo myservo2;
//Servo myservo3;
//Servo myservo4;
//Servo myservo5;



//myservo1.attach(9);
//myservo2.attach(10);
//myservo3.attach(A7);
//myservo4.attach(A8);
//myservo5.attach(A9);

//delay(5);
