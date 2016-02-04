#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Sensor.h"

class Motor {

	private:
		Adafruit_DCMotor *myMotor1;
		Adafruit_DCMotor *myMotor2;
		Adafruit_DCMotor *myMotor3;
		Adafruit_DCMotor *myMotor4;
		Sensor sensor;

	public:
		Motor();
    Adafruit_MotorShield AFMS;
		void moveMotor1(int dir,int angle);
		void moveMotor2(int dir,int angle);
		void moveMotor3(int dir,int angle);
		void moveMotor4(int dir,int angle);
    void stopMotors();
	
};

#endif
