#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Motor.h"
#include "Sensor.h"

Motor::Motor() {
	AFMS = Adafruit_MotorShield();
	myMotor1 = AFMS.getMotor(1);
	myMotor2 = AFMS.getMotor(2);
	myMotor3 = AFMS.getMotor(3);
	myMotor4 = AFMS.getMotor(4);
}

void Motor::moveMotor1(int dir,int angle) {
	int initialSensorVal = sensor.getSensor1Value();
	// Current calibration -> 1 degree => 5 points in sensor
	int changeRequired = 5 * angle;

	if(dir == 0)
		myMotor1->run(BACKWARD);
	else if(dir ==1)
		myMotor1->run(FORWARD);

	myMotor1->setSpeed(75);
	int currentSensorVal = sensor.getSensor1Value();
	while(abs(currentSensorVal - initialSensorVal) < changeRequired) {
		currentSensorVal = sensor.getSensor1Value();
	}
	myMotor1->setSpeed(0);
}

void Motor::moveMotor2(int dir,int angle) {
	int initialSensorVal = sensor.getSensor2Value();
	// Current calibration -> 1 degree => 5 points in sensor
	int changeRequired = 5 * angle;

	if(dir == 0)
		myMotor2->run(BACKWARD);
	else if(dir ==1)
		myMotor2->run(FORWARD);

	myMotor2->setSpeed(75);
	int currentSensorVal = sensor.getSensor2Value();
	while(abs(currentSensorVal - initialSensorVal) < changeRequired) {
		currentSensorVal = sensor.getSensor2Value();
	}
	myMotor2->setSpeed(0);
}

void Motor::moveMotor3(int dir,int angle) {
	int initialSensorVal = sensor.getSensor3Value();
	// Current calibration -> 1 degree => 5 points in sensor
	int changeRequired = 5 * angle;

	if(dir == 0)
		myMotor3->run(BACKWARD);
	else if(dir ==1)
		myMotor3->run(FORWARD);

	myMotor3->setSpeed(75);
	int currentSensorVal = sensor.getSensor3Value();
	while(abs(currentSensorVal - initialSensorVal) < changeRequired) {
		currentSensorVal = sensor.getSensor3Value();
	}
	myMotor3->setSpeed(0);
}

void Motor::moveMotor4(int dir,int angle) {
	int initialSensorVal = sensor.getSensor4Value();
	// Current calibration -> 1 degree => 5 points in sensor
	int changeRequired = 5 * angle;

	if(dir == 0)
		myMotor4->run(BACKWARD);
	else if(dir ==1)
		myMotor4->run(FORWARD);

	myMotor4->setSpeed(75);
	int currentSensorVal = sensor.getSensor4Value();
	while(abs(currentSensorVal - initialSensorVal) < changeRequired) {
		currentSensorVal = sensor.getSensor4Value();
	}
	myMotor4->setSpeed(0);
}

