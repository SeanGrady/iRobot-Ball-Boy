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
// TODO : Add support for speed as well
/*
 * UP: 1  Max: 230
 * Down: 0  Min: 35
 */
void Motor::moveMotor1(int dir,int angle) {
	int initialSensorVal = sensor.getSensor1Value();
	// Current calibration -> 1 degree => 5 points in sensor
	int changeRequired = 5 * angle;

	if(dir == 0)
		myMotor1->run(BACKWARD);
	else if(dir ==1)
		myMotor1->run(FORWARD);

	myMotor1->setSpeed(230);
	int currentSensorVal = sensor.getSensor1Value();
	while(abs(currentSensorVal - initialSensorVal) < changeRequired) {
		currentSensorVal = sensor.getSensor1Value();
    if(dir == 1 && currentSensorVal > 900){
      break;
    }
    else if(dir == 0 && currentSensorVal < 660){
      break;
    }
	}
	myMotor1->setSpeed(0);
}

/*
 * For this sensor direction 0 moves the arm downward. 1 upward
 * Maximum point upward is 775, downward is 336, 90 degrees 439
 */
void Motor::moveMotor2(int dir,int angle) {
	int initialSensorVal = sensor.getSensor2Value();
	// Current calibration -> 1 degree => 5 points in sensor
	int changeRequired = 5 * angle;

	if(dir == 0)
		myMotor2->run(BACKWARD);
	else if(dir ==1)
		myMotor2->run(FORWARD);

	myMotor2->setSpeed(230);
	int currentSensorVal = sensor.getSensor2Value();
	while(abs(currentSensorVal - initialSensorVal) < changeRequired) {      
		currentSensorVal = sensor.getSensor2Value();
    if(dir == 1 && currentSensorVal > 775){
      break;
    }
    else if(dir == 0){
      if(currentSensorVal < 630){
        break;
      }
    }
	}
	myMotor2->setSpeed(0);
}

void Motor::moveMotor3(int dir,int angle) {
	int initialSensorVal = sensor.getSensor3Value();
	// Current calibration -> 1 degree => 5 points in sensor
	int changeRequired = 5 * angle;

  int temp = 0;  

	if(dir == 0)
		myMotor3->run(BACKWARD);
	else if(dir ==1)
		myMotor3->run(FORWARD);

	myMotor3->setSpeed(75);
	int currentSensorVal = sensor.getSensor3Value();
	while(abs(currentSensorVal - initialSensorVal) < changeRequired) {
		currentSensorVal = sensor.getSensor3Value();

    
    if(temp == 10000){
      break;
    }
    else if(dir == 1 && currentSensorVal > 775){
      break;
    }
    else if(dir == 0 && currentSensorVal < 600){
      break;
    }
    temp++;
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

	myMotor4->setSpeed(50);
	delay(angle);
	myMotor4->setSpeed(0);
}

void Motor::stopMotors(){
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  myMotor3->setSpeed(0);
  myMotor4->setSpeed(0);

}


