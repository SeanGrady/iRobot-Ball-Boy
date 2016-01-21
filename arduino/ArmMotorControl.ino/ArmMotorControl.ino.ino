#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  AFMS.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  char code;
  int sensorid;
  int angle;
  int dir;

  while(1) {
    if(Serial.available()) {
      code = Serial.read();
      if (code == 's') {
        //Serial.println("Received s!");
        String sensorReadings = "";
        sensorReadings += String(getSensor1Value());
        sensorReadings += " ";
        sensorReadings += String(getSensor2Value());
        sensorReadings += " ";
        sensorReadings += String(getSensor3Value());
        sensorReadings += " ";
        sensorReadings += String(getSensor4Value());
        sensorReadings += " ";
        sensorReadings += String(getSensor5Value());
        Serial.println(sensorReadings);
      } else if (code == 'm') {
        long temp = Serial.parseInt();
        sensorid = (int) temp / 10000;
        temp = temp % 10000;
        // Dir -> 0 => backward, 1 => forward
        dir = temp / 1000;
        angle = temp % 1000;
        /*Serial.print("Motor id: ");
        Serial.print(sensorid);
        Serial.print(" Direction bit : ");
        Serial.print(dir);
        Serial.print(" Angle : ");
        Serial.print(angle);
        Serial.println(); */

        //Serial.println("trying to move the motor");
        switch (sensorid) {
          case 1:
            moveMotor1(dir,angle);
            break;
          case 2:
            moveMotor2(dir,angle);
            break;
          case 3:
            moveMotor3(dir,angle);
            break;
          case 4:
            moveMotor4(dir,angle);
            break;
          default:
            //Serial.println("Got a wrong motor id");
            break;
        }
      }
    }
  }
}

void calibrateSensor1(int duration) {
  Serial.println("Initial Sensor Reading : ");
  Serial.print(getSensor1Value());
  
  myMotor1->run(FORWARD);
  myMotor1->setSpeed(50);
  delay(duration);
  myMotor1->setSpeed(0);

  Serial.println("Final Sensor Reading : ");
  Serial.print(getSensor1Value());
  
  myMotor1->run(BACKWARD);
  myMotor1->setSpeed(50);
  delay(duration);
  myMotor1->setSpeed(0);
}

void moveMotor1(int dir,int angle) {
  int initialSensorVal = getSensor1Value();
  // Current calibration -> 1 degree => 5 points in sensor
  int changeRequired = 5 * angle;
  
  if(dir == 0)
    myMotor1->run(BACKWARD);
  else if(dir ==1)
    myMotor1->run(FORWARD);
    
  myMotor1->setSpeed(50);
  int currentSensorVal = getSensor1Value();
  while(abs(currentSensorVal - initialSensorVal) < changeRequired) {
    currentSensorVal = getSensor1Value();
  }
  myMotor1->setSpeed(0);
}

void moveMotor2(int dir,int angle) {
  if(dir == 0)
    myMotor2->run(BACKWARD);
  else
    myMotor2->run(FORWARD);
    
  for (int i=0; i<180; i++) {
    myMotor2->setSpeed(i);  
    delay(10);
  }
  for (int i=180; i!=0; i--) {
    myMotor2->setSpeed(i);  
    delay(10);
  }
}

void moveMotor3(int dir,int angle) {
  if(dir == 0)
    myMotor3->run(BACKWARD);
  else
    myMotor3->run(FORWARD);
    
  for (int i=0; i<180; i++) {
    myMotor3->setSpeed(i);  
    delay(10);
  }
  for (int i=180; i!=0; i--) {
    myMotor3->setSpeed(i);  
    delay(10);
  }
}

void moveMotor4(int dir,int angle) {
  if(dir == 0)
    myMotor4->run(BACKWARD);
  else
    myMotor4->run(FORWARD);
    
  for (int i=0; i<180; i++) {
    myMotor4->setSpeed(i);  
    delay(10);
  }
  for (int i=180; i!=0; i--) {
    myMotor4->setSpeed(i);  
    delay(10);
  }
}

int getSensor1Value() {
  return analogRead(sensor1);
}

int getSensor2Value() {
  return analogRead(sensor2);
}

int getSensor3Value() {
  return analogRead(sensor3);
}

int getSensor4Value() {
  return analogRead(sensor4);
}

int getSensor5Value() {
  return analogRead(sensor5);
}
