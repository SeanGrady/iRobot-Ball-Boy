/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

// Analog Sensors
int sensor1 = A0;
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A4;


// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test! \r\n");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  Serial.print("After Begin \r\n");
 
}

void loop() {
  uint8_t i;
  char motorNumber;
  int sensorid;
  int angle;

  printf( "Engage Motor number: \r\n");
  while(!Serial.available()){
    
  }
  
  motorNumber = Serial.read();

  if(motorNumber == 'a'){
    moveMotor(myMotor1);
  }
  else if(motorNumber == 'b'){
    moveMotor(myMotor2);
  }
  else if(motorNumber == 'c'){
    moveMotor(myMotor3);
  }
  else if(motorNumber == 'd'){
    moveMotor(myMotor4);
  }
  else if(motorNumber == 's'){
       Serial.println("Received s!");
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
   } 
   else if (motorNumber == 'm') {
       long temp = Serial.parseInt();
       sensorid = (int) temp / 1000;
       angle = temp % 1000;
       Serial.print("Motor id: ");
       Serial.print(sensorid);
       Serial.print(" Angle : ");
       Serial.print(angle);
       Serial.println();
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

void moveMotor(Adafruit_DCMotor *myMotor){
  uint8_t i;
  char dir;

  Serial.print("Direction: a: forward, b: backward \r\n");
  while(!Serial.available()){
    
  }
    
  dir = Serial.read();

  Serial.print("direction read: \r\n" );
  Serial.print(dir);

  if(dir == 'a'){
    myMotor->run(FORWARD);
    for (i=0; i<180; i++) {
      myMotor->setSpeed(i);  
      delay(10);
    }
    for (i=180; i!=0; i--) {
      myMotor->setSpeed(i);  
      delay(10);
    }
    
  }
  
  else if(dir == 'b'){
    myMotor->run(BACKWARD);
    for (i=0; i<180; i++) {
      myMotor->setSpeed(i);  
      delay(10);
    }
    for (i=180; i!=0; i--) {
      myMotor->setSpeed(i);  
      delay(10);
    }
  }
  else{}

  myMotor->run(RELEASE);
  delay(1000);
}

