
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Sensor.h"
#include "Motor.h"


Sensor s;
Motor m;

void setup() {
  Serial.begin(9600);
  m.AFMS.begin();
}

void loop() {

  char code;
  long sensorid;
  int angle;
  int dir;
  
  while(1) {

    if(Serial.available()) {
      code = Serial.read();
      if(code == 's') {

        Serial.println(s.getSensorValues());
      
      } else if(code == 'm') {
        
        long temp = Serial.parseInt();
            sensorid = (long) temp / 10000;
            temp = temp % 10000;
            // Dir -> 0 => backward, 1 => forward
            dir = temp / 1000;
            angle = temp % 1000;
            Serial.println(sensorid);

            switch (sensorid) {
          case 1:
            m.moveMotor1(dir,angle);
            break;
          case 2:
            m.moveMotor2(dir,angle);
            break;
          case 3:
            m.moveMotor3(dir,angle);
            Serial.println("Motor 3");
            break;
          case 4:
            Serial.println("Starting Motor 4");
            m.moveMotor4(dir,angle);
            Serial.println("Motor 4");
            break;
          default:
            //Serial.println("Got a wrong motor id");
            break;
            }

      }
      else if(code == 'd'){
        m.stopMotors();
      }
    }
  }
}
