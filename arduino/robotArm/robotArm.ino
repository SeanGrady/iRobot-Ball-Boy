
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Sensor.h"
#include "Motor.h"
#include "Ultrasound.h"

Sensor s;
Motor m;
Ultrasound u;

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

      /**
       *  S returns current sensor values of the joints.
       */
      if(code == 's') {

        Serial.println(s.getSensorValues());
      
      }
      /** 
       *  U: This option followed by 5 digits 
       *  first: motor number
       *  second: direction
       *  third -fifth: 3 digi angle ex: 050 = 50 degrees
       *  
       */
      else if(code == 'm') {
        
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
      /**
       *    U option followed by eithe 1,2,3 returns the ultrasound
       *    reading. 1 = front, 2 = left, 3 = right. 
       *    
       *    Return Type: String
       */
      else if(code == 'u') {
        //Serial.println("Getting ultrasound distances");
        int uval = Serial.parseInt();
        
        long dist = 0;
        if(uval == 1) {
          // Front
          dist = u.getDistance(0);
          Serial.println(String(dist));
        } else if(uval == 2) {
          // Left
          Serial.println(String(u.getDistance(1)));
        } else if(uval == 3) {
          // Right
          Serial.println(String(u.getDistance(2)));
        }
      }
    }
  }
}
