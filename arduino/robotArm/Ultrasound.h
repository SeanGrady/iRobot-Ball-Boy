#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include "Arduino.h"

class Ultrasound
{
  /* 
    0,1 -> front sensor
    2,3 -> left sensor
    4,5 -> right sensor
  */
  int triggerPin[3] = {0,2,4};
  int echoPin[3] = {1,3,5};
public:
	Ultrasound();
	long getDistance(int channel);
};

#endif
