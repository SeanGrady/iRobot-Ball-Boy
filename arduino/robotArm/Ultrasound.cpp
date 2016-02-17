#include "Arduino.h"
#include "Ultrasound.h"

Ultrasound::Ultrasound() {
  // Nothing to do on the constructor
  for(int i=0; i<3; ++i) {
    pinMode(triggerPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
  }
}

long Ultrasound::getDistance(int channel) {
  long distance = 0;
  long duration = 0;

  // Keep the pin low initially
  digitalWrite(triggerPin[channel], LOW);
  delayMicroseconds(10);

  // Strobe pulse
  digitalWrite(triggerPin[channel], HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin[channel], LOW);

  // Take the reading
  duration = pulseIn(echoPin[channel], HIGH);

  distance = duration/58.2;

  return distance;
}
