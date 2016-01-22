#include "Arduino.h"
#include "Sensor.h"

Sensor::Sensor() {
	// Serial.println("Sensor object initialized");
}

int Sensor::getSensor1Value() {
	return analogRead(sensor1);
}

int Sensor::getSensor2Value() {
	return analogRead(sensor2);
}

int Sensor::getSensor3Value() {
	return analogRead(sensor3);
}

int Sensor::getSensor4Value() {
	return analogRead(sensor4);
}

int Sensor::getSensor5Value() {
	return analogRead(sensor5);
}

String Sensor::getSensorValues() {
	String result = "";
	result += String(getSensor1Value());
    result += " ";
    result += String(getSensor2Value());
    result += " ";
    result += String(getSensor3Value());
    result += " ";
    result += String(getSensor4Value());
    result += " ";
    result += String(getSensor5Value());
    return result;
}