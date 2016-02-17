#ifndef SENSOR_H
#define SENSOR_H

class Sensor
{

	private:
		int sensor1 = A0;
		int sensor2 = A1;
		int sensor3 = A2;
		int sensor4 = A3;
		int sensor5 = A4;

	public:
		Sensor();
		int getSensor1Value();
		int getSensor2Value();
		int getSensor3Value();
		int getSensor4Value();
		int getSensor5Value();
		String getSensorValues();
	
};


#endif