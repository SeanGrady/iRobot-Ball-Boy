int triggerPin1 = 3;
int echoPin1 = 4;
int triggerPin2 = 5;
int echoPin2 = 6;
int triggerPin3 = 7;
int echoPin3 = 8;
int triggerPin4 = 9;
int echoPin4 = 10;

long getDistance(int triggerPin, int echoPin) {
  long distance = 0;
  long duration = 0;

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration/58.2;

  return distance;
}

void setup() {
  pinMode(triggerPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(triggerPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(triggerPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()) {
    char in = Serial.read();
    if(in == 'd') {
      long distanceFront = getDistance(triggerPin1, echoPin1);
      long distanceLeft = getDistance(triggerPin2, echoPin2);
      long distanceBack = getDistance(triggerPin3, echoPin3);
      long distanceRight = getDistance(triggerPin4, echoPin4);

      String result = String("");
      String distance = String(distanceFront);
      result.concat( " " + distance);
      distance = String(distanceLeft);
      result.concat(" " + distance);
      distance = String(distanceBack);
      result.concat(" "+distance);
      distance = String(distanceRight);
      result.concat(" "+distance+"\r\n");

      Serial.print(result);
    }
  }
}