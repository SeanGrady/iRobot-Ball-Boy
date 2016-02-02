triggerPin = 3;
echoPin = 4;

void setup() {
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  long distance = 0;
  long duration = 0;

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration/58.2;

  Serial.println("Distance : ");
  Serial.print(distance);

  delay(500);
}
