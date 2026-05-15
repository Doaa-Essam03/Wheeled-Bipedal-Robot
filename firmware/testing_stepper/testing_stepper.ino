#define stepPin 3
#define dirPin 4
void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {

  digitalWrite(dirPin, HIGH);

  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }

  delay(1000);

  digitalWrite(dirPin, LOW);

  for(int x = 0; x < 200; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);

    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);
  }

  delay(1000);
}