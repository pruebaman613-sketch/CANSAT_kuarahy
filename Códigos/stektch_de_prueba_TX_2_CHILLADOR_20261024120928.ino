const int CHICHARRA = 4;
float altitud = 0; 

void setup() {
  pinMode(CHICHARRA, OUTPUT);
}

void loop() {
  if (altitud < 5) {
    digitalWrite(CHICHARRA, HIGH);
  } else {
    digitalWrite(CHICHARRA, LOW);
  }

  delay(100);
}