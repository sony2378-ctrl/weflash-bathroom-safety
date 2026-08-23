#define DOOR_PIN 4

void setup() {
  Serial.begin(115200);
  pinMode(DOOR_PIN, INPUT_PULLUP);
}

void loop() {
  Serial.println(digitalRead(DOOR_PIN));
  delay(200);
}
