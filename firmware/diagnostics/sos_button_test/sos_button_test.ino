#define SOS_BUTTON_PIN 5

const unsigned long DEBOUNCE_TIME = 50;

int lastRawState;
int stableState;
unsigned long lastChangeTime = 0;

void setup() {
  Serial.begin(115200);

  // SOS 버튼 COM/NO 접점 사용
  // GPIO5 ↔ 버튼 ↔ GND
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);

  delay(100);

  lastRawState = digitalRead(SOS_BUTTON_PIN);
  stableState = lastRawState;

  Serial.println("=== SOS 버튼 테스트 ===");

  if (stableState == LOW) {
    Serial.println("SOS 눌림");
  } else {
    Serial.println("SOS 대기");
  }
}

void loop() {
  int rawState = digitalRead(SOS_BUTTON_PIN);

  if (rawState != lastRawState) {
    lastRawState = rawState;
    lastChangeTime = millis();
  }

  if (millis() - lastChangeTime >= DEBOUNCE_TIME) {
    if (stableState != rawState) {
      stableState = rawState;

      if (stableState == LOW) {
        Serial.println("========== SOS 눌림 ==========");
      } else {
        Serial.println("SOS 버튼 해제");
      }
    }
  }
}
