#define LD_RX 16   // ESP가 받는 핀 <- LD TX
#define LD_TX 17   // ESP가 보내는 핀 -> LD RX

HardwareSerial LD(1);

// 설정 모드 진입
const uint8_t ENTER_CONFIG[] = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x04, 0x00,
  0xFF, 0x00,
  0x01, 0x00,
  0x04, 0x03, 0x02, 0x01
};

// 설정 모드 종료
const uint8_t EXIT_CONFIG[] = {
  0xFD, 0xFC, 0xFB, 0xFA,
  0x02, 0x00,
  0xFE, 0x00,
  0x04, 0x03, 0x02, 0x01
};

void printIncoming(unsigned long ms) {
  unsigned long start = millis();

  while (millis() - start < ms) {
    while (LD.available()) {
      uint8_t b = LD.read();

      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  LD.begin(256000, SERIAL_8N1, LD_RX, LD_TX);

  Serial.println("=== LD2410C GPIO16/17 TEST ===");

  // 기존 자동 출력 데이터 비우기
  while (LD.available()) LD.read();

  Serial.println();
  Serial.println("[1] GPIO17 -> LD : ENTER CONFIG 전송");

  LD.write(ENTER_CONFIG, sizeof(ENTER_CONFIG));
  LD.flush();

  Serial.println("[2] GPIO16 <- LD : ACK 확인");
  printIncoming(1000);

  Serial.println();
  Serial.println("[3] EXIT CONFIG 전송");

  LD.write(EXIT_CONFIG, sizeof(EXIT_CONFIG));
  LD.flush();

  Serial.println("[4] 종료 ACK 확인");
  printIncoming(1000);

  Serial.println();
  Serial.println("TEST END");
}

void loop() {
}
