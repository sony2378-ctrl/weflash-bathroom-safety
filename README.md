# WEFLASH

ESP32-S3 두 대와 C1001·LD2410C 밀리미터파 센서를 이용해 개인공간의 재실,
움직임, 낙상 및 장시간 무반응을 감지합니다. 위험상황이 확인되면 ESP-NOW로
외부 제어부에 전송해 BF395 경광등·사이렌을 작동하고, 휴대폰 웹 화면에서
상태 확인과 경보 해제를 할 수 있습니다.

## 최종 소스코드

- [천장 센서부 최종 코드](firmware/final/ceiling_final_v35/ceiling_final_v35.ino)
- [외부 제어부 최종 코드](firmware/final/external_final_v35/external_final_v35.ino)
- [컴파일·업로드 및 시험 방법](firmware/final/README.md)
- [부품별 진단 코드](firmware/diagnostics/README.md)

실기 확인이 끝난 조합은 **천장 센서부 3.5.3 + 외부 제어부 3.5.0**입니다.
두 장치는 패킷 v2, ESP-NOW 채널 6과 동일한 HMAC 키를 사용합니다.

## 저장소 구조

```text
firmware/
├─ final/
│  ├─ ceiling_final_v35/       천장 센서부 최종 코드
│  └─ external_final_v35/      외부 제어부 최종 코드
└─ diagnostics/                센서·입력·릴레이·통신 시험 코드

third_party/
└─ DFRobot_HumanDetection/     C1001 통신 확인 기능을 추가한 라이브러리
```

## 하드웨어 연결

| 장치 | 연결 |
|---|---|
| C1001 | TX→GPIO4, RX→GPIO5, 5V/GND |
| LD2410C | TX→GPIO16, RX→GPIO17, 5V/GND |
| 문센서 | GPIO4/GND (`INPUT_PULLUP`) |
| SOS 버튼 | GPIO5/GND (`INPUT_PULLUP`) |
| 릴레이 CH1 | IN→GPIO17, VCC→5V, GND→GND |
| BF395 | 12V 전원을 릴레이 COM/NO 접점으로 스위칭 |

천장부의 두 센서는 5V/GND 공통 전원 레일을 사용합니다. BF395는 12V 제품이므로
DC-DC의 5V 출력에 연결하면 안 됩니다.

## 개발 환경

- Arduino IDE 2.x 또는 Arduino CLI 1.5.1
- ESP32 Arduino Core 3.3.11
- 보드: `ESP32S3 Dev Module`
- C1001 라이브러리: `third_party/DFRobot_HumanDetection`의 수정본

## 업로드

1. 각 최종 코드 폴더의 `project_config.example.h`를 `project_config.h`로 복사합니다.
2. 두 설정 파일에 동일한 32바이트 HMAC 키를 입력합니다.
3. 외부 제어부의 Wi-Fi 비밀번호를 8자 이상으로 설정합니다.
4. 외부 제어부 코드를 먼저 업로드한 뒤 천장 센서부 코드를 업로드합니다.
5. 빈 공간에서 전원을 켜고 C1001 초기화가 끝날 때까지 약 30~60초 기다립니다.
6. 휴대폰을 `WEFLASH-ALARM`에 연결하고 `http://192.168.4.1`을 엽니다.

상세 시험 순서와 C1001 설정값은 [최종 펌웨어 안내](firmware/final/README.md)를
참고하십시오. 버전별 수정 내용은 [변경 기록](CHANGELOG.md)에 정리되어 있습니다.

## 참고

- 실제 Wi-Fi 비밀번호와 HMAC 키가 포함된 `project_config.h`는 저장소에 포함하지 않습니다.
- 공개된 예시 비밀번호와 키를 실제 설치에 그대로 사용하지 마십시오.
- 센서 기준값은 설치 높이와 공간 구조에 맞춰 현장에서 조정해야 합니다.
- 본 작품은 대회 시연용 시제품이며 의료기기나 공식 비상경보 설비를 대체하지 않습니다.
- [외부 라이브러리 및 라이선스](THIRD_PARTY_NOTICES.md) · [보안 안내](SECURITY.md)
