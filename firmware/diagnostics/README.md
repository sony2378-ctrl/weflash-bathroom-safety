# 진단 코드 사용 안내

이 폴더는 부품 하나씩 확인하는 시험용이다. 실제 설치에는
`../final/`의 천장부·외부부 코드만 사용한다.

| 폴더 | 확인 대상 |
|---|---|
| `c1001_fall_test` | C1001 연결, 설정 저장, 낙상·정지 원시값 |
| `ld2410_uart_test` | LD2410C 기본 UART 출력 |
| `ld2410_bidirectional_test` | LD2410C 양방향 UART 연결 |
| `ceiling_dual_sensor_test` | C1001과 LD2410C를 동시에 연결한 상태 |
| `door_sensor_test` | 문센서 열림·닫힘 |
| `door_gpio_diagnostic` | 문센서 GPIO 전기 상태 |
| `sos_button_test` | SOS 버튼 입력 |
| `external_controller_final_test` | 문센서·SOS·릴레이 하드웨어 통합 시험 |
| `espnow_ceiling_sender_test` | ESP-NOW 진단 송신 |
| `espnow_external_receiver_test` | ESP-NOW 진단 수신 |

## 주의

- C1001 관련 두 진단 코드는 실기에서 저장이 확인된 낙상 확정 5초를 사용한다.
- C1001 `RANGE/움직임값`은 cm가 아니라 0~100 움직임 강도값이다.
- ESP-NOW 진단 송신·수신 코드는 서로 한 쌍인 구형 시험 패킷 v1이다.
  최종 패킷 v2와 호환되지 않으므로 한쪽에 최종 코드를 섞지 않는다.
- `external_controller_final_test`라는 이름의 스케치도 부품 시험용이다.
  모바일 웹과 최종 경보 로직은 `../final/external_final_v35`를 사용한다.
- 최종 코드와 C1001 진단 코드는 수정된
  `third_party/DFRobot_HumanDetection` 라이브러리의
  `getFallStateChecked()`를 사용한다. 공식 라이브러리만 설치하면 컴파일되지 않는다.
