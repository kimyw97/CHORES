// 4개 모터 핀 설정
const int stepPins[4] = {2, 3, 4, 12};  // 예: X-STEP(D2), Y-STEP(D3), Z-STEP(D4), E-STEP(D12)
const int dirPins[4]  = {5, 6, 7, 13};  // 예: X-DIR(D5), Y-DIR(D6), Z-DIR(D7), E-DIR(D13)
const int enPin = 8;                    // ENABLE 핀 (공통)

// 모터/스크류 스펙 설정
const int stepsPerRev = 200;            // 1바퀴 = 200 스텝 (Full Step)
const float leadScrew_mm_per_rev = 8.0;  // 리드스크류 이동량 (1회전당 8mm)

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
  pinMode(enPin, OUTPUT);

  digitalWrite(enPin, LOW); // 모터 활성화
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    if (cmd.length() >= 2) {
      char dir = cmd.charAt(0);           // 첫 글자 (U 또는 D)
      int distance_cm = cmd.substring(1).toInt(); // 숫자 부분 (거리)

      if (dir == 'U' || dir == 'D') {
        moveMotors(distance_cm, dir == 'U');
      }
    }
  }
}

void moveMotors(int distance_cm, bool upward) {
  float distance_mm = distance_cm * 10.0;
  int steps = (distance_mm / leadScrew_mm_per_rev) * stepsPerRev;

  // 방향 설정 (4개 모터 모두)
  for (int i = 0; i < 4; i++) {
    digitalWrite(dirPins[i], upward ? HIGH : LOW);
  }

  // 스텝 출력 (4개 모터 동시에)
  for (int i = 0; i < steps; i++) {
    for (int j = 0; j < 4; j++) {
      digitalWrite(stepPins[j], HIGH);
    }
    delayMicroseconds(500); // 속도 조절
    for (int j = 0; j < 4; j++) {
      digitalWrite(stepPins[j], LOW);
    }
    delayMicroseconds(500);
  }
}
