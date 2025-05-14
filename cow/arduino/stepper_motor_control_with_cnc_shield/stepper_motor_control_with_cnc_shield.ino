// 4 Motor pin setup
const int stepPins[4] = {2, 3, 4, 12};  // Example: X-STEP(D2), Y-STEP(D3), Z-STEP(D4), E-STEP(D12)
const int dirPins[4]  = {5, 6, 7, 13};  // Example: X-DIR(D5), Y-DIR(D6), Z-DIR(D7), E-DIR(D13)
const int enPin = 8;                    // ENABLE pin (shared)

// Motor/lead screw specifications
const int stepsPerRev = 200;            // 1 revolution = 200 steps (Full Step)
const float leadScrew_mm_per_rev = 8.0; // Lead screw travel per revolution

void setup() {
  Serial.begin(115200);
  Serial.println("Stepper motor controller initialized.");

  for (int i = 0; i < 4; i++) {
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
  }
  pinMode(enPin, OUTPUT);

  digitalWrite(enPin, LOW); // Enable motors
  Serial.println("Motors enabled.");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // remove possible carriage return or extra spaces

    Serial.print("Received command: ");
    Serial.println(cmd);

    if (cmd.length() >= 2) {
      char dir = cmd.charAt(0); // First character (U or D)
      int distance_cm = cmd.substring(1).toInt(); // Distance in cm

      if (dir == 'U' || dir == 'D') {
        Serial.print("Parsed direction: ");
        Serial.println(dir == 'U' ? "Upward" : "Downward");

        Serial.print("Parsed distance: ");
        Serial.print(distance_cm);
        Serial.println(" cm");

        moveMotors(distance_cm, dir == 'U');
      } else {
        Serial.println("Invalid direction. Use 'U' or 'D'.");
      }
    } else {
      Serial.println("Invalid command format.");
    }
  }
}

void moveMotors(int distance_cm, bool upward) {
  float distance_mm = distance_cm * 10.0;
  int steps = (distance_mm / leadScrew_mm_per_rev) * stepsPerRev;

  Serial.print("Calculated steps: ");
  Serial.println(steps);

  // Set direction for all motors
  for (int i = 0; i < 4; i++) {
    digitalWrite(dirPins[i], upward ? HIGH : LOW);
  }

  Serial.println("Starting motor movement...");

  // Step all motors simultaneously
  for (int i = 0; i < steps; i++) {
    for (int j = 0; j < 4; j++) {
      digitalWrite(stepPins[j], HIGH);
    }
    delayMicroseconds(500);
    for (int j = 0; j < 4; j++) {
      digitalWrite(stepPins[j], LOW);
    }
    delayMicroseconds(500);
  }

  Serial.println("Motor movement complete.");
}
