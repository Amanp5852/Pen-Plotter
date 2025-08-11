#include <Arduino.h>
#include <EEPROM.h>

// ==== EEPROM SETTINGS ====
#define EEPROM_SIZE 64
#define EEPROM_ADDR_CORR 0  // Starting address for correction factors

// ==== Pin Definitions ====
#define MOTOR_A_IN1 32
#define MOTOR_A_IN2 14
#define MOTOR_B_IN1 27
#define MOTOR_B_IN2 12

#define LIMIT_X_PIN 18
#define LIMIT_Y_PIN 19
#define ENCODER_A_PIN 34
#define ENCODER_B_PIN 35

#define MAX_PWM 255
#define HOMING_PWM 150
#define LOOP_INTERVAL 20 // ms PID loop
#define SETTLE_DELAY 150 // ms pause at corners

// === Motion parameters ===
const float MM_PER_PULSE = 0.34;

// === Feedforward scaling ===
float dutyScaleA = 0.93;
float dutyScaleB = 1.00;

// === PID Gains ===
float Kp = 4.0;
float Ki = 0.3;
float Kd = 0.05;  // Added small derivative gain for better control

// === Directional correction factors ===
float CORR_RIGHT = 1.0;
float CORR_DOWN  = 1.0;
float CORR_LEFT  = 1.0;
float CORR_UP    = 1.0;

// === Encoder Tracking ===
volatile long pulsesA = 0;
volatile long pulsesB = 0;
volatile int dirA = 0;
volatile int dirB = 0;

// === PID Variables ===
float integralA = 0, prevErrorA = 0;
float integralB = 0, prevErrorB = 0;

// ==== Interrupts ====
void IRAM_ATTR onPulseA() { pulsesA += dirA; }
void IRAM_ATTR onPulseB() { pulsesB += dirB; }

// ==== Movement Queue ====
#define MOVE_QUEUE_SIZE 10
struct MoveCmd {
  String dir;
  float mm;
  int speedPWM;
};
MoveCmd moveQueue[MOVE_QUEUE_SIZE];
int queueStart = 0;
int queueEnd = 0;

bool isMoving = false;
unsigned long settleStartTime = 0;
bool isSettling = false;

// ==== Function prototypes ====
void driveMotor(int motorID, int pwm);
void stopMotors();
int mmToPulses(float mm);
void home();
void moveCoreXY_Optimized(float mmX, float mmY, int speedPWM);
void saveCorrectionFactors();
void loadCorrectionFactors();
void calibrate();
void enqueueMove(String dir, float mm, int speedPWM);
bool dequeueMove(MoveCmd &cmd);

// ==== Movement Queue Functions ====
void enqueueMove(String dir, float mm, int speedPWM) {
  int nextEnd = (queueEnd + 1) % MOVE_QUEUE_SIZE;
  if (nextEnd != queueStart) {
    moveQueue[queueEnd].dir = dir;
    moveQueue[queueEnd].mm = mm;
    moveQueue[queueEnd].speedPWM = speedPWM;
    queueEnd = nextEnd;
  }
}

bool dequeueMove(MoveCmd &cmd) {
  if (queueStart == queueEnd) return false;
  cmd = moveQueue[queueStart];
  queueStart = (queueStart + 1) % MOVE_QUEUE_SIZE;
  return true;
}

// ==== Setup ====
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  pinMode(LIMIT_X_PIN, INPUT_PULLUP);
  pinMode(LIMIT_Y_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onPulseA, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onPulseB, FALLING);

  loadCorrectionFactors();

  Serial.println("CoreXY ready. Type 'help' for commands.");
}

// ==== Main loop ====
void loop() {
  static MoveCmd currentMove;

  if (!isMoving && !isSettling) {
    if (dequeueMove(currentMove)) {
      float correctedMM = currentMove.mm;
      if (currentMove.dir == "left") correctedMM *= CORR_LEFT;
      else if (currentMove.dir == "up") correctedMM *= CORR_UP;
      else if (currentMove.dir == "right") correctedMM *= CORR_RIGHT;
      else if (currentMove.dir == "down") correctedMM *= CORR_DOWN;

      moveCoreXY_Optimized(
        (currentMove.dir == "left") ? -correctedMM : (currentMove.dir == "right") ? correctedMM : 0,
        (currentMove.dir == "up") ? correctedMM : (currentMove.dir == "down") ? -correctedMM : 0,
        currentMove.speedPWM
      );
      isMoving = true;
    }
  }

  if (isMoving) {
    // Once moveCoreXY_Optimized completes, movement done.
    isMoving = false;
    isSettling = true;
    settleStartTime = millis();
    stopMotors();
  }

  if (isSettling && millis() - settleStartTime >= SETTLE_DELAY) {
    isSettling = false;
  }

  if (Serial.available() && !isMoving && !isSettling) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "home") {
      home();
    }
    else if (cmd.startsWith("left") || cmd.startsWith("right") ||
             cmd.startsWith("up") || cmd.startsWith("down")) {
      int space = cmd.indexOf(' ');
      if (space > 0) {
        String dir = cmd.substring(0, space);
        float mm = cmd.substring(space + 1).toFloat();
        enqueueMove(dir, mm, MAX_PWM * 0.6);
      }
    }
    else if (cmd.startsWith("square")) {
      int space = cmd.indexOf(' ');
      if (space > 0) {
        float side = cmd.substring(space + 1).toFloat();
        enqueueMove("right", side, MAX_PWM * 0.6);
        enqueueMove("down", side, MAX_PWM * 0.6);
        enqueueMove("left", side, MAX_PWM * 0.6);
        enqueueMove("up", side, MAX_PWM * 0.6);
      }
    }
    else if (cmd == "stop") {
      stopMotors();
      queueStart = queueEnd = 0;
    }
    else if (cmd == "calibrate") {
      calibrate();
    }
    else if (cmd == "help") {
      Serial.println("Commands:");
      Serial.println("home                - Home the machine");
      Serial.println("left X / right X    - Move X mm");
      Serial.println("up X / down X       - Move X mm");
      Serial.println("square X            - Draw square of side X mm (right > down > left > up)");
      Serial.println("calibrate           - Run calibration routine");
      Serial.println("stop                - Stop all motion and clear queue");
    }
  }
}

// ==== Motor Control ====
void driveMotor(int motorID, int pwm) {
  int pin1 = (motorID == 0) ? MOTOR_A_IN1 : MOTOR_B_IN1;
  int pin2 = (motorID == 0) ? MOTOR_A_IN2 : MOTOR_B_IN2;

  int out = constrain(abs(pwm), 0, MAX_PWM);
  if (pwm > 0) {
    if (motorID == 0) dirA = 1; else dirB = 1;
    analogWrite(pin1, out); analogWrite(pin2, 0);
  } else if (pwm < 0) {
    if (motorID == 0) dirA = -1; else dirB = -1;
    analogWrite(pin1, 0); analogWrite(pin2, out);
  } else {
    if (motorID == 0) dirA = 0; else dirB = 0;
    analogWrite(pin1, 0); analogWrite(pin2, 0);
  }
}

void stopMotors() {
  driveMotor(0, 0);
  driveMotor(1, 0);
}

// ==== Motion ====
int mmToPulses(float mm) {
  return round(mm / MM_PER_PULSE);
}

void moveCoreXY_Optimized(float mmX, float mmY, int speedPWM) {
  int targetPulsesX = mmToPulses(mmX);
  int targetPulsesY = mmToPulses(mmY);

  int targetA = targetPulsesX + targetPulsesY;
  int targetB = targetPulsesX - targetPulsesY;

  long startA = pulsesA;
  long startB = pulsesB;

  int dirSignA = (targetA >= 0) ? 1 : -1;
  int dirSignB = (targetB >= 0) ? 1 : -1;

  integralA = 0; integralB = 0;
  prevErrorA = 0; prevErrorB = 0;

  unsigned long lastUpdate = millis();

  while (true) {
    unsigned long now = millis();
    if (now - lastUpdate >= LOOP_INTERVAL) {
      lastUpdate = now;

      long currA = pulsesA - startA;
      long currB = pulsesB - startB;

      // Check if movement complete in both motors
      bool doneA = (dirSignA > 0) ? (currA >= targetA) : (currA <= targetA);
      bool doneB = (dirSignB > 0) ? (currB >= targetB) : (currB <= targetB);

      if (doneA && doneB) {
        break;
      }

      int errorA = targetA - currA;
      int errorB = targetB - currB;

      integralA += errorA;
      integralB += errorB;

      float derivA = errorA - prevErrorA;
      float derivB = errorB - prevErrorB;

      float pwmA = (Kp * errorA + Ki * integralA + Kd * derivA) * dutyScaleA;
      float pwmB = (Kp * errorB + Ki * integralB + Kd * derivB) * dutyScaleB;

      prevErrorA = errorA;
      prevErrorB = errorB;

      // Clamp PWM to speed limits
      pwmA = constrain(pwmA, -speedPWM, speedPWM);
      pwmB = constrain(pwmB, -speedPWM, speedPWM);

      driveMotor(0, pwmA);
      driveMotor(1, pwmB);

      // Debug output (can slow down, remove if needed)
      Serial.printf("ErrA: %d, PWM_A: %.1f | ErrB: %d, PWM_B: %.1f\n", errorA, pwmA, errorB, pwmB);
    }
  }
  stopMotors();
}

// ==== Homing ====
void home() {
  Serial.println("Starting homing...");
  while (digitalRead(LIMIT_X_PIN) == HIGH || digitalRead(LIMIT_Y_PIN) == HIGH) {
    int dx = 0, dy = 0;
    if (digitalRead(LIMIT_X_PIN) == HIGH) dx = -50;
    if (digitalRead(LIMIT_Y_PIN) == HIGH) dy = -50;

    int motorA = dx + dy;
    int motorB = dx - dy;

    driveMotor(0, motorA > 0 ? HOMING_PWM : (motorA < 0 ? -HOMING_PWM : 0));
    driveMotor(1, motorB > 0 ? HOMING_PWM : (motorB < 0 ? -HOMING_PWM : 0));
    delay(10);
  }
  stopMotors();
  pulsesA = 0; pulsesB = 0;
  Serial.println("Homed.");
}

// ==== EEPROM Functions ====
void saveCorrectionFactors() {
  EEPROM.put(EEPROM_ADDR_CORR, CORR_RIGHT);
  EEPROM.put(EEPROM_ADDR_CORR + sizeof(float), CORR_DOWN);
  EEPROM.put(EEPROM_ADDR_CORR + 2*sizeof(float), CORR_LEFT);
  EEPROM.put(EEPROM_ADDR_CORR + 3*sizeof(float), CORR_UP);
  EEPROM.commit();
}

void loadCorrectionFactors() {
  EEPROM.get(EEPROM_ADDR_CORR, CORR_RIGHT);
  EEPROM.get(EEPROM_ADDR_CORR + sizeof(float), CORR_DOWN);
  EEPROM.get(EEPROM_ADDR_CORR + 2*sizeof(float), CORR_LEFT);
  EEPROM.get(EEPROM_ADDR_CORR + 3*sizeof(float), CORR_UP);

  if (isnan(CORR_RIGHT) || isnan(CORR_DOWN) || isnan(CORR_LEFT) || isnan(CORR_UP)) {
    CORR_RIGHT = CORR_DOWN = CORR_LEFT = CORR_UP = 1.0;
  }
  Serial.printf("Loaded Corrections: R=%.3f D=%.3f L=%.3f U=%.3f\n", CORR_RIGHT, CORR_DOWN, CORR_LEFT, CORR_UP);
}

// ==== Calibration Routine ====
void calibrate() {
  Serial.println("Starting calibration: target = 50mm per move.");
  home();
  delay(500);

  float measured[4];
  String dirNames[4] = {"right", "down", "left", "up"};

  for (int i = 0; i < 4; i++) {
    enqueueMove(dirNames[i], 50, MAX_PWM * 0.6);
    while (isMoving || isSettling || queueStart != queueEnd) { loop(); }
    Serial.printf("Measure %s move (mm): ", dirNames[i].c_str());
    while (!Serial.available()) delay(10);
    measured[i] = Serial.readStringUntil('\n').toFloat();
  }

  CORR_RIGHT = 50.0 / measured[0];
  CORR_DOWN  = 50.0 / measured[1];
  CORR_LEFT  = 50.0 / measured[2];
  CORR_UP    = 50.0 / measured[3];

  saveCorrectionFactors();

  Serial.printf("New Corrections: R=%.3f D=%.3f L=%.3f U=%.3f\n", CORR_RIGHT, CORR_DOWN, CORR_LEFT, CORR_UP);
  Serial.println("Calibration complete.");
}
