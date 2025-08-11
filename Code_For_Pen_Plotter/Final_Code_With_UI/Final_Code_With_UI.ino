#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>

// ==== EEPROM SETTINGS ====
#define EEPROM_SIZE 128
#define EEPROM_ADDR_CORR 0 // Starting address for correction factors
#define EEPROM_ADDR_POS (EEPROM_ADDR_CORR + 4 * sizeof(float)) // store emergency stop pulsesA, pulsesB

// ==== Pin Definitions ====
#define MOTOR_A_IN1 32
#define MOTOR_A_IN2 14
#define MOTOR_B_IN1 27
#define MOTOR_B_IN2 12

#define LIMIT_X_PIN 18
#define LIMIT_Y_PIN 19
#define ENCODER_A_PIN 34
#define ENCODER_B_PIN 35

#define EMERGENCY_STOP_PIN 33

#define MAX_PWM 255
#define HOMING_PWM 150
#define LOOP_INTERVAL 20 // ms PID loop
#define SETTLE_DELAY 150 // ms pause at corners

// === Motion parameters ===
// The MM_PER_PULSE value has been calibrated using the provided data:
// 46mm divided by 30 pulses = 1.5333mm per pulse.
const float MM_PER_PULSE = 1.5333;

// === Feedforward scaling ===
// Calibrated with user data: 30 pulses (target) / 31 pulses (actual) = 0.96774
float dutyScaleA = 0.96774;
float dutyScaleB = 0.96774;

// === PID Gains ===
float Kp = 5.0; // Increased from 4.0 for a stronger, faster response to errors
float Ki = 0.5; // Increased from 0.3 to better correct for steady-state errors
float Kd = 0.1; // Increased from 0.05 to help dampen overshoot from the higher Kp

// === Directional correction factors ===
float CORR_RIGHT = 1.0;
float CORR_DOWN = 1.0;
float CORR_LEFT = 1.0;
float CORR_UP = 1.0;

// === Encoder Tracking ===
volatile long pulsesA = 0;
volatile long pulsesB = 0;
volatile int dirA = 0;
volatile int dirB = 0;

// Saved position before emergency stop
long savedPulsesA = 0;
long savedPulsesB = 0;

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
    int pulses;
    int speedPWM;
};
MoveCmd moveQueue[MOVE_QUEUE_SIZE];
int queueStart = 0;
int queueEnd = 0;

bool isMoving = false;
unsigned long settleStartTime = 0;
bool isSettling = false;

bool emergencyStopActive = false;

// === WiFi & Web Server Configuration ===
// *** Changed to Access Point mode ***
const char* ssid = "ESP32";
const char* password = "12345";

WebServer server(80);

// ==== Function prototypes ====
void driveMotor(int motorID, int pwm);
void stopMotors();
void home();
void moveCoreXY_Optimized(float mmX, float mmY, int speedPWM);
void moveCoreXY_Pulses(int pulsesX, int pulsesY, int speedPWM);
void saveCorrectionFactors();
void loadCorrectionFactors();
void calibrate();
void enqueueMove(String dir, float mm, int speedPWM);
void enqueueMovePulses(String dir, int pulses, int speedPWM);
bool dequeueMove(MoveCmd &cmd);
void saveEmergencyPosition(long pulsesA_val, long pulsesB_val);
void loadEmergencyPosition(long &pulsesA_val, long &pulsesB_val);
void handleEmergencyStop();
void nicolas(int sidePulses); // New function prototype to accept pulse count

// ==== Web Server Handlers ====
void handleRoot();
void handleCommand();

// ==== Movement Queue Functions ====
void enqueueMove(String dir, float mm, int speedPWM) {
    int nextEnd = (queueEnd + 1) % MOVE_QUEUE_SIZE;
    if (nextEnd != queueStart) {
        moveQueue[queueEnd].dir = dir;
        moveQueue[queueEnd].pulses = round(mm / MM_PER_PULSE); // Convert mm to pulses
        moveQueue[queueEnd].speedPWM = speedPWM;
        queueEnd = nextEnd;
    }
}

void enqueueMovePulses(String dir, int pulses, int speedPWM) {
    int nextEnd = (queueEnd + 1) % MOVE_QUEUE_SIZE;
    if (nextEnd != queueStart) {
        moveQueue[queueEnd].dir = dir;
        moveQueue[queueEnd].pulses = pulses;
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

    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP); // Emergency stop button, active LOW

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onPulseA, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onPulseB, FALLING);

    loadCorrectionFactors();

    // Load saved emergency position, if any
    loadEmergencyPosition(savedPulsesA, savedPulsesB);

    // === WiFi & Web Server Setup ===
    // *** Changed to Access Point mode for direct connection from phone/PC ***
    Serial.printf("Creating Access Point: %s\n", ssid);
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP Address: ");
    Serial.println(IP);
    
    server.on("/", handleRoot);
    server.on("/command", handleCommand);
    server.begin();

    Serial.println("CoreXY ready. Use the web interface to control it.");
}

// ==== Main loop ====
void loop() {
    server.handleClient(); // Handle incoming web requests
    static MoveCmd currentMove;

    // Check emergency stop button state (active LOW)
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW && !emergencyStopActive) {
        emergencyStopActive = true;
        handleEmergencyStop();
    }

    if (!isMoving && !isSettling && !emergencyStopActive) {
        if (dequeueMove(currentMove)) {
            int pulsesX = 0, pulsesY = 0;
            int pulses = currentMove.pulses;

            if (currentMove.dir == "left") pulsesX = -pulses;
            else if (currentMove.dir == "right") pulsesX = pulses;
            else if (currentMove.dir == "up") pulsesY = pulses;
            else if (currentMove.dir == "down") pulsesY = -pulses;
            else if (currentMove.dir == "diag_ur") { // up-right diagonal
                pulsesX = pulses;
                pulsesY = pulses;
            }
            else if (currentMove.dir == "diag_ul") { // up-left diagonal
                pulsesX = -pulses;
                pulsesY = pulses;
            }
            else if (currentMove.dir == "diag_dr") { // down-right diagonal
                pulsesX = pulses;
                pulsesY = -pulses;
            }
            else if (currentMove.dir == "diag_dl") { // down-left diagonal
                pulsesX = -pulses;
                pulsesY = -pulses;
            }

            moveCoreXY_Pulses(pulsesX, pulsesY, currentMove.speedPWM);
            isMoving = true;
        }
    }

    if (isMoving) {
        // Movement just finished (since moveCoreXY functions are blocking)
        isMoving = false;
        isSettling = true;
        settleStartTime = millis();
        stopMotors();
    }

    if (isSettling && millis() - settleStartTime >= SETTLE_DELAY) {
        isSettling = false;
    }
}

// ==== Emergency Stop Handler ====
void handleEmergencyStop() {
    Serial.println("!!! Emergency stop activated !!!");

    // Save current position to EEPROM
    saveEmergencyPosition(pulsesA, pulsesB);
    savedPulsesA = pulsesA;
    savedPulsesB = pulsesB;

    // Stop all motors
    stopMotors();

    // Clear motion queue
    queueStart = queueEnd = 0;

    // Home the system
    home();

    Serial.println("System homed after emergency stop.");
    Serial.println("Type 'restart' to return to last position.");

    // Wait until emergency stop button released (debounce)
    while (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
        delay(50);
    }
    emergencyStopActive = false;
}

// ==== EEPROM Functions ====
void saveCorrectionFactors() {
    EEPROM.put(EEPROM_ADDR_CORR, CORR_RIGHT);
    EEPROM.put(EEPROM_ADDR_CORR + sizeof(float), CORR_DOWN);
    EEPROM.put(EEPROM_ADDR_CORR + 2 * sizeof(float), CORR_LEFT);
    EEPROM.put(EEPROM_ADDR_CORR + 3 * sizeof(float), CORR_UP);
    EEPROM.commit();
}

void loadCorrectionFactors() {
    EEPROM.get(EEPROM_ADDR_CORR, CORR_RIGHT);
    EEPROM.get(EEPROM_ADDR_CORR + sizeof(float), CORR_DOWN);
    EEPROM.get(EEPROM_ADDR_CORR + 2 * sizeof(float), CORR_LEFT);
    EEPROM.get(EEPROM_ADDR_CORR + 3 * sizeof(float), CORR_UP);

    if (isnan(CORR_RIGHT) || isnan(CORR_DOWN) || isnan(CORR_LEFT) || isnan(CORR_UP)) {
        CORR_RIGHT = CORR_DOWN = CORR_LEFT = CORR_UP = 1.0;
    }
    Serial.printf("Loaded Corrections: R=%.3f D=%.3f L=%.3f U=%.3f\n", CORR_RIGHT, CORR_DOWN, CORR_LEFT, CORR_UP);
}

// Save emergency stop position pulses to EEPROM
void saveEmergencyPosition(long pulsesA_val, long pulsesB_val) {
    EEPROM.put(EEPROM_ADDR_POS, pulsesA_val);
    EEPROM.put(EEPROM_ADDR_POS + sizeof(long), pulsesB_val);
    EEPROM.commit();
    Serial.printf("Saved emergency position: pulsesA=%ld, pulsesB=%ld\n", pulsesA_val, pulsesB_val);
}

// Load emergency stop position pulses from EEPROM
void loadEmergencyPosition(long &pulsesA_val, long &pulsesB_val) {
    EEPROM.get(EEPROM_ADDR_POS, pulsesA_val);
    EEPROM.get(EEPROM_ADDR_POS + sizeof(long), pulsesB_val);
    // If EEPROM values are invalid (e.g. large), reset to zero
    if (pulsesA_val > 1000000 || pulsesA_val < -1000000) pulsesA_val = 0;
    if (pulsesB_val > 1000000 || pulsesB_val < -1000000) pulsesB_val = 0;
    Serial.printf("Loaded emergency position: pulsesA=%ld, pulsesB=%ld\n", pulsesA_val, pulsesB_val);
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
void moveCoreXY_Optimized(float mmX, float mmY, int speedPWM) {
    int targetPulsesX = round(mmX / MM_PER_PULSE);
    int targetPulsesY = round(mmY / MM_PER_PULSE);

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

            Serial.printf("ErrA: %d, PWM_A: %.1f | ErrB: %d, PWM_B: %.1f\n", errorA, pwmA, errorB, pwmB);
        }
    }
    stopMotors();
}

void moveCoreXY_Pulses(int pulsesX, int pulsesY, int speedPWM) {
    int targetA = pulsesX + pulsesY;
    int targetB = pulsesX - pulsesY;

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

            bool doneA = (dirSignA > 0) ? (currA >= targetA) : (currA <= targetA);
            bool doneB = (dirSignB > 0) ? (currB >= targetB) : (currB <= targetB);

            if (doneA && doneB) break;

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

            pwmA = constrain(pwmA, -speedPWM, speedPWM);
            pwmB = constrain(pwmB, -speedPWM, speedPWM);

            driveMotor(0, pwmA);
            driveMotor(1, pwmB);

            Serial.printf("ErrA: %d, PWM_A: %.1f | ErrB: %d, PWM_B: %.1f\n", errorA, pwmA, errorB, pwmB);
        }
    }
    stopMotors();
}

// ==== Homing routine ====
void home() {
    Serial.println("Homing started...");
    // Drive motors slowly towards limit switches until triggered

    // Drive motor A negative until LIMIT_X_PIN LOW
    driveMotor(0, -HOMING_PWM);
    while (digitalRead(LIMIT_X_PIN) == HIGH) {
        delay(10);
    }
    stopMotors();

    // Drive motor B negative until LIMIT_Y_PIN LOW
    driveMotor(1, -HOMING_PWM);
    while (digitalRead(LIMIT_Y_PIN) == HIGH) {
        delay(10);
    }
    stopMotors();

    // Reset pulses to zero at home
    noInterrupts();
    pulsesA = 0;
    pulsesB = 0;
    interrupts();

    Serial.println("Homing done. Position reset to zero.");
}

// ==== Calibration routine (stub) ====
void calibrate() {
    Serial.println("Calibration routine running...");
    // Insert calibration steps if needed
    Serial.println("Calibration complete.");
}

// ==== Nicolas Function ====
void nicolas(int sidePulses) {
    Serial.println("Drawing nicolas pattern with pulse counts...");

    // A to E points of the square and roof based on pulse counts:
    // Assuming a coordinate system where A = (0, 0)
    // B = (0, sidePulses)
    // D = (sidePulses, sidePulses)
    // E = (sidePulses, 0)
    // C = (sidePulses/2, sidePulses + sidePulses/2)

    int straightSpeed = MAX_PWM * 0.6;
    int diagonalSpeed = MAX_PWM * 0.8; // Increased speed for diagonals

    // 1. A to B: Move from point A(0,0) to B(0, sidePulses)
    // This is a vertical move of 'sidePulses' pulses.
    Serial.println("Step 1: Moving from A to B (Up)");
    moveCoreXY_Pulses(0, sidePulses, straightSpeed);
    delay(SETTLE_DELAY);

    // 2. B to C: The first part of the roof.
    // Move from B(0, sidePulses) to C(sidePulses/2, sidePulses + sidePulses/2).
    // We use floating point math for precision on the roof diagonals to prevent
    // truncation errors if `sidePulses` is an odd number.
    float halfSideFloat = (float)sidePulses / 2.0;
    int roofDeltaX = round(halfSideFloat);
    int roofDeltaY = round(halfSideFloat);
    Serial.printf("Step 2: Moving from B to C (diag up-right) with deltaX=%d, deltaY=%d\n", roofDeltaX, roofDeltaY);
    moveCoreXY_Pulses(roofDeltaX, roofDeltaY, diagonalSpeed);
    delay(SETTLE_DELAY);

    // 3. C to D: The second part of the roof.
    // Move from C(sidePulses/2, sidePulses + sidePulses/2) to D(sidePulses, sidePulses).
    Serial.printf("Step 3: Moving from C to D (diag down-right) with deltaX=%d, deltaY=%d\n", roofDeltaX, -roofDeltaY);
    moveCoreXY_Pulses(roofDeltaX, -roofDeltaY, diagonalSpeed);
    delay(SETTLE_DELAY);

    // 4. D to E: Move from point D(sidePulses, sidePulses) to E(sidePulses, 0).
    // This is a vertical move of 'sidePulses' pulses in the negative direction.
    Serial.println("Step 4: Moving from D to E (down)");
    moveCoreXY_Pulses(0, -sidePulses, straightSpeed);
    delay(SETTLE_DELAY);

    // 5. E to B: The diagonal line from the bottom-right to top-left.
    // Move from E(sidePulses, 0) to B(0, sidePulses).
    // This requires a change of deltaX = -sidePulses and deltaY = +sidePulses.
    Serial.printf("Step 5: Moving from E to B (diag up-left) with deltaX=%d, deltaY=%d\n", -sidePulses, sidePulses);
    moveCoreXY_Pulses(-sidePulses, sidePulses, diagonalSpeed);
    delay(SETTLE_DELAY);

    // 6. B to D: The horizontal line across the top of the square.
    // Move from B(0, sidePulses) to D(sidePulses, sidePulses).
    // This is a horizontal move of 'sidePulses' pulses.
    Serial.println("Step 6: Moving from B to D (right)");
    moveCoreXY_Pulses(sidePulses, 0, straightSpeed);
    delay(SETTLE_DELAY);

    // 7. D to A: The diagonal line from the top-right to bottom-left.
    // Move from D(sidePulses, sidePulses) to A(0, 0).
    // This requires a change of deltaX = -sidePulses and deltaY = -sidePulses.
    Serial.printf("Step 7: Moving from D to A (diag down-left) with deltaX=%d, deltaY=%d\n", -sidePulses, -sidePulses);
    moveCoreXY_Pulses(-sidePulses, -sidePulses, diagonalSpeed);
    delay(SETTLE_DELAY);

    // 8. A to E: The horizontal line across the bottom of the square.
    // Move from A(0,0) to E(sidePulses, 0).
    // This is a horizontal move of 'sidePulses' pulses.
    Serial.println("Step 8: Moving from A to E (right)");
    moveCoreXY_Pulses(sidePulses, 0, straightSpeed);
    delay(SETTLE_DELAY);

    Serial.println("Nicolas pattern complete.");
}

// ==== Web Server Handler Implementations ====

const char* HTML_CONTENT = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>CoreXY Plotter Control</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Roboto+Mono:wght@400;700&display=swap');

        :root {
            --primary-color: #3f51b5;
            --accent-color: #ff5722;
            --bg-color: #f5f5f5;
            --card-bg-color: #ffffff;
            --shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }

        body {
            font-family: 'Roboto Mono', monospace;
            background-color: var(--bg-color);
            margin: 0;
            padding: 10px;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            box-sizing: border-box;
        }

        .main-container {
            width: 100%;
            max-width: 1000px;
            display: flex;
            flex-wrap: wrap;
            gap: 15px;
            justify-content: center;
        }

        .card {
            background-color: var(--card-bg-color);
            padding: 15px;
            border-radius: 8px;
            box-shadow: var(--shadow);
            flex: 1 1 250px; /* Adjust flex properties for responsive side-by-side */
            max-width: 400px;
        }

        .card h2 {
            margin-top: 0;
            color: var(--primary-color);
            border-bottom: 2px solid var(--primary-color);
            padding-bottom: 8px;
            font-size: 1.2rem;
        }

        .button-group {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            margin-bottom: 10px;
        }

        .button-group.single-row {
            grid-template-columns: 1fr;
        }

        .button {
            padding: 12px;
            border: none;
            border-radius: 5px;
            font-size: 14px;
            cursor: pointer;
            transition: background-color 0.3s, transform 0.1s;
            font-family: 'Roboto Mono', monospace;
            font-weight: bold;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
            text-align: center;
        }

        .button:hover {
            transform: translateY(-2px);
        }

        .button:active {
            transform: translateY(0);
        }

        .btn-primary {
            background-color: var(--primary-color);
            color: white;
        }

        .btn-primary:hover {
            background-color: #303f9f;
        }

        .btn-accent {
            background-color: var(--accent-color);
            color: white;
        }

        .btn-accent:hover {
            background-color: #e64a19;
        }

        .btn-danger {
            background-color: #e53935;
            color: white;
        }

        .btn-danger:hover {
            background-color: #c62828;
        }

        .input-group {
            display: flex;
            gap: 8px;
            align-items: center;
            margin-bottom: 8px;
        }

        .input-group input {
            flex-grow: 1;
            padding: 8px;
            border: 1px solid #ccc;
            border-radius: 5px;
            font-family: 'Roboto Mono', monospace;
        }

        .status-log {
            background-color: #f9f9f9;
            border: 1px solid #ddd;
            padding: 10px;
            height: 120px;
            overflow-y: auto;
            border-radius: 5px;
            white-space: pre-wrap;
            font-size: 12px;
        }
    </style>
</head>
<body>
    <div class="main-container">
        <div class="card">
            <h2>System Control</h2>
            <div class="button-group">
                <button class="button btn-primary" onclick="sendCommand('home')">Home</button>
                <button class="button btn-danger" onclick="sendCommand('emergency')">Emergency Stop</button>
            </div>
            <div class="button-group single-row">
                <button class="button btn-primary" onclick="sendCommand('stop')">Stop All</button>
            </div>
        </div>

        <div class="card">
            <h2>Jog Controls</h2>
            <div class="input-group">
                <input type="number" id="jog-mm" value="10" placeholder="Distance in mm">
                <button class="button btn-primary" onclick="sendJogCommand('up')">Up</button>
                <button class="button btn-primary" onclick="sendJogCommand('down')">Down</button>
            </div>
            <div class="input-group">
                <input type="number" id="jog-mm-horiz" value="10" placeholder="Distance in mm">
                <button class="button btn-primary" onclick="sendJogCommand('left')">Left</button>
                <button class="button btn-primary" onclick="sendJogCommand('right')">Right</button>
            </div>
        </div>

        <div class="card">
            <h2>Drawing Patterns</h2>
            <div class="input-group">
                <input type="number" id="square-input" value="100" placeholder="Side Length (mm)">
                <button class="button btn-accent" onclick="sendPatternCommand('square')">Draw Square</button>
            </div>
            <div class="input-group">
                <input type="number" id="squarepulses-input" value="100" placeholder="Side Length (pulses)">
                <button class="button btn-accent" onclick="sendPatternCommand('squarepulses')">Draw Square (Pulses)</button>
            </div>
            <div class="input-group">
                <input type="number" id="nicolas-input" value="100" placeholder="Base Length (pulses)">
                <button class="button btn-accent" onclick="sendPatternCommand('nicolas')">Draw House Pattern</button>
            </div>
        </div>

        <div class="card">
            <h2>Status Log</h2>
            <pre id="log" class="status-log">Ready...</pre>
        </div>
    </div>

    <script>
        const logElement = document.getElementById('log');
        const jogMmInput = document.getElementById('jog-mm');
        const jogMmHorizInput = document.getElementById('jog-mm-horiz');

        function sendCommand(cmd) {
            log(`Sending command: ${cmd}`);
            fetch(`/command?cmd=${cmd}`)
                .then(response => response.text())
                .then(data => log(`Response: ${data}`))
                .catch(error => log(`Error: ${error}`));
        }

        function sendJogCommand(dir) {
            let val;
            if (dir === 'up' || dir === 'down') {
                val = jogMmInput.value;
            } else {
                val = jogMmHorizInput.value;
            }
            if (!val) {
                log('Please enter a value for the jog distance.');
                return;
            }
            log(`Sending jog command: ${dir} ${val}mm`);
            fetch(`/command?cmd=move&dir=${dir}&val=${val}`)
                .then(response => response.text())
                .then(data => log(`Response: ${data}`))
                .catch(error => log(`Error: ${error}`));
        }

        function sendPatternCommand(pattern) {
            let inputId = pattern + '-input';
            let val = document.getElementById(inputId).value;
            if (!val) {
                log('Please enter a value for the pattern.');
                return;
            }
            log(`Sending pattern command: ${pattern} with value ${val}`);
            fetch(`/command?cmd=${pattern}&val=${val}`)
                .then(response => response.text())
                .then(data => log(`Response: ${data}`))
                .catch(error => log(`Error: ${error}`));
        }

        function log(message) {
            logElement.innerHTML += '> ' + message + '\n';
            logElement.scrollTop = logElement.scrollHeight;
        }
    </script>
</body>
</html>
)rawliteral";

void handleRoot() {
    server.send(200, "text/html", HTML_CONTENT);
}

void handleCommand() {
    String cmd = server.arg("cmd");
    String val = server.arg("val");
    String dir = server.arg("dir");

    Serial.printf("Received command: %s with value %s and direction %s\n", cmd.c_str(), val.c_str(), dir.c_str());

    if (cmd == "home") {
        home();
    } else if (cmd == "emergency") {
        handleEmergencyStop();
    } else if (cmd == "stop") {
        stopMotors();
        queueStart = queueEnd = 0;
    } else if (cmd == "move") {
        float mm = val.toFloat();
        enqueueMove(dir, mm, MAX_PWM * 0.6);
    } else if (cmd == "square") {
        float mm = val.toFloat();
        enqueueMove("right", mm, MAX_PWM * 0.6);
        enqueueMove("down", mm, MAX_PWM * 0.6);
        enqueueMove("left", mm, MAX_PWM * 0.6);
        enqueueMove("up", mm, MAX_PWM * 0.6);
    } else if (cmd == "squarepulses") {
        int pulses = val.toInt();
        enqueueMovePulses("right", pulses, MAX_PWM * 0.6);
        enqueueMovePulses("down", pulses, MAX_PWM * 0.6);
        enqueueMovePulses("left", pulses, MAX_PWM * 0.6);
        enqueueMovePulses("up", pulses, MAX_PWM * 0.6);
    } else if (cmd == "nicolas") {
        int pulses = val.toInt();
        nicolas(pulses);
    }
    server.send(200, "text/plain", "OK");
}
