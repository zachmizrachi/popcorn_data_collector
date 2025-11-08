#include <Servo.h>

// === Stepper / A4988 Setup ===
const int dirPin = 2;
const int stepPin = 3;

// A4988 control pins
const int enablePin = 4; // connect to A4988 ENABLE (active LOW)
const int sleepPin  = 10; // optional: connect to A4988 SLEEP (active LOW). Set to -1 if unused.

// === Servo Setup ===
const int receiveServoPin = 5;
const int dumpServoPin = 6;

Servo receiveServo;
Servo dumpServo;

// === Servo Position Constants ===
const int RECEIVE_BASE_POS = 100;
const int RECEIVE_POS = 40;

const int DUMP_BASE_POS = 0;
const int DUMP_POS = 180;

// === DC Motor (L298) Setup ===
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// === Stepper Control States ===
bool isStepperRunning = false;   // overall cycle enabled/disabled
bool isDCMotorRunning = false;
bool stepperActivePhase = false; // true = ON phase (pulsing), false = OFF phase (disabled)

// === Timing ===
unsigned long lastStepTime = 0;      // micros() time of last half-toggle
unsigned long phaseStartTime = 0;    // millis() time when current phase started

// adjust these to your desired times
const unsigned long STEPPER_ON_TIME  = 1000;  // ms motor pulses for this long
const unsigned long STEPPER_OFF_TIME = 750;  // ms motor driver disabled for this long

// step pulse timing: microseconds per half-toggle (so full period = 2 * stepInterval)
const unsigned long stepInterval = 4000; // microseconds (4 ms per half pulse = 125 Hz toggles)
bool stepPinState = LOW;

// === Setup ===
void setup() {
  // Stepper basic pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW); // CCW default
  digitalWrite(stepPin, LOW); // safe idle

  // A4988 control pins
  pinMode(enablePin, OUTPUT);
  if (sleepPin >= 0) pinMode(sleepPin, OUTPUT);

  // Set A4988 initial state: disabled and awake (safe)
  digitalWrite(enablePin, HIGH); // disable outputs (ACTIVE LOW)
  if (sleepPin >= 0) digitalWrite(sleepPin, HIGH); // keep awake (active LOW)

  // Servo setup
  receiveServo.attach(receiveServoPin);
  dumpServo.attach(dumpServoPin);
  receiveServo.write(RECEIVE_BASE_POS);
  dumpServo.write(DUMP_BASE_POS);

  // DC motor setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  // Serial setup
  Serial.begin(9600);
  Serial.println("=== Command List ===");
  Serial.println("1 = Start Stepper ON/OFF Cycle");
  Serial.println("2 = Stop Stepper Immediately");
  Serial.println("3 = Move Receive Servo → RECEIVE_POS");
  Serial.println("4 = Move Receive Servo → BASE_POS");
  Serial.println("5 = Move Dump Servo → DUMP_POS");
  Serial.println("6 = Move Dump Servo → BASE_POS");
  Serial.println("7 = Run DC Motor");
  Serial.println("8 = Stop DC Motor");
  Serial.println("====================");
}

void loop() {
  handleSerial();
  update_stepper_cycle();  // non-blocking stepper cycle
}

// === Serial handler ===
void handleSerial() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '\n' || command == '\r') return;

    switch (command) {
      case '1': start_stepper_cycle(); break;
      case '2': stop_stepper_cycle();  break;
      case '3': move_receive_to_target(); break;
      case '4': move_receive_to_base();   break;
      case '5': move_dump_to_target();    break;
      case '6': move_dump_to_base();      break;
      case '7': run_dc_motor();           break;
      case '8': stop_dc_motor();          break;
      default:
        Serial.print("Invalid command: ");
        Serial.println(command);
    }
  }
}

// === Stepper cycle control ===
void start_stepper_cycle() {
  if (isStepperRunning) {
    Serial.println("Stepper cycle already running.");
    return;
  }
  // Enable driver and start ON phase
  isStepperRunning = true;
  stepperActivePhase = true; // start with ON phase
  phaseStartTime = millis();
  lastStepTime = micros();

  // enable A4988 (active LOW)
  digitalWrite(enablePin, LOW);

  // ensure SLEEP is awake if used
  if (sleepPin >= 0) digitalWrite(sleepPin, HIGH);

  Serial.println("Stepper ON/OFF cycle started (driver enabled).");
}

void stop_stepper_cycle() {
  if (!isStepperRunning) {
    Serial.println("Stepper already stopped.");
    return;
  }

  // Immediately stop pulsing and disable driver to remove coil current
  isStepperRunning = false;
  stepperActivePhase = false;
  digitalWrite(stepPin, LOW);    // no stray pulses
  digitalWrite(enablePin, HIGH); // disable outputs -> reduces current/heat

  Serial.println("Stepper cycle stopped immediately (driver disabled).");
}

void update_stepper_cycle() {
  if (!isStepperRunning) return;

  unsigned long nowMillis = millis();

  if (stepperActivePhase) {
    // ON phase: ensure driver enabled
    digitalWrite(enablePin, LOW); // enable driver (active LOW)
    if (sleepPin >= 0) digitalWrite(sleepPin, HIGH); // wake

    // if ON phase is over, switch to OFF
    if (nowMillis - phaseStartTime >= STEPPER_ON_TIME) {
      stepperActivePhase = false;
      phaseStartTime = nowMillis;

      // disable driver immediately to remove holding current
      digitalWrite(stepPin, LOW);
      digitalWrite(enablePin, HIGH); // disable outputs
      Serial.println("Stepper OFF phase (driver disabled).");
      return;
    }

    // Generate step pulses (non-blocking)
    unsigned long nowMicros = micros();
    if (nowMicros - lastStepTime >= stepInterval) {
      lastStepTime = nowMicros;
      stepPinState = !stepPinState;
      digitalWrite(stepPin, stepPinState);
    }

  } else {
    // OFF phase: driver should already be disabled
    if (nowMillis - phaseStartTime >= STEPPER_OFF_TIME) {
      // Start next ON phase
      stepperActivePhase = true;
      phaseStartTime = nowMillis;
      lastStepTime = micros();
      digitalWrite(enablePin, LOW); // re-enable outputs
      Serial.println("Stepper ON phase (driver enabled).");
    }
  }
}

// === Servo functions ===
void move_receive_to_target() {
  receiveServo.write(RECEIVE_POS);
  Serial.println("Receive Servo → RECEIVE_POS");
}
void move_receive_to_base() {
  receiveServo.write(RECEIVE_BASE_POS);
  Serial.println("Receive Servo → BASE_POS");
}
void move_dump_to_target() {
  dumpServo.write(DUMP_POS);
  Serial.println("Dump Servo → DUMP_POS");
}
void move_dump_to_base() {
  dumpServo.write(DUMP_BASE_POS);
  Serial.println("Dump Servo → BASE_POS");
}

// === DC Motor functions ===
void run_dc_motor() {
  if (isDCMotorRunning) {
    Serial.println("DC Motor already running.");
    return;
  }
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 255);
  isDCMotorRunning = true;
  Serial.println("DC Motor running forward at full speed...");
}
void stop_dc_motor() {
  if (!isDCMotorRunning) {
    Serial.println("DC Motor already stopped.");
    return;
  }
  analogWrite(enA, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  isDCMotorRunning = false;
  Serial.println("DC Motor stopped.");
}
