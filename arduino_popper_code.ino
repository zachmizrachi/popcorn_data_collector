#include <Servo.h>

// === Stepper / A4988 Setup ===
const int dirPin = 2;
const int stepPin = 3;

// A4988 control pins
const int enablePin = 4; // connect to A4988 ENABLE (active LOW)
const int sleepPin  = 10; // optional: connect to A4988 SLEEP (active LOW). Set to -1 if unused.

// === Servo Setup ===
const int armServoPin = 5;
const int dumpServoPin = 6;

Servo armServo;
Servo dumpServo;

// === Servo Position Constants ===
const int ARM_POP_POS = 100;
const int ARM_RECEIVE_POS = 40;
const int ARM_DUMP_POS = 70;  // mid-position between POP and RECEIVE

const int DUMP_BASE_POS = 0;
const int DUMP_POS = 180;

// === DC Motor (L298) Setup ===
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// === Stepper Control States ===
bool isStepperRunning = false;
bool isStepperContinuous = false;   // continuous stepper mode
bool isDCMotorRunning = false;
bool stepperActivePhase = false;

// === Servo State Tracking ===
enum ArmState { ARM_AT_POP, ARM_AT_RECEIVE, ARM_AT_DUMP, UNKNOWN };
ArmState armState = ARM_AT_RECEIVE;

// === Dump State Tracking ===
enum DumpState { DUMP_UP_POS, DUMP_DOWN_POS };
DumpState dumpState = DUMP_UP_POS;  // initial safe position

// === Timing ===
unsigned long lastStepTime = 0;
unsigned long phaseStartTime = 0;
const unsigned long STEPPER_ON_TIME  = 1000;
const unsigned long STEPPER_OFF_TIME = 750;
const unsigned long stepInterval = 4000;
bool stepPinState = LOW;

// === DC Motor Pulse Timing ===
bool dcPulseActive = false;
unsigned long dcPulseStartTime = 0;
const unsigned long DC_PULSE_DURATION = 200;  // 0.2 sec

// Helper for readable prints
const char* armStateToString(ArmState state) {
  switch (state) {
    case ARM_AT_POP: return "ARM_AT_POP";
    case ARM_AT_RECEIVE: return "ARM_AT_RECEIVE";
    case ARM_AT_DUMP: return "ARM_AT_DUMP";
    default: return "UNKNOWN";
  }
}
const char* dumpStateToString(DumpState state) {
  switch (state) {
    case DUMP_UP_POS: return "DUMP_UP_POS";
    case DUMP_DOWN_POS: return "DUMP_DOWN_POS";
    default: return "UNKNOWN";
  }
}

// === Setup ===
void setup() {
  // Stepper setup
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, LOW);

  pinMode(enablePin, OUTPUT);
  if (sleepPin >= 0) pinMode(sleepPin, OUTPUT);

  digitalWrite(enablePin, HIGH);
  if (sleepPin >= 0) digitalWrite(sleepPin, HIGH);

  // Servo setup
  armServo.attach(armServoPin);
  dumpServo.attach(dumpServoPin);
  armServo.write(ARM_POP_POS);
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
  Serial.println("0 = Move Arm Servo → ARM_DUMP_POS");
  Serial.println("1 = Start Stepper ON/OFF Cycle");
  Serial.println("2 = Stop Stepper Immediately (also stops continuous)");
  Serial.println("3 = Move Arm Servo → ARM_RECEIVE_POS (only if dump is UP)");
  Serial.println("4 = Move Arm Servo → ARM_POP_POS (only if dump is UP)");
  Serial.println("5 = Move Dump Servo → DUMP_POS (only if Arm is at ARM_DUMP_POS)");
  Serial.println("6 = Move Dump Servo → BASE_POS (sets dump to UP)");
  Serial.println("7 = Pulse DC Motor (0.2 sec)");
  Serial.println("8 = Stop DC Motor");
  Serial.println("C = Run Stepper Continuously (no pulsing)");
  Serial.println("====================");
}

void loop() {
  handleSerial();
  update_stepper_cycle();
  update_dc_pulse();

  // Continuous stepper mode
  if (isStepperContinuous) {
    unsigned long nowMicros = micros();
    if (nowMicros - lastStepTime >= stepInterval) {
      lastStepTime = nowMicros;
      stepPinState = !stepPinState;
      digitalWrite(stepPin, stepPinState);
    }
  }
}

// === Serial handler ===
void handleSerial() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '\n' || command == '\r') return;

    switch (command) {
      case '0': move_arm_to_dump();      break;
      case '1': start_stepper_cycle();   break;
      case '2': stop_stepper_cycle();    break;
      case '3': move_arm_to_receive();   break;
      case '4': move_arm_to_pop();       break;
      case '5': move_dump_to_target();   break;
      case '6': move_dump_to_base();     break;
      case '7': pulse_dc_motor();        break;  // updated behavior
      case '8': stop_dc_motor();         break;
      case 'C': run_stepper_continuous(); break;
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

  isStepperRunning = true;
  stepperActivePhase = true;  // start with ON phase
  phaseStartTime = millis();
  lastStepTime = micros();

  // enable A4988 driver (active LOW)
  digitalWrite(enablePin, LOW);

  // wake driver if sleep pin is used
  if (sleepPin >= 0) digitalWrite(sleepPin, HIGH);

  Serial.println("Stepper ON/OFF cycle started (driver enabled).");
}

void stop_stepper_cycle() {
  if (!isStepperRunning && !isStepperContinuous) {
    Serial.println("Stepper already stopped.");
    return;
  }

  isStepperRunning = false;
  isStepperContinuous = false;
  stepperActivePhase = false;

  digitalWrite(stepPin, LOW);
  digitalWrite(enablePin, HIGH);  // disable outputs

  Serial.println("Stepper stopped (all modes disabled).");
}

void update_stepper_cycle() {
  if (!isStepperRunning) return;

  unsigned long nowMillis = millis();

  if (stepperActivePhase) {
    // ON phase
    digitalWrite(enablePin, LOW);
    if (sleepPin >= 0) digitalWrite(sleepPin, HIGH);

    // Switch to OFF phase
    if (nowMillis - phaseStartTime >= STEPPER_ON_TIME) {
      stepperActivePhase = false;
      phaseStartTime = nowMillis;

      digitalWrite(stepPin, LOW);
      digitalWrite(enablePin, HIGH);  // disable outputs
      Serial.println("Stepper OFF phase (driver disabled).");
      return;
    }

    // Step pulses
    unsigned long nowMicros = micros();
    if (nowMicros - lastStepTime >= stepInterval) {
      lastStepTime = nowMicros;
      stepPinState = !stepPinState;
      digitalWrite(stepPin, stepPinState);
    }

  } else {
    // OFF phase
    if (nowMillis - phaseStartTime >= STEPPER_OFF_TIME) {
      stepperActivePhase = true;
      phaseStartTime = nowMillis;
      lastStepTime = micros();
      digitalWrite(enablePin, LOW);  // re-enable
      Serial.println("Stepper ON phase (driver enabled).");
    }
  }
}

// === Continuous stepper mode ===
void run_stepper_continuous() {
  if (isStepperContinuous) {
    Serial.println("Stepper already running continuously.");
    return;
  }

  // Stop any pulsing cycle that may be active
  if (isStepperRunning) {
    isStepperRunning = false;
    stepperActivePhase = false;
    Serial.println("Stopped pulsing cycle — switching to continuous mode.");
  }

  // Enable A4988 driver (active LOW)
  digitalWrite(enablePin, LOW);
  if (sleepPin >= 0) digitalWrite(sleepPin, HIGH);

  isStepperContinuous = true;
  lastStepTime = micros();
  stepPinState = LOW;
  digitalWrite(stepPin, LOW);

  Serial.println("Stepper running continuously (driver enabled, no pulsing).");
}

// === Servo functions ===
void move_arm_to_receive() {
  if (dumpState != DUMP_UP_POS) {
    Serial.println("⚠️ Cannot move Arm → ARM_RECEIVE_POS: Dump not in UP position!");
    return;
  }
  armServo.write(ARM_RECEIVE_POS);
  armState = ARM_AT_RECEIVE;
  Serial.print("Arm Servo → ARM_RECEIVE_POS | DumpState: ");
  Serial.println(dumpStateToString(dumpState));
}

void move_arm_to_pop() {
  if (dumpState != DUMP_UP_POS) {
    Serial.println("⚠️ Cannot move Arm → ARM_POP_POS: Dump not in UP position!");
    return;
  }
  armServo.write(ARM_POP_POS);
  armState = ARM_AT_POP;
  Serial.print("Arm Servo → ARM_POP_POS | DumpState: ");
  Serial.println(dumpStateToString(dumpState));
}

void move_arm_to_dump() {
  armServo.write(ARM_DUMP_POS);
  armState = ARM_AT_DUMP;
  Serial.println("Arm Servo → ARM_DUMP_POS");
}

void move_dump_to_target() {
  Serial.print("Current Arm State: ");
  Serial.println(armStateToString(armState));

  if (armState != ARM_AT_DUMP) {
    Serial.println("⚠️ Cannot move Dump Servo → DUMP_POS: Arm not in ARM_DUMP_POS!");
    return;
  }

  dumpServo.write(DUMP_POS);
  dumpState = DUMP_DOWN_POS;
  Serial.println("Dump Servo → DUMP_POS (Authorized)");
}

void move_dump_to_base() {
  dumpServo.write(DUMP_BASE_POS);
  dumpState = DUMP_UP_POS;
  Serial.println("Dump Servo → BASE_POS (Dump UP)");
}

// === DC Motor functions ===
void pulse_dc_motor() {
  if (isDCMotorRunning || dcPulseActive) {
    Serial.println("⚠️ DC Motor already active or pulsing.");
    return;
  }

  // Run DC motor forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 255);
  isDCMotorRunning = true;
  dcPulseActive = true;
  dcPulseStartTime = millis();

  Serial.println("⚡ DC Motor pulse started (0.2 sec)...");
}

void update_dc_pulse() {
  if (dcPulseActive && millis() - dcPulseStartTime >= DC_PULSE_DURATION) {
    stop_dc_motor();
    dcPulseActive = false;
    Serial.println("✅ DC Motor pulse completed.");
  }
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

  Serial.println("🛑 DC Motor stopped.");
}
