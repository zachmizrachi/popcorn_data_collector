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
  Serial.println("2 = Stop Stepper Immediately");
  Serial.println("3 = Move Arm Servo → ARM_RECEIVE_POS (only if dump is UP)");
  Serial.println("4 = Move Arm Servo → ARM_POP_POS (only if dump is UP)");
  Serial.println("5 = Move Dump Servo → DUMP_POS (only if Arm is at ARM_DUMP_POS)");
  Serial.println("6 = Move Dump Servo → BASE_POS (sets dump to UP)");
  Serial.println("7 = Run DC Motor");
  Serial.println("8 = Stop DC Motor");
  Serial.println("====================");
}

void loop() {
  handleSerial();
  update_stepper_cycle();
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
      case '7': run_dc_motor();          break;
      case '8': stop_dc_motor();         break;
      default:
        Serial.print("Invalid command: ");
        Serial.println(command);
    }
  }
}

// === Stepper cycle control ===
void start_stepper_cycle() { /* ... unchanged ... */ }
void stop_stepper_cycle() { /* ... unchanged ... */ }
void update_stepper_cycle() { /* ... unchanged ... */ }

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
