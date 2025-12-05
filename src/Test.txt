#include <Servo.h>
#include <L298Nlib.h>
#include <Ultrasonic.h>

// === Pins ===
#define TRIG_PIN A0
#define ECHO_PIN A1
#define SERVO_PIN 9

#define M1_IN1 4
#define M1_IN2 5
#define M2_IN1 6
#define M2_IN2 7

// === Constants ===
#define TURN_DURATION 1500
#define SERVO_DELAY 1500
#define ACTION_DELAY 1500
#define PING_INTERVAL 70
#define VACUUM 8

// === Components ===
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);
L298N_Motor motorLeft(M1_IN1, M1_IN2, -1);
L298N_Motor motorRight(M2_IN2, M2_IN1, -1);
Servo myservo;

// === Robot State ===
enum RobotState {
  IDLE,
  BACKING_UP,
  LOOKING_RIGHT,
  LOOKING_LEFT,
  TURNING
};

RobotState state = IDLE;
unsigned long stateStartTime = 0;
unsigned long lastPingTime = 0;

int distance = 20;
int distanceR = 0;
int distanceL = 0;

// === Function Prototypes ===
int readPing();
void moveForward();
void moveBackward();
void moveStop();

void setup() {
  Serial.begin(115200);
  myservo.attach(SERVO_PIN);
  myservo.write(90); // Center the servo
  pinMode(VACUUM, OUTPUT);
  digitalWrite(VACUUM, HIGH);
  delay(1000); // Initial stabilization
  distance = readPing();
}

void loop() {
  unsigned long currentTime = millis();

  switch (state) {
    case IDLE:
      if (currentTime - lastPingTime >= PING_INTERVAL) {
        lastPingTime = currentTime;
        distance = readPing();
        Serial.print("Distance: ");
        Serial.println(distance);

        if (distance <= 15) {
          moveStop();
          Serial.println("Obstacle detected!");
          state = BACKING_UP;
          stateStartTime = currentTime;
        } else {
          moveForward();
        }
      }
      break;

    case BACKING_UP:
      if (currentTime - stateStartTime < ACTION_DELAY) {
        moveBackward();
      } else {
        moveStop();
        myservo.write(0); // Look right
        state = LOOKING_RIGHT;
        stateStartTime = currentTime;
      }
      break;

    case LOOKING_RIGHT:
      if (currentTime - stateStartTime >= SERVO_DELAY) {
        distanceR = readPing();
        Serial.print("Right Distance: ");
        Serial.println(distanceR);
        myservo.write(150); // Look left
        state = LOOKING_LEFT;
        stateStartTime = currentTime;
      }
      break;

    case LOOKING_LEFT:
      if (currentTime - stateStartTime >= SERVO_DELAY) {
        distanceL = readPing();
        Serial.print("Left Distance: ");
        Serial.println(distanceL);
        myservo.write(60); // Center
        state = TURNING;
        stateStartTime = currentTime;
      }
      break;

    case TURNING:
      if (distanceR >= distanceL) {
        Serial.println("Turning Right");
        motorLeft.run(FORWARD);
        motorRight.run(BACKWARD);
      } else {
        Serial.println("Turning Left");
        motorLeft.run(BACKWARD);
        motorRight.run(FORWARD);
      }

      if (currentTime - stateStartTime >= TURN_DURATION) {
        moveForward();
        state = IDLE;
      }
      break;
  }
}

// === Sensor Read (Erick Simões library) ===
int readPing() {
  int cm = ultrasonic.read();  // returns distance in cm
  if (cm <= 0 || cm > 200) {
    cm = 250; // fallback if out of range
  }
  return cm;
}

// === Motion Functions ===
void moveForward() {
  motorLeft.run(FORWARD);
  motorRight.run(FORWARD);
  digitalWrite(VACUUM, HIGH);
}

void moveBackward() {
  motorLeft.run(BACKWARD);
  motorRight.run(BACKWARD);
  digitalWrite(VACUUM, LOW);
}

void moveStop() {
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
  digitalWrite(VACUUM, HIGH);
}
