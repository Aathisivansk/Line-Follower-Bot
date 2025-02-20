#include <QTRSensors.h>

// Motor driver control pins
#define LEFT_MOTOR_A1  5  // PWM-capable pin for left motor
#define LEFT_MOTOR_A2  6  // PWM-capable pin for left motor

#define RIGHT_MOTOR_B1 10  // PWM-capable pin for right motor
#define RIGHT_MOTOR_B2 11 // PWM-capable pin for right motor

// IR Sensor Configuration
#define NUM_OF_SENSORS 8
QTRSensors qtr;
#define LEDON_PIN 7  // Connect LEDON pin to D7
#define ANALOG_PIN1 A0
#define ANALOG_PIN2 A1
#define ANALOG_PIN3 A2
#define ANALOG_PIN4 A3
#define ANALOG_PIN5 A4
#define ANALOG_PIN6 A5
#define ANALOG_PIN7 A6
#define ANALOG_PIN8 A7

uint16_t sensorValues[NUM_OF_SENSORS];

// PID Control Variables
float integral = 0;
float lastError = 0;
float Kp = 0.8, Ki = 0.008, Kd = 0.2;
#define BASE_SPEED 150

// Buttons
#define CALIBRATE_BUTTON 2
#define START_BUTTON 3

void setup() {
    Serial.begin(9600);
    pinMode(LEDON_PIN, OUTPUT);
    digitalWrite(LEDON_PIN, HIGH);
    pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);

    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){ANALOG_PIN1,ANALOG_PIN2,ANALOG_PIN3,ANALOG_PIN4,ANALOG_PIN5,ANALOG_PIN6,ANALOG_PIN7,ANALOG_PIN8}, NUM_OF_SENSORS);

    pinMode(LEFT_MOTOR_A1, OUTPUT);
    pinMode(LEFT_MOTOR_A2, OUTPUT);
    pinMode(RIGHT_MOTOR_B1, OUTPUT);
    pinMode(RIGHT_MOTOR_B2, OUTPUT);

    
}

void loop() {
  if (digitalRead(CALIBRATE_BUTTON) == LOW) {
    Serial.println("Starting Calibration...");
    calibrateSensors();
    Serial.println("Calibration Complete!");
  }
  if (digitalRead(START_BUTTON) == LOW) {
    while (digitalRead(START_BUTTON) == LOW); // Debounce
    while (true) { // Continuous line following
      followLine();
      if (digitalRead(START_BUTTON) == LOW) break; // Exit on button press
    }
  }
}

void calibrateSensors() {
  // Replace moveLeft/moveRight with forward/backward
  for (int i = 0; i < 100; i++) {
    if (i % 20 < 10) {
      setMotorSpeed(100, 100); // Forward
    } else {
      setMotorSpeed(-100, -100); // Backward
    }
    qtr.calibrate();
    delay(10);
  }
  stopMotors();
}

void followLine() {
    
    int position = qtr.readLineBlack(sensorValues);
    int error = position - 3500;
    integral += error;
    integral = constrain(integral, -1000, 1000); // Adjust limits as needed
    float derivative = error - lastError;
    lastError = error;
    
    int turn = (Kp * error) + (Ki * integral) + (Kd * derivative);
    Serial.print("Sensor Values: ");
    for (int i = 0; i < NUM_OF_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        Serial.print(" ");
    }
    Serial.println();
    
    Serial.print("Position: ");
    Serial.println(position);
    Serial.print("Error: ");
    Serial.println(turn);
    int leftSpeed = BASE_SPEED + turn;
    int rightSpeed = BASE_SPEED - turn;
    
    setMotorSpeed(leftSpeed, rightSpeed);
}

// Function to control motor speed and direction
void setMotorSpeed(int leftSpeed, int rightSpeed) {

    leftSpeed = constrain(leftSpeed, -75, 200);
    rightSpeed = constrain(rightSpeed, -75, 200);
    Serial.println(leftSpeed);
    Serial.println(rightSpeed);

    // Control left motor
    if (leftSpeed > 0) {
        analogWrite(LEFT_MOTOR_A1, leftSpeed);
        digitalWrite(LEFT_MOTOR_A2, LOW);
    } else {
        analogWrite(LEFT_MOTOR_A1, 0);
        analogWrite(LEFT_MOTOR_A2, abs(leftSpeed));
    }

    // Control right motor
    if (rightSpeed > 0) {
        analogWrite(RIGHT_MOTOR_B1, rightSpeed);
        digitalWrite(RIGHT_MOTOR_B2, LOW);
    } else {
        analogWrite(RIGHT_MOTOR_B1, 0);
        analogWrite(RIGHT_MOTOR_B2, abs(rightSpeed));
    }
}

void moveLeft() {
    digitalWrite(LEFT_MOTOR_A1, LOW);
    digitalWrite(LEFT_MOTOR_A2, HIGH);
    digitalWrite(RIGHT_MOTOR_B1, HIGH);
    digitalWrite(RIGHT_MOTOR_B2, LOW);
}

void moveRight() {
    digitalWrite(LEFT_MOTOR_A1, HIGH);
    digitalWrite(LEFT_MOTOR_A2, LOW);
    digitalWrite(RIGHT_MOTOR_B1, LOW);
    digitalWrite(RIGHT_MOTOR_B2, HIGH);
}

void stopMotors() {
    digitalWrite(LEFT_MOTOR_A1, LOW);
    digitalWrite(LEFT_MOTOR_A2, LOW);
    digitalWrite(RIGHT_MOTOR_B1, LOW);
    digitalWrite(RIGHT_MOTOR_B2, LOW);
}
