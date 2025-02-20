#include <QTRSensors.h>

// Motor driver control pins
#define LEFT_MOTOR_A1  5  // PWM-capable pin for left motor
#define LEFT_MOTOR_A2  6  // PWM-capable pin for left motor

#define RIGHT_MOTOR_B1 10  // PWM-capable pin for right motor
#define RIGHT_MOTOR_B2 11 // PWM-capable pin for right motor

// IR Sensor Configuration
#define NUM_OF_SENSORS 8
QTRSensorsAnalog qtr((unsigned char[]){A1, A2, A3, A4, A5, A6, A7, A8}, NUM_OF_SENSORS);\
#define LEDON_PIN 7  // Connect LEDON pin to D7
uint16_t sensorValues[NUM_OF_SENSORS];

// PID Control Variables
float integral = 0;
float lastError = 0;
float Kp = 0.8, Ki = 0.05, Kd = 0.2;
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
    pinMode(LEFT_MOTOR_A1, OUTPUT);
    pinMode(LEFT_MOTOR_A2, OUTPUT);
    pinMode(RIGHT_MOTOR_B1, OUTPUT);
    pinMode(RIGHT_MOTOR_B2, OUTPUT);

    Serial.println("Starting Calibration...");
    calibrateSensors();
    Serial.println("Calibration Complete!");
}

void loop() {
    if (digitalRead(START_BUTTON) == LOW) {
        followLine();
    }
}

void calibrateSensors() {
    for (int i = 0; i < 100; i++) {
        if (i % 20 < 10) {
            moveLeft();
        } else {
            moveRight();
        }
        qtr.calibrate();
        delay(100);
    }
    stopMotors();
}

void followLine() {
    int position = qtr.readLineBlack(sensorValues);
    int error = position - 3500;
    integral += error;
    float derivative = error - lastError;
    lastError = error;
    
    int turn = (Kp * error) + (Ki * integral) + (Kd * derivative);
    int leftSpeed = BASE_SPEED + turn;
    int rightSpeed = BASE_SPEED - turn;
    
    setMotorSpeed(leftSpeed, rightSpeed);
}

// Function to control motor speed and direction
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    Serial.println("Setting Motor speed.......");

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
