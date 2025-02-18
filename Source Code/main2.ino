#include <QTRSensors.h>

// Motor driver control pins
#define IN1 A1
#define IN2 A2
#define IN3 A3
#define IN4 A4
#define ENA 5
#define ENB 6

// IR Sensor Configuration
#define NUM_OF_SENSORS 8
QTRSensorsRC qtr;
uint16_t sensorValues[NUM_OF_SENSORS];

// PID Control Variables
float integral = 0;
float lastError = 0;
float Kp = 0.8, Ki = 0.0, Kd = 0.2;  // Tuned PID constants
#define BASE_SPEED 150  // Base speed for straight paths

// Buttons for calibration and start
#define CALIBRATE_BUTTON A0
#define START_BUTTON A5

// Finish line detection threshold
#define FINISH_THRESHOLD 800  // Adjust based on sensor readings for black square

// Function to read sensors and calculate error
int readSensors() {
    int position = qtr.readLineBlack(sensorValues);
    int desiredPosition = 3500;  // Midpoint of the sensor range
    int error = position - desiredPosition;
    return error;
}

// Function to control motor speed and direction
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Control left motor
    if (leftSpeed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, -leftSpeed);
    }

    // Control right motor
    if (rightSpeed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, -rightSpeed);
    }
}

// PID Controller for Line Following
void followLine() {
    int error = readSensors();
    
    integral += error;
    float derivative = error - lastError;

    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    correction = constrain(correction, -BASE_SPEED, BASE_SPEED);

    int leftMotorSpeed = BASE_SPEED - correction;
    int rightMotorSpeed = BASE_SPEED + correction;

    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

    lastError = error;
}

// Function to detect finish line (black square)
bool detectFinishLine() {
    for (int i = 0; i < NUM_OF_SENSORS; i++) {
        if (sensorValues[i] > FINISH_THRESHOLD) {
            return true;  // Finish line detected
        }
    }
    return false;  // No finish line detected
}

// Function to handle intersections
void handleIntersection() {
    if (sensorValues[0] > FINISH_THRESHOLD && sensorValues[7] > FINISH_THRESHOLD) {
        // Intersection detected
        // Example: Turn left
        setMotorSpeed(-150, 150);  // Rotate left
        delay(500);  // Adjust delay based on turning speed
    }
}

// Function to auto-calibrate sensors
void autoCalibrate() {
    for (int i = 0; i < 150; i++) {  // Adjust loop count for better calibration
        qtr.calibrate();  // Read and store min/max sensor values
        
        // Make the bot move in a small circle (tweak speeds as needed)
        setMotorSpeed(80, -80);  // Left motor forward, Right motor backward
        
        delay(20);  // Small delay for smooth motion
    }
    
    // Stop the bot after calibration
    setMotorSpeed(0, 0);
}

void setup() {
    Serial.begin(9600);

    // Setup for BUTTONS
    pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);

    // Configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){2, 3, 4, 7, 8, 9, 12, 13}, NUM_OF_SENSORS);

    // Setup for Motor Driver
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Wait for calibration button press
    Serial.println("Press CALIBRATE button to begin calibration...");
    while (digitalRead(CALIBRATE_BUTTON) == HIGH);  // Wait until button is pressed
    delay(200);  // Debounce
    autoCalibrate();
    delay(2000);  // Delay before starting the bot

    // Wait for start button press
    Serial.println("Press START button to begin line following...");
    while (digitalRead(START_BUTTON) == HIGH);  // Wait until button is pressed
    delay(200);  // Debounce
}

void loop() {
    if (detectFinishLine()) {
        // Stop the bot when finish line is detected
        setMotorSpeed(0, 0);
        Serial.println("Finish line detected! Bot stopped.");
        while (true);  // Infinite loop to stop the bot
    } else {
        followLine();  // Continue line following
    }
}