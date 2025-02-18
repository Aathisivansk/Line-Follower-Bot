#include <QTRSensors.h>

// Motor driver control pins
#define LEFT_MOTOR_FWD 5
#define LEFT_MOTOR_BACK 6
#define RIGHT_MOTOR_FWD 10
#define RIGHT_MOTOR_BACK 11

// IR Sensor Configuration
#define NUM_OF_SENSORS 8
QTRSensorsRC qtr;
uint16_t sensorValues[NUM_OF_SENSORS];

// PID Control Variables
float integral = 0;
float lastError = 0;
float Kp = 0.5, Ki = 0.5, Kd = 0.5;
#define BASE_SPEED 150

//Some more Buttons
#define CALIBRATE_BUTTON A0
#define START_BUTTON A1

// Function to read sensors and calculate error
int readSensors() {
    int position = qtr.readLineBlack(sensorValues);
    int desiredPosition = 3500; // Midpoint of the sensor range
    int error = position - desiredPosition;
    return error;
}

// Function to control motor speed
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    if (leftSpeed > 0) {
        analogWrite(LEFT_MOTOR_FWD, leftSpeed);
        digitalWrite(LEFT_MOTOR_BACK, LOW);
    } else {
        analogWrite(LEFT_MOTOR_BACK, -leftSpeed);
        digitalWrite(LEFT_MOTOR_FWD, LOW);
    }

    if (rightSpeed > 0) {
        analogWrite(RIGHT_MOTOR_FWD, rightSpeed);
        digitalWrite(RIGHT_MOTOR_BACK, LOW);
    } else {
        analogWrite(RIGHT_MOTOR_BACK, -rightSpeed);
        digitalWrite(RIGHT_MOTOR_FWD, LOW);
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


void setup()
{

    //Setup for BUTTONS
    pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);

    //Setup for IR sensors
    for (int i = 0; i < NUM_OF_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
    

    //Setup for Motor Driver
    pinMode(LEFT_MOTOR_FWD, OUTPUT);
    pinMode(LEFT_MOTOR_BACK, OUTPUT);
    pinMode(RIGHT_MOTOR_FWD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACK, OUTPUT);

    while (digitalRead(CALIBRATE_BUTTON) == HIGH);  // Wait until button is pressed
    delay(200); // Debounce
    autoCalibrate();
    delay(2000);  // Delay before starting the bot

    // Wait for start button press
    Serial.println("Press START button to begin line following...");
    while (digitalRead(START_BUTTON) == HIGH);  // Wait until button is pressed
    delay(200); // Debounce
}

void loop()
{
    followLine();
}