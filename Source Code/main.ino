⁷#include <QTRSensors.h>

// Motor driver control pins
#define LEFT_MOTOR_A1  5  // PWM-capable pin for left motor
#define LEFT_MOTOR_A2  6  // PWM-capable pin for left motor

#define RIGHT_MOTOR_B1 10  // PWM-capable pin for right motor
#define RIGHT_MOTOR_B2 11 // PWM-capable pin for right motor

// IR Sensor Configuration
#define NUM_OF_SENSORS 8
QTRSensorsRC qtr;
uint16_t sensorValues[NUM_OF_SENSORS];

// PID Control Variables
float integral = 0;
float lastError = 0;
float Kp = 0.8, Ki = 0.08, Kd = 0.2;
#define BASE_SPEED 100

//Some more Buttons
#define CALIBRATE_BUTTON A0
#define START_BUTTON A5

// Function to read sensors and calculate error
int readSensors() {
    int position = qtr.readLineBlack(sensorValues);
    int desiredPosition = 3500; // Midpoint of the sensor range
    int error = position - desiredPosition;
    return error;
}

// Function to control motor speed and direction
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // Control left motor
    if (leftSpeed > 0) {
        analogWrite(LEFT_MOTOR_A1, leftSpeed);
        analogWrite(LEFT_MOTOR_A2, 0);
    } else {
        analogWrite(LEFT_MOTOR_A1, 0);
        analogWrite(LEFT_MOTOR_A2, -leftSpeed);
    }

    // Control right motor
    if (rightSpeed > 0) {
        analogWrite(RIGHT_MOTOR_B1, rightSpeed);
        analogWrite(RIGHT_MOTOR_B2, 0);
    } else {
        analogWrite(RIGHT_MOTOR_B1, 0);
        analogWrite(RIGHT_MOTOR_B2, -rightSpeed);
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
    Serial.begin(9600);

    //Setup for BUTTONS
    pinMode(CALIBRATE_BUTTON, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);

    // configure the sensors
    const uint8_t sensorPins[] = {2, 3, 4, 7, 8, 9, 12, 13};
qtr.setTypeRC();
qtr.setSensorPins(sensorPins, 8);  // 8 is the number of sensors

    pinMode(LEFT_MOTOR_A1, OUTPUT);
    pinMode(LEFT_MOTOR_A2, OUTPUT);
    pinMode(RIGHT_MOTOR_B1, OUTPUT);
    pinMode(RIGHT_MOTOR_B2, OUTPUT);

void setup() {
    pinMode(A5, INPUT_PULLUP);  // Enable internal pull-up resistor

    while (digitalRead(CALIBRATE_BUTTON) == HIGH);  // Wait until calibration button is pressed
    delay(200);  // Debounce
    autoCalibrate();
    delay(2000);  // Delay before starting the bot

    // Wait for start button press
    Serial.println("Press START button to begin line following...");
    while (digitalRead(A5) == HIGH);  // Wait until A5 switch is pressed
    delay(200);  // Debounce
}

void loop() {
    followLine();  // Start line-following once the switch is pressed
}