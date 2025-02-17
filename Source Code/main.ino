#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Motor driver contol pins NOTE: Change pin occordingly
#define LEFT_MOTOR_FWD 5
#define LEFT_MOTOR_BACK 6
#define RIGHT_MOTOR_FWD 10
#define RIGHT_MOTOR_BACK 11

//OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH,SCREEN_HEIGHT,&Wire,OLED_RESET);

//IR SENSOR PINS
#define NUM_OF_SENSORS 8
int sensorPins[NUM_OF_SENSORS] = {2,3,4,7,8,9,12,13};

bool isRunning = false;

// Sensor positions for weighted average calculation
int sensorPositions[NUM_OF_SENSORS] = {-3, -2, -1, 0, 1, 2, 3, 4};

//PID logics Variables
float integral = 0;
float lastError = 0;

float kp = 0.5;
float ki = 0.5;
float kd = 0.5;

#define baseSpeed 150


// Function to read sensors and calculate error
int readSensors() {
    int weightSum = 0;
    int sensorSum = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        int sensorValue = digitalRead(sensorPins[i]) == LOW ? 1 : 0; // Assuming black = LOW
        weightSum += sensorValue * sensorPositions[i];
        sensorSum += sensorValue;
    }

    // If no sensor detects the line, return last error to prevent losing track
    if (sensorSum == 0) return lastError;
    
    int error = weightSum / sensorSum;
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

    int leftMotorSpeed = BASE_SPEED - correction;
    int rightMotorSpeed = BASE_SPEED + correction;

    setMotorSpeed(leftMotorSpeed, rightMotorSpeed);

    lastError = error;
}

void setup()
{
    //Setup for IR sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
    
    //Setup for Motor Driver
    pinMode(LEFT_MOTOR_FWD, OUTPUT);
    pinMode(LEFT_MOTOR_BWD, OUTPUT);
    pinMode(RIGHT_MOTOR_FWD, OUTPUT);
    pinMode(RIGHT_MOTOR_BWD, OUTPUT);

    // Initialize OLED display
    Serial.begin(9600);  // Debugging

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        for (;;);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("PID Line Follower");
    display.display();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
    
    pinMode(LEFT_MOTOR_FWD, OUTPUT);
    pinMode(LEFT_MOTOR_BACK, OUTPUT);
    pinMode(RIGHT_MOTOR_FWD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACK, OUTPUT);
}

void loop()
{
    // Update OLED Display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Line Follower Bot");

    // Show motor speed (To be updated when motor control is added)
    display.setCursor(0, 45);
    display.print("Speed: ");
    display.print(motorSpeed);
    display.println(" %");

    display.display();

    followLine();
}