#include <QTRSensors.h>

// Motor driver control pins
#define LEFT_MOTOR_A1  5  // PWM-capable pin for left motor
#define LEFT_MOTOR_A2  6  // PWM-capable pin for left motor

#define RIGHT_MOTOR_B1 10  // PWM-capable pin for right motor
#define RIGHT_MOTOR_B2 11 // PWM-capable pin for right motor

// IR Sensor Configuration
#define NUM_OF_SENSORS 8
QTRSensors qtr;
uint16_t sensorValues[NUM_OF_SENSORS];

// PID Control Variables  Serial.Println("Variable calibration");
float integral = 0;
float lastError = 0;
float Kp = 0.8, Ki = 0.05, Kd = 0.2;
#define BASE_SPEED 150

//Some more Buttons
#define CALIBRATE_BUTTON A0
#define START_BUTTON A5

// Function to read sensors and calculate error
int readSensors() {
    Serial.println("Sensor reading and error calculation");
    int position = qtr.readLineBlack(sensorValues);

    //To print the sensor values
    Serial.print("Sensor Values: [ ");
    for (int i = 0; i < NUM_OF_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        if (i < NUM_OF_SENSORS - 1) {
            Serial.print(", ");
        }
    }
    Serial.println(" ]");

    int desiredPosition = 3500; // Midpoint of the sensor range
    int error = position - desiredPosition;
    return error;
}

// Function to control motor speed and direction
void setMotorSpeed(int leftSpeed, int rightSpeed) {
    Serial.println("Setting Motor speed.......");

    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
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

// PID Controller for Line 
void followLine()
 {
    Serial.println("PID Control");
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
    Serial.println("auto calibration");
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

    uint8_t sensorPins[] = {2, 3, 4, 7, 8, 9, 12, 13};

    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins(sensorPins, NUM_OF_SENSORS)
    qtr.emittersOn();

    Serial.println("Calibrating sensors...");
    for (int i = 0; i < 400; i++) {
        qtr.calibrate();
        delay(5);
    }
    Serial.println("Calibration complete!");

    pinMode(LEFT_MOTOR_A1, OUTPUT);
    pinMode(LEFT_MOTOR_A2, OUTPUT);
    pinMode(RIGHT_MOTOR_B1, OUTPUT);
    pinMode(RIGHT_MOTOR_B2, OUTPUT);

    // Wait for start button press
    Serial.println("Press START button to begin line following...");
    Serial.println(digitalRead(A5));
    while (digitalRead(A5) == HIGH);  // Wait until A5 switch is pressed
    delay(200);  // Debounce
}

void loop() {
    followLine();  // Start line-following once the switch is pressed
}
