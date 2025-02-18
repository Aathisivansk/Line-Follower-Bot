// ================================================= Program changes needed if DRV8833 driver is used ==============================
// Define motor control pins for DRV8833
#define LEFT_MOTOR_A1  5  // PWM-capable pin for left motor
#define LEFT_MOTOR_A2  6  // PWM-capable pin for left motor

#define RIGHT_MOTOR_B1 9  // PWM-capable pin for right motor
#define RIGHT_MOTOR_B2 10 // PWM-capable pin for right motor

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

// Setup function
void setup() {
    pinMode(LEFT_MOTOR_A1, OUTPUT);
    pinMode(LEFT_MOTOR_A2, OUTPUT);
    pinMode(RIGHT_MOTOR_B1, OUTPUT);
    pinMode(RIGHT_MOTOR_B2, OUTPUT);
}

//
