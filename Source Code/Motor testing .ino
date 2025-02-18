// Motor driver control pins
#define LEFT_MOTOR_FWD 5
#define LEFT_MOTOR_BACK 6
#define RIGHT_MOTOR_FWD 10
#define RIGHT_MOTOR_BACK 11

void setup()
{
    //Setup for Motor Driver
    pinMode(LEFT_MOTOR_FWD, OUTPUT);
    pinMode(LEFT_MOTOR_BACK, OUTPUT);
    pinMode(RIGHT_MOTOR_FWD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACK, OUTPUT);
}

void loop()
{
    forward();
    delay(3000);

    backward();
    delay(3000);

    stopMotor();
    delay(1000);
    
}

void forward()
{
    analogWrite(LEFT_MOTOR_FWD,200);
    analogWrite(LEFT_MOTOR_BACK,0);
    analogWrite(RIGHT_MOTOR_FWD,200);
    analogWrite(RIGHT_MOTOR_BACK,0);

}

void backward()
{
    analogWrite(LEFT_MOTOR_FWD, 0);
    analogWrite(LEFT_MOTOR_BACK,200);
    analogWrite(RIGHT_MOTOR_FWD, 0);
    analogWrite(RIGHT_MOTOR_BACK,200);
}

void stopMotor()
{
    //Turn OFF all motors
    analogWrite(LEFT_MOTOR_FWD,0);
    analogWrite(LEFT_MOTOR_BACK,0);
    analogWrite(RIGHT_MOTOR_FWD,0);
    analogWrite(RIGHT_MOTOR_BACK,0);
}
