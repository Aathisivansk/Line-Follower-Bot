// Motor driver contol pins NOTE: Change pin occordingly
#define MOT_A1_PIN 7
#define MOT_A2_PIN 9
#define MOT_B1_PIN 10
#define MOT_B2_PIN 11

void setup()
{
    //setup all motor pins to OUTPUT
    pinMode(MOT_A1_PIN, OUTPUT);
    pinMode(MOT_A2_PIN, OUTPUT);
    pinMode(MOT_B1_PIN, OUTPUT);
    pinMode(MOT_B2_PIN, OUTPUT);
}

void loop()
{
    forward();
    delay(2000);

    backward();
    delay(2000);

    stopMotor();
    delay(2000);
    
}

void forward()
{
    //Turn ON Motor
    analogWrite(MOT_A1_PIN,150);
    digitalWrite(MOT_A2_PIN,LOW);
    analogWrite(MOT_B1_PIN,150);
    digitalWrite(MOT_B2_PIN,LOW);
}

void backward()
{
    analogWrite(MOT_A1_PIN,0);
    analogWrite(MOT_A2_PIN,150);
    analogWrite(MOT_B1_PIN,0);
    analogWrite(MOT_B2_PIN,150);
}

void stopMotor()
{
    //Turn OFF all motors
    digitalWrite(MOT_A1_PIN, LOW);
    digitalWrite(MOT_A2_PIN, LOW);
    digitalWrite(MOT_B1_PIN, LOW);
    digitalWrite(MOT_B2_PIN, LOW);
}
