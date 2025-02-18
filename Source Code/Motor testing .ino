// Motor driver control pins
#define IN1 A1
#define IN2 A2
#define IN3 A3
#define IN4 A4
#define ENA 5
#define ENB 6

void setup()
{
    //Setup for Motor Driver
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    Serial.begin(9600);

}

void loop()
{
    forward();
    delay(2000);

    backward();
    delay(2000);

}

void forward()
{
    Serial.println("Moving Forward");
    // Move Motor A forward at medium speed
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 150);
    // Move Motor B forward at medium speed
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 150);


}

void backward()
{
    Serial.println("Moving Backward");
    // Move Motor A forward at medium speed
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 150);
    // Move Motor B forward at medium speed
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 150);
}
