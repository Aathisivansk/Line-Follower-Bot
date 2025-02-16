#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Motor driver contol pins NOTE: Change pin occordingly
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

//OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH,SCREEN_HEIGHT,&Wire,OLED_RESET);

//IR SENSOR PINS
#define NUM_OF_SENSORS 5
int sensorPins[NUM_OF_SENSORS] = {2,3,4,5,6};

//START BUTTON and LED INDICATOR
#define START_BOTTON 7
#define LED_INDICATOR 13

bool isRunning = false;

//PID logics Variables
int baseSpeed;
int errorSum;
int lastError;

#define Ki
#define Kp
#define Kd

void setup()
{

}

void loop()
{
    
}