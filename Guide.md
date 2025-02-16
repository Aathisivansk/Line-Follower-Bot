# Line Follower Robot - Arduino Nano

## **Component Description and Usage**

### **1. Arduino Nano**
- Acts as the **main controller** for reading sensor values, controlling motors, and handling the OLED display.
- Operates at **5V logic** and has **PWM-supported pins** for motor control.

---

### **2. OLED Display (SSD1306 - I2C)**
#### **Connections:**
| OLED Pin | Arduino Nano Pin |
|----------|-----------------|
| VCC      | 5V              |
| GND      | GND             |
| SCL      | A5 (I2C Clock)  |
| SDA      | A4 (I2C Data)   |

#### **Programming Instructions:**
1. Install required libraries:  
   - **Adafruit SSD1306**  
   - **Adafruit GFX**  
   *(In Arduino IDE: Sketch > Include Library > Manage Libraries > Search & Install)*
2. Example code to initialize and print data:
   ```cpp
   #include <Wire.h>
   #include <Adafruit_GFX.h>
   #include <Adafruit_SSD1306.h>

   #define SCREEN_WIDTH 128
   #define SCREEN_HEIGHT 64
   Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

   void setup() {
       display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
       display.clearDisplay();
       display.setTextSize(1);
       display.setTextColor(WHITE);
       display.setCursor(10, 10);
       display.println("OLED Ready");
       display.display();
   }

   void loop() { }
   ```

---

### **3. IR Sensor Array (Digital Output)**
- Used for **line tracking**.
- **Detects black (LOW) and white (HIGH)** surfaces.

#### **Connections:**
| IR Sensor | Arduino Nano Pin |
|-----------|-----------------|
| Sensor 1  | D2              |
| Sensor 2  | D3              |
| Sensor 3  | D4              |
| Sensor 4  | D7              |
| Sensor 5  | D8              |

#### **Programming Instructions:**
```cpp
#define NUM_SENSORS 5
int sensorPins[NUM_SENSORS] = {2, 3, 4, 7, 8};

void setup() {
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    Serial.print("Sensors: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(digitalRead(sensorPins[i]));
        Serial.print(" ");
    }
    Serial.println();
    delay(100);
}
```

---

### **4. L298 Mini Motor Driver**
#### **Connections:**
| L298 Mini Pin | Arduino Nano Pin |
|--------------|-----------------|
| VCC          | 5V              |
| GND          | GND             |
| IN1          | D10             |
| IN2          | D11             |
| IN3          | D5 (PWM)        |
| IN4          | D6 (PWM)        |

#### **Programming Instructions:**
```cpp
#define IN1 10
#define IN2 11
#define PWM1 5
#define PWM2 6

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
}

void loop() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM1, 150);  // 0 to 255 for speed
}
```

---

### **5. Start Button & LED Indicator**
#### **Connections:**
| Component | Arduino Nano Pin |
|-----------|-----------------|
| Start Button | D14 (A0) |
| LED Indicator | D15 (A1) |

#### **Programming Instructions:**
```cpp
#define START_BUTTON A0
#define LED_INDICATOR A1

bool isRunning = false;

void setup() {
    pinMode(START_BUTTON, INPUT_PULLUP);
    pinMode(LED_INDICATOR, OUTPUT);
}

void loop() {
    if (digitalRead(START_BUTTON) == LOW) {
        delay(200); // Debounce
        isRunning = !isRunning;
        digitalWrite(LED_INDICATOR, isRunning ? HIGH : LOW);
    }
}
```

---

## **Next Steps**
✅ **Ensure all components are connected correctly.**  
✅ **Upload the respective codes to Arduino Nano.**  
⏭️ **Proceed with integrating PID control for smooth movement.**
