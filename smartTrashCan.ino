#include <Servo.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 

LiquidCrystal_I2C lcd(0x27, 16, 2); 
Servo lidServo;

int trigPin = 9;
int echoPin = 10;
int led = 13;
long duration;
int distance;

// --- Settings ---
int servoSpeed = 50;   
int defaultPos = 45;   
int currentPos = 45;   
bool moveLeft = true;  

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  lidServo.attach(3);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led, OUTPUT);
  
  lidServo.write(defaultPos); 
}

void loop() {
  // --- Ultrasonic Sensor Logic ---
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Trigger if hand is detected (less than 20 cm)
  if (distance > 0 && distance < 20) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Working!   "); 
    digitalWrite(led, HIGH);

    // --- STEP 1: SLOW MOVE TO OPEN (Left 0 or Right 90) ---
    if (moveLeft) {
      for (int pos = currentPos; pos >= 0; pos -= 1) {
        lidServo.write(pos);
        delay(servoSpeed);
      }
      currentPos = 0;
    } else {
      for (int pos = currentPos; pos <= 90; pos += 1) {
        lidServo.write(pos);
        delay(servoSpeed);
      }
      currentPos = 90;
    }

    // --- NEW LOGIC: HOLD POSITION ---
    // This loop keeps the lid open as long as your hand is detected
    do {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(5);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.034 / 2;
      
      delay(100); // Short delay to stabilize sensor readings
    } while (distance > 0 && distance < 20);

    // Optional: Keep it open for 1 extra second after hand is removed
    delay(1000); 

    // --- STEP 2: SLOW BOUNCE BACK TO DEFAULT (45) ---
    if (currentPos > defaultPos) {
      for (int pos = currentPos; pos >= defaultPos; pos -= 1) {
        lidServo.write(pos);
        delay(servoSpeed);
      }
    } else {
      for (int pos = currentPos; pos <= defaultPos; pos += 1) {
        lidServo.write(pos);
        delay(servoSpeed);
      }
    }

    currentPos = defaultPos;  
    moveLeft = !moveLeft;     // Flip direction for next time
    digitalWrite(led, LOW);
    lcd.clear();
  } 
  else {
    // --- IDLE STATE ---
    lcd.setCursor(0, 0);
    lcd.print("Hello User!     "); 
    lidServo.write(defaultPos); 
    currentPos = defaultPos;
  }

  delay(100); 
}
