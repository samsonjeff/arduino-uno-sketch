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

int servoSpeed = 20; 
int currentPos = 54; // Starts at default (54 degrees)
bool moveLeft = true; // Toggle variable to track direction

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // POWER ON MESSAGE
  lcd.setCursor(0, 0);
  lcd.print("Hello User!");
  delay(2000);
  lcd.clear();
  
  lidServo.attach(3);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led, OUTPUT);
  
  // Set to default position
  lidServo.write(54); 
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

  if (distance > 0 && distance < 30) {
    lcd.setCursor(0, 0);
    lcd.print("Hello World    "); 
    digitalWrite(led, HIGH);

    if (moveLeft) {
      // --- MOVE TO LEFT (0) ---
      for (int pos = currentPos; pos >= 0; pos -= 1) {
        lidServo.write(pos);
        delay(servoSpeed);
      }
      currentPos = 0;
    } else {
      // --- MOVE TO RIGHT (90) ---
      for (int pos = currentPos; pos <= 90; pos += 1) {
        lidServo.write(pos);
        delay(servoSpeed);
      }
      currentPos = 90;
    }

    delay(3000); // Stay at position for 3 seconds

    // --- RETURN TO DEFAULT (54) ---
    if (currentPos > 54) {
      for (int pos = currentPos; pos >= 54; pos -= 1) {
        lidServo.write(pos);
        delay(servoSpeed);
      }
    } else {
      for (int pos = currentPos; pos <= 54; pos += 1) {
        lidServo.write(pos);
        delay(servoSpeed);
      }
    }
    
    currentPos = 54;         // Reset position tracker
    moveLeft = !moveLeft;    // Toggle direction for the NEXT detection
    digitalWrite(led, LOW);
    lcd.clear();
  } else {
    // If no sense detected, ensure it stays at 54 (your default)
    // Note: You mentioned 45 in your text, but 54 in your requirement. 
    // I am using 54 to match your "default" setting.
    lidServo.write(54); 
  }

  delay(100); 
}