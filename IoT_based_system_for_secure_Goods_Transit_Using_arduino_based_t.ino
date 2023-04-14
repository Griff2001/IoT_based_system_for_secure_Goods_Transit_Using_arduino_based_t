#include <SoftwareSerial.h>
#include <AFMotor.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Servo.h>

SoftwareSerial bluetooth(10, 9); // RX | TX arduino side
char B;
AF_DCMotor leftmotor(4);
AF_DCMotor rightmotor(3);
int LEDPIN = 13;

LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial mySerial(8, 9); //SIM800L Tx & Rx is connected to Arduino #8 & #9

const int servoPin = 3; // Servo pin
const int touchPin = 7; // Pushtouch pin
int ledPin = 5;
int touchState = 0;
int directionState = 0;
Servo myservo;
int pos = 0;

void setup()
{
  bluetooth.begin(9600); //Default Baud for comm, it may be different for your Module.
  Serial.begin(115200);
  Serial.println("WISETHINGZ BLUETOOTH CAR");
  pinMode(LEDPIN, OUTPUT);

  lcd.init();         // initialize the lcd
  lcd.backlight();    // Turn on the LCD screen backlight

  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(9600);

  //Begin serial communication with Arduino and SIM800L
  mySerial.begin(9600);

  Serial.println("Initializing...");
  delay(1000);

  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();

  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();

  mySerial.println("AT+CNMI=1,2,0,0,0"); // Configuration for receiving SMS
  updateSerial();

  lcd.begin(16, 2);
  lcd.print(" Touch Based");
  lcd.setCursor(0, 1);
  lcd.print("Door Lock System");
  myservo.attach(3);
  pinMode(touchPin, INPUT);
  pinMode(ledPin, OUTPUT);
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    mySerial.write(Serial.read()); //Forward what Serial received to Software Serial Port
  }
  while (mySerial.available())
  {
    Serial.write(mySerial.read()); //Forward what Software Serial received to Serial Port
  }
}

void sendCoordinates()
{
  mySerial.println("AT+CMGS=\"+254740390420\""); //number to send
  updateSerial();
  mySerial.print("Find the coordinates below; Latitude: -0° 10' 0.4512 Longitude: 35° 57' 52.9452"); //text content
  updateSerial();
  mySerial.write(26);
}

void loop()
{
  // Check for incoming SMS
  if (mySerial.available())
  {
    String message = mySerial.readString();
    if (message.indexOf("coordinates") >= 0)
    {
      sendCoordinates();
    }
  }

  // Check touch state
  touchState = digitalRead(touchPin);
  if (directionState == 0)
  {
    if (touchState == HIGH)
    {
      directionState = 1;
      for (pos = 0; pos < 180; pos += 1)
        digitalWrite(ledPin, 1);
      lcd.clear();
      lcd.print("Status: Unlocked");
      lcd.setCursor(0, 1);
    }
  }
      //bluetooth car
      if (bluetooth.available()) {
    B = bluetooth.read();
    Serial.println(B);

    if (B == 'F') {
      Serial.println("forward moving");
      leftmotor.run(FORWARD);
      leftmotor.setSpeed(254);
      rightmotor.run(FORWARD);
      rightmotor.setSpeed(254);
    }
    if (B == 'B') {
      Serial.println("backward moving");
      leftmotor.run(BACKWARD);
      leftmotor.setSpeed(254);
      rightmotor.run(BACKWARD);
      rightmotor.setSpeed(254);
    }
    if (B == 'R') {
      Serial.println("right moving");
      leftmotor.run(FORWARD);
      leftmotor.setSpeed(254);
      rightmotor.run(BACKWARD);
      rightmotor.setSpeed(254);
    }
    if (B == 'L') {
      Serial.println("left moving");
      leftmotor.run(BACKWARD);
      leftmotor.setSpeed(254);
      rightmotor.run(FORWARD);
      rightmotor.setSpeed(254);
    }
    if (B == 'S') {
      Serial.println("stop moving");
      leftmotor.run(RELEASE);
      rightmotor.run(RELEASE);
    }
  }
      
