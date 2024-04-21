#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "dht.h"

// variables
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
int lcdBacklightPin = 5;
bool lcdOn = false;

int pirButton = 8;
int pirPin = 13;
int pirState = LOW;
int pirVal = 0;
bool motion = false;

dht DHT;
int tempSensorPin = A0;

int lightSensorPin = A1;
int lightSensorVal = 0;

int e1Pin = 10; // motor 1 enable pin
int m1Pin = 12; // motor 1 direction pin
int fanSpeedLow = 100;
int fanSpeedMedium = 200;
int fanSpeedHigh = 255;
int fanSpeed = 0;

int redLEDPin = 7;
int yellowLEDPin = 6;
int greenLEDPin = 5;
bool redLED = false;
bool yellowLED = false;
bool greenLED = false;

int voltageSensorPin = A3;
float adcVoltage = 0;
float voltageSensorVal;
float voltage;
const float factor = 5.128;    // reduction factor for voltage sensor shield
const float refVoltage = 5.00; // reference voltage
float R1 = 30000.0;
float R2 = 7500.0;

float battPercent = 0;

// LCD display
void lcdDisplay(bool isLcdOn)
{
    if (isLcdOn)
    {
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("This is a LCD");
        lcd.setCursor(2, 1);
        lcd.print(":D");
        Serial.print("LCD Display = ");
        Serial.println("On");
        lcd.backlight();
    }
    else if (!isLcdOn)
    {
        lcd.clear();
        Serial.print("LCD Display = ");
        Serial.println("Off");
        lcd.noBacklight(); // turn off backlight
    }
}

// motion detection (PIR sensor)
void motionDetection()
{
    int pirVal = digitalRead(pirPin);       // Read PIR sensor value
    int buttonVal = digitalRead(pirButton); // Read button value
    Serial.print("Motion sensor = ");

    // Check if motion is detected or button is pressed
    if (pirVal == HIGH || buttonVal == HIGH)
    {
        Serial.println(HIGH);
    }
    else
    {
        Serial.println(LOW);
    }
}

// motor driver (fan)
void motorDriver(int fanSpeed)
{
    if (fanSpeed > 0)
    {
        Serial.print("Fan = ");
        Serial.println("On");

        digitalWrite(m1Pin, HIGH);
        if (fanSpeed == 1)
        {
            analogWrite(e1Pin, fanSpeedLow);
        }
        else if (fanSpeed == 2)
        {
            analogWrite(e1Pin, fanSpeedMedium);
        }
        else if (fanSpeed == 3)
        {
            analogWrite(e1Pin, fanSpeedHigh);
        }
    }
    else
    {
        digitalWrite(m1Pin, LOW);
        analogWrite(e1Pin, 0);
        Serial.print("Fan = ");
        Serial.println("Off");
    }
}

// temp sensor
void tempSensor()
{
    DHT.read11(tempSensorPin);

    Serial.print("Current humidity = ");
    Serial.print(DHT.humidity);
    Serial.print("%  ");
    Serial.print("temperature = ");
    Serial.print(DHT.temperature);
    Serial.println("C  ");
}

// light sensor
void lightSensor()
{
    lightSensorVal = analogRead(lightSensorPin); // when no light, value will be lower; when got light, value will be higher
    Serial.print("Light sensor = ");
    Serial.println(lightSensorVal);
}

// LEDs
void ledLights(bool red, bool yellow, bool green)
{
    if (red)
    {
        digitalWrite(redLEDPin, HIGH);
        Serial.println("Red LED = On");
    }
    else
    {
        digitalWrite(redLEDPin, LOW);
        Serial.println("Red LED = Off");
    }

    if (yellow)
    {
        digitalWrite(yellowLEDPin, HIGH);
        Serial.println("Yellow LED = On");
    }
    else
    {
        digitalWrite(yellowLEDPin, LOW);
        Serial.println("Yellow LED = Off");
    }

    if (green)
    {
        digitalWrite(greenLEDPin, HIGH);
        Serial.println("Green LED = On");
    }
    else
    {
        digitalWrite(greenLEDPin, LOW);
        Serial.println("Green LED = Off");
    }
}

void voltSensor()
{
    voltageSensorVal = analogRead(voltageSensorPin);

    adcVoltage = (voltageSensorVal * refVoltage) / 1024.0;

    voltage = adcVoltage / (R2 / (R1 + R2));
}

void calculateBattPercent()
{
    battPercent = (voltage / 9) * 100;

    Serial.print("Battery percentage = ");
    Serial.print(battPercent);
    Serial.println("%");
}

void setup()
{
    Serial.begin(9600);

    // lcd
    lcd.init();
    lcd.backlight();

    // pir motion sensor
    pinMode(pirPin, INPUT); // declare pir sensor as input
    pinMode(pirButton, INPUT);

    // motor driver
    pinMode(m1Pin, OUTPUT);
    digitalWrite(m1Pin, LOW); // close the fan by default

    // LEDs
    pinMode(redLEDPin, OUTPUT);
    pinMode(yellowLEDPin, OUTPUT);
    pinMode(greenLEDPin, OUTPUT);

    // voltage sensor
    pinMode(voltageSensorPin, INPUT);
}

void loop()
{
    if (Serial.available() > 0)
    {
        // parse incoming data
        String message = Serial.readStringUntil('\n');
        Serial.println(message);

        if (message == "Red LED On")
        {
            redLED = true;
        }
        if (message == "Red LED Off")
        {
            redLED = false;
        }
        if (message == "Yellow LED On")
        {
            yellowLED = true;
        }
        if (message == "Yellow LED Off")
        {
            yellowLED = false;
        }
        if (message == "Green LED On")
        {
            greenLED = true;
        }
        if (message == "Green LED Off")
        {
            greenLED = false;
        }
        if (message == "LCD On")
        {
            lcdOn = true;
        }
        if (message == "LCD Off")
        {
            lcdOn = false;
        }
        if (message == "Fan Off")
        {
            fanSpeed = 0;
        }
        if (message == "Fan 1")
        {
            fanSpeed = 1;
        }
        if (message == "Fan 2")
        {
            fanSpeed = 2;
        }
        if (message == "Fan 3")
        {
            fanSpeed = 3;
        }
    }
    lcdDisplay(lcdOn);

    motionDetection();
    motorDriver(fanSpeed);

    tempSensor();

    lightSensor();

    ledLights(redLED, greenLED, yellowLED);

    voltSensor();

    calculateBattPercent();
}