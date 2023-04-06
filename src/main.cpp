#include <Arduino.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

#define neoPixelPin 21

#define in1 35
#define in2 45
#define in3 36
#define in4 37
#define SOUND_SPEED 0.034

#define direction 40
#define step 41

#define SERVO_1 39
#define SERVO_2 42

#define stepButton 0

#define pot 1

Servo servo1;
Servo servo2;

Adafruit_NeoPixel neoPixel = Adafruit_NeoPixel(1, neoPixelPin, NEO_RGB + NEO_KHZ800);

// servo
// Stopped value
const int servo1_neutral = 70;
const int servo2_neutral = 71;
// Forward values
int servo_forward = 135;
// Reverse
int servo_reverse = 40;

// ultrasonic sensor variables
#define WINDOW_SIZE 5
long duration;
float distanceCm;
int INDEX = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;
class Distance
{
  // Class Member Variables
  // These are initialized at startup
  int trigPin;
  int echoPin;
  long OnTime;  // microseconds of on-time
  long OffTime; // microseconds of off-time

  // These maintain the current state
  int distState;
  unsigned long previousMicrosDist; // will store last time dist was updated

  // Constructor - creates a Distance
  // and initializes the member variables and state
public:
  Distance(int tPin, int ePin, long on, long off)
  {
    OnTime = on;
    OffTime = off;
    trigPin = tPin;
    echoPin = ePin;
    previousMicrosDist = 0;

    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

    distState = LOW;
  }

  float Update()
  {
    // check to see if it's time to change the state of the sensor
    unsigned long currentMicros = micros();

    if ((distState == HIGH) && (currentMicros - previousMicrosDist >= OnTime))
    {
      distState = LOW;                    // Turn it off
      previousMicrosDist = currentMicros; // Remember the time
      digitalWrite(trigPin, distState);   // Update the actual sensor

      // Reads the echoPin, returns the sound wave travel time in microseconds
      duration = pulseIn(echoPin, HIGH);

      // Calculate the distance
      distanceCm = duration * SOUND_SPEED / 2;
      // Serial.println(distanceCm);
    }
    else if ((distState == LOW) && (currentMicros - previousMicrosDist >= OffTime))
    {
      distState = HIGH;                   // turn it on
      previousMicrosDist = currentMicros; // Remember the time
      digitalWrite(trigPin, distState);   // Update the actual sensor
    }

    SUM = SUM - READINGS[INDEX];
    READINGS[INDEX] = distanceCm;
    SUM = SUM + distanceCm;
    INDEX = (INDEX + 1) % WINDOW_SIZE;

    AVERAGED = SUM / WINDOW_SIZE;

    return AVERAGED;
  }
};

class Drive
{
  const int high = 255;
  const int low = 0;
  int pin1;
  int pin2;
  int pwmOutput;

public:
  Drive(int firstPin, int secPin)
  {

    pin1 = firstPin;
    pin2 = secPin;
    // initalize pins
    // pinMode(pin1, OUTPUT);
    // pinMode(pin2, OUTPUT);

    // set motors to standby
    digitalWrite(pin1, low);
    digitalWrite(pin2, low);
  }

  void standby()
  {
    digitalWrite(pin1, low);
    digitalWrite(pin2, low);
  }

  // drives the motors forward, takes a speed value from 0 to 100
  void forward(int speed)
  {
    pwmOutput = map(speed, 0, 100, 0, 255);
    analogWrite(pin1, pwmOutput);
    analogWrite(pin2, low);
  }

  // drives the motors backwards, takes a speed value from 0 to 100
  void reverse(int speed)
  {
    pwmOutput = map(speed, 0, 100, 0, 255);
    analogWrite(pin2, pwmOutput);
    analogWrite(pin1, low);
  }

  void stop()
  {
    analogWrite(pin1, high);
    analogWrite(pin2, high);
  }
};

class StepperMotor
{
  int dirPin;
  int stepPin;
  unsigned long previousMicros;
  long stepDelayForward = 765;
  long stepDelayReverse = 765;
  int stepAlt = 1;

public:
  StepperMotor(int inDirPin, int inStepPin)
  {
    dirPin = inDirPin;
    stepPin = inStepPin;
    previousMicros = 0;

    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, LOW);
  }

  void forward()
  {
    unsigned long currentMillis = micros();
    // long stepSpeed = map(analogRead(pot),0,4096,0,1000);

    if (currentMillis - previousMicros > stepDelayForward)
    {

      digitalWrite(dirPin, LOW);
      // Serial.println(stepSpeed);
      stepAlt = stepAlt * -1;
      stepAlt == 1 ? digitalWrite(stepPin, HIGH) : digitalWrite(stepPin, LOW);
      // stepAlt == 1 ? Serial.println("HIGH") : Serial.println("LOW");

      previousMicros = currentMillis;
    }
  }
  void reverse()
  {
    unsigned long currentMillis = micros();
    // long stepSpeed = map(analogRead(pot),0,4096,0,1000);

    if (currentMillis - previousMicros > stepDelayReverse)
    {
      digitalWrite(dirPin, HIGH);
      // Serial.println(stepSpeed);
      stepAlt = stepAlt * -1;
      stepAlt == 1 ? digitalWrite(stepPin, HIGH) : digitalWrite(stepPin, LOW);
      // stepAlt == 1 ? Serial.println("HIGH") : Serial.println("LOW");

      previousMicros = currentMillis;
    }
  }

  void stop()
  {
    digitalWrite(stepPin, LOW);
  }
};

void changeColor(int color)
{
  switch (color)
  {
  case 0: // green
    neoPixel.setPixelColor(0, neoPixel.Color(0, 255, 0));
    neoPixel.show();
    break;
  case 1: // purple
    neoPixel.setPixelColor(0, neoPixel.Color(255, 0, 255));
    neoPixel.show();
    break;
  case 2: // pink
    neoPixel.setPixelColor(0, neoPixel.Color(255,20,147));
    neoPixel.show();
    break;
  
  case 3: // blue
    neoPixel.setPixelColor(0, neoPixel.Color(0, 0, 255));
    neoPixel.show();
    break;
  }
}

unsigned long previousMillisStart;
unsigned long currentMillisStart;

int stage = 0; // integer that iterates through what stage the robot is in
bool finish;
bool start;

const int sensorDistance = 7.5; // max sensor distance in cm

// time trackers used for stage 1
unsigned long previousMillisStepper;
unsigned long currentMillisStepper;
const long stepperRunTime = 16000; // how long the steppers run for in milliseconds

// time trackers used for stage 4
unsigned long previousMillisMotors;
unsigned long currentMillisMotors;
const long motorRunTime = 3000; // how long the motors run for in milliseconds

// time trackers used for stage 2
unsigned long previousMillisServos;
unsigned long currentMillisServos;
const long servosRunTime = 6000; // how long the servos run for in milliseconds

int mode = 0;

Drive motor1(in1, in2);
Drive motor2(in3, in4);
StepperMotor step1(direction, step);
Distance dist1(4, 5, 10, 2); // defining the distance sensor, first 2 params are the pins

void setup()
{
  Serial.begin(9600);

  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);

  pinMode(stepButton, INPUT_PULLUP);

  previousMillisStart = 0;
  previousMillisStepper = 0;
  previousMillisMotors = 0;
  previousMillisServos = 0;

  neoPixel.begin();

  finish = false; 
  start = false;
}

void loop()
{
  currentMillisStart = millis();

  mode = map(analogRead(pot), 0, 4096, 0, 4);

  switch (mode)
  {
  case 0: // main stage, runs the sequence of actions once button is pressed
    changeColor(mode);
    if (digitalRead(stepButton) == LOW)
    {
      start = true;
    }
    if (!finish && currentMillisStart - previousMillisStart > 2000 && start && mode == 0)
    {
      switch (stage)
      {
      case 0:
        // drive forward untill sensor detects edge of table
        if (dist1.Update() <= sensorDistance)
        {
          Serial.println(dist1.Update());
          motor1.forward(100); // value in here dictates speed, values can range from 0 - 100
          motor2.forward(100);
          // Serial.println("CASE 0 FORWARD");
        }
        else
        { // once the robot has reached the ledge turn on the brakes on the motors
          motor1.stop();
          motor2.stop();
          stage += 1;
          // Serial.println("CASE 0 STOP");
          previousMillisStepper = millis();
        }
        break;
      case 1:
        currentMillisStepper = millis();
        if (currentMillisStepper - previousMillisStepper < stepperRunTime)
        { // runs the stepper for the desired stepperRunTime in ms
          step1.forward();
          // Serial.println("CASE 1 FORWARD");
        }
        else
        {
          stage += 1;          
          previousMillisMotors = millis();
          // Serial.println("CASE 1 STOP");
        }
        break;
      case 2:
        currentMillisMotors = millis();
        if (currentMillisMotors - previousMillisMotors < 2000)
        { // drives forward for the desired motorRunTime in ms
          motor1.forward(50);
          motor2.forward(50);
        }
        else
        { // once the motors have run for their runtime this will set the motors into standby mode
          motor1.standby();
          motor2.standby();
          stage += 1;
          previousMillisServos = millis();
        }
        break;

      case 3:
        

        currentMillisServos = millis();

        if (currentMillisServos - previousMillisServos < servosRunTime)
        {
          servo1.write(servo_forward);
          servo2.write(servo_forward);
        }
        else
        {
          servo1.write(servo1_neutral);
          servo2.write(servo2_neutral);
          stage += 1;
          previousMillisMotors = millis();
        }
        break;
      case 4:
        currentMillisMotors = millis();
        if (currentMillisMotors - previousMillisMotors < 5000)
        { // drives forward for the desired motorRunTime in ms
          motor1.forward(100);
          motor2.forward(100);
        }
        else
        { // once the motors have run for their runtime this will set the motors into standby mode
          motor1.standby();
          motor2.standby();
          stage += 1;
          previousMillisStepper = millis();
        }
        

        break;

      case 5:
        // copied from case 1
        currentMillisStepper = millis();
        if (currentMillisStepper - previousMillisStepper < stepperRunTime)
        {
          step1.reverse();
        }
        else
        {
          stage += 1;
          previousMillisMotors = millis();
        }
        
        break;

      case 6:
        // drive forward untill sensor detects edge of table
        if (dist1.Update() <= 9 && dist1.Update() != 0 || dist1.Update() > 1500)
        {
          motor1.forward(75); // value in here dictates speed, values can range from 0 - 100
          motor2.forward(75);
          // Serial.println("CASE 0 FORWARD");
        }
        else
        { // once the robot has reached the ledge turn on the brakes on the motors
          motor1.stop();
          motor2.stop();
          stage += 1;
          // Serial.println("CASE 0 STOP");
          previousMillisMotors = millis();
          
        }
        break;

      case 7:
        currentMillisMotors = millis();
        if (currentMillisMotors - previousMillisMotors < motorRunTime)
        { // drives forward for the desired motorRunTime in ms
          motor1.reverse(75);
          motor2.reverse(75);
        }
        else
        { // once the motors have run for their runtime this will set the motors into standby mode
          motor1.standby();
          motor2.standby();
          finish = true;
          //stage += 1;
        }
        break;
      }
    }
    else
    {
      Serial.println("DONE");
    }
    break;
  case 1: // debug mode 1, runs the steppers in reverse when button is pressed
    changeColor(mode);
    if (digitalRead(stepButton) == LOW)
    {
      step1.reverse();
    }
    break;

  case 2: // debug mode 2, runs the steppers forward when button is pressed
      changeColor(mode);
    if (digitalRead(stepButton) == LOW)
    {
      step1.forward();
    }
    break;

  case 3: // debug mode 3, runs the servos when the button is pressed
    changeColor(mode);
    if (digitalRead(stepButton) == LOW)
    {
      servo1.write(servo_reverse);
      servo2.write(servo_reverse);
    }
    else
    {
      servo1.write(servo1_neutral);
      servo2.write(servo2_neutral);
    }
    break;
  }
}