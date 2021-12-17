
#include <Wire.h>
#include <ZumoShield.h>

#include <ZumoMotors.h>
#include <Pushbutton.h>


ZumoReflectanceSensorArray reflectanceSensors;
Pushbutton button(ZUMO_BUTTON);

// Define an array for holding sensor values.
#define NUM_SENSORS 6
uint16_t sensorValues[NUM_SENSORS];


#define QTR_THRESHOLD  1000 // microseconds
#define cms  10

bool useEmitters = true;

uint8_t selectedSensorIndex = 0;

#define LED_PIN 13
#define FORWARD_SPEED     200
const int trigPin = 4;
const int echoPin = 1;

ZumoMotors motors;

void setup()
{
  reflectanceSensors.init();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
   button.waitForButton();
}



void loop()
{

  //Ultrasonic sensor

  long duration, cm;
  digitalWrite(LED_PIN, HIGH);
  motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  // Read the reflectance sensors.
  reflectanceSensors.read(sensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  printReadingsToSerial();


  

  //delay(100);

  if ((sensorValues[0] > QTR_THRESHOLD || sensorValues[1] > QTR_THRESHOLD || sensorValues[2] > QTR_THRESHOLD || sensorValues[3] > QTR_THRESHOLD || sensorValues[4] > QTR_THRESHOLD || sensorValues[5] > QTR_THRESHOLD) && cm < cms)
  {   
    digitalWrite(LED_PIN, LOW);   
    motors.setSpeeds(-FORWARD_SPEED, -FORWARD_SPEED);   
    delay(100);   
    
    for (int speed = 0; speed >= -400; speed--)
  {
    motors.setLeftSpeed(speed);
    motors.setRightSpeed(-speed);
    delay(2);
  }
  }

}



// Prints a line with all the sensor readings to the serial
// monitor.
void printReadingsToSerial()
{
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d %4d %c\n",
    sensorValues[0],
    sensorValues[1],
    sensorValues[2],
    sensorValues[3],
    sensorValues[4],
    sensorValues[5],
    useEmitters ? 'E' : 'e'
  );
  Serial.print(buffer);
}


long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
