#define EXE_INTERVAL 1000


//Libraries for Ultrasonic sensor
#include <ZumoMotors.h>
#include <Pushbutton.h>

// Defining constant
const int trigPin = 4;
const int echoPin = 1;


ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

#define SPEED           200 // Maximum motor speed when going straight; variable speed when turning


unsigned long lastExecutedMillis = 0; // vairable to save the last executed time

void setup() {
  /*******************
   *  your setup code
   *******************/
   Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  
  // Ultrasonic sensor serial monitor
  
  long duration, cm;

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

  //delay(100);

  if (currentMillis - lastExecutedMillis >= EXE_INTERVAL) {
    lastExecutedMillis = currentMillis; // save the last executed time

    /******************
     * your code block
     ******************/
     Serial.print("   Straight");
  }
  
  if(cm < 8){      // run backwards and spin      
      motors.setSpeeds(-SPEED, -SPEED);   
      delay(100);  
       
      for (int speed = 0; speed >= -400; speed--)
        {
          motors.setLeftSpeed(speed);
          motors.setRightSpeed(-speed);
          delay(2);
          }
      }
}
\

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
