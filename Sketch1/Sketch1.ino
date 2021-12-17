/* This program uses the Zumo Shield's onboard magnetometer to help
 * the Zumo make a random turn. It uses ZumoMotors, Pushbutton, and 
 * ZumoIMU.
 *
 * This program first calibrates the compass to account for offsets in
 * its output. Calibration is accomplished in setup().
 *
 * In loop(), The driving angle then changes its offset by a random value 
 * from -45 to 45 degrees (90 degrees)from the heading every second. 
 *
 * It is important to note that stray magnetic fields from electric
 * current (including from the Zumo's own motors) and the environment
 * (for example, steel rebar in a concrete floor) might adversely
 * affect readings from the compass and make them less reliable.
 */

//Libraries for Magnetometer & zumo IR sensors
#include <Wire.h>
#include <ZumoShield.h>

//Libraries for Ultrasonic sensor
#include <ZumoMotors.h>
#include <Pushbutton.h>

//IR receiver
#include <IRremote.h>


#define SPEED           200 // Maximum motor speed when going straight; variable speed when turning
#define TURN_BASE_SPEED 200 // Base speed when turning (added to variable speed)

#define CALIBRATION_SAMPLES 70  // Number of compass readings to take when calibrating

#define LED_PIN 13


// Define an array for holding sensor values.
#define NUM_SENSORS 6
uint16_t sensorValues[NUM_SENSORS];
#define QTR_THRESHOLD  1500 // microseconds
#define cms  10

bool useEmitters = true;
uint8_t selectedSensorIndex = 0;


// Defining constant
const int trigPin = 4;
const int echoPin = 1;


int c = 0;


// IR
const int RECV_PIN = A0;
IRrecv irrecv(RECV_PIN);
decode_results results;


// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#define DEVIATION_THRESHOLD 25

ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
ZumoIMU imu;
ZumoIMU::vector<int16_t> m_max; // maximum magnetometer values, used for calibration
ZumoIMU::vector<int16_t> m_min; // minimum magnetometer values, used for calibration


//Initialize millis
unsigned long previousMillis = 0;        // will store last time LED was updated




//------------------------------------------------------------------------------------//

void setup()
{
  reflectanceSensors.init();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  //IR
  irrecv.enableIRIn();
  irrecv.blink13(true);

   //----------------------------------------------------------------------------------//

// Setup will calibrate our compass by finding maximum/minimum magnetic readings

   calibration_magneto();
}




void loop()
{
  // Magentometer serial monitor

  float heading, relative_heading;
  int speed;
  static float target_heading = averageHeading();

  // Heading is given in degrees away from the magnetic vector, increasing clockwise
  heading = averageHeading();

  // This gives us the relative heading with respect to the target angle
  relative_heading = relativeHeading(heading, target_heading);

  Serial.print("Target heading: ");
  Serial.print(target_heading);
  Serial.print("    Actual heading: ");
  Serial.print(heading);
  Serial.print("    Difference: ");
  Serial.print(relative_heading);

 
  //----------------------------------------------------------------------//

  // ULTRASONIC

      // Ultrasonic sensor serial monitor
  
  long duration, cm;
  digitalWrite(LED_PIN, HIGH);

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

  //----------------------------------------------------------------------//

  // Read the reflectance sensors.
  
  reflectanceSensors.read(sensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  printReadingsToSerial();   
  
  //----------------------------------------------------------------------//

  // Moving and orientation

  if (c==1){back_home();}

  
  // If the Zumo has turned to the direction it wants to be pointing, go straight and then do another turn
  else if(abs(relative_heading) < DEVIATION_THRESHOLD)
  { 
    unsigned long currentMillis = millis();
    straight();
    find_food(cm, target_heading);
    object_detection(cm, target_heading);
    

    
    if (currentMillis - previousMillis >= 1000) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      
      // Turn off motors and wait a short time to reduce interference from motors
      pose_before_redirection();

      // Turn 90 degrees relative to the direction we are pointing.
      // This will help account for variable magnetic field, as opposed
      // to using fixed increments of 90 degrees from the initial
      // heading (which might have been measured in a different magnetic
      // field than the one the Zumo is experiencing now).
      // Note: fmod() is floating point modulo
      target_heading = fmod(averageHeading() + random(-45,45), 360);
    }

    //delay(1000);

    //object_detection();
    
    
  
  }

  // TURN
  else
  {
    // To avoid overshooting, the closer the Zumo gets to the target
    // heading, the slower it should turn. Set the motor speeds to a
    // minimum base amount plus an additional variable amount based
    // on the heading difference.

    speed = SPEED*relative_heading/180;

    if (speed < 0)
      speed -= TURN_BASE_SPEED;
    else
      speed += TURN_BASE_SPEED;

    motors.setSpeeds(speed, -speed);

    Serial.print("   Turn");
  }
  Serial.println();

}







//-------------------------------------------------------------------------//
// Magnetometer

// Converts x and y components of a vector to a heading in degrees.
// This calculation assumes that the Zumo is always level.
template <typename T> float heading(ZumoIMU::vector<T> v)
{
  float x_scaled =  2.0*(float)(v.x - m_min.x) / (m_max.x - m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - m_min.y) / (m_max.y - m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

// Yields the angle difference in degrees between two headings
float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  // constrain to -180 to 180 degree range
  if (relative_heading > 180)
    relative_heading -= 360;
  if (relative_heading < -180)
    relative_heading += 360;

  return relative_heading;
}

// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
float averageHeading()
{
  ZumoIMU::vector<int32_t> avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    imu.readMag();
    avg.x += imu.m.x;
    avg.y += imu.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}


// ---------------------------------------------------------------------------------//
// Function for Ultrasonic sensor to caluclate distance

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
  

// FUnctions




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


         
void object_detection(long cm, long target_heading){
  
  if(cm < cms){      // run backwards and spin      
        digitalWrite(LED_PIN, LOW);   
        motors.setSpeeds(-SPEED, -SPEED);   
        delay(100);  

        if (random(1,2) == 2){
          motors.setSpeeds(-SPEED, SPEED);
          delay(500);
          }
        else{
          motors.setSpeeds(SPEED, -SPEED);
          delay(500);
          }
        motors.setSpeeds(SPEED, SPEED);
        delay(500);

        // Turn off motors and wait a short time to reduce interference from motors
      pose_before_redirection();

      // Turn 90 degrees relative to the direction we are pointing.
      // This will help account for variable magnetic field, as opposed
      // to using fixed increments of 90 degrees from the initial
      // heading (which might have been measured in a different magnetic
      // field than the one the Zumo is experiencing now).
      // Note: fmod() is floating point modulo
      target_heading = fmod(averageHeading() + random(-45,45), 360);

         }
  }


void find_food(long cm, long target_heading){
    if ((sensorValues[0] > QTR_THRESHOLD || sensorValues[1] > QTR_THRESHOLD || sensorValues[2] > QTR_THRESHOLD || sensorValues[4] > QTR_THRESHOLD || sensorValues[5] > QTR_THRESHOLD) && cm < cms)
  {
    digitalWrite(LED_PIN, LOW);   
    motors.setSpeeds(0, 0);   
    delay(5000);
    motors.setSpeeds(-SPEED, -SPEED);
    delay(100);  

        if (random(1,2) == 2){
          motors.setSpeeds(-SPEED, SPEED);
          delay(500);
          }
        else{
          motors.setSpeeds(SPEED, -SPEED);
          delay(500);
          }
      //  motors.setSpeeds(SPEED, SPEED);
     //   delay(1000);

/*
    //CONTINUE SEARCHING

        // Turn off motors and wait a short time to reduce interference from motors
      pose_before_redirection();

      // Turn 90 degrees relative to the direction we are pointing.
      // This will help account for variable magnetic field, as opposed
      // to using fixed increments of 90 degrees from the initial
      // heading (which might have been measured in a different magnetic
      // field than the one the Zumo is experiencing now).
      // Note: fmod() is floating point modulo
      target_heading = fmod(averageHeading() + random(-45,45), 360);

      */

      c = 1;
  }
  }

void back_home(){
  

  if (irrecv.decode()){
        straight();
        delay(1000);

        Serial.print("IR_beacon");
        Serial.println(results.value, HEX);
        irrecv.resume();
  }
  else{
    motors.setLeftSpeed(SPEED);
    motors.setRightSpeed(-SPEED);
    delay(100);
  }
  
  }


void straight(){
  motors.setSpeeds(SPEED, SPEED);
  Serial.print("   Straight");
  }

void pose_before_redirection(){
      // Turn off motors and wait a short time to reduce interference from motors
    motors.setSpeeds(0, 0);

   // while(currentMillis < lastExecutedMillis + 1000){}
    delay(100);
    }






void calibration_magneto(){
  // The highest possible magnetic value to read in any direction is 32767
  // The lowest possible magnetic value to read in any direction is -32767
  ZumoIMU::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
  unsigned char index;

  // Initialize the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initialize IMU
  imu.init();

  // Enables accelerometer and magnetometer
  imu.enableDefault();

  imu.configureForCompassHeading();

  button.waitForButton();

  Serial.println("starting calibration");

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(SPEED);
  motors.setRightSpeed(-SPEED);

  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    imu.readMag();

    running_min.x = min(running_min.x, imu.m.x);
    running_min.y = min(running_min.y, imu.m.y);

    running_max.x = max(running_max.x, imu.m.x);
    running_max.y = max(running_max.y, imu.m.y);

    Serial.println(index);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  // Store calibrated values in m_max and m_min
  m_max.x = running_max.x;
  m_max.y = running_max.y;
  m_min.x = running_min.x;
  m_min.y = running_min.y;
  button.waitForButton();
  }
