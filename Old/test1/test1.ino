


void calibrate() {
  
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector running_min = { 32767,  32767,  32767};
  LSM303::vector running_max = {-32767, -32767, -32767};
  unsigned char index;
  
  // Initialize the compass chip
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  // configure the LSM303 chip
  // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  // 220 Hz compass update rate
  compass.writeReg(LSM303::CRB_REG_M, 0x60);
  compass.writeReg(LSM303::CRA_REG_M, 0x1C);

  // Wait till the robot is in a good place to spin.
  button.waitForButton();
  Serial.println("starting calibration");

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setSpeeds(200, -200);

  int counter = 70;
  
  while (counter > 0) {

    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    // capture new mins an max values
    if(running_min.x > compass.m.x) {
      running_min.x = compass.m.x;
    }
    if(running_min.y > compass.m.y) {
      running_min.y = compass.m.y;
    }
    if(running_max.x < compass.m.x) {
      running_max.x = compass.m.x;
    }
    if(running_max.y < compass.m.y) {
      running_max.y = compass.m.y;
    }

    // decrement counter by one
    counter = counter - 1;
    
    // sample with fine granularity
    delay(50);
  }

  // stop the robot from spinning
  motors.setSpeeds(0, 0);

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;
}


void setup() {

  // Set up the serial connection.
  Serial.begin(9600);

  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Run the calibration process.
  calibrate();

  // Wait to start the loop.
  button.waitForButton();
}



void loop() {

  float direction = 0;
  int   count     = 0;
 
  // Compute average heading over ten samples to reduce sensor noise
  while(count < 10 ) {
    compass.read();
    direction = direction + compass.heading();
    count = count + 1;
  }
  direction = direction / count;
  
  // display average heading
  Serial.println(direction);

  // wait one second
  delay(1000);
}
