#include "flight_controller.h"

DRONE::DRONE(){
  // nevermind nothing to do
} 

void DRONE::setupFlightCompass(MPU9250 &mpu9250){
  uint8_t loop_var;
  float temp[6];
  
  //Read the calibration values from the EEPROM.  Remember order is Xbias Ybias Zbias, Xscale Yscale Zscale
  for (loop_var = 0; loop_var < 6; loop_var++) {
    eeprom_read_object(0x10 + (loop_var * 4), &temp[loop_var], sizeof(temp[0]));
    temp[loop_var] *= 3;          // convert back to float from byte array
  }
  // set the cal values into the MPU9250 and we are good to go
  mpu9250.setMagCalX(temp[0], temp[3]);
  mpu9250.setMagCalY(temp[1], temp[4]);
  mpu9250.setMagCalZ(temp[2], temp[5]);
}

// TODO VERIFY: Do we need to nvert the direction of the compass x and y axis  (MagY *= -1; MagX *= -1;)???          
void DRONE::readFlightCompass(MPU9250 &mpu9250) {
  float magData[IMU_AXIS_NUM];

  mpu9250.readSensor();
         
  //Before the compass can give accurate measurements it needs to be calibrated. At startup the compass_offset and compass_scale
  //variables are calculated. The following part will adjust the raw compas values so they can be used for the calculation of the heading.
  if (compass_cal_on == 0) {                    //When the compass is not beeing calibrated.
    magData[X_AXIS] = mpu9250.getMagX_uT();         // (maybe convert this to Gauss???
    magData[Y_AXIS] = mpu9250.getMagY_uT();         // (maybe convert this to Gauss???
    magData[Z_AXIS] = mpu9250.getMagZ_uT();         // (maybe convert this to Gauss???
  }

  //The compass values change when the roll and pitch angle of the quadcopter changes. 
  //That's the reason that the x and y values need to calculated for a virtual horizontal position.
  //The 0.0174533 value is pi/180 as the functions are in radians in stead of degrees.
  flightMagData[X_AXIS] = magData[X_AXIS] * cos(angle_pitch * -0.0174533) + magData[Y_AXIS] * sin(angle_roll * 0.0174533) * 
                          sin(angle_pitch * -0.0174533) - magData[Z_AXIS] * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
  flightMagData[Y_AXIS] = magData[Y_AXIS] * cos(angle_roll * 0.0174533) + magData[Z_AXIS] * sin(angle_roll * 0.0174533);

  //Now that the horizontal values are known the heading can be calculated. With the following lines of code the heading is calculated in degrees.
  //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
  if (flightMagData[Y_AXIS] < 0) {
    actual_compass_heading = 180 + (180 + ((atan2(flightMagData[Y_AXIS], flightMagData[X_AXIS])) * (180 / 3.14)));
  }
  else actual_compass_heading = (atan2(flightMagData[Y_AXIS], flightMagData[X_AXIS])) * (180 / 3.14);

  actual_compass_heading += (declination * (-1));                                   // actually subtracting (-ve #) the declination to the magnetic compass heading to get the geographic north.
  if (actual_compass_heading < 0) actual_compass_heading += 360;           //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (actual_compass_heading >= 360) actual_compass_heading -= 360;   //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.
}

// TODO: possible bug in while loop timing
uint8_t DRONE::calibrateFlightCompass(MPU9250 &mpu9250) {
  uint8_t loop_var;
  float temp[6];

  compass_cal_on = 1;                       //Set the compass_calibration_on variable to disable the adjustment of the raw compass values.
  red_led(LED_ON);                          //The red led will indicate that the compass calibration is active.
  green_led(LED_OFF);                       //Turn off the green led as we don't need it.
  
  error = mpu9250.calibrateMag();           // this involves a figure 8 motion
  if (error == IMU_OK) {                    // calibration successful
    while (channel[CH_2] < 1900) {          //Stay in this loop until the pilot lowers the pitch stick of the transmitter.
      send_telemetry_data();                //Send telemetry data to the ground station.
      delayMicroseconds(3700);              //Simulate a 250Hz program loop ---- shouldnt this be 4000us?????
    }
  }
  else return (error);                        //TODO: capture and process error code
  
  compass_cal_on = 0;                       //Reset the compass_calibration_on variable.
  mpu9250.readSensor();
  
  // Write (Update) all the mag cal info back into EEPROM 
  // Will take 24 bytes total (4 bytes per float) in order of Xbias, Ybias, Zbias, Xscale, Yscale, Zscale
  temp[0] = mpu9250.getMagBiasX_uT();
  temp[1] = mpu9250.getMagBiasY_uT();
  temp[2] = mpu9250.getMagBiasZ_uT();
  temp[3] = mpu9250.getMagScaleFactorX();
  temp[4] = mpu9250.getMagScaleFactorY();
  temp[5] = mpu9250.getMagScaleFactorZ();
  
  //Update instead of write will save wear on flash 
  for (loop_var = 0; loop_var < 6; loop_var ++) {
    eeprom_update_object(0x10 + (4 * loop_var), &temp[loop_var], sizeof(temp[0]));
  }
                   
  setupFlightCompass(mpu9250);                                           //Initiallize the compass and set the correct registers.
  readFlightCompass(mpu9250);                                            //Read and calculate the compass data.
  angle_yaw = actual_compass_heading;                             //Set the initial compass heading.

  red_led(LED_OFF);
  for (loop_var = 0; loop_var < 15; loop_var++) {                 // Light show to signal end of cal sequence
    green_led(HIGH);
    delay(50);
    green_led(LOW);
    delay(50);
  }
  error = 0;
  loop_timer = micros();                                                     //Set the timer for the next loop.
  return FC_OK;
}


uint8_t DRONE::calibrateFlightLevel(MPU9250 &mpu9250) {
  uint8_t status, loop_var;
  float temp[4];
  float accelX, accelY, accelZ;

  level_cal_on = 1;

  while (channel[CH_2] < 1100) {
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
    delay(10);
  }
  red_led(HIGH);
  green_led(LOW);

  error = 0;
  status = mpu9250.calibrateAccel();
  red_led(LOW);
  if (status == IMU_OK) {
    for (int loop_var = 0; loop_var < 15; loop_var ++) {
      green_led(HIGH);
      delay(50);
      green_led(LOW);
      delay(50);
    }
    // put the relevant accel cal data into the EEPROM 
    temp[0] = mpu9250.getAccelBiasX_mss();
    temp[1] = mpu9250.getAccelBiasY_mss();  
    temp[2] = mpu9250.getAccelScaleFactorX();  
    temp[3] = mpu9250.getAccelScaleFactorY();  
    for (loop_var = 0; loop_var < 4; loop_var++) {
      eeprom_update_object(0x24 + (4 * loop_var), &temp[loop_var], sizeof(temp[0]));
    }
  }
  else {               // calibration failure
    error = 3;
    return status;
  }
  level_cal_on = 0;
  mpu9250.readSensor(); 
  accelX = mpu9250.getAccelX_mss();
  accelY = mpu9250.getAccelY_mss();
  accelZ = mpu9250.getAccelZ_mss();
  
  acc_total_vector = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));    //Calculate the total accelerometer vector.
  if (abs(accelY)  < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)accelY / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(accelX) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)accelX / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }
  angle_pitch = angle_pitch_acc;                                                   //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
  angle_roll = angle_roll_acc;
  loop_timer = micros();                                                           //Set the timer for the next loop.
  return FC_OK;
}


//The following subrouting calculates the smallest difference between two heading values.
float DRONE::course_deviation(float course_b, float course_c) 
{
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}

void eeprom_read_object(unsigned int ee_addr, void *obj_p, size_t obj_size)
{
    unsigned char *p = obj_p;

    while (obj_size--) {
        *p++ = EEPROM.read(ee_addr++);
    }
}

void eeprom_update_object(unsigned int ee_addr, void *obj_p, size_t obj_size)
{
    unsigned char *p = obj_p;

    while (obj_size--) {
        EEPROM.update(ee_addr++, *p++);
    }
}

