#include "flight_controller.h"

extern uint8_t number_used_sats, return_to_home_set;
extern float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
float aX, aY, aZ, gX, gY, gZ;
float roll_level_adjust, pitch_level_adjust;

//int16_t loop_counter;
//uint32_t loop_timer;    
//int32_t channel[RC_CHANNEL_NUM], channel_base[RC_CHANNEL_NUM];   

//int16_t esc_1, esc[CH_2], esc[CH_3], esc[CH_4];
float declination = 13.8f;
float low_battery_warning = 10.5f;

int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
int16_t acc_z_average_short[26], acc_z_average_long[51];

TwoWire myI2C(I2C1_SDA, I2C1_SCL);
MPU9250 mpu9250(myI2C, MPU9250_ADDRESS);
BMP280 bmp280;
DRONE myDrone;
PID_CONTROL pid;


void setup() {
  
  SerialUSB.begin(57600);
  delay(250);

  pinMode(GREEN_LED, OUTPUT);                                         //Set PD6 as output.
  pinMode(RED_LED, OUTPUT);                                         //Set PD7 as output.
  pinMode(BLUE_LED, OUTPUT);                                         //Set PD7 as output.
  green_led(LED_OFF);                                               //Set output PC7 low.
  red_led(LED_OFF);                                                 //Set output PB14 low.
  blue_led(LED_ON);                                                 //Set output PB7 high.
  
  pinMode(ADC_VBAT_IN, INPUT_ANALOG);

  timer_setup();
  delay(50);
  gps_setup();                                                  //Set the baud rate and output refreshrate of the GPS module.

  //Check if the MPU-9250 is responding.
  myDrone.error = mpu9250.begin();
  while (myDrone.error != 0) {                                  //Stay in this loop because the MPU-9250 did not responde.
    myDrone.error = 1;                                          //Set the error status to 1.
    myDrone.error_signal();                                      //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //Check if the BMP280 barometer is responding.
  myDrone.error = bmp280.probe();
  while (myDrone.error != 0) {                                  //Stay in this loop because the BMP280 did not responde.
    myDrone.error = 3;                                          //Set the error status to 2.
    myDrone.error_signal();                                      //Show the error via the red LED.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  mpu9250.setup();                                              //Initiallize the gyro and set the correct registers.
  myDrone.setupFlightCompass(mpu9250);                               //Initiallize the compass and set the correct registers.
  myDrone.readFlightCompass(mpu9250);                                //Read and calculate the compass data.
  myDrone.angle_yaw = myDrone.actual_compass_heading;           //Set the initial compass heading.

  //Create a 5 second delay before calibration.
  int count_var;
  for (count_var = 0; count_var < 1250; count_var++) {      //1250 loops of 4 microseconds = 5 seconds.
    red_led((count_var % 125 == 0) ? LED_ON : LED_OFF);         //Every 125 loops (500ms) -- Change the led status.
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;                                                //Set start back to 0.
  mpu9250.calibrateGyro();                                      //Calibrate the gyro offset.

  //Wait until the receiver is active.
  while (channel[CH_1] < 990 || channel[CH_2] < 990 || channel[CH_3] < 990 || channel[CH_4] < 990)  {
    myDrone.error = 4;                                          //Set the error status to 4.
    myDrone.error_signal();                                      //Show the error via the red LED.
    delay(4);                                                   //Delay 4ms to simulate a 250Hz loop
  }
  myDrone.error = 0;                                                    //Reset the error status to 0.


  //When everything is done, turn off the led.
  red_led(LED_OFF);                                                 //Set output PB4 low.

  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = myDrone.check_battery_voltage();

  bmp280.initialize();
  
  //The BMP280 needs a few readings to stabilize ????
  int start;
  for (start = 0; start < 100; start++) {                       //This loop runs 100 times.
    bmp280.getFlightData(myDrone);                              //Read and calculate the barometer data.
    delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  }
  bmp280.actual_pressure = 0;                                   //Reset the pressure calculations.

  //Before starting the avarage accelerometer value is preloaded into the variables.
  for (start = 0; start <= 24; start++) acc_z_average_short[start] = acc_z;
  for (start = 0; start <= 49; start++) acc_z_average_long[start] = acc_z;
  acc_z_average_short_total = acc_z * 25;
  acc_z_average_long_total = acc_z * 50;
  myDrone.start = 0;

  if (myDrone.motor_idle_speed < 1000)myDrone.motor_idle_speed = 1000;          //Limit the minimum idle motor speed to 1000us.
  if (myDrone.motor_idle_speed > 1200)myDrone.motor_idle_speed = 1200;          //Limit the maximum idle motor speed to 1200us.

  loop_timer = micros();                                        //Set the timer for the first loop.
} // end setup function


void loop() {

  /// Online adjustable settings 
  //uint32_t setting_adjust_timer;
  //uint16_t setting_click_counter;
  //uint8_t previous_channel_6;
  //float adjustable_setting_1, adjustable_setting_2, adjustable_setting_3;
  float battery_voltage;
  int16_t manual_throttle, throttle, takeoff_throttle;
	//int16_t manual_takeoff_throttle = 0;
		
  static int32_t receiver_watchdog = 0;
  
  if(receiver_watchdog < 750) receiver_watchdog ++;
  if(receiver_watchdog == 750 && myDrone.start == 2){
    channel[CH_1] = 1500;
    channel[CH_2] = 1500;
    channel[CH_3] = 1500;
    channel[CH_4] = 1500;
    myDrone.error = 8;
    if (number_used_sats > 5){
      if(home_point_recorded == 1)channel[CH_5] = 2000;
      else channel[CH_5] = 1750;
    }
    else channel[CH_5] = 1500;    
  }
  //Some functions are only accessible when the quadcopter is off.
  if (myDrone.start == 0) {
    
    if (channel[CH_1] > 1900 && channel[CH_2] < 1100 && channel[CH_3] > 1900 && channel[CH_4] > 1900) 
    {
      myDrone.calibrateFlightCompass(mpu9250);     //###### For compass calibration move both sticks to the top right. #########//
    }

    if (channel[CH_1] < 1100 && channel[CH_2] < 1100 && channel[CH_3] > 1900 && channel[CH_4] < 1100)
    {
      myDrone.calibrateFlightLevel(mpu9250);       //###### Level calibration move both sticks to the top left. #########//
    }
    // TODO: Add back in change settings feature
    /*
    if (channel[CH_6] >= 1900 && previous_channel[CH_6] == 0) {
      previous_channel[CH_6] = 1;
      if (setting_adjust_timer > millis()) setting_click_counter++;
      else setting_click_counter = 0;
      setting_adjust_timer = millis() + 1000;
      if (setting_click_counter > 3) {
        setting_click_counter = 0;
        change_settings();
      }
    }
    if (channel[CH_6] < 1900)previous_channel[CH_6] = 0;
  }*/

  myDrone.heading_lock = 0;
  if (channel[CH_6] > 1200) myDrone.heading_lock = 1;                                           //If channel 6 is between 1200us and 1600us the flight mode is 2

  myDrone.flight_mode = 1;                                                                 //In all other situations the flight mode is 1;
  if (channel[CH_5] >= 1200 && channel[CH_5] < 1600) myDrone.flight_mode = 2;                       //If channel 6 is between 1200us and 1600us the flight mode is 2
  if (channel[CH_5] >= 1600 && channel[CH_5] < 1950) myDrone.flight_mode = 3;                       //If channel 6 is between 1600us and 1900us the flight mode is 3
  if (channel[CH_5] >= 1950 && channel[CH_5] < 2100) {
    if (waypoint_set == 1 && myDrone.home_point_recorded == 1 && myDrone.start == 2) myDrone.flight_mode = 4;
    else myDrone.flight_mode = 3;
  }

  if (myDrone.flight_mode <= 3) {
    return_to_home_step = 0;
    return_to_home_lat_factor = 0;
    return_to_home_lon_factor = 0;
  }

  myDrone.return_to_home(return_to_home_step);                                                                //Jump to the return to home step program.
  myDrone.flight_mode_signal();                                                            //Show the flight_mode via the green LED.
  myDrone.error_signal();                                                                  //Show the error via the red LED.
  
  mpu9250.readSensor();
  aX = mpu9250.getAccelX_mss();
  aY = mpu9250.getAccelY_mss();
  aZ = mpu9250.getAccelZ_mss();
  gX = mpu9250.getGyroX_rads();
  gY = mpu9250.getGyroY_rads();
  gZ = mpu9250.getGyroZ_rads();
  
  bmp280.getFlightData(myDrone);                                                                //Read and calculate the barometer data.
  myDrone.readFlightCompass(mpu9250);                                                                  //Read and calculate the compass data.

  if (gps_add_counter >= 0) gps_add_counter--;

  myDrone.read_gps();

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  myDrone.gyro_roll_input = (myDrone.gyro_roll_input * 0.7) + (((float)myDrone.gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  myDrone.gyro_pitch_input = (myDrone.gyro_pitch_input * 0.7) + (((float)myDrone.gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  myDrone.gyro_yaw_input = (myDrone.gyro_yaw_input * 0.7) + (((float)myDrone.gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

    //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  myDrone.angle_pitch += myDrone.gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  myDrone.angle_roll += myDrone.gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
  myDrone.angle_yaw += myDrone.gyro_yaw * 0.0000611;                                        //Calculate the traveled yaw angle and add this to the angle_yaw variable.
  if (myDrone.angle_yaw < 0) myDrone.angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (myDrone.angle_yaw >= 360) myDrone.angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  myDrone.angle_pitch -= myDrone.angle_roll * sin(myDrone.gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  myDrone.angle_roll += myDrone.angle_pitch * sin(myDrone.gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  myDrone.angle_yaw -= myDrone.course_deviation(myDrone.angle_yaw, myDrone.actual_compass_heading) / 1200.0;       //Calculate the difference between the gyro and compass heading and make a small correction.
  if (myDrone.angle_yaw < 0) myDrone.angle_yaw += 360;                                             //If the compass heading becomes smaller then 0, 360 is added to keep it in the 0 till 360 degrees range.
  else if (myDrone.angle_yaw >= 360) myDrone.angle_yaw -= 360;                                     //If the compass heading becomes larger then 360, 360 is subtracted to keep it in the 0 till 360 degrees range.


  //Accelerometer angle calculations
  myDrone.acc_total_vector = sqrt((aX * aX) + (aY * aY) + (aZ * aZ));    //Calculate the total accelerometer vector.

  if (abs(aY) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)aY / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(aX) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)aX / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.

  vertical_acceleration_calculations();                                            //Calculate the vertical accelration.

  channel_base[CH_1] = channel[CH_1];                                                      //Normally channel[CH_1] is the pid_roll_setpoint input.
  channel_base[CH_2] = channel[CH_2];                                                      //Normally channel[CH_2] is the pid_pitch_setpoint input.
  gps_man_adjust_heading = angle_yaw;                                              //
  //When the heading_lock mode is activated the roll and pitch pid setpoints are heading dependent.
  //At startup the heading is registerd in the variable course_lock_heading.
  //First the course deviation is calculated between the current heading and the course_lock_heading.
  //Based on this deviation the pitch and roll controls are calculated so the responce is the same as on startup.
  if (myDrone.heading_lock == 1) {
    heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
    channel_base[CH_1] = 1500 + ((float)(channel[CH_1] - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel[CH_2] - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
    channel_base[CH_2] = 1500 + ((float)(channel[CH_2] - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel[CH_1] - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
    gps_man_adjust_heading = course_lock_heading;

  }
  if (myDrone.flight_mode >= 3 && waypoint_set == 1) {
    myDrone.pid_roll_setpoint_base = 1500 + gps_roll_adjust;
    myDrone.pid_pitch_setpoint_base = 1500 + gps_pitch_adjust;
  }
  else {
    myDrone.pid_roll_setpoint_base = channel_base[CH_1];
    myDrone.pid_pitch_setpoint_base = channel_base[CH_2];
  }

  //Because we added the GPS adjust values we need to make sure that the control limits are not exceded.
  if (myDrone.pid_roll_setpoint_base > 2000)myDrone.pid_roll_setpoint_base = 2000;
  if (myDrone.pid_roll_setpoint_base < 1000) myDrone.pid_roll_setpoint_base = 1000;
  if (myDrone.pid_pitch_setpoint_base > 2000) myDrone.pid_pitch_setpoint_base = 2000;
  if (myDrone.pid_pitch_setpoint_base < 1000) myDrone.pid_pitch_setpoint_base = 1000;

  pid.calculate_pid();                                                                 //Calculate the pid outputs based on the receiver inputs.

  myDrone.start_stop_takeoff();                                                            //Starting, stopping and take-off detection

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(pinmap_function(digitalPinToPinName(ADC_VBAT_IN), PinMap_ADC))) / 1410.1;

  //Turn on the led if battery voltage is to low. Default setting is 10.5V
  if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && myDrone.error == 0) myDrone.error = 1;


  //The variable base_throttle is calculated in the following part. It forms the base throttle for every motor.
  if (takeoff_detected == 1 && myDrone.start == 2) {                                         //If the quadcopter is started and flying.
    throttle = channel[CH_3] + takeoff_throttle;                                         //The base throttle is the receiver throttle channel + the detected take-off throttle.
    if (myDrone.flight_mode >= 2) {                                                          //If altitude mode is active.
      throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle;    //The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/Nju9rvZOjVQ
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  if (myDrone.start == 2) {                                                                //The motors are started.
    if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
    esc[CH_1] = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc[CH_2] = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc[CH_3] = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc[CH_4] = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

    if (battery_voltage < 12.40 && battery_voltage > 6.0) {                        //Is the battery connected?
      esc[CH_1] += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-1 pulse for voltage drop.
      esc[CH_2] += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-2 pulse for voltage drop.
      esc[CH_3] += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-3 pulse for voltage drop.
      esc[CH_4] += (12.40 - battery_voltage) * battery_compensation;                   //Compensate the esc-4 pulse for voltage drop.
    }

    if (esc[CH_1] < myDrone.motor_idle_speed) esc[CH_1] = myDrone.motor_idle_speed;                        //Keep the motors running.
    if (esc[CH_2] < myDrone.motor_idle_speed) esc[CH_2] = myDrone.motor_idle_speed;                        //Keep the motors running.
    if (esc[CH_3] < myDrone.motor_idle_speed) esc[CH_3] = myDrone.motor_idle_speed;                        //Keep the motors running.
    if (esc[CH_4] < myDrone.motor_idle_speed) esc[CH_4] = myDrone.motor_idle_speed;                        //Keep the motors running.

    if (esc[CH_1] > 2000)esc[CH_1] = 2000;                                                 //Limit the esc-1 pulse to 2000us.
    if (esc[CH_2] > 2000)esc[CH_2] = 2000;                                                 //Limit the esc-2 pulse to 2000us.
    if (esc[CH_3] > 2000)esc[CH_3] = 2000;                                                 //Limit the esc-3 pulse to 2000us.
    if (esc[CH_4] > 2000)esc[CH_4] = 2000;                                                 //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc[CH_1] = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-1.
    esc[CH_2] = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-2.
    esc[CH_3] = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-3.
    esc[CH_4] = 1000;                                                                  //If start is not 2 keep a 1000us pulse for ess-4.
  }

  set_esc_outputs(1000, 1000, 1000, 1000);  
  myDrone.send_telemetry_data();                                                           //Send telemetry data to the ground station.

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
  //that the loop time is still 4000us and no longer! More information can be found on
  //the Q&A page:
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

  if (micros() - loop_timer > 4050)myDrone.error = 2;                                      //Output an error if the loop time exceeds 4050us.
  while (micros() - loop_timer < 4000);                                            //We wait until 4000us are passed.
  loop_timer = micros();                                                           //Set the timer for the next loop.
}
