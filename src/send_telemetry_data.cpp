#include "flight_controller.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part sends the telemetry data to the ground station.
//The output for the serial monitor is PB0. Protocol is 1 start bit, 8 data bits, no parity, 1 stop bit.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DRONE::send_telemetry_data(void) 
{
  tel_loop_counter++;                                                                 //Increment the telemetry_loop_counter variable.
  if (tel_loop_counter == 1)tel_send_byte = 'J';                                //Send a J as start signature.
  if (tel_loop_counter == 2)tel_send_byte = 'B';                                //Send a B as start signature.
  if (tel_loop_counter == 3) {
    check_byte = 0;
    tel_send_byte = error;                              //Send the error as a byte.
  }
  if (tel_loop_counter == 4)tel_send_byte = flight_mode + return_to_home_step;                        //Send the flight mode as a byte.
  if (tel_loop_counter == 5)tel_send_byte = battery_voltage * 10;               //Send the battery voltage as a byte.
  if (tel_loop_counter == 6) {
    tel_buffer_byte = temperature;                                                    //Store the temperature as it can change during the next loop.
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the temperature variable.
  }
  if (tel_loop_counter == 7)tel_send_byte = tel_buffer_byte >> 8;         //Send the last 8 bytes of the temperature variable.
  if (tel_loop_counter == 8)tel_send_byte = angle_roll + 100;                   //Send the roll angle as a byte. Adding 100 prevents negative numbers.
  if (tel_loop_counter == 9)tel_send_byte = angle_pitch + 100;                  //Send the pitch angle as a byte. Adding 100 prevents negative numbers.
  if (tel_loop_counter == 10)tel_send_byte = start;                             //Send the error as a byte.
  if (tel_loop_counter == 11) {
    if (start == 2) {                                                                       //Only send the altitude when the quadcopter is flying.
      tel_buffer_byte = 1000 + ((float)(ground_pressure - actual_pressure) * 0.117);  //Calculate the altitude and add 1000 to prevent negative numbers.
    }
    else {
      tel_buffer_byte = 1000;                                                         //Send and altitude of 0 meters if the quadcopter isn't flying.
    }
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the altitude variable.
  }
  if (tel_loop_counter == 12)tel_send_byte = tel_buffer_byte >> 8;        //Send the last 8 bytes of the altitude variable.

  if (tel_loop_counter == 13) {
    tel_buffer_byte = 1500 + takeoff_throttle;                                        //Store the take-off throttle as it can change during the next loop.
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the take-off throttle variable.
  }
  if (tel_loop_counter == 14)tel_send_byte = tel_buffer_byte >> 8;        //Send the last 8 bytes of the take-off throttle variable.
  if (tel_loop_counter == 15) {
    tel_buffer_byte = angle_yaw;                                                      //Store the compass heading as it can change during the next loop.
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the compass heading variable.
  }
  if (tel_loop_counter == 16)tel_send_byte = tel_buffer_byte >> 8;        //Send the last 8 bytes of the compass heading variable.
  if (tel_loop_counter == 17)tel_send_byte = heading_lock;                      //Send the heading_lock variable as a byte.
  if (tel_loop_counter == 18)tel_send_byte = number_used_sats;                  //Send the number_used_sats variable as a byte.
  if (tel_loop_counter == 19)tel_send_byte = fix_type;                          //Send the fix_type variable as a byte.
  if (tel_loop_counter == 20) {
    tel_buffer_byte = l_lat_gps;                                                      //Store the latitude position as it can change during the next loop.
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the latitude position variable.
  }
  if (tel_loop_counter == 21)tel_send_byte = tel_buffer_byte >> 8;        //Send the next 8 bytes of the latitude position variable.
  if (tel_loop_counter == 22)tel_send_byte = tel_buffer_byte >> 16;       //Send the next 8 bytes of the latitude position variable.
  if (tel_loop_counter == 23)tel_send_byte = tel_buffer_byte >> 24;       //Send the next 8 bytes of the latitude position variable.
  if (tel_loop_counter == 24) {
    tel_buffer_byte = l_lon_gps;                                                      //Store the longitude position as it can change during the next loop.
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the longitude position variable.
  }
  if (tel_loop_counter == 25)tel_send_byte = tel_buffer_byte >> 8;        //Send the next 8 bytes of the longitude position variable.
  if (tel_loop_counter == 26)tel_send_byte = tel_buffer_byte >> 16;       //Send the next 8 bytes of the longitude position variable.
  if (tel_loop_counter == 27)tel_send_byte = tel_buffer_byte >> 24;       //Send the next 8 bytes of the longitude position variable.

  if (tel_loop_counter == 28) {
    tel_buffer_byte = adjustable_setting_1 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 1.
  }
  if (tel_loop_counter == 29)tel_send_byte = tel_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 1.
  if (tel_loop_counter == 30) {
    tel_buffer_byte = adjustable_setting_2 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 2.
  }
  if (tel_loop_counter == 31)tel_send_byte = tel_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 2.
  if (tel_loop_counter == 32) {
    tel_buffer_byte = adjustable_setting_3 * 100;                                     //Store the adjustable setting 1 as it can change during the next loop.
    tel_send_byte = tel_buffer_byte;                                            //Send the first 8 bytes of the adjustable setting 3.
  }
  if (tel_loop_counter == 33)tel_send_byte = tel_buffer_byte >> 8;        //Send the next 8 bytes of the adjustable setting 3.


  if (tel_loop_counter == 34)tel_send_byte = check_byte;                        //Send the check-byte.


  //After 125 loops the tel_loop_counter variable is reset. This way the telemetry data is send every half second.
  if (tel_loop_counter == 125)tel_loop_counter = 0;                             //After 125 loops reset the tel_loop_counter variable

  //Send the tel_send_byte via the serial protocol via ouput PB0.
  //Send a start bit first.
  if (tel_loop_counter <= 34) {
    check_byte ^= tel_send_byte;
    GPIOB_BASE->BSRR = 0b1 << 16;                                                             //Reset output PB0 to 0 to create a start bit.
    delayMicroseconds(104);                                                                   //Delay 104us (1s/9600bps)
    for (tel_bit_counter = 0; tel_bit_counter < 8; tel_bit_counter ++) {    //Create a loop fore every bit in the
      if (tel_send_byte >> tel_bit_counter & 0b1) GPIOB_BASE->BSRR = 0b1 << 0;    //If the specific bit is set, set output PB0 to 1;
      else GPIOB_BASE->BSRR = 0b1 << 16;                                                      //If the specific bit is not set, reset output PB0 to 0;
      delayMicroseconds(104);                                                                 //Delay 104us (1s/9600bps)
    }
    //Send a stop bit
    GPIOB_BASE->BSRR = 0b1 << 0;                                                              //Set output PB0 to 1;
  }
}
