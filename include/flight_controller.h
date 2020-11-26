/*------------------ TODO / possible bugs / future enhancements ---------------------------------------- 
// Rev 1: 11/25/2020 - Wayne Brenckle
//
// 1) FC.cpp ln 40(readFlightCompass) VERIFY: Do we need to invert the direction of the x and y axis  
//    i.e. MagY *= -1; MagX *= -1; --- Dont think so but need to be sure
//
// 2) Standardize / consolodate error handling across classes. 
//
// 3) Check if we need to do any unit conversions / standardizations 
//    i.e. We are using uT instead of Gauss for mag fields, preassure is in hPa (everywhere?)
//
// 4) Implement nRF telemetry over SPI instead of Virtual (bit-banging) Serial port 
//    Will also need SPI for eMMC transfers once use cases are found (see enhancement section below)
//
// 5) Code cleanup: 
//     a) Global variables -- really shouldnt be any, nor the need to include every .h with 
//        every .cpp file.  
//
//     b) BMP280 class is a mess.  Longer term probably makes sense for DRONE class to 
//        inherit both MPU9250 and MP280 classes.
//
// 6) We dont need to use arduino.  Move to ST Cube and HAL/LL would give much more control.
//    i.e. no way I want to start implementing DMA or FreeRTOS in this framework
//
// 7) Any better file organization?  --- MPU9250.cpp is over 1000 lines !!!!!!
//    Should driver files have ANY? flight controller specific code????          
//
// 8) Future enhancements: 
//       a) Proximity sensor for obstacle avoidance
//       b) BL-Heli ESCs with one-shot (need to implement DMA first, also FreeRTOS?)
//       b) FPV camera system, can this be brought into flight controller control??
//       c) Any way to incorporate GoPro / Gimball ?  Subject follow / track 
//       d) Uses for SD storage?  Camera storage / Black box? EEPROM backup is only for sure use case right now
//       e) Look at MCU choices once final code size is in --- L4 is likely overkill, but does
//          power consumption / battery life justify it? i.e. LPUART etc 
//-----------------------------------------------------------------------------------------------------*/

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdio.h>
#include <math.h>
#include <EEPROM.h>
#include "BMP280.h"
#include "MPU9250.h"
#include "quaternionFilters.h"
#include "PID.h"

/*-----------------------
  STM32L496 PERIPHERALS
-----------------------*/
//-----INPUT TIMER-----//
#define CH_RISING     1
#define CH_FALLING    2
#define TIM15_CH1    PF9   // FS-iA6 PPM input
//-----OUTPUT TIMER----//
#define TIM1_CH1     PE9   // ESC1 PWM OUT
#define TIM1_CH2     PE11  // ESC2 PWM OUT   
#define TIM1_CH3     PE13  // ESC3 PWM OUT
#define TIM1_CH4     PE14  // ESC4 PWM OUT
//-----LED GPIOs-------//
#define GREEN_LED    PC7
#define RED_LED      PB14
#define BLUE_LED     PB7
//-----I2C1 BUS--------//
#define I2C1_SDA     PB9
#define I2C1_SCL     PB8
//-----ADC VBAT--------//
#define ADC_VBAT_IN  PA3   // MAYBE USE PC3?
//------USART2---------//
#define USART2_TX    PD5
#define USART2_RX    PD6
//------USART3---------//
#define USART3_TX    PD8
#define USART3_RX    PD9
//------LPUART1--------//
#define LPUART1_TX   PC1
#define LPUART1_RX   PC0
//------SDMMC----------//
//#define SDIO0      PC8
//#define SDIO1      PC9
//#define SDIO2      PC10
//#define SDIO3      PC11
//#define SDIO_SK    PC12
//#define SDIO_CMD   PD2
//------SPI1 BUS-------//
//#define SPI1_MOSI  PA6
//#define SPI1_MISO  PA7
//#define SPI1_SCK   PA5
//#define SPI1_CS    PD14 (Actually SPI_NSS, need to handle carefully in SW)
//#define SPI1_DR    PD15
//------SPI2 BUS-------//
//#define SPI2_MOSI  PB5
//#define SPI2_MISO  PB4
//#define SPI2_SCK   PB3
//#define SPI2_CS    PA4  (Actually SPI_NSS, need to handle carefully in SW)
//#define SPI2_DR    PF5
//-----POWER IN/OUT----//
//#define 5VIN   (CN8)P9
//#define 3V3OUT (CN8)P7
//#define LPWR_WAKE  PA0

enum { CH_1, CH_2, CH_3, CH_4, CH_5, CH_6 };  
enum { X_AXIS, Y_AXIS, Z_AXIS }; 
enum { LED_OFF, LED_ON };

#define RC_CHANNEL_NUM 6
#define MOTOR_NUM      4
#define IMU_AXIS_NUM   3

#define IMU_OK 1    // TODO: MPU9250 library returns -ve errors, need to implement a handler to parse
#define FC_OK  1

#define MPU9250_ADDRESS  0x68

#define SERIAL_DEBUG 0

extern float declination;                 // actually needs to be negative: Set the declination (for San Jose) between the magnetic and geographic north.
extern float battery_voltage, low_battery_warning, gps_man_adjust_heading;          // Set the battery warning at 10.5V (default = 10.5V).
extern float angle_yaw, angle_pitch, angle_roll;
extern float pid_output_roll, pid_output_pitch, pid_output_yaw, pid_output_altitude;
extern float gps_roll_adjust, gps_pitch_adjust;
extern float heading_lock_course_deviation;
extern uint8_t gps_add_counter, waypoint_set, takeoff_detected;
extern int16_t loop_counter; 
extern uint32_t loop_timer;    
extern int32_t channel[RC_CHANNEL_NUM], channel_base[RC_CHANNEL_NUM], esc[MOTOR_NUM];

extern void red_led(int8_t level);
extern void green_led(int8_t level);
extern void blue_led(int8_t level);
extern float check_battery_voltage(void);
extern void eeprom_read_object(unsigned int ee_addr, void *obj_p, size_t obj_size);
extern void eeprom_update_object(unsigned int ee_addr, void *obj_p, size_t obj_size);
extern void timer_setup(void);
extern void gps_setup(void); 
extern void vertical_acceleration_calculations(void);
extern void set_esc_ouputs(int32_t channel1, int32_t channel2, int32_t channel3, int32_t channel4);


class DRONE {
	
	public:
		//----- flight_controller.cpp ---------//
    	DRONE(void); // base type
		
		void setupFlightCompass(MPU9250 &mpu9250);
    	void readFlightCompass(MPU9250 &mpu9250);
    	uint8_t calibrateFlightCompass(MPU9250 &mpu9250);
    	uint8_t calibrateFlightLevel(MPU9250 &mpu9250);
		float course_deviation(float course_b, float course_c);
		void read_gps(void); 
		void start_stop_takeoff(void);
		void return_to_home(uint8_t go_home);
		void send_telemetry_data(void); 
		//void calculate_pid(void); 
		void flight_mode_signal(void); 
		void error_signal(void);
		float check_battery_voltage(void);

		// TODO: implement these in this class
		//void set_esc_outputs (uint32_t channel1, uint32_t channel2, uint32_t channel3, uint32_t channel4);
		//void green_led(int8_t level);
		//void red_led(int8_t level);
		//void blue_led(int8_t level);

		uint8_t check_byte, start;
		uint8_t error, error_counter, error_led, warning;
		uint8_t flight_mode, flight_mode_counter, flight_mode_led;
		uint8_t takeoff_detected, manual_alt_change;
		uint8_t tel_send_byte, tel_bit_counter, tel_loop_counter;
		uint32_t tel_buffer_byte;
		
		// TODO: where are other GPS Values????
		uint8_t number_used_sats, home_point_recorded;

		int16_t manual_throttle, throttle, takeoff_throttle;
		int16_t manual_takeoff_throttle = 0, motor_idle_speed = 1100;
		
		uint8_t last_channel[MOTOR_NUM];
		int32_t channel_start[RC_CHANNEL_NUM], previous_channel[RC_CHANNEL_NUM], channel_base[2];
		int32_t pid_roll_setpoint_base, pid_pitch_setpoint_base, pid_yaw_setpoint_base;
		int32_t receiver_watchdog;
		int32_t acc_total_vector, acc_total_vector_at_start;
		
		uint32_t error_timer, flight_mode_timer;

		float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
		float gyro_roll, gyro_pitch, gyro_yaw, gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
		//float acc_pitch_cal_value, acc_roll_cal_value;
		float dummy_float;

		//Compass variables
		uint8_t compass_cal_on, level_cal_on, heading_lock;
		int16_t flightMagData[IMU_AXIS_NUM];            // values adjusted for tilt/roll
		//int16_t compass_cal_values[6];
		float actual_compass_heading; //compass_x_horizontal, compass_y_horizontal;
		//float compass_scale_y, compass_scale_z;
		//int16_t compass_offset_x, compass_offset_y, compass_offset_z;
		float course_a, course_b, course_c, base_course_mirrored, actual_course_mirrored;
		float course_lock_heading, heading_lock_course_deviation;
		float compass_scale_deviation[2];

		/*
float l_lon_gps_float_adjust, l_lat_gps_float_adjust, gps_man_adjust_heading;
float return_to_home_lat_factor, return_to_home_lon_factor, return_to_home_move_factor;
uint8_t home_point_recorded;
int32_t lat_gps_home, lon_gps_home;
*/



}; // Class DRONE

#endif /* FLIGHT_CONTROLLER_H */

