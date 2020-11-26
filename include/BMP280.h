/*	BMP280.h------------------modified by Wayne Brenckle
	Bosch BMP280 pressure sensor library using the Arduino framework.
	This library uses I2C connection.
	Uses floating-point equations from BMP280 datasheet.   */

#ifndef BMP280_h
#define BMP280_h

#include "flight_controller.h"

#define BMP280_ADDR 0x76 // 7-bit address

class BMP280 {
	public:
		BMP280(void) :  dig_T1(0), dig_P1(0), dig_T2(0), dig_T3(0), dig_P2(0), dig_P3(0), dig_P4(0), dig_P5(0), dig_P6(0), dig_P7(0), dig_P8(0), dig_P9(0), tFine(0)
    	{
    	};

		/**< declaring prototypes */
  		uint8_t probe  (const uint8_t address = BMP280_ADDR);
  				
		void setEnabled(const uint8_t enable=1);     // Enable Sensor or set it to Standby (1 or 0)

		uint8_t getEnabled(void);

		void setPressureOversampleRatio(const uint8_t sampleRatio = 16);

		void setTemperatureOversampleRatio(const uint8_t sampleRatio = 2);

		void setFilterRatio(const uint8_t filterRatio = 0);

		void setStandby(const uint16_t ms = 0);

		void readTrimming(void);

		void reset(void);                    // trigger a software-reboot of the sensor

  		uint8_t initialize(void);               // set up the sensor for basic operation, IF found at address
  		
  		uint8_t checkMeasurement(void);       // check for new data, return 1 when Measurement is ready 

  		uint8_t awaitMeasurement(void);        // wait (and check) for new data in a busy-loop, TODO: give functionpointer to sleepfunction */

		void triggerMeasurement(void);		  // only used when in manual/standby mode	

		void getRawPressure(uint32_t &);
		void getPressure(uint32_t&);
		void getPressure(float&);
		
		void getAltitude(float&);
		
		void getRawTemperature(uint32_t &);
		void getTemperature(int32_t&);
		void getTemperature(float&);

		void getMeasurement(float&);
		void getCalcValues(uint32_t, uint32_t);

		void getFlightData(DRONE &myDrone);

		//Pressure variables--------TODO: MOVE THIS AND getFlightData() to DRONE Class?????????
		uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
		float actual_pressure, pressure_slow, pressure_fast, pressure_diff;
		float ground_pressure, altutude_hold_pressure, return_to_home_decrease;
		float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
		float pid_error_gain_altitude, pid_throttle_gain_altitude;
		float pid_i_gain_altitude, pid_p_gain_altitude, pid_max_altitude;
		uint8_t parachute_rotating_mem_location;
		int32_t parachute_buffer[35], parachute_throttle;
		float pressure_parachute_previous;
		int32_t pressure_rotating_mem[50], pressure_total_avarage;
		uint8_t pressure_rotating_mem_location;
		float pressure_rotating_mem_actual;
 		float calc_temperature, calc_pressure, calc_altitude;
		
	private:
	
		/** ######### Register-Map ################################################################# */
    
		// CALIBRATION DATA, 25 Register. 0x88 - 0xA1
    	static const uint8_t REG_DIG_T1                     =(0x88);    // watch out - switched MSB/LSB
    	static const uint8_t REG_DIG_T2                     =(0x8A);
    	static const uint8_t REG_DIG_T3                     =(0x8C);
    	static const uint8_t REG_DIG_P1                     =(0x8E);
    	static const uint8_t REG_DIG_P2                     =(0x90);
    	static const uint8_t REG_DIG_P3                     =(0x92);
    	static const uint8_t REG_DIG_P4                     =(0x94);
    	static const uint8_t REG_DIG_P5                     =(0x96);
    	static const uint8_t REG_DIG_P6                     =(0x98);
    	static const uint8_t REG_DIG_P7                     =(0x9A);
    	static const uint8_t REG_DIG_P8                     =(0x9C);
    	static const uint8_t REG_DIG_P9                     =(0x9E);

    	static const uint8_t REG_ID						    =(0xD0);
    	static const uint8_t		VAL_ID					=(0x58);

    	static const uint8_t REG_RESET					    =(0xE0);
    	static const uint8_t 	VAL_RESET				    =(0xB6); 	// write it to trigger POR

    	static const uint8_t REG_STATUS					    =(0xF3);
    	static const uint8_t 	MSK_STATUS_MEASURING	    =(1<<3);	// 1 when conversion is running
    	static const uint8_t 	MSK_STATUS_IMUPDATE		    =(1<<0);	// 1 when NVM data is copied to image registers

    	static const uint8_t REG_CTRL_MEAS				    =(0xF4);
    	static const uint8_t 	MSK_CTRL_OSRS_T			    =(B11100000);
    	static const uint8_t 		VAL_CTRL_OSRS_T00	    =(B00000000); // skip measurement
    	static const uint8_t 		VAL_CTRL_OSRS_T01	    =(B00100000); // 1x (no oversampling)
    	static const uint8_t 		VAL_CTRL_OSRS_T02	    =(B01000000); // 2x --> 17bit, 2m�C
    	static const uint8_t 		VAL_CTRL_OSRS_T04	    =(B01100000); // 4x  	(brings no improvement)
    	static const uint8_t 		VAL_CTRL_OSRS_T08	    =(B10000000); // 8x		(brings no improvement)
    	static const uint8_t 		VAL_CTRL_OSRS_T16	    =(B10100000); // 16x	(brings no improvement)
    	static const uint8_t 	MSK_CTRL_OSRS_P			    =(B00011100);
    	static const uint8_t 		VAL_CTRL_OSRS_P00	    =(B00000000); // skip measurement
    	static const uint8_t 		VAL_CTRL_OSRS_P01	    =(B00000100); // 1x (no oversampling)
    	static const uint8_t 		VAL_CTRL_OSRS_P02	    =(B00001000); // 2x
    	static const uint8_t 		VAL_CTRL_OSRS_P04	    =(B00001100); // 4x
    	static const uint8_t 		VAL_CTRL_OSRS_P08	    =(B00010000); // 8x
    	static const uint8_t 		VAL_CTRL_OSRS_P16	    =(B00010100); // 16x --> 20 bit, 0.16 Pa
    	static const uint8_t 	MSK_CTRL_MODE			    =(B00000011);
    	static const uint8_t 		VAL_MODE_SLEEP		    =(B00000000);	// low power
    	static const uint8_t 		VAL_MODE_FORCED		    =(B00000001);	// manual
    	static const uint8_t 		VAL_MODE_NORMAL		    =(B00000011); 	// automatic

    	static const uint8_t REG_CONFIG					    =(0xF5);
    	static const uint8_t 	MSK_CONFIG_T_SB			    =(B11100000);
    	static const uint8_t 		VAL_SB_0000			    =(B00000000);
    	static const uint8_t 		VAL_SB_0062			    =(B00100000);
    	static const uint8_t 		VAL_SB_0125			    =(B01000000);
    	static const uint8_t 		VAL_SB_0250			    =(B01100000);
    	static const uint8_t 		VAL_SB_0500			    =(B10000000);
    	static const uint8_t 		VAL_SB_1000			    =(B10100000);
    	static const uint8_t 		VAL_SB_2000			    =(B11000000);
    	static const uint8_t 		VAL_SB_4000			    =(B11100000);
    	static const uint8_t 	MSK_CONFIG_FILTER		    =(B00011100);
    	static const uint8_t 		VAL_FILTER_00		    =(B00000000);	// full BW
    	static const uint8_t 		VAL_FILTER_02 		    =(B00000100);	// 0.223 * ODR
    	static const uint8_t 		VAL_FILTER_04 		    =(B00001000);	// 0.092 * ODR
    	static const uint8_t 		VAL_FILTER_08 		    =(B00001100);	// 0.042 * ODR
    	static const uint8_t 		VAL_FILTER_16 		    =(B00010000);	// 0.021 * ODR
    	static const uint8_t 	MSK_CONFIG_SPI3W_EN		    =(B00000001);	// 1 = activate SPI-Mode

    	static const uint8_t REG_PRESS_MSB				    =(0xF7);
    	static const uint8_t REG_PRESS_LSB				    =(0xF8);
    	static const uint8_t REG_PRESS_XLSB				    =(0xF9); 	// bit 4-7 usable

    	static const uint8_t REG_TEMP_MSB				    =(0xFA);
    	static const uint8_t REG_TEMP_LSB				    =(0xFB);
    	static const uint8_t REG_TEMP_XLSB				    =(0xFC);	// bit 4-7 usable

    	uint16_t    dig_T1, dig_P1;
    	int16_t     dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    	int32_t     tFine;
		short oversampling, oversampling_t;
		long signed int t_fine;
		char error;

		uint8_t probeAddress(const uint8_t);
  		void    write       (const uint8_t, const uint8_t, const uint8_t *, const uint8_t);
  		void    writeByte   (const uint8_t, const uint8_t, const uint8_t);
  		void    writeCMD    (const uint8_t, const uint8_t);
  		uint8_t readByte    (const uint8_t, const uint8_t);
  		void    read        (const uint8_t, const uint8_t, uint8_t *,     const uint8_t);
  		void    setRegister (const uint8_t, const uint8_t, const uint8_t, const uint8_t);
  		uint8_t getRegister (const uint8_t, const uint8_t, const uint8_t);
		
};

#endif  //bmp280.h
