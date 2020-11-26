/* 	BMP280.cpp --- modified by Wayne Brenckle
	
	Bosch BMP280 pressure sensor library for the Arduino framework.
	This library uses I2C connection.
	Uses floating-point equations from BMP280 datasheet.  */


#include "flight_controller.h"


uint8_t BMP280::probe(const uint8_t address)
{
    Wire.beginTransmission(address);
    if (Wire.endTransmission(true)==0) return 1; // found something
    else                               return 0; // no response
};

uint8_t BMP280::probeAddress(const uint8_t address)
{
    return probe(address);
};

void BMP280::write(const uint8_t address, const uint8_t register_address, const uint8_t write_value[], const uint8_t length=1)
{
    if (!length) return;
    Wire.beginTransmission(address);
    Wire.write(register_address);
    uint8_t counter;
    counter = 0;
    while (counter < length)
    {
        Wire.write(write_value[counter]);
        counter++;
    }
    Wire.endTransmission(true);
};

void BMP280::writeByte(const uint8_t address, const uint8_t register_address, const uint8_t write_value)
{
    Wire.beginTransmission(address);
    Wire.write(register_address);
    Wire.write(write_value);
    Wire.endTransmission(true);
};

void BMP280::writeCMD(const uint8_t address, const uint8_t cmd)
{
    Wire.beginTransmission(address);
    Wire.write(cmd);
    Wire.endTransmission();
};

void BMP280::read(const uint8_t address, const uint8_t registeraddress, uint8_t buff[], const uint8_t length=1)
{
    Wire.beginTransmission(address); 	// Adress + WRITE (0)
    Wire.write(registeraddress);
    Wire.endTransmission(false); 		// No Stop Condition, for repeated Talk

    if (!length) return;
    Wire.requestFrom(address, length); 	// Address + READ (1)
    uint8_t _i;
    _i=0;
    while(Wire.available())
    {
        buff[_i] = Wire.read();
        _i++;
    }

    Wire.endTransmission(true); 		// Stop Condition
};

uint8_t BMP280::readByte(const uint8_t address, const uint8_t register_address)
{
    uint8_t _readvalue;
    read(address, register_address, &_readvalue, 1);
    return _readvalue;
};

void BMP280::setRegister(const uint8_t address, const uint8_t registeraddress, const uint8_t mask, const uint8_t writevalue)
{
    uint8_t _setting;
    read(address, registeraddress, &_setting, 1 );
    _setting     &= ~mask;
    _setting     |= (writevalue&mask);
    writeByte(address, registeraddress, _setting);
};

uint8_t BMP280::getRegister(const uint8_t address, const uint8_t registeraddress, const uint8_t mask)
{
    uint8_t _setting;
    read(address, registeraddress, &_setting, (uint8_t)1 );
    return (_setting & mask);
};

/**< Enable / Disable the Sensor */
void BMP280::setEnabled(const uint8_t enable)
{
    uint8_t _value;
    if (enable) _value=VAL_MODE_NORMAL;
    else        _value=0;
    setRegister(BMP280_ADDR,REG_CTRL_MEAS, MSK_CTRL_MODE, _value);
};

/**< read Enable / Disable - Status */
uint8_t BMP280::getEnabled(void)
{
    return (3 & readByte(BMP280_ADDR,REG_CTRL_MEAS));
};

/**< do a software reset */
void BMP280::reset(void)
{
    writeByte(BMP280_ADDR,REG_RESET, VAL_RESET);
};

/**<  */
void BMP280::setPressureOversampleRatio(const uint8_t sampleRatio)
{
    uint8_t _value;
    if 		(sampleRatio > 15) 	_value = VAL_CTRL_OSRS_P16;
    else if (sampleRatio > 7)	_value = VAL_CTRL_OSRS_P08;
    else if (sampleRatio > 3)	_value = VAL_CTRL_OSRS_P04;
    else if (sampleRatio > 1)	_value = VAL_CTRL_OSRS_P02;
    else if (sampleRatio > 0)	_value = VAL_CTRL_OSRS_P01;
    else  						_value = VAL_CTRL_OSRS_P00; // disable!!!
    setRegister(BMP280_ADDR,REG_CTRL_MEAS, MSK_CTRL_OSRS_P, _value);
};

void BMP280::setTemperatureOversampleRatio(const uint8_t sampleRatio)
{
    uint8_t _value;
    if 		(sampleRatio > 15) 	_value = VAL_CTRL_OSRS_T16;
    else if (sampleRatio > 7)	_value = VAL_CTRL_OSRS_T08;
    else if (sampleRatio > 3)	_value = VAL_CTRL_OSRS_T04; // more isn't better
    else if (sampleRatio > 1)	_value = VAL_CTRL_OSRS_T02; // 2 should be maximum
    else if (sampleRatio > 0)	_value = VAL_CTRL_OSRS_T01;
    else  						_value = VAL_CTRL_OSRS_T00; // disable!!!
    setRegister(BMP280_ADDR,REG_CTRL_MEAS, MSK_CTRL_OSRS_T, _value);
};

void BMP280::setFilterRatio(const uint8_t filterRatio)
{
    uint8_t _value;
    if 		(filterRatio > 15) 	_value = VAL_FILTER_16;
    else if (filterRatio > 7)	_value = VAL_FILTER_08;
    else if (filterRatio > 3)	_value = VAL_FILTER_04;
    else if (filterRatio > 1)	_value = VAL_FILTER_02;
    else  						_value = VAL_FILTER_00; // disable!!!
    setRegister(BMP280_ADDR,REG_CONFIG, MSK_CONFIG_FILTER, _value);
};

void BMP280::setStandby(const uint16_t ms)
{
    uint8_t _value;
    if 		(ms > 3000) _value = VAL_SB_4000;
    else if (ms > 1500)	_value = VAL_SB_2000;
    else if (ms >  750)	_value = VAL_SB_1000;
    else if (ms >  350)	_value = VAL_SB_0500;
    else if (ms >  180)	_value = VAL_SB_0250;
    else if (ms >   90)	_value = VAL_SB_0125;
    else if (ms >   31)	_value = VAL_SB_0062;
    else  				_value = VAL_SB_0000; // disable!!!
    setRegister(BMP280_ADDR,REG_CONFIG, MSK_CONFIG_T_SB, _value);
}

/**< initialize */
uint8_t BMP280::initialize(void)
{
    if (probe(BMP280_ADDR)==0) return 0;

    reset();
    delay(4);
	// setup gaming mode 4xPressure OS, 1xTemp OS, Filter coef = 16, standby = 0 (full BW)
    setPressureOversampleRatio(4);
    setTemperatureOversampleRatio(1);
    setFilterRatio();   // any filter would drastically increasing measurement time
    setStandby();       // same is true for standby time obviously.  Current measure time is = 12ms 
	// make sure we are not in SPI mode
    setRegister(BMP280_ADDR, REG_CONFIG, MSK_CONFIG_SPI3W_EN, 0);

    readTrimming();
    setEnabled(1);
    return 1;
};

void BMP280::readTrimming(void)
{
    uint8_t _value[2];
    read(BMP280_ADDR, REG_DIG_T1, _value, 2);
    dig_T1 = uint16_t((uint16_t(_value[1]<<8)) | _value[0]);
    read(BMP280_ADDR, REG_DIG_T2, _value, 2);
    dig_T2 = int16_t((_value[1]<<8) | _value[0]);
    read(BMP280_ADDR, REG_DIG_T3, _value, 2);
    dig_T3 = int16_t((_value[1]<<8) | _value[0]);

    read(BMP280_ADDR, REG_DIG_P1, _value, 2);
    dig_P1 = uint16_t((uint16_t(_value[1]<<8)) | _value[0]);
    read(BMP280_ADDR, REG_DIG_P2, _value, 2);
    dig_P2 = int16_t((_value[1]<<8) | _value[0]);
    read(BMP280_ADDR, REG_DIG_P3, _value, 2);
    dig_P3 = int16_t((_value[1]<<8) | _value[0]);

    read(BMP280_ADDR, REG_DIG_P4, _value, 2);
    dig_P4 = int16_t((_value[1]<<8) | _value[0]);
    read(BMP280_ADDR, REG_DIG_P5, _value, 2);
    dig_P5 = int16_t((_value[1]<<8) | _value[0]);
    read(BMP280_ADDR, REG_DIG_P6, _value, 2);
    dig_P6 = int16_t((_value[1]<<8) | _value[0]);

    read(BMP280_ADDR, REG_DIG_P7, _value, 2);
    dig_P7 = int16_t((_value[1]<<8) | _value[0]);
    read(BMP280_ADDR, REG_DIG_P8, _value, 2);
    dig_P8 = int16_t((_value[1]<<8) | _value[0]);
    read(BMP280_ADDR, REG_DIG_P9, _value, 2);
    dig_P9 = int16_t((_value[1]<<8) | _value[0]);
};


/**< disables continious Mode! (enable(1)) */
void BMP280::triggerMeasurement(void)
{
    setRegister(BMP280_ADDR,REG_CTRL_MEAS, MSK_CTRL_MODE, VAL_MODE_FORCED);
};

uint8_t BMP280::checkMeasurement(void)
{
    return !(MSK_STATUS_MEASURING & readByte(BMP280_ADDR, REG_STATUS));
};

/**<  if you started a measurement and want to actively wait for it to finish */
uint8_t BMP280::awaitMeasurement(void)
{
    uint8_t _counter = 0;
    while(checkMeasurement()==0)
    {
        if(++_counter > 250) return 0; //Error out after max of 500ms for a read
        delay(2);
    }
    return 1; // Measurement finished
};

void BMP280::getCalcValues(uint32_t raw_temp, uint32_t raw_pressure)
{
	
	/*volatile uint32_t temp[3];
	temp[2]=rx_buff[3];
	temp[1]=rx_buff[4];
	temp[0]=rx_buff[5];
	temperature_raw = (temp[2]<<12)+(temp[1]<<4)+(temp[0]>>4);

	temp[2]=rx_buff[0];
	temp[1]=rx_buff[1];
	temp[0]=rx_buff[2];
	pressure_raw=(temp[2]<<12)+(temp[1]<<4)+(temp[0]>>4);*/

	double var1, var2;
	var1=(((double)raw_temp)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
	var2=((((double)raw_temp)/131072.0-((double)dig_T1)/8192.0)*(((double)raw_temp)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
	double t_fine = (int32_t)(var1+var2);
	volatile float T = (var1+var2)/5120.0;

	var1=((double)t_fine/2.0)-64000.0;
	var2=var1*var1*((double)dig_P6)/32768.0;
	var2=var2+var1*((double)dig_P5)*2.0;
	var2=(var2/4.0)+(((double)dig_P4)*65536.0);
	var1=(((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((double)dig_P1);
	volatile double p=1048576.0-(double)raw_pressure;
	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((double)dig_P9)*p*p/2147483648.0;
	var2=p*((double)dig_P8)/32768.0;
	p=p+(var1+var2+((double)dig_P7))/16.0;

	calc_temperature = T;
	calc_pressure = p;
	calc_altitude = 44330.0f*(1-powf(calc_pressure/101325.0f,1.0f/5.255f));
	//or ...  altitude=((powf(101325.0/pressure, 1/5.257f)-1)*(temperature+273.15f))/0.0065f;
}

void BMP280::getRawPressure(uint32_t &pascal){
	
	uint8_t value[3];
    read(BMP280_ADDR, REG_PRESS_MSB, value, 3);
	pascal = ((value[0] *256.0) + value[1] + (value[3]/256.0)) * 16;	//20bit UP
}

/**<  gives airpressure in Pascal */
void BMP280::getPressure(uint32_t& pascal)
{
    uint8_t _value[3];
    read(BMP280_ADDR, REG_PRESS_MSB, _value, 3);

    int32_t var1, var2, adc;
    adc     = (uint32_t( uint16_t(_value[0] << 8) | _value[1])<<4) | (_value[2]>>4);
    var1 = (((int32_t)tFine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0) return; // avoid exception caused by division by zero

    pascal = (((uint32_t)(((int32_t)1048576)-adc)-(var2>>12)))*3125;
    if (pascal < 0x80000000)    pascal = (pascal << 1) / ((uint32_t)var1);
    else                        pascal = (pascal / (uint32_t)var1) * 2;

    var1 = (((int32_t)dig_P9) * ((int32_t)(((pascal>>3) * (pascal>>3))>>13)))>>12;
    var2 = (((int32_t)(pascal>>2)) * ((int32_t)dig_P8))>>13;
    pascal = (uint32_t)((int32_t)pascal + ((var1 + var2 + dig_P7) >> 4));
};

void BMP280::getPressure(float& pascal)
{
    uint32_t iPascal;
    getPressure(iPascal);
    pascal = float(iPascal);
}

/**<  gives pressure-values */
void BMP280::getMeasurement(float& pascal)
{
    getPressure(pascal);
};

/**<  gives the number of meters above sea level */
void BMP280::getAltitude(float& meter)
{
    uint32_t iPascal;
    getPressure(iPascal);

    meter = 44330.0*(1-pow(float(iPascal)/101325.0,1.0/5.255));

};

void BMP280::getRawTemperature(uint32_t &celsius){
	
	uint8_t value[3];
    read(BMP280_ADDR, REG_TEMP_MSB, value, 3);
	//int32_t adc = (uint32_t( uint16_t(value[0] << 8) | value[1])<<4) | (value[2]>>4);
    
	//float factor = pow(2, 4);
	//uT = (( (value[0] *256.0) + value[1] + (value[3]/256.0))) * factor ;	//20bit UT
	celsius = ((value[0] *256.0) + value[1] + (value[3]/256.0)) * 16 ;	//20bit UT
}

/**<  gives temperature in degree celsius */
void BMP280::getTemperature(int32_t& millicelsius)
{
    uint8_t value[3];
    read(BMP280_ADDR, REG_TEMP_MSB, value, 3);

    int32_t var1, var2, adc;
    adc     = (uint32_t( uint16_t(value[0] << 8) | value[1])<<4) | (value[2]>>4);
    var1    = ((((adc>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2    = (((((adc>>4) - ((int32_t)dig_T1)) * ((adc>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3))>>14;
    tFine   = var1 + var2;
    millicelsius = (tFine * 50 + 1280) >> 8;

};

void BMP280::getTemperature(float &celsius)
{
    int32_t iTemperature;
    getTemperature(iTemperature);
    celsius = float(iTemperature) / 1000;
};


void BMP280::getFlightData(DRONE &myDrone) 
{ 
    //Every time this function is called the barometer_counter variable is incremented. 
    //This way a specific action is executed at the correct moment. This is needed ?????? 
    //because requesting data from the BMP280 takes around 12ms to complete????? 
    //TODO----- Verify time to take measurement
	static uint8_t barometer_counter = 0, temperature_counter = 0, average_temperature_mem_location = 0;
	barometer_counter++;

	if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1.
		if (temperature_counter == 0) {                                             //And the temperature counter is 0.
			//Get temperature data from BMP280
			// Store the temperature in a 5 location rotating memory to prevent temperature spikes.
			raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
			getRawTemperature(raw_temperature_rotating_memory[average_temperature_mem_location]);
			raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
			average_temperature_mem_location++;
			if (average_temperature_mem_location == 5) average_temperature_mem_location = 0;
			raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
		}
		else {
			//Get pressure data from BMP280
			getRawPressure(raw_pressure);		
		}

    	temperature_counter ++;                                                     //Increase the temperature_counter variable.
    	if (temperature_counter == 20) {                                            //When the temperature counter equals 20.
    		temperature_counter = 0;                                                  //Reset the temperature_counter variable.
    	}
    }
	getCalcValues(raw_temperature, raw_pressure);
	if (barometer_counter == 2) {                                                                              //If the barometer_counter variable equals 2.
	  //To get a smoother pressure value we will use a 20 location rotating memory.
	  pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
	  pressure_rotating_mem[pressure_rotating_mem_location] = calc_pressure;                                    //Calculate the new change between the actual pressure and the previous measurement.
	  pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
	  pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
	  if (pressure_rotating_mem_location == 20) pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
	  pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.	
	  //To get better results we will use a complementary fillter that can be adjusted by the fast average.
	  pressure_slow = pressure_slow * (float)0.985 + pressure_fast * (float)0.015;
	  pressure_diff = pressure_slow - pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
	  if (pressure_diff > 8)pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
	  if (pressure_diff < -8)pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
	  //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
	  if (pressure_diff > 1 || pressure_diff < -1)pressure_slow -= pressure_diff / 6.0;
	  actual_pressure = pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
	}

	if (barometer_counter == 3) {                                                                               //When the barometer counter is 3	
	  barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
	  //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
	  //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
	  if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
	  parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
	  parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
	  parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
	  pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
	  parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
		if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.	
		if (myDrone.flight_mode >= 2 && takeoff_detected == 1) {                                                          //If the quadcopter is in altitude mode and flying.
			if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
			//When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
			//will indicate if the altitude of the quadcopter is changed by the pilot.
			manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
			myDrone.manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
			if (channel[CH_3] > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
				manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
				pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
				myDrone.manual_throttle = (channel[CH_3] - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
			}
			if (channel[CH_3] < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
				manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
				pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
				myDrone.manual_throttle = (channel[CH_3] - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
			}	
			//Calculate the PID output of the altitude hold.
			pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
			float pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.	
			//To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
			//The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
			pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
			if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
				pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
				if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
			}	
			//In the following section the I-output is calculated. It's an accumulation of errors over time.
			//The time factor is removed as the program loop runs at 250Hz.
			pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
			if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
			else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
			//In the following line the PID-output is calculated.
			//P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
			//I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
			//D = pid_d_gain_altitude * parachute_throttle.
			pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
			//To prevent extreme PID-output the output must be limited.
			if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
			else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
		}	
		//If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
		else if (myDrone.flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
			pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
			pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
			pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
			myDrone.manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
			manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
		}
	}
}

