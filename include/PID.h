#ifndef _PID_H_
#define _PID_H_

#include "flight_controller.h"

/*-----------------------------/
/ PID gain and limit settings  /
/-----------------------------*/

//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
extern float battery_compensation;

extern float pid_p_gain_altitude;           //Gain setting for the altitude P-controller (default = 1.4).
extern float pid_i_gain_altitude;           //Gain setting for the altitude I-controller (default = 0.2).
extern float pid_d_gain_altitude;          //Gain setting for the altitude D-controller (default = 0.75).
extern int pid_max_altitude;                //Maximum output of the PID-controller (+/-).


class PID_CONTROL{

     public:
          PID_CONTROL();
          void calculate_pid(void);

          float pid_error_temp;
          float pid_i_mem_roll, pid_pitch_roll, pid_output_roll, pid_last_roll_d_error;
          float pid_i_mem_pitch, pid_output_pitch, pid_last_pitch_d_error;
          float pid_i_mem_yaw, pid_output_yaw, pid_last_yaw_d_error;
          int32_t roll_level_adjust, pitch_level_adjust; 

}
#endif // _PID_H_