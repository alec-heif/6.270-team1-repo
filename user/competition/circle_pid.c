//
//  circle_pid.c
//  
//
//  Created by aheifetz on 1/17/13.
//
//

#include <stdio.h>
#include <umain.c>


//port that the gyro is plugged into
#define GYRO_PORT 20
//conversion from gyro magic units to real degrees. never change this unless we get a new gyro
#define GYRO_DEGREE_CONVERSION 1351388 //DO NOT CHANGE THIS EVER OMG
//how long to wait in usetup to calibrate gyro
#define GYRO_CALIB_WAIT_TIME 2500L

//PID constants for maintaining current heading while driving
#define KP_CIRCLE 1.5 //@BROILED5
#define KI_CIRCLE 0
#define KD_CIRCLE 0
