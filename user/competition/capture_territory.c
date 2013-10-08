//
//  capture_territory.c
//  
//
//  Created by aheifetz on 1/19/13.
//
//

#include <stdio.h>
#include <umain.c>

#define SPINNER_SPEED 165
#define SPINNER_MOTOR_A 4
#define SPINNER_MOTOR_B 5

void spin_spinner(int spin_speed){
    motor_set_vel(SPINNER_MOTOR_A, spin_speed);
    motor_set_vel(SPINNER_MOTOR_B, spin_speed);
}

void stop_spinner(){
    motor_set_vel(SPINNER_MOTOR_A, 0);
    motor_set_vel(SPINNER_MOTOR_A, 0);
}


