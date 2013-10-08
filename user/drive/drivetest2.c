//
//  drivetest2.c
//  
//
//  Created by aheifetz on 1/11/13.
//
//

#include <stdio.h>
#include <joyos.h>
/**
 *INSTRUCTIONS: First, set the left motor and right motor values in int umain(void) - where the first two comments are.
 *Then, see below instructions.
 *
 *Push Start to start moving forwards at full power, then Stop to stop.
 *Then push Start to start moving backwards at full power, then Stop to stop.
 *Then rotate clockwise 25%. 
 *Then rotate cw 50%.
 *Then rotate cw 75%
 *Then rotate cw 100%.
 *Then rotate counterclockwise 25%.
 *Then rotate ccw 50%.
 *Then rotate ccw 75%
 *Then rotate ccw 100%.
 *Then repeat.
 *
 *To try other motor values, just change the motor 
 **/

static uint8_t const LEFT_MOT_1 = 0;
static uint8_t const LEFT_MOT_2 = 1;
static uint8_t const RIGHT_MOT_1 = 2;
static uint8_t const RIGHT_MOT_2 = 3;
#define SPINNER_MOTOR_PORT_1 4
#define SPINNER_MOTOR_PORT_2 5

int usetup(){
    return 0;
    
}
/*
void drive_straight(int speed){
    set_motors(speed, speed);
}
*/


void set_spinners(int pwr){
    motor_set_vel(SPINNER_MOTOR_PORT_1, pwr);
    motor_set_vel(SPINNER_MOTOR_PORT_2, pwr);
}

void set_spinners_individ(int pwr1, int pwr2){
    motor_set_vel(SPINNER_MOTOR_PORT_1, pwr1);
    motor_set_vel(SPINNER_MOTOR_PORT_2, pwr2);
}

void set_motor_all(int leftPwr1, int leftPwr2, int rightPwr1, int rightPwr2){
    motor_set_vel(LEFT_MOT_1, leftPwr1);
    motor_set_vel(LEFT_MOT_2, leftPwr2);
    motor_set_vel(RIGHT_MOT_1, rightPwr1);
    motor_set_vel(RIGHT_MOT_2, rightPwr2);
}

void run_drive_test(int leftPwr1, int leftPwr2, int rightPwr1, int rightPwr2){
    go_click();
    set_motor_all(leftPwr1, leftPwr2, rightPwr1, rightPwr2);
    while(!stop_press()){
        printf("L1 Current: %d, L2 Current: %d, R1 Current: %d, R2 Current: %d\n", motor_get_current(LEFT_MOT_1), motor_get_current(LEFT_MOT_2), motor_get_current(RIGHT_MOT_1), motor_get_current(RIGHT_MOT_2));
    }
    //stop_motors();
}

void stop_motors(){
    set_motors(0, 0);
}

void set_motors(int leftPwr, int rightPwr){
    motor_set_vel(LEFT_MOT_1, leftPwr);
    motor_set_vel(LEFT_MOT_2, leftPwr);
    motor_set_vel(RIGHT_MOT_1, rightPwr);
    motor_set_vel(RIGHT_MOT_2, rightPwr);
}

/*
void run_mini_drive_test(){
    go_click();
    motor_set_vel(0, 255);
    stop_click();
    stop_motors();
    go_click();
    motor_set_vel(0, 255);
    stop_click();
    go_click()
}
*/
int umain(void){
    while(1){
        go_click();
        set_spinners_individ(225, 0);
        stop_click();
        set_spinners(0);
        go_click();
        set_spinners_individ(0, 225);
        stop_click();
        set_spinners(0);
        go_click();
        set_spinners_individ(225, 225);
        stop_click();
        set_spinners(0);
        /*run_drive_test(64, 64, 64, 64);
        pause(50);
        run_drive_test(128, 128, 128, 128);
        pause(50);
        run_drive_test(196, 196, 196, 196);
        pause(50);
        run_drive_test(-64, -64, -64, -64);
        pause(50);
        run_drive_test(-128, -128, -128, -128);
        pause(50);
        run_drive_test(-196, -196, -196, -196);
        pause(50);*/
    }
}
