//
//  vps_test_1.c
//  
//
//  Created by aheifetz on 1/11/13.
//
//

#include <stdio.h>
#include <joyos.h>
#include <math.h>

/**
 *INSTRUCTIONS:
 */
extern volatile uint8_t robot_id;

int usetup(void){
    robot_id = 1;
    return 0;
}

static uint8_t const LEFT_MOT_1 = 0;
static uint8_t const LEFT_MOT_2 = 1;
static uint8_t const RIGHT_MOT_1 = 2;
static uint8_t const RIGHT_MOT_2 = 3;

static float const PIXELS_TO_INCHES = 96/4096; //divide by # of pixels, multiply by

//static uint8_t go_pressed = 0; //c doesn't have a bool type, so 0 = false and 1 = true.
//static uint8_t stop_pressed = 0;

static int const TARGETS[3][2] = {{0, 0},{800, 800},{-800, -800}};

float get_distance(int16_t x_dist, int16_t y_dist){
    float dist_squared = ((((float)x_dist * (float)x_dist)) + (((float)y_dist * (float)y_dist)));
    float returned_value = sqrt(dist_squared);
    return returned_value;
}

void update(){
    copy_objects();
    int current_x = game.coords[0].x;// * PIXELS_TO_INCHES;
    int current_y = game.coords[0].y;// * PIXELS_TO_INCHES;
    float current_angle = (((float)game.coords[0].theta) * 180.0) / 2048;
    
    int16_t target_x = TARGETS[0][0];// * PIXELS_TO_INCHES;
    int16_t target_y = TARGETS[0][1];// * PIXELS_TO_INCHES;
    
    float x_distance = (target_x - current_x);
    float y_distance = (target_y - current_y);
    float target_distance = get_distance(x_distance, y_distance);
    
    float target_angle = (float)atan2(y_distance, x_distance)*(180/M_PI);
    float angle_difference = target_angle - current_angle;
    if(angle_difference < -180){
        angle_difference += 360;
    }
    if(angle_difference > 180){
        angle_difference -=360;
    }
    
    
    printf("\nnew version current_x: %04d, current_y: %04d, current_angle: %.2f, target_distance: %.2f, target_angle: %.2f angle_distance: %.2f", current_x, current_y, current_angle, target_distance, target_angle, angle_difference);
    
}

 /*
void drive_straight(int speed){
    set_motors(speed, speed);
}

void run_drive_test(int left_pwr, int right_pwr){
    go_click();
    set_motors(left_pwr, right_pwr);
    stop_click();
    stop_motors();
}

void stop_motors(){
    set_motors(0, 0);
}

void set_motors(int leftPwr, int rightPwr){
    motor_set_vel(LEFT_MOT_1, leftPwr);
    motor_set_vel(LEFT_MOT_2, leftPwr);
    motor_set_vel(RIGHT_MOT_1, rightPwr);
    motor_set_vel(RIGHT_MOT_2, rightPwr);
}*/

void umain(void){
    while(1){
        update();
        pause(500);
    }
}
