//
//  navigation_with_drive_test.c
//  
//
//  Created by aheifetz on 1/15/13.
//
//


#include <stdio.h>
#include <joyos.h>
#include <math.h>
#include <lib/pid.h>
#include <gyro.h>

extern volatile uint8_t robot_id; //required for using vps

//Defines a point on the field
typedef struct location{ 
    int x;
    int y;
} Location;

//defines general information about the current state of the game
typedef struct field_state{ 
    Location curr_loc; //obvious
    Location target_loc; //where we're moving to
    float curr_angle; //current orientation
    float target_dist; //distance to target_loc
    float target_angle; //angle that the robot must be facing to drive straight towards target_loc (goes from 0 to 360, 0 is at pos-x)
    float angle_difference; //difference between current angle and target angle (from -180 to 180)
    uint8_t stage; //whether we're at the first instant of the game, spinning, or driving toward a target, 
    uint8_t substage; //while at DRIVE_STAGE, whether we're accelerating, at max speed, decelerating, or stopped
    uint32_t curr_time; //current time
    uint32_t stored_time; //time at which the last "event" happened. an "event" is any change in stage or substage.
} Field_State;

//motor slots on happyboard
#define LEFT_MOT_1 0
#define LEFT_MOT_2 1
#define RIGHT_MOT_1 2
#define RIGHT_MOT_2 3

//for clarity
#define FALSE 0 
#define TRUE 1

//for converting vps pixels into real world units. 4096 pixels = 96 inches
#define PIXELS_TO_INCHES 96/4096 
#define INCHES_TO_PIXELS 4096/96

//stages
#define SETUP_STAGE 0
#define SPINNING_STAGE 1 //rotating in place to face target
#define DRIVING_STAGE 2 //driving towards target

//used as a parameter for rotate() method
#define CLOCKWISE -1
#define COUNTER_CLOCKWISE 1
#define STOP 0

#define DO_WE_WAIT_TO_START_DRIVING 0 //@BROILED1
#define HOW_LONG_DO_WE_WAIT_TO_START_DRIVING 0 //@BROILED2

//various substages of DRIVE_STAGE
#define DRIVE_ACCELERATE 0
#define DRIVE_DRIVE 1
#define DRIVE_DECELERATE 2
#define DRIVE_STOP 3

//miliseconds to accelerate
#define DRIVE_ACCELERATE_TIME 100 

//miliseconds to decelerate
#define DRIVE_DECELERATE_TIME 100
//distance before target to start decelerating
#define DRIVE_DECELERATE_DISTANCE 2

//PID constants for rotating in place
#define KP_SPIN 3
#define KI_SPIN 0
#define KD_SPIN 0

//PID constants for maintaining current heading while driving
#define KP_DRIVE 1.5 //@BROILED5
#define KI_DRIVE 0
#define KD_DRIVE 0

//max number of degrees away from target heading that we need to be to start driving toward target
#define ROTATION_ACCEPTABLE_ERROR 10 //@BROILED4

//max velocity of motors while spinning in place
#define MAX_SPIN_SPEED 48 //@BROILED3
//max velocity of motors while driving toward target
#define MAX_DRIVE_SPEED 128 //@BROILED6

//port that the gyro is plugged into
#define GYRO_PORT 20
//conversion from gyro magic units to real degrees. never change this unless we get a new gyro
#define GYRO_DEGREE_CONVERSION 1351388 //DO NOT CHANGE THIS EVER OMG
//how long to wait in usetup to calibrate gyro
#define GYRO_CALIB_WAIT_TIME 2500L

//declared here so used as a global
Field_State field_state;

int reset = 0;

//provids the desired velocity at a given time while driving
//ugh i HATE to have to make this a global variable when it's only used in one place but the PID structure requires it...annoying.
float target_vel; 

//which location to target from TARGETS (used for testing)
int TARGETS[3][2] = {{0, 0},{800, 800},{-800, -800}};
uint8_t curr_target_index = 0;

//set up gyroscope and tell the vps what robot we are.
//runs before umain and a few seconds before the start of the competition
int usetup(void){
    robot_id = 1; //team 1
    //gyro_init(GYRO_PORT, GYRO_DEGREE_CONVERSION, GYRO_CALIB_WAIT_TIME);
    return 0;
}

float get_sign_f(float num){
    if(num < 0.0){
        return ((float)-1.0);
    }
    if(num > 0.0){
        return ((float)1.0);
    }
    return ((float)0.0);
}

//obvious
float pythagorean(int16_t x_dist, int16_t y_dist){
    float dist_squared = ((((float)x_dist * (float)x_dist)) + (((float)y_dist * (float)y_dist)));
    float returned_value = sqrt(dist_squared);
    return returned_value;
}

//easy way to make sure two motors on one wheel aren't given different powers
void set_motors(int leftPwr, int rightPwr){
    motor_set_vel(LEFT_MOT_1, leftPwr);
    motor_set_vel(LEFT_MOT_2, leftPwr);
    motor_set_vel(RIGHT_MOT_1, rightPwr);
    motor_set_vel(RIGHT_MOT_2, rightPwr);
}

//obvious
void stop_motors(){
    set_motors(0, 0);
}

//rot_pwr = positive power on one wheel, negative on the other
//direction is for clockwise or counterclockwise
void rotate(int rot_pwr, int direction){
    rot_pwr = abs(rot_pwr);
    if(direction == CLOCKWISE)
       set_motors(rot_pwr, -rot_pwr);
    if(direction == COUNTER_CLOCKWISE)
        set_motors(-rot_pwr, rot_pwr);
    if(direction == STOP)
        stop_motors();
}

//takes values on one scale (i.e. from 0 to 100) and converts them to a new scale (i.e. from 0 to 255).
float scale_values(float vel, float old_scale, float new_scale){
    vel = vel * new_scale / old_scale;
    return vel;
}

//make sure velocity is inputted properly to motor_set_vel. also sets velocities to within the max speeds
int format_velocity(float vel){
    switch(field_state.stage){
        case SPINNING_STAGE:
            if(fabs(vel) > MAX_SPIN_SPEED)
                vel = MAX_SPIN_SPEED * get_sign_f(vel);
            return (int)vel;
        case DRIVING_STAGE:
            if(fabs(vel) > MAX_DRIVE_SPEED)
                vel = MAX_DRIVE_SPEED * get_sign_f(vel);
            return (int)vel;
    }
}

//returns the sign of an int (pos. or neg.)
int get_sign(int num){
    if(num < 0){
        return -1;
    }
    if(num > 0){
        return 1;
    }
    return 0;
}


//gets info from vps and updates field_state
void update_field(){
    copy_objects();
    int current_x = game.coords[0].x;
    int current_y = game.coords[0].y;
    field_state.curr_loc.x = current_x;
    field_state.curr_loc.y = current_y;
    field_state.curr_angle = (((float)game.coords[0].theta) * 180.0) / 2048;
    
    int target_x = game.coords[1].x;
    int target_y = game.coords[1].y;

    if(field_state.stage == DRIVING_STAGE && target_x != field_state.target_loc.x && target_y != field_state.target_loc.y){
        //if we've finished decelerating, start rotating towards new target
        if(field_state.substage == DRIVE_STOP){
            printf("New Target acquired! curr_x=%d, curr_y=%d, curr_angle=%f, old_target_x=%d, old_target_y=%d, new_target_x=%d, new_target_y=%d\n", field_state.curr_loc.x, field_state.curr_loc.y, field_state.curr_angle, field_state.target_loc.x, field_state.target_loc.y, target_x, target_y);
            field_state.target_loc.x = target_x;
            field_state.target_loc.y = target_y;
        }
        //if we're still driving, start decelerating
        if(field_state.substage == DRIVE_DRIVE){
            field_state.substage = DRIVE_STOP;
            field_state.target_loc.x = target_x;
            field_state.target_loc.y = target_y;
        }
    }
    //in first iteration of game, initialize these to make sure we don't try to check their values when they don't exist
    if(field_state.stage == SETUP_STAGE){
        field_state.target_loc.x = target_x;
        field_state.target_loc.y = target_y;
        field_state.substage = DRIVE_STOP;
    }
    
    float x_distance = (field_state.target_loc.x - current_x);
    float y_distance = (field_state.target_loc.y - current_y);
    
    field_state.target_dist = pythagorean(x_distance, y_distance);
    field_state.target_angle = (float)atan2(y_distance, x_distance)*(180/M_PI);
    field_state.curr_time = get_time();
    //int temp = frob_read();
    //KP_DRIVE = scale_values((float)temp, 1023.0, 4.0);
}

//input function for rotation PID, basically just returns angle_difference
float update_angle_error(){
    field_state.angle_difference = field_state.target_angle - field_state.curr_angle;
    if(field_state.angle_difference < -180){
        field_state.angle_difference += 360;
    }
    if(field_state.angle_difference > 180){
        field_state.angle_difference -=360;
    }
    return field_state.angle_difference;
}

//output function for rotation PID, basically just sets rotate() to a new value
void update_rotation(float pwr_i){
    int pwr = format_velocity(pwr_i);
    int direction = (int)(get_sign_f(field_state.angle_difference));
    rotate(pwr, direction);
}

//runs during drive pid
void update_velocities(float pwr_change_t){
    update_angle_error();
    int pwr_change = format_velocity(fabs(pwr_change_t));
    int temp_target_vel = (int)(fabs(target_vel));
    if(temp_target_vel == 0){
        set_motors(0,0);
        return;
    }
    if(field_state.angle_difference < 0){
        //printf("lm: %d, rm: %d\n", temp_target_vel, temp_target_vel - pwr_change);
        set_motors(temp_target_vel, temp_target_vel - pwr_change);
    }
    else{
        set_motors(temp_target_vel - pwr_change, temp_target_vel);
    }
}

//start accelerating
void start_drive_accelerate(){
    field_state.stored_time = get_time();
    field_state.stage = DRIVING_STAGE;
    field_state.substage = DRIVE_ACCELERATE;
    gyro_set_degrees(field_state.curr_angle);
}

//start driving with constant speed
void start_drive_drive(){
    field_state.stage = DRIVING_STAGE;
    field_state.stored_time = get_time();
    field_state.substage = DRIVE_DRIVE;
    //gyro_set_degrees(field_state.curr_angle);
}

//start decelerating
void start_drive_decelerate(){
    field_state.stored_time = get_time();
    field_state.substage = DRIVE_DECELERATE;
    gyro_set_degrees(field_state.curr_angle);
}

//finish decelerating
void start_drive_stop(){
    set_motors(0,0);
    pause(100);
    field_state.stored_time = get_time();
    field_state.substage = DRIVE_STOP;
    field_state.stage = SPINNING_STAGE;
    printf("Stopped robot at x=%d, y=%d, angle=%f!\n", field_state.curr_loc.x, field_state.curr_loc.y, field_state.curr_angle);
    //gyro_set_degrees(field_state.curr_angle);
    //field_state.stage = SPINNING_STAGE;
}

//main function. runs when we press go.
void umain(void){
    //pause(5000);
    field_state.stage = SETUP_STAGE; //runs one frame of update_field() at SETUP_STAGE to initialize variables, then starts turning towards first target
    update_field();
    field_state.stage = SPINNING_STAGE;
    //uint8_t spin_pid_on = FALSE; //bool used for initialization. c doesn't have bools so I use uint8_t's (FALSE == 0, TRUE == 1)
    uint8_t drive_pid_on = FALSE;
    //initialize PID for spinning in place
    //struct pid_controller spinning_PID;
    //init_pid(&spinning_PID, KP_SPIN, KI_SPIN, KD_SPIN, &update_angle_error, &update_rotation);
    
    //initialize PID for maintaining heading while driving
    struct pid_controller driving_PID;
    init_pid(&driving_PID, KP_DRIVE, KI_DRIVE, KD_DRIVE, &update_angle_error, &update_velocities);
    
    while(1){
        update_field();
        if(field_state.stage == SPINNING_STAGE){
            //printf("Spin, clx: %d, cly: %d, ca: %f, ta: %f, error: %f\n", field_state.curr_loc.x, field_state.curr_loc.y, field_state.curr_angle, field_state.target_angle, field_state.angle_difference);
            //if(spin_pid_on == FALSE){ //prepare to start the spin_pid process
                //spinning_PID.goal = 0; //we want angle_difference to be 0, i.e. the robot's orientation = desired orientation
                //spinning_PID.enabled = true;
                //spin_pid_on = TRUE; //don't run this initial setup again until the next time we start spinning
                //gyro_set_degrees(field_state.curr_angle); //tell the gyro that we are currently facing where the vps tells us we're facing
            //}
            //updates PID until robot facing within an acceptable error of desired orientation
            update_angle_error();
            if(fabs(field_state.angle_difference) >= ROTATION_ACCEPTABLE_ERROR){
                int direction;
                if(field_state.angle_difference < 0)
                    direction = CLOCKWISE;
                else{
                    direction = COUNTER_CLOCKWISE;
                }
                rotate(MAX_SPIN_SPEED, direction);
                //printf("Current Angle: %f, Target Angle: %f, Difference: %f, Direction: %d\n", field_state.curr_angle, field_state.target_angle, field_state.angle_difference, direction);
                continue;
            }
            else { //transition to driving toward the target.
                
                //spinning_PID.enabled = false;
                //spin_pid_on = FALSE;
                printf("Stopped at A: %f, Ta: %f, E: %f", field_state.curr_angle, field_state.target_angle, field_state.angle_difference);
                set_motors(0,0);
                pause(100);
                start_drive_drive();
                continue;
            }
        }
        
        if(field_state.stage == DRIVING_STAGE){
            //printf("Driving...\n");
            switch(field_state.substage){
                /*case DRIVE_ACCELERATE:
                    if(field_state.curr_time - field_state.stored_time > DRIVE_ACCELERATE_TIME){
                        target_vel = MAX_DRIVE_SPEED;
                        start_drive_drive();
                        break;
                    }
                    target_vel = format_velocity(scale_values(field_state.curr_time - field_state.stored_time, DRIVE_ACCELERATE_TIME, MAX_DRIVE_SPEED));
                    break;*/
                case DRIVE_DRIVE:
                    /*if((field_state.target_dist * PIXELS_TO_INCHES) < DRIVE_DECELERATE_DISTANCE){ 
                        target_vel = 0;//format_velocity(scale_values(field_state.target_dist, DRIVE_DECELERATE_DISTANCE, MAX_DRIVE_SPEED));
                        field_state.stored_time = get_time();
                        field_state.substage = DRIVE_STOP;
                        field_state.
                        set_motors(0,0);
                        gyro_set_degrees(field_state.curr_angle);
                        break;
                    }*/
                    target_vel = MAX_DRIVE_SPEED;
                    break;
                /*case DRIVE_DECELERATE:
                    if(field_state.curr_time - field_state.stored_time > DRIVE_DECELERATE_TIME){
                        stop_motors();
                        start_drive_stop();
                        driving_PID.enabled = false;
                        drive_pid_on = false;
                        continue;
                    }
                    target_vel = format_velocity(scale_values(DRIVE_DECELERATE_TIME - (field_state.curr_time - field_state.stored_time), DRIVE_DECELERATE_TIME, MAX_DRIVE_SPEED));
                    break; */
                case DRIVE_STOP: //should never actually make it here but just in case
                    start_drive_stop();
                    field_state.stored_time = get_time();
                    driving_PID.enabled = FALSE;
                    drive_pid_on = FALSE;
                    continue;
                    //gyro_set_degrees(field_state.curr_angle);
            }
            if(drive_pid_on == FALSE){
                driving_PID.goal = 0;
                driving_PID.enabled = TRUE;
                drive_pid_on = TRUE;
                //gyro_set_degrees(field_state.curr_angle);
            }
            update_angle_error();
            update_pid(&driving_PID);
            //printf("Drive, clx: %d, cly: %d, ca: %f, ta: %f, error: %f\n, ", field_state.curr_loc.x, field_state.curr_loc.y, field_state.curr_angle, field_state.target_angle, field_state.angle_difference);
        }
    }
}
















