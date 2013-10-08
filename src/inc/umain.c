//
//  navigation_with_drive_test.c
//  
//
//  Created by aheifetz on 1/16/13.
//
//

#include <stdio.h>
#include <joyos.h>
#include <math.h>
#include <lib/pid.h>
#include <gyro.h>
#include <lib/log.h>
//#include <umain.c>
//#include <in_field.c>

//extern volatile uint8_t robot_id; //required for using vps

//Defines a point on the field
typedef struct location{
    int x;
    int y;
} Location;

//defines general information about the current state of the game
typedef struct field_state{ 
    Location curr_loc; //obvious
    Location opponent_loc;
    Location target_loc; //where we're moving to
    Location target_loc_waypoint;
    Location target_loc_plane; //location of the end loc of a vector from the center through the target_loc to the edge of the field.
    Location curr_loc_plus_delta;
    Location start_drive_loc;
    float curr_angle; //current orientation
    float target_dist; //distance to target_loc
    float target_angle; //angle that the robot must be facing to drive straight towards target_loc (goes from 0 to 360, 0 is at pos-x)
    float angle_difference; //difference between current angle and target angle (from -180 to 180)
    uint8_t sextant;
    int8_t color;
    float distance_increment;
    uint8_t stage; //whether we're at the first instant of the game, spinning, or driving toward a target, 
    uint8_t substage; //while at DRIVE_STAGE, whether we're accelerating, at max speed, decelerating, or stopped
    uint32_t curr_time; //current time
    uint32_t stored_time; //time at which the last "event" happened. an "event" is any change in stage or substage.
    uint32_t start_drive_time;
    uint32_t start_dump_time;
    struct pid_controller circle_PID;
    int drive_direction;
    int score;
    uint8_t pid_enabled;
    uint32_t start_time;
    uint32_t update_time;
    uint8_t balls_held;
    uint8_t we_want_to_dump;
    uint32_t encoder_value;
    uint8_t last_substage;
    uint32_t last_encoder_update;
    uint8_t robot_stopped;
    uint8_t tries;
    
    } Field_State;


int bisecting_points[12];
int territory_pivot_points[12];
int lever_pivot_points[12];

//motor slots on happyboard
#define LEFT_MOT_1 0
#define LEFT_MOT_2 1
#define RIGHT_MOT_1 2
#define RIGHT_MOT_2 3
#define SPINNER_MOTOR_PORT_1 4
#define SPINNER_MOTOR_PORT_2 5

#define ENCODER_UPDATE_TIME 200


//multiply coordinates by 114.5/126.5
#define VPS_RATIO 0.9051 

#define BLUE 1
#define RED -1
//for clarity
#define FALSE 0 
#define TRUE 1

#define US 1
#define OPPONENT 1

#define CAPTURE_NOT_MINE 20
#define MINE_NOT_CAPTURE 21
#define CAPTURE_AND_MINE 22
#define LEAVE 23

//for converting vps pixels into real world units. 4096 pixels = 96*2/sqrt(3) inches
#define PIXELS_TO_INCHES 110.851/4096 .0
#define INCHES_TO_PIXELS 4096.0/110.851

#define ROBOT_WIDTH 406.5

//stages
#define SETUP_STAGE 0
#define SPINNING_STAGE 100 //rotating in place to face target
#define DRIVING_STAGE 101 //driving towards target

//used as a parameter for rotate() method
#define CLOCKWISE -1
#define COUNTER_CLOCKWISE 1
#define STOP 0

#define RUN_CIRCLE_AT_START 1

#define DO_WE_WAIT_TO_START_DRIVING 0 //@BROILED1
#define HOW_LONG_DO_WE_WAIT_TO_START_DRIVING 0 //@BROILED2

//various substages of DRIVE_STAGE
#define DRIVE_ACCELERATE 0
#define DRIVE_DRIVE 1
#define DRIVE_DECELERATE 2
#define DRIVE_STOP 3

#define FREEWHEEL_ENCODER_PORT 24
#define CLICKS_PER_INCH 1.61

//miliseconds to accelerate
#define DRIVE_ACCELERATE_TIME 100
#define APPROACH_ACCELERATE_TIME 50
#define BACKUP_ACCELERATE_TIME 50

#define FORWARD 1
#define BACKWARD -1

//miliseconds to decelerate
#define DRIVE_DECELERATE_TIME 100
//distance before target to start decelerating
#define DRIVE_DECELERATE_DISTANCE 2

//PID constants for rotating in place
#define KP_SPIN 3
#define KI_SPIN 0
#define KD_SPIN 0


//max number of degrees away from target heading that we need to be to start driving toward target
#define ROTATION_ACCEPTABLE_ERROR 10 //@BROILED4

//max velocity of motors while spinning in place
#define MAX_SPIN_SPEED 48 //@BROILED3
//max velocity of motors while driving toward target
#define MAX_DRIVE_SPEED 128 //@BROILED6

//port that the gyro is plugged into
#define GYRO_PORT 20
//conversion from gyro magic units to real degrees. never change this unless we get a new gyro
#define GYRO_DEGREE_CONVERSION 1398794.44444 //DO NOT CHANGE THIS EVER OMG
//how long to wait in usetup to calibrate gyro
#define GYRO_CALIB_WAIT_TIME 2500L

#define FIRST_STAGE 10
#define SECOND_STAGE 11
#define THIRD_STAGE 12

#define ACCELERATE_SUBSTAGE 20
#define DRIVE_SUBSTAGE 21
#define DECELERATE_SUBSTAGE 22
#define LEVER_APPROACH_SUBSTAGE 23
#define LEVER_SUBSTAGE 24
#define LEVER_RETREAT_SUBSTAGE 25
#define TERRITORY_APPROACH_SUBSTAGE 26
#define TERRITORY_SUBSTAGE 27
#define TERRITORY_RETREAT_SUBSTAGE 28
#define DUMPING_SUBSTAGE 29
#define TRANSITION_SUBSTAGE 30
#define PIVOT_SUBSTAGE 31
#define DUMPING_RETREAT_SUBSTAGE 32

#define TARGET_CIRCLE_VEL 205

#define CRITICAL_ANGLE 195

#define DUMP_ADDED_DISTANCE ROBOT_WIDTH - 40


#define CIRCLE_DIRECTION 1
#define ERROR 19

#define DUMP_BASE_CASE 1400

uint32_t time_until_dump = 4000;
uint32_t time_during_dump = DUMP_BASE_CASE;


#define CIRCLE_DESIRED_DISTANCE (23 * INCHES_TO_PIXELS)



#define KP_DRIVE 1.6
#define KP_APPROACH 2.5

float KP_CIRCLE = KP_DRIVE;
float KI_CIRCLE = 0;
float KD_CIRCLE = 0;

#define THINK_ABOUT_DUMPING_TIME 110000

#define CIRCLE_ACCELERATE_TIME 200
#define CIRCLE_DECELERATE_TIME 100
#define CIRCLE_DECELERATE_DISTANCE_FIRST 14*INCHES_TO_PIXELS
#define CIRCLE_DECELERATE_DISTANCE_SECOND 8*INCHES_TO_PIXELS
#define TERRITORY_TIMEOUT_TIME 4000
#define LEVER_APPROACH_TIME 1200
#define LEVER_TIMEOUT_TIME 4500
#define TRANSITION_TIME 1700
#define CLICKY_CLICKY_TIME 320

#define DRIVE_TIMEOUT_TIME 500

#define ROTATION_THRESHOLD 20

#define SERVO_DOWN 400
#define SERVO_MIDDLE 300
#define SERVO_UP 175

#define WALL_ACCELERATE_TIME 50
#define WALL_DECELERATE_DISTANCE 4*INCHES_TO_PIXELS

#define PIVOT_DISTANCE 1264.4

#define GEARBOX_POINT 0
#define BALL_POINT 1


//declared here so used as a global
Field_State field_state;

int reset = 0;

//provids the desired velocity at a given time while driving
//ugh i HATE to have to make this a global variable when it's only used in one place but the PID structure requires it...annoying.
int target_vel;
int current_vel;
int accelerate_time;
int decelerate_distance;

//which location to target from TARGETS (used for testing)
int TARGETS[3][2] = {{0, 0},{800, 800},{-800, -800}};
uint8_t curr_target_index = 0;

//set up gyroscope and tell the vps what robot we are.
//runs before umain and a few seconds before the start of the competition
/*int usetup(void){
    robot_id = 1; //team 1
    //gyro_init(GYRO_PORT, GYRO_DEGREE_CONVERSION, GYRO_CALIB_WAIT_TIME);
    return 0;
}*/

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

float pythagorean_loc(Location start_loc, Location end_loc){
    int x_dist = end_loc.x - start_loc.x;
    int y_dist = end_loc.y - start_loc.y;
    return pythagorean(x_dist, y_dist);
}

//easy way to make sure two motors on one wheel aren't given different powers
void set_motors(int leftPwr, int rightPwr){
    motor_set_vel(LEFT_MOT_1, leftPwr);
    motor_set_vel(LEFT_MOT_2, leftPwr);
    motor_set_vel(RIGHT_MOT_1, rightPwr);
    motor_set_vel(RIGHT_MOT_2, rightPwr);
}

void set_spinners(int pwr){
    motor_set_vel(SPINNER_MOTOR_PORT_1, pwr);
    motor_set_vel(SPINNER_MOTOR_PORT_2, pwr);
}

void set_spinners_indivit(int pwr1, int pwr2){
    motor_set_vel(SPINNER_MOTOR_PORT_1, pwr1);
    motor_set_vel(SPINNER_MOTOR_PORT_2, pwr2);
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
        case FIRST_STAGE:
            if(vel > target_vel)
                vel = target_vel;
            return (int)vel;
            break;
        case SECOND_STAGE:
            if(vel > target_vel)
                vel = target_vel;
            return (int)vel;
            break;
        default:
            if(vel > target_vel)
                vel = target_vel;
            return (int)vel;
            break;
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

float mod_f(float angle, float modder){
    if(angle < (-1 * modder)){
        angle += (2 * modder);
    }
    if(angle > modder){
        angle -= (2 * modder);
    }
    return angle;
}

float reverse_mod(float angle, float demodder, float sign){
    if(angle*sign > 0){
        angle = angle + sign*2*demodder;
    }
}


//gets info from vps and updates field_state

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

Location get_scaled_loc(Location target_loc, float distance){
    int old_x_distance = target_loc.x;
    int old_y_distance = target_loc.y;
    
    float old_target_distance = pythagorean(old_x_distance, old_y_distance);
    float new_target_distance = distance;
    float ratio = new_target_distance / old_target_distance;
    int new_x_distance = (int)(old_x_distance * ratio);
    int new_y_distance = (int)(old_y_distance * ratio);
    Location returned_loc;
    returned_loc.x = new_x_distance;
    returned_loc.y = new_y_distance;
    return returned_loc;
}

Location get_scaled_loc_int_param(int target_loc_x, int target_loc_y, float distance){
    Location temp_loc;
    temp_loc.x = target_loc_x;
    temp_loc.y = target_loc_y;
    return get_scaled_loc(temp_loc, distance);
}


uint8_t mod_ui(int sextant, uint8_t modder){
    if(sextant < 0)
        sextant += modder;
    return (uint8_t)((int)sextant%(int)modder);
}

//main function. runs when we press go.
/*void umain(void){
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
                    break;
                case DRIVE_DRIVE:
                    /*if((field_state.target_dist * PIXELS_TO_INCHES) < DRIVE_DECELERATE_DISTANCE){ 
                        target_vel = 0;//format_velocity(scale_values(field_state.target_dist, DRIVE_DECELERATE_DISTANCE, MAX_DRIVE_SPEED));
                        field_state.stored_time = get_time();
                        field_state.substage = DRIVE_STOP;
                        field_state.
                        set_motors(0,0);
                        gyro_set_degrees(field_state.curr_angle);
                        break;
                    }
                    target_vel = MAX_DRIVE_SPEED;
                    break;
                /*case DRIVE_DECELERATE:
                    if(field_state.curr_time - field_state.stored_time > DRIVE_DECELERATE_TIME){
                 
                        int old_x_distance = field_state.target_loc.x - current_x;
                        int old_y_distance = field_state.target_loc.y - current_y;
                        float old_target_distance = pythagorean(old_x_distance, old_y_distance);
                        float new_target_distance = DRIVE_DECELERATE_DISTANCE * INCHES_TO_PIXELS;
                        float ratio = new_target_distance / old_target_distance;
                        int new_x_distance = (int)(old_x_distance * ratio);
                        int new_y_distance = (int)(old_y_distance * ratio);
                        field_state.target_loc.x = current_x + new_x_distance;
                        field_state.target_loc.y = current_y + new_y_distance;
                    
                        stop_motors();
                        start_drive_stop();
                        driving_PID.enabled = false;
                        drive_pid_on = false;
                        continue;
                    }
                    target_vel = format_velocity(scale_values(DRIVE_DECELERATE_TIME - (field_state.curr_time - field_state.stored_time), DRIVE_DECELERATE_TIME, MAX_DRIVE_SPEED));
                    break; 
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
}*/
















