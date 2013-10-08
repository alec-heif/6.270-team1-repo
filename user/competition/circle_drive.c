//
//  circle_drive.c
//  
//
//  Created by aheifetz on 1/19/13.
//
//

#include <in_field.c>

extern volatile uint8_t robot_id;

#define SECOND 2
#define THIRD 3

#define TWO_RL_02 12
#define TWO_RL_01 9
#define TWO_RL_00 4
#define ONE_RL_00 3
#define ONE_RL_01 7
#define ONE_RL_10 6
#define ONE_RL_11 10
#define ZERO_RL_1 8
#define ZERO_RL_0 5

//set for counter-clockwise circle direction. change to -1 for clockwise.


/*Location get_lever_pivot(int sextant){
    sextant = sextant%6;
    Location target_pivot;
    target_pivot.x = lever_pivot_points[(2*sextant)%12];
    target_pivot.y = lever_pivot_points[(2*sextant + 1)%12];
    return target_pivot;
}

Location get_territory_pivot(uint8_t sextant){
    sextant = sextant%6;
    Location target_pivot;
    target_pivot.x = territory_pivot_points[(2*sextant)%12];
    target_pivot.y = territory_pivot_points[(2*sextant + 1)%12];
    return target_pivot;
}*/

uint8_t timeout_counter = 0;


uint8_t rate_limit(uint8_t sextant){
    return game.territories[sextant].rate_limit;
}

uint8_t owner(uint8_t sextant){
    return game.territories[sextant].owner;
}

uint8_t balls_left(uint8_t sextant){
    return game.territories[sextant].remaining;
}

int dump_color(uint8_t sextant){
    switch(sextant){
        case 0:
            return RED;
            break;
        case 1:
            return RED;
            break;
        case 2:
            return RED;
            break;
        case 3:
            return RED;
            break;
        case 4:
            return RED;
            break;
        case 5:
            return BLUE;
            break;
    }
}

void start_driving_somewhere(Location somewhere){
    target_vel = TARGET_CIRCLE_VEL;
    current_vel = 0;
    field_state.stored_time = get_time();
    field_state.start_drive_time = get_time();
    accelerate_time = CIRCLE_ACCELERATE_TIME;
    decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
    field_state.drive_direction = BACKWARD;
    KP_CIRCLE = KP_DRIVE;
    field_state.circle_PID.enabled = true;
    field_state.pid_enabled = true;
    
    field_state.substage = DRIVE_SUBSTAGE;
    //set_new_destination(field_state.curr_loc_plus_delta, somewhere);//get_territory_pivot_point_loc((temp_sextant - 1)%6));
    
}


uint8_t what_to_do(uint8_t sextant){
    if(owner(sextant) != US){
        if(rate_limit(sextant) <= 15 && balls_left(sextant) > 0){
            return CAPTURE_AND_MINE;
        }
        else{
            return CAPTURE_NOT_MINE;
        }
    }
    else{
        if(rate_limit(sextant) <= 15 && balls_left(sextant) > 0){
            return MINE_NOT_CAPTURE;
        }
        else{
            return LEAVE;
        }
    }
}


void retreat_from_territory(){
    KP_CIRCLE = KP_APPROACH;
    field_state.substage = TERRITORY_RETREAT_SUBSTAGE;
    
    //SET MOTORS TO LEVER ARC THINGY
    target_vel = 96;
    current_vel = 96;
    set_spinners(0);
    //accelerate_time = CIRCLE_ACCELERATE_TIME;
    //decelerate_distance = CIRCLE_DECELERATE_DISTANCE;
    //KP_CIRCLE = 1.5;
    //set_motors(55, 120);
    KP_CIRCLE = KP_APPROACH;
    field_state.stored_time = get_time();
    field_state.drive_direction = FORWARD;
    field_state.pid_enabled = TRUE;
    field_state.circle_PID.enabled = true;
    servo_set_pos(1, SERVO_UP);
    set_new_destination(field_state.curr_loc_plus_delta, get_territory_pivot_point_loc(get_current_sextant(field_state.curr_loc_plus_delta)));
}

void retreat_from_lever(){
    KP_CIRCLE = KP_APPROACH;
    field_state.substage = LEVER_RETREAT_SUBSTAGE;
    
    //SET MOTORS TO LEVER ARC THINGY
    target_vel = 96;
    current_vel = 96;
    set_spinners(0);
    //accelerate_time = CIRCLE_ACCELERATE_TIME;
    //decelerate_distance = CIRCLE_DECELERATE_DISTANCE;
    //KP_CIRCLE = 1.5;
    //set_motors(55, 120);
    KP_CIRCLE = KP_APPROACH;
    field_state.stored_time = get_time();
    field_state.drive_direction = BACKWARD;
    field_state.pid_enabled = TRUE;
    field_state.circle_PID.enabled = true;
    servo_set_pos(1, SERVO_UP);
    set_new_destination(field_state.curr_loc_plus_delta, get_lever_pivot_point_loc(get_current_sextant(field_state.curr_loc_plus_delta)));
}

void retreat_from_dumping(){
    KP_CIRCLE = KP_APPROACH;
    field_state.substage = DUMPING_RETREAT_SUBSTAGE;
    
    //SET MOTORS TO LEVER ARC THINGY
    target_vel = 96;
    current_vel = 96;
    set_spinners(0);
    servo_set_pos(0, 300);
    //accelerate_time = CIRCLE_ACCELERATE_TIME;
    //decelerate_distance = CIRCLE_DECELERATE_DISTANCE;
    //KP_CIRCLE = 1.5;
    //set_motors(55, 120);
    KP_CIRCLE = KP_APPROACH;
    field_state.stored_time = get_time();
    field_state.drive_direction = FORWARD;
    field_state.pid_enabled = TRUE;
    field_state.circle_PID.enabled = true;
    servo_set_pos(1, SERVO_UP);
    set_new_destination(field_state.curr_loc_plus_delta, get_dump_location_robot(get_current_sextant(field_state.curr_loc_plus_delta)));
}


void do_what_to_do(uint8_t sextant){
    uint8_t what_to_do_now = what_to_do(sextant);
    //whf("Do what to do ran for sextant %d and we should do %d\n", sextant, what_to_do_now);
    switch(what_to_do_now){
        case CAPTURE_AND_MINE:
            field_state.drive_direction = BACKWARD;
            if(field_state.substage == TERRITORY_APPROACH_SUBSTAGE)
                field_state.drive_direction = FORWARD;
            break;
        case CAPTURE_NOT_MINE:
            field_state.drive_direction = BACKWARD;
            if(field_state.substage == TERRITORY_APPROACH_SUBSTAGE)
                field_state.drive_direction = FORWARD;
            break;
        case MINE_NOT_CAPTURE:
            field_state.drive_direction = FORWARD;
            if(field_state.substage == LEVER_SUBSTAGE)
                field_state.drive_direction = BACKWARD;
            break;
    }
    if(sextant != get_current_sextant(field_state.curr_loc_plus_delta)){
        if(field_state.substage == TERRITORY_RETREAT_SUBSTAGE || field_state.substage == LEVER_RETREAT_SUBSTAGE || field_state.substage == DUMPING_RETREAT_SUBSTAGE){
            //we just backed out of a thing, pivot toward the counter-clockwise target and let pivot substage do the rest
            target_vel = TARGET_CIRCLE_VEL;
            current_vel = 0;
            field_state.substage = PIVOT_SUBSTAGE;
            accelerate_time = CIRCLE_ACCELERATE_TIME;
            decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
            KP_CIRCLE = KP_DRIVE;
            field_state.stored_time = get_time();
            field_state.circle_PID.enabled = true;
            field_state.pid_enabled = true;
        }
        else{
            target_vel = TARGET_CIRCLE_VEL;
            current_vel = 0;
            accelerate_time = CIRCLE_ACCELERATE_TIME;
            decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
            KP_CIRCLE = KP_DRIVE;
            field_state.start_drive_time = get_time();
            field_state.circle_PID.enabled = true;
            field_state.pid_enabled = true;
            field_state.substage = DRIVE_SUBSTAGE;

        }
    }
    else{
        //we probably just finished a dump
        if(field_state.substage == TERRITORY_RETREAT_SUBSTAGE || field_state.substage == LEVER_RETREAT_SUBSTAGE || field_state.substage == DUMPING_RETREAT_SUBSTAGE){
            //we just backed out of a thing, pivot toward the counter-clockwise target and let pivot substage do the rest
            target_vel = TARGET_CIRCLE_VEL;
            current_vel = 0;
            field_state.substage = PIVOT_SUBSTAGE;
            accelerate_time = CIRCLE_ACCELERATE_TIME;
            decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
            KP_CIRCLE = KP_DRIVE;
            field_state.stored_time = get_time();
            field_state.circle_PID.enabled = true;
            field_state.pid_enabled = true;
        }
        else{
            target_vel = TARGET_CIRCLE_VEL;
            current_vel = 0;
            accelerate_time = CIRCLE_ACCELERATE_TIME;
            decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
            KP_CIRCLE = KP_DRIVE;
            field_state.start_drive_time = get_time();
            field_state.circle_PID.enabled = true;
            field_state.pid_enabled = true;
            field_state.substage = DRIVE_SUBSTAGE;
            
        }

    }
    switch(what_to_do_now){
        case CAPTURE_AND_MINE:
            //printf("cap+mine\n");
            set_new_destination(field_state.curr_loc_plus_delta, get_territory_pivot_point_loc(sextant));
            break;
        case CAPTURE_NOT_MINE:
            set_new_destination(field_state.curr_loc_plus_delta, get_territory_pivot_point_loc(sextant));
            break;
        case MINE_NOT_CAPTURE:
            servo_set_pos(1, SERVO_UP);
            set_new_destination(field_state.curr_loc_plus_delta, get_lever_pivot_point_loc(sextant));
            break;
    }
    //printf("Final Substage: %d", field_state.substage);
}


float get_angle_error_circle(){
    //uint32_t dt = field_state.curr_time - field_state.stored_time;
    float target_angle;
    if(field_state.substage == TERRITORY_APPROACH_SUBSTAGE || field_state.substage == LEVER_APPROACH_SUBSTAGE){
        Location heading_target_loc;
        if(field_state.drive_direction == BACKWARD){
            heading_target_loc = get_territory_loc(get_current_sextant(field_state.curr_loc_plus_delta));
            field_state.target_angle = mod_f((float)(degrees(atan2(heading_target_loc.y - field_state.curr_loc_plus_delta.y, heading_target_loc.x - field_state.curr_loc_plus_delta.x)) + 180), 180);
        }
        else{
            heading_target_loc = get_lever_loc(get_current_sextant(field_state.curr_loc_plus_delta));
            field_state.target_angle = mod_f((float)(degrees(atan2(heading_target_loc.y - field_state.curr_loc_plus_delta.y, heading_target_loc.x - field_state.curr_loc_plus_delta.x))), 180);
        }
        
    }
    else if(field_state.substage == DRIVE_SUBSTAGE || field_state.substage == DUMPING_SUBSTAGE || field_state.substage == PIVOT_SUBSTAGE || field_state.substage == LEVER_RETREAT_SUBSTAGE || field_state.substage == TERRITORY_RETREAT_SUBSTAGE || field_state.substage == DUMPING_RETREAT_SUBSTAGE){
        if(field_state.drive_direction == BACKWARD)
            field_state.target_angle = mod_f((float)(degrees(atan2(field_state.target_loc_waypoint.y - field_state.curr_loc_plus_delta.y, field_state.target_loc_waypoint.x - field_state.curr_loc_plus_delta.x)) + 180), 180);
        else
            field_state.target_angle = mod_f((float)(degrees(atan2(field_state.target_loc_waypoint.y - field_state.curr_loc_plus_delta.y, field_state.target_loc_waypoint.x - field_state.curr_loc_plus_delta.x))), 180);
    }
    float current_angle;
    current_angle = mod_f(gyro_get_degrees(), 180);
    float error = (field_state.target_angle - current_angle);
    error = mod_f(error, 180);
    ////printf("Target Angle: %f, Current Angle: %f, Error: %f\n", target_angle, current_angle, error);
    return error;
}


void robot_backup(){
    set_motors(field_state.drive_direction * 128, field_state.drive_direction * 128);
    pause(400);
    set_motors(0, 0);
    pause(100);
}

//DEFINES:
/*****
 TWO_RL_02 12
 TWO_RL_01 9
 TWO_RL_00 4
 ONE_RL_00 3
 ONE_RL_01 7
 ONE_RL_10 = 6
 ONE_RL_11 = 10
 ZERO_RL_1 = 8
 ZERO_RL_0 = 5
 
 
 */

uint8_t get_target_territory(){
    Location curr_loc = field_state.curr_loc_plus_delta;
    uint8_t curr_sextant = get_current_sextant(curr_loc);
    uint8_t ccw_sextant = mod_ui(curr_sextant + 1, 6);
    uint8_t cw_sextant = mod_ui(curr_sextant - 1, 6);
    uint8_t opponent_sextant = get_current_sextant(field_state.opponent_loc);
    uint8_t two_ccw_sextant = mod_ui(curr_sextant + 2, 6);
    uint8_t two_cw_sextant = mod_ui(curr_sextant + 2, 6);
    uint8_t three_sextant = mod_ui(curr_sextant + 3, 6);
    uint8_t dump_loc;
    //printf("CS: %d, CCW: %d, CW: %d, 2CCW: %d, 2CW: %d, 3S: %d, OS: %d\n", curr_sextant, ccw_sextant, cw_sextant, two_ccw_sextant, two_cw_sextant, three_sextant, opponent_sextant);
    if(opponent_sextant != curr_sextant && (owner(curr_sextant) != US || (balls_left(curr_sextant) > 0 && rate_limit(curr_sextant) < ZERO_RL_0 ))){
            return curr_sextant;
    }
    //printf("Past First, ");
    if(opponent_sextant == two_ccw_sextant && balls_left(ccw_sextant) > 0 && rate_limit(ccw_sextant) < ONE_RL_00)
        return ccw_sextant;
    //printf("Past Second, ");
    if(opponent_sextant == two_cw_sextant && balls_left(cw_sextant) > 0 && rate_limit(cw_sextant) < ONE_RL_00)
        return cw_sextant;
    //printf("Past Third, ");

    if((balls_left(ccw_sextant) > 0 && rate_limit(ccw_sextant) < ONE_RL_00 && owner(ccw_sextant) != US) && opponent_sextant != ccw_sextant)
        return ccw_sextant;
    //printf("Past Fourth, ");

    if((balls_left(cw_sextant) > 0 && rate_limit(cw_sextant) < ONE_RL_00 && owner(cw_sextant) != US) && opponent_sextant != cw_sextant)
        return cw_sextant;
    //printf("Past Fifth, ");

    if(((owner(ccw_sextant) != US && balls_left(ccw_sextant) > 0 && rate_limit(ccw_sextant) < ONE_RL_01) || (owner(ccw_sextant) == US && balls_left(ccw_sextant) > 0 && rate_limit(ccw_sextant) < ONE_RL_00)) && opponent_sextant != ccw_sextant)
       return ccw_sextant;
    //printf("Past Sixth, ");

    if((opponent_sextant == two_ccw_sextant && ((owner(ccw_sextant) != US && balls_left(ccw_sextant) > 0 && rate_limit(ccw_sextant) < ONE_RL_11) || (owner(ccw_sextant) == US && balls_left(ccw_sextant) > 0 && rate_limit(ccw_sextant) < ONE_RL_10))) && opponent_sextant != ccw_sextant)
        return ccw_sextant;
    //printf("Past Seventh, ");
    
    if(((owner(cw_sextant) != US && balls_left(cw_sextant) > 0 && rate_limit(cw_sextant) < ONE_RL_01) || (owner(cw_sextant) == US && balls_left(cw_sextant) > 0 && rate_limit(cw_sextant) < ONE_RL_00)) && opponent_sextant != cw_sextant)
        return cw_sextant;
    //printf("Past 8th, ");

    if((opponent_sextant == two_cw_sextant && ((owner(cw_sextant) != US && balls_left(cw_sextant) > 0 && rate_limit(cw_sextant) < ONE_RL_11) || (owner(cw_sextant) == US && balls_left(cw_sextant) > 0 && rate_limit(cw_sextant) < ONE_RL_10))) && opponent_sextant != cw_sextant)
        return cw_sextant;
    //printf("Past 9th, ");

    /****START AGGRESSION****/
    if(opponent_sextant == three_sextant && owner(two_ccw_sextant) != US && balls_left(two_ccw_sextant) > 0 && rate_limit(two_ccw_sextant) < TWO_RL_01)
        return two_ccw_sextant;
    //printf("Past 10, ");

    if(opponent_sextant == three_sextant && owner(two_cw_sextant) != US && balls_left(two_cw_sextant) > 0 && rate_limit(two_cw_sextant) < TWO_RL_01)
        return two_cw_sextant;
    //printf("Past 11, ");

    if(opponent_sextant != ccw_sextant && opponent_sextant != two_ccw_sextant && owner(two_ccw_sextant) != US && balls_left(two_ccw_sextant) > 0 && rate_limit(two_ccw_sextant) < TWO_RL_01)
        return two_ccw_sextant;
    //printf("Past 12, ");

    if(opponent_sextant != cw_sextant && opponent_sextant != two_cw_sextant && owner(two_cw_sextant) != US && balls_left(two_cw_sextant) > 0 && rate_limit(two_cw_sextant) < TWO_RL_01)
        return two_cw_sextant;
    //printf("Past 13, ");

    /*******END AGGRESSION********/
    if(opponent_sextant != ccw_sextant && opponent_sextant != two_ccw_sextant && opponent_sextant != three_sextant && owner(ccw_sextant) != US && ((owner(two_ccw_sextant) != US && rate_limit(two_ccw_sextant) < TWO_RL_02) || (owner(two_ccw_sextant) == US && rate_limit(two_ccw_sextant) < TWO_RL_01)))
        return ccw_sextant;
    //printf("Past 14, ");

    if(opponent_sextant != cw_sextant && opponent_sextant != two_cw_sextant && opponent_sextant != three_sextant && owner(cw_sextant) != US && ((owner(two_cw_sextant) != US && rate_limit(two_cw_sextant) < TWO_RL_02) || (owner(two_cw_sextant) == US && rate_limit(two_cw_sextant) < TWO_RL_01)))
        return cw_sextant;
    //printf("Past 15, ");

    if(opponent_sextant != ccw_sextant && opponent_sextant != two_ccw_sextant && owner(ccw_sextant) != US && ((owner(two_ccw_sextant) != US && rate_limit(two_ccw_sextant) < TWO_RL_02 && balls_left(two_ccw_sextant) > 0) || (owner(two_ccw_sextant) == US && rate_limit(two_ccw_sextant) < TWO_RL_01)))
        return ccw_sextant;
    //printf("Past 16, ");

    if(opponent_sextant != cw_sextant && opponent_sextant != two_cw_sextant && owner(cw_sextant) != US && ((owner(two_cw_sextant) != US && rate_limit(two_cw_sextant) < TWO_RL_02 && balls_left(two_cw_sextant) > 0) || (owner(two_cw_sextant) == US && rate_limit(two_cw_sextant) < TWO_RL_01 && balls_left(two_cw_sextant) > 0)))
        return cw_sextant;
    //printf("Past 17, ");

    if(dump_color(curr_sextant) == field_state.color && opponent_sextant != curr_sextant && field_state.balls_held > 0){
        field_state.we_want_to_dump = TRUE;
        dump_loc = pick_dump_sextant();
        return dump_loc;
    }
    //printf("Past 18, ");

    if(opponent_sextant != ccw_sextant && owner(ccw_sextant) != US)
        return ccw_sextant;
    //printf("Past 19, ");

    if(opponent_sextant != cw_sextant && owner(cw_sextant) != US)
        return cw_sextant;
    //printf("Past First, ");

    if((((balls_left(curr_sextant) > 0 && rate_limit(curr_sextant) < ZERO_RL_1) || owner(curr_sextant) != US) && opponent_sextant != curr_sextant))
        return curr_sextant;
    //printf("Past 20, ");

    if(opponent_sextant == cw_sextant && (owner(two_ccw_sextant) != US || (rate_limit(two_ccw_sextant) < TWO_RL_00 && balls_left(two_ccw_sextant) > 0)))
        return two_ccw_sextant;
    //printf("Past 21, ");

    if(opponent_sextant == ccw_sextant && (owner(two_cw_sextant) != US || (rate_limit(two_cw_sextant) < TWO_RL_00 && balls_left(two_cw_sextant) > 0)))
        return two_cw_sextant;
    //printf("Past 22, ");

    if(field_state.balls_held > 0){
        field_state.we_want_to_dump = TRUE;
        dump_loc = pick_dump_sextant();
        return dump_loc;
    }
    //printf("Past 23, finished.\n\n");

    return curr_sextant;
}


/*uint8_t get_num_sextants_with_balls(uint8_t start_sextant, uint8_t end_sextant, int direction){
    uint8_t curr_sextant = start_sextant;
    switch(direction){
        case COUNTER_CLOCKWISE:
            break;
        case CLOCKWISE:
            if(target_sextant > curr_sextant){
                target_sextant -= 6;
            }
            return (uint8_t)(curr_sextant - target_sextant);
            break;
        case 0:
            return 0;
            break;
    }
}*/

void run_pivot_subroutine(){
    if(field_state.substage == PIVOT_SUBSTAGE && fabs(get_angle_error_circle()) < ROTATION_THRESHOLD){
        if(field_state.we_want_to_dump){
            get_your_ass_to_a_toilet();
        }
        else{
            if(field_state.stage != FIRST_STAGE){
                field_state.start_drive_time = get_time();
                field_state.substage = DRIVE_SUBSTAGE;
            }
            else{
                field_state.substage = DRIVE_SUBSTAGE;
                target_vel = TARGET_CIRCLE_VEL;
                current_vel = 0;
                accelerate_time = CIRCLE_ACCELERATE_TIME;
                decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
                KP_CIRCLE = KP_DRIVE;
                field_state.stored_time = get_time();
                field_state.start_drive_time = get_time();
                field_state.drive_direction = BACKWARD;
                field_state.circle_PID.enabled = true;
                field_state.pid_enabled = true;
            }
        }
        if(current_vel == 0)
            current_vel = target_vel;
    }
}


void update_circle_velocities(float error){
    int pwr_change = format_velocity(fabs(error));
    int dir = field_state.drive_direction;
    if(field_state.substage == DRIVE_SUBSTAGE){
        /*if(fabs(mod_f(field_state.target_angle - mod_f(gyro_get_degrees(), 180), 180)) < ROTATION_THRESHOLD){
         if(current_vel == 0)
         current_vel = target_vel;
         }*/
        if(error*dir > 0){
            set_motors(dir*current_vel, dir*current_vel - dir*pwr_change);
            ////printf("Left Vel: %d, Right Vel: %d\n", current_vel, current_vel - pwr_change);
        }
        else{
            set_motors(dir*current_vel - dir*pwr_change, dir*current_vel);
            ////printf("Left Vel: %d, Right Vel: %d\n", current_vel - pwr_change, current_vel);
        }
    }
    else if(field_state.substage == TERRITORY_APPROACH_SUBSTAGE || field_state.substage == LEVER_APPROACH_SUBSTAGE || field_state.substage == DUMPING_SUBSTAGE || field_state.substage == PIVOT_SUBSTAGE || field_state.substage == LEVER_RETREAT_SUBSTAGE || field_state.substage == TERRITORY_RETREAT_SUBSTAGE || field_state.substage == DUMPING_RETREAT_SUBSTAGE){
        if(fabs(mod_f(field_state.target_angle - mod_f(gyro_get_degrees(), 180), 180)) < ROTATION_THRESHOLD){
            if(current_vel == 0){
                //used to know when we started moving toward the lever/territory
                field_state.stored_time = get_time();
                current_vel = target_vel;
            }
        }
        if(error*dir > 0){
            set_motors(dir*current_vel + dir*pwr_change, dir*current_vel - dir*pwr_change);
            ////printf("Left Vel: %d, Right Vel: %d\n", current_vel, current_vel - pwr_change);
        }
        else{
            set_motors(dir*current_vel - dir*pwr_change, dir*current_vel + dir*pwr_change);
            ////printf("Left Vel: %d, Right Vel: %d\n", current_vel - pwr_change, current_vel);
        }
    }
}

uint8_t is_decelerating(){
    if((field_state.target_loc.x == field_state.target_loc_waypoint.x && field_state.target_loc.y == field_state.target_loc_waypoint.y) && (pythagorean(field_state.target_loc.x - field_state.curr_loc_plus_delta.x, field_state.target_loc.y - field_state.curr_loc_plus_delta.y) < decelerate_distance)){
        //current_vel = 0;
        //target_vel = 0;
        //stop_motors();
        return true;

    }
        return false;
}


void accelerate(){
    if(is_decelerating())
        return;
    else if(current_vel < target_vel && (field_state.substage == DRIVE_SUBSTAGE || field_state.substage == DUMPING_SUBSTAGE)){
        if(field_state.curr_time < field_state.start_drive_time + accelerate_time){
            current_vel = format_velocity(scale_values(field_state.curr_time - field_state.start_drive_time, accelerate_time, target_vel));
        }
        else{
            current_vel = target_vel;
        }
    }
}

void decelerate(){
    if(is_decelerating()){
        if(field_state.substage != TERRITORY_SUBSTAGE && field_state.substage != LEVER_SUBSTAGE && field_state.substage != TRANSITION_SUBSTAGE && field_state.substage != PIVOT_SUBSTAGE){
            float distance = (pythagorean(field_state.target_loc.x - field_state.curr_loc_plus_delta.x, field_state.target_loc.y - field_state.curr_loc_plus_delta.y));
            //float new_target_vel = scale_values(pythagorean(field_state.target_loc.x - field_state.curr_loc_plus_delta.x, field_state.target_loc.y - field_state.curr_loc_plus_delta.y), decelerate_distance, target_vel);
            if(distance < CIRCLE_DECELERATE_DISTANCE_FIRST){
                current_vel = 96;
                KP_CIRCLE = 1.5;
            }
            if(distance < CIRCLE_DECELERATE_DISTANCE_SECOND){
                KP_CIRCLE = 2.5;
                current_vel = 64;
            }
        }
    }
}



float get_target_pivot_angle(Location curr_loc){
    return 0;
}

void get_next_move(){
    return;
}
                  
                  
int get_acceptable_error(){
    switch(field_state.substage){
        case TERRITORY_APPROACH_SUBSTAGE:
            return (int)(0*INCHES_TO_PIXELS);
            break;
        case LEVER_APPROACH_SUBSTAGE:
            return (int)(0*INCHES_TO_PIXELS);
            break;
        case TERRITORY_RETREAT_SUBSTAGE:
            return (int)(4*INCHES_TO_PIXELS);
            break;
        case LEVER_RETREAT_SUBSTAGE:
            return (int)(3*INCHES_TO_PIXELS);
            break;
        case DRIVE_SUBSTAGE:
            if(target_vel != 64 && target_vel !=96)
                return (int)(6*(INCHES_TO_PIXELS));
            else
                return (int)(4*INCHES_TO_PIXELS);
            break;
        default:
            return (int)(3*INCHES_TO_PIXELS);
            break;
    }
}

void take_a_dump(){
    field_state.we_want_to_dump = TRUE;
    /*if(field_state.substage == PIVOT_SUBSTAGE){
        field_state.we_want_to_dump = TRUE;
        target_vel = TARGET_CIRCLE_VEL;
        field_state.stored_time = get_time();
        field_state.start_drive_time = get_time();
        field_state.substage = DRIVE_SUBSTAGE;
        accelerate_time = CIRCLE_ACCELERATE_TIME;
        decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
        KP_CIRCLE = KP_DRIVE;
        field_state.drive_direction = BACKWARD;
        field_state.circle_PID.enabled = true;
        field_state.pid_enabled = true;
        uint8_t dump_sextant = pick_dump_sextant();
        Location dump_loc = get_dump_location_robot(dump_sextant);
        set_new_destination(field_state.curr_loc_plus_delta, dump_loc);
        uint8_t travel_sextants = get_sextant_difference(get_current_sextant(field_state.curr_loc_plus_delta), get_current_sextant(dump_loc), get_waypoint_direction(field_state.curr_loc_plus_delta, dump_loc, gyro_get_degrees()));
        //time_until_dump += 1500*travel_sextants;
        time_during_dump += 100*field_state.balls_held;
    }*/
    if(field_state.substage == TERRITORY_SUBSTAGE){
        retreat_from_territory();
    }
        servo_set_pos(0, 150);
}





void update_field(){
    copy_objects();
    int current_x = (int)(game.coords[0].x * VPS_RATIO);
    int current_y = (int)(game.coords[0].y * VPS_RATIO);
    field_state.opponent_loc.x = (int)(game.coords[1].x * VPS_RATIO);
    field_state.opponent_loc.y = (int)(game.coords[1].y * VPS_RATIO);
    
    float dist_to_opponent = pythagorean(field_state.opponent_loc.x - current_x, field_state.opponent_loc.y - current_y);
    
    float current_theta = (((float)game.coords[0].theta) * 180.0) / 2048;
    field_state.curr_loc_plus_delta.x = current_x; //+ get_delta(field_state.curr_loc).x;
    field_state.curr_loc_plus_delta.y = current_y; //+ get_delta(field_state.curr_loc).y;
    uint8_t sextant = get_current_sextant(field_state.curr_loc_plus_delta);
    uint8_t old_sextant = get_current_sextant(field_state.curr_loc);

    uint8_t has_encoder_moved = FALSE;
    float dist_to_target = pythagorean_loc(field_state.curr_loc_plus_delta, field_state.target_loc);
    if(get_time() > field_state.last_encoder_update + ENCODER_UPDATE_TIME){
        field_state.last_encoder_update = get_time();
        if(encoder_read(FREEWHEEL_ENCODER_PORT) != field_state.encoder_value){
            field_state.robot_stopped = FALSE;
            field_state.encoder_value = encoder_read(FREEWHEEL_ENCODER_PORT);
            timeout_counter=0;
        }
        else{
            if(field_state.substage == field_state.last_substage && (field_state.substage == DRIVE_SUBSTAGE || field_state.substage == LEVER_APPROACH_SUBSTAGE || field_state.substage == TERRITORY_RETREAT_SUBSTAGE || field_state.substage == LEVER_RETREAT_SUBSTAGE) && dist_to_target > 12*INCHES_TO_PIXELS){
                field_state.robot_stopped = TRUE;
                timeout_counter++;
            }
            else if(get_current_sextant(field_state.opponent_loc) == sextant){
                field_state.robot_stopped = TRUE;
                timeout_counter++;
            }
        }
        field_state.last_substage = field_state.substage;
    }

    if(field_state.robot_stopped){
        int dir = field_state.drive_direction;
        if(timeout_counter >= 2){
            if(timeout_counter%2 == 0)
                dir = dir;
            else
                dir = -1 * dir;
        }
        if(timeout_counter < 4)
            set_motors(-dir*205, -dir*205);
        else{
            set_motors(-dir*205, dir*205);
            pause(200);
            set_motors(-dir*205, dir * 205);
        }
        pause(500);
        stop_motors();
        copy_objects();
        pause(100);
        field_state.last_encoder_update = get_time();
        current_vel = 0;
        field_state.substage = PIVOT_SUBSTAGE;
        Location curr_loc;
        curr_loc.x = game.coords[0].x;
        curr_loc.y = game.coords[0].y;
        set_new_destination(field_state.curr_loc_plus_delta, field_state.target_loc);
        encoder_reset(FREEWHEEL_ENCODER_PORT);
        field_state.encoder_value = encoder_read(FREEWHEEL_ENCODER_PORT);
        field_state.robot_stopped = FALSE;
    }
    
    if(field_state.stage != FIRST_STAGE && get_current_sextant(field_state.opponent_loc) == sextant){
        uint8_t new_target = get_target_territory();
        do_what_to_do(new_target);
        field_state.substage = DRIVE_SUBSTAGE;
    }
    
    ////printf("Current X: %d, Current Y: %d\n", current_x, current_y);

    
    ////printf("Current Distance: %f, Waypoint Distance: %f, Target Distance: %f, Distance Increment: %f\n", pythagorean(field_state.curr_loc_plus_delta.x, field_state.curr_loc_plus_delta.y), pythagorean(field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y), pythagorean(field_state.target_loc.x, field_state.target_loc.y), field_state.distance_increment);
    float dist_to_waypoint = pythagorean_loc(field_state.curr_loc_plus_delta, field_state.target_loc_waypoint);
    if(sextant != old_sextant ){
        uint8_t target_sextant = get_current_sextant(field_state.target_loc);
        /*int direction;
        if((get_angle_error_circle() > 90 && dist_to_waypoint < CIRCLE_DECELERATE_DISTANCE_SECOND) && (sextant == old_sextant)){
            //we're about to spiral...let's not spiral
            direction = get_waypoint_direction(field_state.curr_loc_plus_delta, field_state.target_loc, gyro_get_degrees());
            if(get_sextant_difference(field_state.curr_loc_plus_delta, field_state.target_loc, direction) > 1){
                //we aren't at the target sextant yet, so set target waypoint to next waypoint
                float distance_increment;
                    switch(direction){
                    case COUNTER_CLOCKWISE:
                            if(target_sextant < sextant)
                                target_sextant += 6;
                            new_distance = bound_waypoint_distance(field_state.distance_increment, field_state.target_loc_waypoint);
                            field_state.target_loc_waypoint = get_scaled_loc_int_param(bisecting_points[(2*sextant+4)%12], bisecting_points[(2*sextant+5)%12], new_distance);
                            break;
                        case CLOCKWISE:
                            if(target_sextant > sextant)
                                target_sextant -= 6;
                            new_distance = bound_waypoint_distance(field_state.distance_increment, field_state.target_loc_waypoint);
                            field_state.target_loc_waypoint = get_scaled_loc_int_param(bisecting_points[mod_ui(2*sextant - 2, 12)], bisecting_points[mod_ui(2*sextant - 1, 12)], new_distance);
                            break;
                    }
                }

            }

        }*/
        //We have broken the plane, get next waypoint, or target if in same sextant
        int direction = get_waypoint_direction(field_state.curr_loc_plus_delta, field_state.target_loc, mod_f(gyro_get_degrees(), 180));
        if(sextant != target_sextant){
            float distance_increment;
            if(sextant == mod_ui(old_sextant + direction, 6)){
                float new_distance;
                switch(direction){
                    case COUNTER_CLOCKWISE:
                        if(target_sextant < sextant)
                            target_sextant += 6;
                        new_distance = bound_waypoint_distance(field_state.distance_increment, field_state.target_loc_waypoint);
                        field_state.target_loc_waypoint = get_scaled_loc_int_param(bisecting_points[(2*sextant+2)%12], bisecting_points[(2*sextant+3)%12], new_distance);
                        break;
                    case CLOCKWISE:
                        if(target_sextant > sextant)
                            target_sextant -= 6;
                        new_distance = bound_waypoint_distance(field_state.distance_increment, field_state.target_loc_waypoint);
                        field_state.target_loc_waypoint = get_scaled_loc_int_param(bisecting_points[(2*sextant)%12], bisecting_points[(2*sextant + 1)%12], new_distance);
                        break;
                }
            }
            field_state.target_loc_plane = get_scaled_loc(field_state.target_loc_waypoint, OUTER_HEX_APATHEM_DIST);

        }
        else{
            field_state.target_loc_waypoint = field_state.target_loc;
        }
        field_state.curr_loc = field_state.curr_loc_plus_delta;
        
    }
    if((field_state.substage == TERRITORY_APPROACH_SUBSTAGE) && current_vel > 0){
        if(get_time() - field_state.stored_time > TERRITORY_TIMEOUT_TIME || game.coords[0].score != field_state.score){
            if(!(field_state.we_want_to_dump)){
                field_state.substage = DRIVE_SUBSTAGE;
                //SET MOTORS TO LEVER ARC THINGY
                if(what_to_do(sextant) == MINE_NOT_CAPTURE){
                    target_vel = 96;
                    current_vel = 96;
                    set_spinners(0);
                    //accelerate_time = CIRCLE_ACCELERATE_TIME;
                    //decelerate_distance = CIRCLE_DECELERATE_DISTANCE;
                    //KP_CIRCLE = 1.5;
                    //set_motors(55, 120);
                    field_state.stored_time = get_time();
                    field_state.drive_direction = FORWARD;
                    field_state.pid_enabled = TRUE;
                    field_state.circle_PID.enabled = true;
                    servo_set_pos(1, SERVO_UP);
                    set_new_destination(field_state.curr_loc_plus_delta, get_lever_pivot_point_loc(sextant));
                }
                else if(what_to_do(sextant) != LEAVE ){
                    set_spinners(0);
                    retreat_from_territory();
                    
                }
                else{
                    set_spinners(0);
                    uint8_t new_target_territory = get_target_territory();
                    int temp_direction = get_waypoint_direction(field_state.curr_loc_plus_delta, get_territory_pivot_point_loc(new_target_territory), mod_f(gyro_get_degrees(), 180));
                    switch(temp_direction){
                        case CLOCKWISE:
                            retreat_from_territory();
                            break;
                        case COUNTER_CLOCKWISE:
                            do_what_to_do(new_target_territory);
                            break;
                        default:
                            retreat_from_territory();
                            break;
                    }
                }
            }
            else{
                retreat_from_territory();
            }
        }
        if(get_time() - field_state.start_time > THINK_ABOUT_DUMPING_TIME){
            field_state.we_want_to_dump = TRUE;
        }
    }
    //printf("\nInitial substage: %d\n", field_state.substage);
    if((field_state.substage == LEVER_APPROACH_SUBSTAGE) && current_vel > 0){
        if(get_time() - field_state.stored_time > LEVER_APPROACH_TIME){
            //printf("\nInitial substage shit.: %d\n", field_state.substage);

            //SET MOTORS TO LEVER ARC THINGY
            //target_vel = 64;
            //current_vel = 64;
            //set_spinners(0);
            //accelerate_time = CIRCLE_ACCELERATE_TIME;
            //decelerate_distance = CIRCLE_DECELERATE_DISTANCE;
            //KP_CIRCLE = 1.5;
            //set_motors(55, 120);
            stop_motors();
            field_state.substage = LEVER_SUBSTAGE;
            field_state.stored_time = get_time();
            field_state.target_loc_waypoint = field_state.curr_loc_plus_delta;
            field_state.target_loc = field_state.curr_loc_plus_delta;
            field_state.pid_enabled = TRUE;
            field_state.stored_time = get_time();
            int score_temp = field_state.score;
            servo_set_pos(1, SERVO_DOWN);
            pause(CLICKY_CLICKY_TIME + 100);
            uint8_t rate_limit_old = rate_limit(get_current_sextant(field_state.curr_loc_plus_delta));
            while(1){
                copy_objects();
                field_state.opponent_loc.x = game.coords[1].x;
                field_state.opponent_loc.y = game.coords[1].y;
                field_state.curr_loc_plus_delta.x = game.coords[0].x;
                field_state.curr_loc_plus_delta.y = game.coords[0].y;
                if(get_time() - field_state.start_time > THINK_ABOUT_DUMPING_TIME){
                    field_state.we_want_to_dump = TRUE;
                }
                if(game.coords[0].score != score_temp){
                    score_temp += 40;
                    field_state.balls_held += 1;
                }
                //printf("Target Territory: %d", get_target_territory());
                if(get_target_territory() == sextant && what_to_do(sextant) == LEAVE && !field_state.we_want_to_dump && rate_limit(sextant) < rate_limit_old && rate_limit(sextant) != 0){
                    //printf("Un-bailed\n");
                    field_state.stored_time = get_time();
                }
                if((game.coords[0].score == field_state.score + 200 || field_state.stored_time + LEVER_TIMEOUT_TIME < get_time()) || balls_left(sextant) == 0){
                    //printf("Started to Bail, ");
                    if(!field_state.we_want_to_dump){
                        pause(50);
                        //printf("Continued to bail, ");
                        /*target_vel = TARGET_CIRCLE_VEL;
                        current_vel = 0;
                        accelerate_time = CIRCLE_ACCELERATE_TIME;
                        decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
                        KP_CIRCLE = KP_DRIVE;
                        field_state.stored_time = get_time();
                        field_state.start_drive_time = get_time();
                        field_state.drive_direction = BACKWARD;
                        field_state.circle_PID.enabled = true;
                        field_state.pid_enabled = true;
                        uint8_t temp_sextant = sextant;
                        if(temp_sextant == 0)
                            temp_sextant += 6;*/
                        uint8_t new_target_territory = get_target_territory();
                        if(new_target_territory != sextant && new_target_territory != ERROR){
                            //printf("three layers into bailing, ");
                            int temp_direction = get_waypoint_direction(field_state.curr_loc_plus_delta, get_territory_pivot_point_loc(new_target_territory), mod_f(gyro_get_degrees() + 180, 180));
                            switch(temp_direction){
                                case CLOCKWISE:
                                    retreat_from_lever();
                                    break;
                                case COUNTER_CLOCKWISE:
                                    do_what_to_do(new_target_territory);
                                    break;
                                default:
                                    retreat_from_lever();
                                    break;
                            }
                            break;
                        }
                        else if(field_state.we_want_to_dump){
                            //printf("3.5 layers into bailing, ");
                            pause(50);
                            if(get_waypoint_direction(field_state.curr_loc_plus_delta, get_dump_location_robot(pick_dump_sextant()), mod_f(gyro_get_degrees(), 180)) == CLOCKWISE){
                                retreat_from_lever();
                                break;
                            }
                            else{
                                get_your_ass_to_a_toilet();
                                break;
                            }
                        }
                        else{
                            //printf("last layer of bailing\n");
                            retreat_from_lever();
                            break;
                        }
                    }
                    else{
                        pause(50);
                        if(get_waypoint_direction(field_state.curr_loc_plus_delta, get_dump_location_robot(pick_dump_sextant()), mod_f(gyro_get_degrees(), 180)) == CLOCKWISE){
                            retreat_from_lever();
                            break;
                        }
                        else{
                            get_your_ass_to_a_toilet();
                            break;
                        }
                    }
                }
                rate_limit_old = rate_limit(sextant);
                servo_set_pos(1, SERVO_MIDDLE);
                pause(CLICKY_CLICKY_TIME);
                servo_set_pos(1, SERVO_DOWN);
                pause(CLICKY_CLICKY_TIME);
            }
        }
    }
    /*if(field_state.substage == TRANSITION_SUBSTAGE){
        if(get_time() - field_state.stored_time > TRANSITION_TIME){
            field_state.substage = LEVER_SUBSTAGE;
            stop_motors();
            field_state.stored_time = get_time();
            field_state.target_loc_waypoint = field_state.curr_loc_plus_delta;
            field_state.target_loc = field_state.curr_loc_plus_delta;
            field_state.pid_enabled = TRUE;
            field_state.circle_PID.enabled = true;
            while(1){
                copy_objects();
                if(game.coords[0].score == field_state.score + 200 || field_state.stored_time + LEVER_TIMEOUT_TIME < get_time() ){
                    field_state.substage = DRIVE_SUBSTAGE;
                    target_vel = 128;
                    current_vel = 0;
                    accelerate_time = CIRCLE_ACCELERATE_TIME;
                    decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
                    KP_CIRCLE = KP_DRIVE;
                    field_state.stored_time = get_time();
                    field_state.start_drive_time = get_time();
                    field_state.drive_direction = BACKWARD;
                    field_state.circle_PID.enabled = true;
                    uint8_t temp_sextant = sextant;
                    if(temp_sextant == 0)
                        temp_sextant += 6;
                    set_new_destination(field_state.curr_loc_plus_delta, get_territory_pivot_point_loc((temp_sextant - 1)%6));
                    break;
                }
                servo_set_pos(1, SERVO_DOWN);
                pause(CLICKY_CLICKY_TIME);
                servo_set_pos(1, SERVO_MIDDLE);
                pause(CLICKY_CLICKY_TIME);
            }
        }
    }*/
    /*else if((field_state.substage == LEVER_APPROACH_SUBSTAGE) && current_vel > 0){
        if(get_time() - field_state.stored_time > TERRITORY_TIMEOUT_TIME || game.coords[0].score != field_state.score){
            field_state.substage = LEVER_APPROACH_SUBSTAGE;
            target_vel = 64;
            current_vel = 0;
            set_spinners(0);
            //accelerate_time = CIRCLE_ACCELERATE_TIME;
            //decelerate_distance = CIRCLE_DECELERATE_DISTANCE;
            //KP_CIRCLE = 1.5;
            //field_state.start_drive_time = get_time();
            field_state.drive_direction = FORWARD;
            set_new_destination(field_state.curr_loc_plus_delta, get_lever_robot_loc((sextant)%6));
            
        }
    }*/
    
    if(get_time() - field_state.start_time > THINK_ABOUT_DUMPING_TIME){
        if(field_state.substage == DRIVE_SUBSTAGE){
            if(!field_state.we_want_to_dump)
                get_your_ass_to_a_toilet();
        }
        field_state.we_want_to_dump = TRUE;
    }
    
    int acceptable_error = get_acceptable_error(field_state.substage);
    dist_to_target = pythagorean(field_state.target_loc.x - field_state.curr_loc_plus_delta.x, field_state.target_loc.y - field_state.curr_loc_plus_delta.y);
    
    if(field_state.target_loc.x == field_state.target_loc_waypoint.x && field_state.target_loc.y == field_state.target_loc_waypoint.y){
        if(dist_to_target < acceptable_error){
            gyro_set_degrees(current_theta);
            /*if(field_state.substage == TERRITORY_APPROACH_SUBSTAGE){
                field_state.substage = TERRITORY_SUBSTAGE;
                stop_motors();
                set_spinners(0);
                current_vel = 0;
                target_vel = 0;
            }
            if(field_state.substage == LEVER_APPROACH_SUBSTAGE){
                field_state.substage = LEVER_SUBSTAGE;
                stop_motors();
                current_vel = 0;
                target_vel = 0;
            }*/
            if(field_state.substage == DRIVE_SUBSTAGE || field_state.substage == TERRITORY_RETREAT_SUBSTAGE || field_state.substage == LEVER_RETREAT_SUBSTAGE || field_state.substage == DUMPING_RETREAT_SUBSTAGE){
                if(field_state.stage == FIRST_STAGE)
                    field_state.stage = SECOND_STAGE;
                if(field_state.target_loc_waypoint.x == get_territory_pivot_point_loc(sextant).x && field_state.target_loc_waypoint.y == get_territory_pivot_point_loc(sextant).y){
                    if(!field_state.we_want_to_dump){
                        if(get_target_territory() == sextant){
                            if(what_to_do(sextant) == CAPTURE_AND_MINE || what_to_do(sextant) == CAPTURE_NOT_MINE){
                                set_new_destination(field_state.curr_loc_plus_delta, get_territory_robot_loc(sextant));
                                set_spinners( 255 * field_state.color);
                                target_vel = 64;
                                current_vel = 0;
                                field_state.start_drive_time = get_time();
                                accelerate_time = WALL_ACCELERATE_TIME;
                                decelerate_distance = (int)(WALL_DECELERATE_DISTANCE);
                                field_state.stored_time = get_time();
                                field_state.substage = TERRITORY_APPROACH_SUBSTAGE;
                                KP_CIRCLE = KP_APPROACH;
                                field_state.drive_direction = BACKWARD;
                            }
                            else if(what_to_do(sextant) == MINE_NOT_CAPTURE){
                                set_new_destination(field_state.curr_loc_plus_delta, get_lever_pivot_point_loc(sextant));
                                target_vel = 96;
                                current_vel = 96;
                                KP_CIRCLE = KP_APPROACH;
                                field_state.stored_time = get_time();
                                field_state.drive_direction = FORWARD;
                                field_state.pid_enabled = TRUE;
                                field_state.circle_PID.enabled = true;
                                servo_set_pos(1, SERVO_UP);
                                field_state.substage = DRIVE_SUBSTAGE;
                            }
                            else{
                                //what to do is LEAVE - which means we want to get to the lever anyway
                                set_new_destination(field_state.curr_loc_plus_delta, get_lever_pivot_point_loc(sextant));
                                target_vel = 96;
                                current_vel = 96;
                                KP_CIRCLE = KP_APPROACH;
                                field_state.stored_time = get_time();
                                field_state.drive_direction = FORWARD;
                                field_state.pid_enabled = TRUE;
                                field_state.circle_PID.enabled = true;
                                servo_set_pos(1, SERVO_UP);
                                field_state.substage = DRIVE_SUBSTAGE;
                            }
                        }
                        else{
                            uint8_t new_target = get_target_territory();
                            if(get_target_territory() != ERROR)
                                do_what_to_do(new_target);
                            else{
                                get_your_ass_to_a_toilet();
                            }
                        }
                    }
                    if(field_state.we_want_to_dump){
                        field_state.substage = PIVOT_SUBSTAGE;
                        uint8_t dump_sextant = pick_dump_sextant();
                        Location dump_loc = get_dump_location_robot(dump_sextant);
                        set_new_destination(field_state.curr_loc_plus_delta, dump_loc);
                        current_vel = 0;
                    }
                }
                else if(field_state.target_loc_waypoint.x == get_lever_pivot_point_loc(sextant).x && field_state.target_loc_waypoint.y == get_lever_pivot_point_loc(sextant).y){
                    if(field_state.substage != LEVER_RETREAT_SUBSTAGE){
                        //we want to go to the lever
                        if(get_target_territory() == sextant){
                            target_vel = 96;
                            current_vel = 0;
                            set_new_destination(field_state.curr_loc_plus_delta, get_lever_robot_loc(sextant));
                            decelerate_distance = (int)(WALL_DECELERATE_DISTANCE);
                            field_state.start_drive_time = get_time();
                            accelerate_time = WALL_ACCELERATE_TIME;
                            KP_CIRCLE = KP_APPROACH;
                            field_state.drive_direction = FORWARD;
                            field_state.substage = LEVER_APPROACH_SUBSTAGE;
                        }
                    }
                    else{
                        if(get_target_territory() != sextant && get_target_territory() != ERROR){
                            do_what_to_do(get_target_territory());
                        }
                        else if(field_state.we_want_to_dump)
                            get_your_ass_to_a_toilet();
                    }
                }
                else if(field_state.target_loc.x == get_dump_location_robot(sextant).x && field_state.target_loc.y == get_dump_location_robot(sextant).y){
                    if(field_state.substage != DUMPING_RETREAT_SUBSTAGE){
                        field_state.substage = DUMPING_SUBSTAGE;
                        set_new_destination(field_state.curr_loc_plus_delta, get_dump_location(sextant));
                        current_vel = 0;
                        target_vel = 64;
                        field_state.start_dump_time = get_time();
                        decelerate_distance = 0;
                        KP_CIRCLE = KP_APPROACH;
                        field_state.drive_direction = BACKWARD;
                    }
                    else{
                        do_what_to_do(get_target_territory());
                    }
                }
            }
        }
    }
    
    run_pivot_subroutine();
    
    if(field_state.substage == DUMPING_SUBSTAGE){
        if(get_time() - field_state.start_dump_time > 1000)
           servo_set_pos(0, 150);
        if(get_time() - field_state.start_dump_time > 1000 + time_during_dump){
            servo_set_pos(0, 300);
            field_state.balls_held = 0;
            time_during_dump = DUMP_BASE_CASE;
            field_state.we_want_to_dump = FALSE;
            retreat_from_dumping();
        }
    }
    if(current_x != field_state.curr_loc.x || current_y != field_state.curr_loc.y || current_theta != field_state.curr_angle){
        field_state.update_time = get_time();
        field_state.curr_loc.x = current_x;
        field_state.curr_loc.y = current_y;
        field_state.curr_angle = current_theta;
        ////printf("Lag: %lu, CX: %d, CY: %d, OX: %d, OY: %d, TX: %d, TY: %d, TWX: %d, TWY: %d, CA: %.2f, G: %.3f, LE: %d, RE: %d\n", get_time() - field_state.stored_time, current_x, current_y, field_state.curr_loc.x, field_state.curr_loc.y, field_state.target_loc.x, field_state.target_loc.y, field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y, field_state.curr_angle, gyro_get_degrees(), encoder_read(LEFT_ENCODER_PORT), encoder_read(RIGHT_ENCODER_PORT));
        //field_state.stored_time = get_time();
        if(field_state.substage == DRIVE_SUBSTAGE)
            gyro_set_degrees(current_theta);
        
    }
    
    field_state.curr_time = get_time() + field_state.start_time;
    
    /*if(field_state.curr_time - field_state.update_time >= DRIVE_TIMEOUT_TIME && !is_decelerating() && field_state.substage == DRIVE_SUBSTAGE && dist_to_target < acceptable_error){
        //We hit a wall...or another robot, but we haven't coded robot-ramming timeouts yet...
        float distance = pythagorean(field_state.curr_loc_plus_delta.x, field_state.curr_loc_plus_delta.y);
        if(fabs(distance - get_min_distance_loc_param(field_state.curr_loc)) < fabs(distance - get_max_distance_loc_param(field_state.curr_loc))){
            //We hit the inner hex
            
        }
    }*/
    
    field_state.score = game.coords[0].score;

}

int usetup(void){
    robot_id = 1; //team 1
    gyro_init(GYRO_PORT, GYRO_DEGREE_CONVERSION, GYRO_CALIB_WAIT_TIME);
    servo_set_pos(0, 150);
    servo_set_pos(1, SERVO_DOWN);
    set_spinners(255);
    pause(500);
    servo_set_pos(0, 300);
    servo_set_pos(1, SERVO_UP);
    set_spinners(0);
    pause(500);
    return 0;
}


void initialize(){
    float skew = 0;//find_skew(field_state.curr_loc);
    build_bisecting_points(skew, &bisecting_points);
    build_lever_pivot_points(&lever_pivot_points);
    build_territory_pivot_points(&territory_pivot_points);
    field_state.stage = FIRST_STAGE;
    field_state.drive_direction = BACKWARD;
    field_state.substage = PIVOT_SUBSTAGE;
    accelerate_time = DRIVE_ACCELERATE_TIME;
    field_state.pid_enabled = TRUE;
    decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST + 4*INCHES_TO_PIXELS;
    field_state.stored_time = get_time();
    field_state.curr_time = get_time();
    field_state.score = 0;
    target_vel = TARGET_CIRCLE_VEL;
    field_state.start_time = get_time();
    uint8_t changed_last_update = FALSE;
    field_state.encoder_value = encoder_read(FREEWHEEL_ENCODER_PORT);
    field_state.balls_held = 0;
    field_state.tries = 0;
    current_vel = 0;
    update_field();
    log_init(30000);
    uint8_t sextant = get_current_sextant(field_state.curr_loc);
    if(sextant == 0){
        field_state.color = BLUE;
    }
    else if(sextant == 3){
        field_state.color = RED;
    }
    else{
        field_state.color = 2;
    }
    Location target_loc = get_territory_pivot_point_loc(mod_ui(sextant - 1, 6));
    set_new_destination(field_state.curr_loc, target_loc);
    update_field();
    servo_set_pos(0, 300);
}


int umain(){
    robot_id = 1;
    copy_objects();
    copy_objects();
    field_state.curr_loc.x = (int)(game.coords[0].x * VPS_RATIO);
    field_state.curr_loc.y = (int)(game.coords[0].y * VPS_RATIO);
    field_state.curr_angle = (((float)game.coords[0].theta) * 180.0) / 2048;
    gyro_set_degrees(field_state.curr_angle);

        
    initialize();
    
    
    /*for(int sextant = 0; sextant < 6; sextant++){
        //printf("TPX: %d, TPY: %d, LPX: %d, LPY: %d\n", get_territory_pivot_point_loc(sextant).x, get_territory_pivot_point_loc(sextant).y, lever_pivot_points[2*sextant], lever_pivot_points[2*sextant + 1]);
    }*/
    init_pid(&field_state.circle_PID, KP_CIRCLE, KI_CIRCLE, KD_CIRCLE, &get_angle_error_circle, &update_circle_velocities);
    field_state.circle_PID.enabled = true;
    field_state.circle_PID.goal = 0;
    
    ////printf("OX: %d, OY: %d, TX: %d, TY: %d, TWX: %d, TWY: %d, CA: %.2f, G: %.3f, LE: %d, RE: %d\n", field_state.curr_loc.x, field_state.curr_loc.y, field_state.target_loc.x, field_state.target_loc.y, field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y, field_state.curr_angle, gyro_get_degrees(), encoder_read(LEFT_ENCODER_PORT), encoder_read(RIGHT_ENCODER_PORT));
    
    while(1){
        update_field();
        accelerate();
        decelerate();
        if(field_state.pid_enabled)
            update_pid(&field_state.circle_PID);
        ////printf("Sextant: %d, OX: %d, OY: %d, TX: %d, TY: %d, TWX: %d, TWY: %d, CA: %.2f, G: %.3f, LE: %d, RE: %d\n", get_current_sextant(field_state.curr_loc), field_state.curr_loc.x, field_state.curr_loc.y, field_state.target_loc.x, field_state.target_loc.y, field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y, field_state.curr_angle, gyro_get_degrees(), encoder_read(LEFT_ENCODER_PORT), encoder_read(RIGHT_ENCODER_PORT));

    }
    return 0;
}
