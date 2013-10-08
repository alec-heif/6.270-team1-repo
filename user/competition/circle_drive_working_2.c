//
//  circle_drive.c
//  
//
//  Created by aheifetz on 1/19/13.
//
//

#include <in_field_working_2.c>

extern volatile uint8_t robot_id;

#define SECOND 2
#define THIRD 3

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
    else if(field_state.substage == DRIVE_SUBSTAGE || field_state.substage == DUMPING_SUBSTAGE || field_state.stage == PIVOT_SUBSTAGE || field_state.stage == LEVER_RETREAT_SUBSTAGE || field_state.substage == TERRITORY_RETREAT_SUBSTAGE){
        if(field_state.drive_direction == BACKWARD)
            field_state.target_angle = mod_f((float)(degrees(atan2(field_state.target_loc_waypoint.y - field_state.curr_loc_plus_delta.y, field_state.target_loc_waypoint.x - field_state.curr_loc_plus_delta.x)) + 180), 180);
        else
            field_state.target_angle = mod_f((float)(degrees(atan2(field_state.target_loc_waypoint.y - field_state.curr_loc_plus_delta.y, field_state.target_loc_waypoint.x - field_state.curr_loc_plus_delta.x))), 180);
    }
    float current_angle;
    current_angle = mod_f(gyro_get_degrees(), 180);
    float error = (field_state.target_angle - current_angle);
    error = mod_f(error, 180);
    //printf("Target Angle: %f, Current Angle: %f, Error: %f\n", target_angle, current_angle, error);
    return error;
}

uint8_t get_target_territory(Location curr_loc){
    uint8_t curr_sextant = get_current_sextant(curr_loc);
    //For now, just get the territory to the right as the target...should change later to be intelligent
    
}

void run_pivot_subroutine(){
    if(field_state.substage == PIVOT_SUBSTAGE && fabs(get_angle_error_circle()) < ROTATION_THRESHOLD){
        
        if(field_state.we_want_to_dump){
            get_your_ass_to_a_toilet();
        }
        else{
            start_driving_somewhere(get_territory_pivot_point_loc(get_target_territory(field_state.curr_loc_plus_delta)));
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
            //printf("Left Vel: %d, Right Vel: %d\n", current_vel, current_vel - pwr_change);
        }
        else{
            set_motors(dir*current_vel - dir*pwr_change, dir*current_vel);
            //printf("Left Vel: %d, Right Vel: %d\n", current_vel - pwr_change, current_vel);
        }
    }
    else if(field_state.substage == TERRITORY_APPROACH_SUBSTAGE || field_state.substage == LEVER_APPROACH_SUBSTAGE || field_state.substage == DUMPING_SUBSTAGE || field_state.substage == PIVOT_SUBSTAGE || field_state.substage == LEVER_RETREAT_SUBSTAGE || field_state.substage == TERRITORY_RETREAT_SUBSTAGE){
        if(fabs(mod_f(field_state.target_angle - mod_f(gyro_get_degrees(), 180), 180)) < ROTATION_THRESHOLD){
            if(current_vel == 0){
                //used to know when we started moving toward the lever/territory
                field_state.stored_time = field_state.curr_time;
                current_vel = target_vel;
            }
        }
        if(error*dir > 0){
            set_motors(dir*current_vel + dir*pwr_change, dir*current_vel - dir*pwr_change);
            //printf("Left Vel: %d, Right Vel: %d\n", current_vel, current_vel - pwr_change);
        }
        else{
            set_motors(dir*current_vel - dir*pwr_change, dir*current_vel + dir*pwr_change);
            //printf("Left Vel: %d, Right Vel: %d\n", current_vel - pwr_change, current_vel);
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
        if(field_state.substage != TERRITORY_SUBSTAGE && field_state.substage != LEVER_SUBSTAGE && field_state.substage != TRANSITION_SUBSTAGE && field_state.stage != PIVOT_SUBSTAGE){
            float distance = (pythagorean(field_state.target_loc.x - field_state.curr_loc_plus_delta.x, field_state.target_loc.y - field_state.curr_loc_plus_delta.y));
            //float new_target_vel = scale_values(pythagorean(field_state.target_loc.x - field_state.curr_loc_plus_delta.x, field_state.target_loc.y - field_state.curr_loc_plus_delta.y), decelerate_distance, target_vel);
            if(distance < CIRCLE_DECELERATE_DISTANCE_FIRST){
                current_vel = 96;
                target_vel = 96;
                KP_CIRCLE = 1.0;
            }
            if(distance < CIRCLE_DECELERATE_DISTANCE_SECOND){
                current_vel = 64;
                target_vel = 64;
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
        case LEVER_APPROACH_SUBSTAGE:
            return (int)(0*INCHES_TO_PIXELS);
            break;
        case TERRITORY_RETREAT_SUBSTAGE:
        case LEVER_RETREAT_SUBSTAGE:
            return (int)(6*INCHES_TO_PIXELS);
            break;
        case DRIVE_SUBSTAGE:
            if(target_vel != 64)
                return (int)(4*(INCHES_TO_PIXELS));
            else
                return(int)(4*INCHES_TO_PIXELS);
            break;
        default:
            return (int)(3*INCHES_TO_PIXELS);
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
    if(field_state.stage == TERRITORY_SUBSTAGE){
        retreat_from_territory();
    }
        servo_set_pos(0, 150);
}


void update_field(){
    copy_objects();
    int current_x = (int)(game.coords[0].x * VPS_RATIO);
    int current_y = (int)(game.coords[0].y * VPS_RATIO);
    uint8_t has_encoder_moved = FALSE;
    /*if(get_time() > field_state.last_encoder_update + ENCODER_UPDATE_TIME){
        if(encoder_read(FREEWHEEL_ENCODER_PORT) != field_state.encoder_value){
            field_state.robot_stopped = TRUE;
            field_state.encoder_value = encoder_read(FREEWHEEL_ENCODER_PORT);
            field_state.
        }
        else{
            field_state.robot_stopped = TRUE;
        }

    }*/
    
    //printf("Current X: %d, Current Y: %d\n", current_x, current_y);
    float current_theta = (((float)game.coords[0].theta) * 180.0) / 2048;
    field_state.curr_loc_plus_delta.x = current_x; //+ get_delta(field_state.curr_loc).x;
    field_state.curr_loc_plus_delta.y = current_y; //+ get_delta(field_state.curr_loc).y;
    uint8_t sextant = get_current_sextant(field_state.curr_loc_plus_delta);
    uint8_t old_sextant = get_current_sextant(field_state.curr_loc);
    
    //printf("Current Distance: %f, Waypoint Distance: %f, Target Distance: %f, Distance Increment: %f\n", pythagorean(field_state.curr_loc_plus_delta.x, field_state.curr_loc_plus_delta.y), pythagorean(field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y), pythagorean(field_state.target_loc.x, field_state.target_loc.y), field_state.distance_increment);
    float dist_to_waypoint = pythagorean_loc(field_state.curr_loc_plus_delta, field_state.target_loc_waypoint);
    if(sextant != old_sextant ){//|| (get_angle_error_circle() > 90 && dist_to_waypoint < CIRCLE_DECELERATE_DISTANCE_SECOND){
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
        int direction = get_waypoint_direction(field_state.curr_loc_plus_delta, field_state.target_loc, gyro_get_degrees());
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
        encoder_reset(LEFT_ENCODER_PORT);
        encoder_reset(RIGHT_ENCODER_PORT);
        
    }
    if((field_state.substage == TERRITORY_APPROACH_SUBSTAGE) && current_vel > 0){
        if(get_time() - field_state.stored_time > TERRITORY_TIMEOUT_TIME || game.coords[0].score != field_state.score){
            if(!(field_state.we_want_to_dump)){
                field_state.substage = DRIVE_SUBSTAGE;
                //SET MOTORS TO LEVER ARC THINGY
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
            else{
                retreat_from_territory();
            }
        }
        if(get_time() - field_state.start_time > THINK_ABOUT_DUMPING_TIME){
            field_state.we_want_to_dump = TRUE;
        }
    }
    if((field_state.substage == LEVER_APPROACH_SUBSTAGE) && current_vel > 0){
        if(get_time() - field_state.stored_time > LEVER_APPROACH_TIME){
            field_state.substage = LEVER_SUBSTAGE;
            //SET MOTORS TO LEVER ARC THINGY
            //target_vel = 64;
            //current_vel = 64;
            //set_spinners(0);
            //accelerate_time = CIRCLE_ACCELERATE_TIME;
            //decelerate_distance = CIRCLE_DECELERATE_DISTANCE;
            //KP_CIRCLE = 1.5;
            //set_motors(55, 120);
            stop_motors();
            field_state.stored_time = get_time();
            field_state.target_loc_waypoint = field_state.curr_loc_plus_delta;
            field_state.target_loc = field_state.curr_loc_plus_delta;
            field_state.pid_enabled = TRUE;
            field_state.stored_time = get_time();
            int score_temp = field_state.score;
            servo_set_pos(1, SERVO_DOWN);
            pause(CLICKY_CLICKY_TIME + 100);
            while(1){
                copy_objects();
                if(get_time() - field_state.start_time > THINK_ABOUT_DUMPING_TIME){
                    field_state.we_want_to_dump = TRUE;
                }
                if(game.coords[0].score != score_temp){
                    score_temp += 40;
                    field_state.balls_held += 1;
                }
                if(game.coords[0].score == field_state.score + 200 || field_state.stored_time + LEVER_TIMEOUT_TIME < get_time() ){
                    if(!field_state.we_want_to_dump){
                        pause(50);
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
                        uint8_t temp_sextant = sextant;
                        if(temp_sextant == 0)
                            temp_sextant += 6;
                        set_new_destination(field_state.curr_loc_plus_delta, get_territory_pivot_point_loc(get_target_territory(field_state.curr_loc_plus_delta)));
                        break;
                    }
                    else{
                        pause(50);
                        get_your_ass_to_a_toilet();
                        break;
                    }
                }
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
    float dist_to_target = pythagorean(field_state.target_loc.x - field_state.curr_loc_plus_delta.x, field_state.target_loc.y - field_state.curr_loc_plus_delta.y);
    
    if(field_state.target_loc.x == field_state.target_loc_waypoint.x && field_state.target_loc.y == field_state.target_loc_waypoint.y){
        if(dist_to_target < acceptable_error){
            gyro_set_degrees(current_theta);
            encoder_reset(LEFT_ENCODER_PORT);
            encoder_reset(RIGHT_ENCODER_PORT);
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
            if(field_state.substage == DRIVE_SUBSTAGE || field_state.substage == TERRITORY_RETREAT_SUBSTAGE || field_state.substage == LEVER_RETREAT_SUBSTAGE){
                if(field_state.stage == FIRST_STAGE)
                    field_state.stage = SECOND_STAGE;
                if(field_state.target_loc.x == get_territory_pivot_point_loc(sextant).x && field_state.target_loc.y == get_territory_pivot_point_loc(sextant).y){
                    if(field_state.substage != TERRITORY_RETREAT_SUBSTAGE){
                        set_new_destination(field_state.curr_loc_plus_delta, get_territory_robot_loc(sextant));
                        set_spinners( 229 * field_state.color);
                        target_vel = 64;
                        current_vel = 0;
                        field_state.start_drive_time = get_time();
                        accelerate_time = WALL_ACCELERATE_TIME;
                        decelerate_distance = (int)(WALL_DECELERATE_DISTANCE);
                        field_state.substage = TERRITORY_APPROACH_SUBSTAGE;
                        KP_CIRCLE = KP_APPROACH;
                        field_state.drive_direction = BACKWARD;
                    }
                    else{
                        field_state.substage = PIVOT_SUBSTAGE;
                        uint8_t dump_sextant = pick_dump_sextant();
                        Location dump_loc = get_dump_location_robot(dump_sextant);
                        set_new_destination(field_state.curr_loc_plus_delta, dump_loc);
                        current_vel = 0;
                    }
                }
                else if(field_state.target_loc.x == get_lever_pivot_point_loc(sextant).x && field_state.target_loc.y == get_lever_pivot_point_loc(sextant).y){
                    if(field_state.substage != LEVER_RETREAT_SUBSTAGE){
                        target_vel = 64;
                        current_vel = 0;
                        set_new_destination(field_state.curr_loc_plus_delta, get_lever_robot_loc(sextant));
                        decelerate_distance = (int)(WALL_DECELERATE_DISTANCE);
                        field_state.start_drive_time = get_time();
                        accelerate_time = WALL_ACCELERATE_TIME;
                        KP_CIRCLE = KP_APPROACH;
                        field_state.drive_direction = FORWARD;
                        field_state.substage = LEVER_APPROACH_SUBSTAGE;
                    }
                    else{
                        field_state.substage = PIVOT_SUBSTAGE;
                        //WE WILL PROBABLY NEVER USE LEVER_RETREAT_SUBSTAGE...
                    }
                }
                else if(field_state.target_loc.x == get_dump_location_robot(sextant).x && field_state.target_loc.y == get_dump_location_robot(sextant).y){
                    field_state.substage = DUMPING_SUBSTAGE;
                    set_new_destination(field_state.curr_loc_plus_delta, get_dump_location(sextant));
                    current_vel = 0;
                    target_vel = 64;
                    field_state.start_dump_time = get_time();
                    decelerate_distance = 0;
                    KP_CIRCLE = KP_APPROACH;
                    field_state.drive_direction = BACKWARD;
                }
            }
        }
    }
    
    run_pivot_subroutine();
    
    if(field_state.substage == DUMPING_SUBSTAGE){
        if(get_time() - field_state.start_dump_time > 4000)
           servo_set_pos(0, 150);
        if(get_time() - field_state.start_dump_time > 4000 + time_during_dump){
            servo_set_pos(0, 300);
        }
    }
    if(current_x != field_state.curr_loc.x || current_y != field_state.curr_loc.y || current_theta != field_state.curr_angle){
        field_state.update_time = get_time();
        field_state.curr_loc.x = current_x;
        field_state.curr_loc.y = current_y;
        field_state.curr_angle = current_theta;
        //printf("Lag: %lu, CX: %d, CY: %d, OX: %d, OY: %d, TX: %d, TY: %d, TWX: %d, TWY: %d, CA: %.2f, G: %.3f, LE: %d, RE: %d\n", get_time() - field_state.stored_time, current_x, current_y, field_state.curr_loc.x, field_state.curr_loc.y, field_state.target_loc.x, field_state.target_loc.y, field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y, field_state.curr_angle, gyro_get_degrees(), encoder_read(LEFT_ENCODER_PORT), encoder_read(RIGHT_ENCODER_PORT));
        //field_state.stored_time = get_time();
        encoder_reset(LEFT_ENCODER_PORT);
        encoder_reset(RIGHT_ENCODER_PORT);
        if(field_state.substage == DRIVE_SUBSTAGE)
            gyro_set_degrees(current_theta);
        
    }
    
    field_state.curr_time = get_time();
    
    //***TIMEOUTS***
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
    decelerate_distance = CIRCLE_DECELERATE_DISTANCE_FIRST;
    field_state.stored_time = get_time();
    field_state.curr_time = get_time();
    target_vel = TARGET_CIRCLE_VEL;
    field_state.start_time = get_time();
    uint8_t changed_last_update = FALSE;
    field_state.encoder_value = encoder_read(FREEWHEEL_ENCODER_PORT);
    field_state.balls_held = 0;
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
        printf("TPX: %d, TPY: %d, LPX: %d, LPY: %d\n", get_territory_pivot_point_loc(sextant).x, get_territory_pivot_point_loc(sextant).y, lever_pivot_points[2*sextant], lever_pivot_points[2*sextant + 1]);
    }*/
    init_pid(&field_state.circle_PID, KP_CIRCLE, KI_CIRCLE, KD_CIRCLE, &get_angle_error_circle, &update_circle_velocities);
    field_state.circle_PID.enabled = true;
    field_state.circle_PID.goal = 0;
    
    //printf("OX: %d, OY: %d, TX: %d, TY: %d, TWX: %d, TWY: %d, CA: %.2f, G: %.3f, LE: %d, RE: %d\n", field_state.curr_loc.x, field_state.curr_loc.y, field_state.target_loc.x, field_state.target_loc.y, field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y, field_state.curr_angle, gyro_get_degrees(), encoder_read(LEFT_ENCODER_PORT), encoder_read(RIGHT_ENCODER_PORT));
    
    while(1){
        update_field();
        accelerate();
        decelerate();
        if(field_state.pid_enabled)
            update_pid(&field_state.circle_PID);
        if(get_time() > 119000){
            
        }
        //printf("Sextant: %d, OX: %d, OY: %d, TX: %d, TY: %d, TWX: %d, TWY: %d, CA: %.2f, G: %.3f, LE: %d, RE: %d\n", get_current_sextant(field_state.curr_loc), field_state.curr_loc.x, field_state.curr_loc.y, field_state.target_loc.x, field_state.target_loc.y, field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y, field_state.curr_angle, gyro_get_degrees(), encoder_read(LEFT_ENCODER_PORT), encoder_read(RIGHT_ENCODER_PORT));

    }
    return 0;
}
