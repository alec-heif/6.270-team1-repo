//
//  in_field.c
//  
//
//  Created by aheifetz on 1/17/13.
//
//

//#include <stdio.h>
#include <umain_working_2.c>


#define OUTER_SKEW_MODIFIER -30
//ROBOT_CENTER_DISTANCE = (25.75/2 + 23.75 - 4.25) * INCHES_TO_PIXELS
#define ROBOT_CENTER_DISTANCE 1196.2 //2
//INNER_HEX_VERTEX_DIST = 25.75/2 * 2/sqrt(3)*INCHES_TO_PIXELS
#define INNER_HEX_VERTEX_DIST 549.3
#define INNER_HEX_APATHEM_DIST 475.7
#define OUTER_HEX_VERTEX_DIST 2048 //48 inches*2/sqrt(3)*INCHES_TO_PIXELS
#define OUTER_HEX_APATHEM_DIST 1773.6 //48 inches * INCHES_TO_PIXELS
#define LEVER_PIVOT_POINT_CENTER_DIST (1264.4 - 100)
#define TERRITORY_PIVOT_POINT_CENTER_DIST (1264.4 - 130)
#define PIVOT_POINT_SEXTANT_DIST (517)


#define RIGHT 1
#define LEFT -1

//assuming 0 degrees is to the right and sextant zero is the one from zero to sixty

//this assumes the outer hex to be "normal" so inner hex has a skew of +30 degrees by default

float radians(float degrees){
    return degrees * M_PI / 180.0;
}

float degrees(float radians){
    return (radians*180.0) / M_PI;
}

uint8_t which_sextant(int32_t cross_product_s0e3, int32_t cross_product_s1e4, int32_t cross_product_s2e5){
    
    //Edited to change from starting sextants being 0 and 3 to 1 and 4
    if(cross_product_s0e3 > 0){
        //Sextant 1, 2, or 3
        if(cross_product_s1e4 > 0){
            //Sextant 2 or 3
            if(cross_product_s2e5 > 0){
                //Sextant 2
                return 2;
            }
            else{
                //Sextant 1
                return 1;
            }
        }
        else{
            //Sextant 0
            return 0;
        }
    }
    else{
        //Sextant 4, 5, or 0
        if(cross_product_s1e4 < 0){
            //Sextant 5 or 0
            if(cross_product_s2e5 < 0){
                //Sextant 5
                return 5;
            }
            else{
                //Sextant 4
                return 4;
            }
        }
        else{
            //Sextant 3
            return 3;
        }
    }
}

float find_skew(Location robot_start){
    int sign = 0;
    int expected_angle;
    if(robot_start.x < 0){
        sign = 1;
        expected_angle = -120;
    }
    else{
        sign = -1;
        expected_angle = 60;
    }
    float leftward_drift = ROBOT_CENTER_DISTANCE*cos(radians(expected_angle)) - robot_start.x;
    float downward_drift = ROBOT_CENTER_DISTANCE*sin(radians(expected_angle)) - robot_start.y;
    int skew_sign = 0;
    if(leftward_drift*sign > 0 && downward_drift*sign > 0){
        //skew is clockwise
        skew_sign = -1;
    }
    else if(leftward_drift*sign < 0 && downward_drift*sign <= 0){
        //skew is counterclockwise
        skew_sign = 1;
    }
    else{
        //assume there is no skew since it's either translated more than it's rotated, in which case skew is less significant and we have bigger problems, or that it's exactly evenly lined up, in which case Isaac and company are awesome
        skew_sign = 0;
    }
    //float start_translation_dist = pythagorean(leftward_drift, downward_drift);
    //float skew = 2 * skew_sign * (asin(start_translation_dist/2.0*ROBOT_CENTER_DISTANCE));
    float current_angle = (float)atan2(robot_start.y, robot_start.x)*(180/M_PI);
    return current_angle - expected_angle;
}

void build_bisecting_points(int skew, int *bp){
    
    //float outer_hex_apathem = OUTER_HEX_VERTEX_DIST*2/sqrt(3);
    //float side_length = OUTER_HEX_VERTEX_DIST;
    for(int sextant = 0; sextant < 3; sextant++){
        Location start_loc;
        Location end_loc;
        start_loc.x = (int)(cos(radians(skew + 60.0*sextant - 30.0)) * OUTER_HEX_APATHEM_DIST);
        start_loc.y = (int)(sin(radians(skew + 60.0*sextant - 30.0)) * OUTER_HEX_APATHEM_DIST);
        end_loc.x = -1 * start_loc.x;
        end_loc.y = -1 * start_loc.y;
        *(bp + 2*sextant) = start_loc.x;
        *(bp + 2*sextant + 1) = start_loc.y;
        *(bp + 2*sextant + 3*2) = end_loc.x;
        *(bp + 2*sextant + 3*2 + 1) = end_loc.y;
    }
}

Location delta_to_lever(uint8_t sextant){
    Location loc;
    int bp_x = bisecting_points[2*sextant];
    int bp_y = bisecting_points[2*sextant + 1];
    float bp_theta = degrees(atan2(bp_y, bp_x));
    loc.x = (int)(cos(radians(bp_theta + 90))*PIVOT_POINT_SEXTANT_DIST);
    loc.y = (int)(sin(radians(bp_theta + 90))*PIVOT_POINT_SEXTANT_DIST);
    return loc;
}

Location delta_to_territory(uint8_t sextant){
    Location loc;
    int bp_x = bisecting_points[(2*(sextant+1))%12];
    int bp_y = bisecting_points[(2*(sextant+1) + 1)%12];
    float bp_theta = degrees(atan2(bp_y, bp_x));
    loc.x = (int)(cos(radians(bp_theta - 90))*PIVOT_POINT_SEXTANT_DIST);
    loc.y = (int)(sin(radians(bp_theta - 90))*PIVOT_POINT_SEXTANT_DIST);
    return loc;
}

Location delta_to_lever_pivot(uint8_t sextant){
    Location loc;
    int bp_x = bisecting_points[2*sextant];
    int bp_y = bisecting_points[2*sextant + 1];
    float bp_theta = degrees(atan2(bp_y, bp_x));
    loc.x = (int)(cos(radians(bp_theta + 180))*(OUTER_HEX_APATHEM_DIST - LEVER_PIVOT_POINT_CENTER_DIST));
    loc.y = (int)(sin(radians(bp_theta + 180))*(OUTER_HEX_APATHEM_DIST - LEVER_PIVOT_POINT_CENTER_DIST));
    return loc;
}

Location delta_to_territory_pivot(uint8_t sextant){
    Location loc;
    int bp_x = bisecting_points[(2*(sextant+1))%12];
    int bp_y = bisecting_points[(2*(sextant+1) + 1)%12];
    float bp_theta = degrees(atan2(bp_y, bp_x));
    loc.x = (int)(cos(radians(bp_theta + 180))*(OUTER_HEX_APATHEM_DIST - TERRITORY_PIVOT_POINT_CENTER_DIST));
    loc.y = (int)(sin(radians(bp_theta + 180))*(OUTER_HEX_APATHEM_DIST - TERRITORY_PIVOT_POINT_CENTER_DIST));
    return loc;
}

Location get_territory_pivot_point_loc(uint8_t sextant){
    Location loc;
    loc.x = territory_pivot_points[2*sextant];
    loc.y = territory_pivot_points[2*sextant + 1];
    return loc;
}

Location get_lever_pivot_point_loc(uint8_t sextant){
    Location loc;
    loc.x = lever_pivot_points[2*sextant];
    loc.y = lever_pivot_points[2*sextant + 1];
    return loc;
}

Location get_territory_loc(uint8_t sextant){
    Location delta_1 = delta_to_territory(sextant);
    //Location delta_2 = delta_to_territory_pivot(sextant);
    //delta_2.x = (int)(delta_2.x / 5);
    //delta_2.y = (int)(delta_2.y / 5);
    Location loc;
    loc.x = bisecting_points[(2*(sextant + 1))%12] + delta_1.x;
    loc.y = bisecting_points[(2*(sextant + 1) + 1)%12] + delta_1.y;
    return loc;
}

Location get_territory_robot_loc(uint8_t sextant){
    Location delta_1 = delta_to_territory(sextant);
    Location delta_2 = delta_to_territory_pivot(sextant);
    delta_2.x = (int)(2 * delta_2.x / 5);
    delta_2.y = (int)(2 * delta_2.y / 5);
    Location loc;
    loc.x = bisecting_points[(2*(sextant + 1))%12] + delta_1.x + delta_2.x;
    loc.y = bisecting_points[(2*(sextant + 1) + 1)%12] + delta_1.y + delta_2.x;
    return loc;
}

Location get_lever_loc(uint8_t sextant){
    Location delta_1 = delta_to_lever(sextant);
    //Location delta_2 = delta_to_lever_pivot(sextant);
    //delta_2.x = (int)(3* delta_2.x / 5);
    //delta_2.y = (int)(3 * delta_2.y / 5);
    Location loc;
    loc.x = bisecting_points[(2*(sextant))%12] + delta_1.x;
    loc.y = bisecting_points[(2*(sextant) + 1)%12] + delta_1.y;
    return loc;
}

Location get_lever_robot_loc(uint8_t sextant){
    Location delta_1 = delta_to_lever(sextant);
    Location delta_2 = delta_to_lever_pivot(sextant);
    delta_2.x = (int)(2* delta_2.x / 5);
    delta_2.y = (int)(2 * delta_2.y / 5);
    Location loc;
    loc.x = bisecting_points[(2*(sextant))%12] + delta_1.x + delta_2.x;
    loc.y = bisecting_points[(2*(sextant) + 1)%12] + delta_1.y + delta_2.y;
    return loc;
}

void build_lever_pivot_points(int *lpb){
    for(int sextant = 0; sextant < 6; sextant++){
        Location loc;
        int bp_x = bisecting_points[(2*(sextant))%12];
        int bp_y = bisecting_points[(2*(sextant) + 1)%12];
        Location delta_1 = delta_to_lever(sextant);
        Location delta_2 = delta_to_lever_pivot(sextant);
        int new_x = bp_x + (delta_1.x) + (delta_2.x);
        int new_y = bp_y + (delta_1.y) + (delta_2.y);
        *(lpb + 2*sextant) = new_x;
        *(lpb + 2*sextant + 1) = new_y;
    }
}

void build_territory_pivot_points(int *tpb){
    for(int sextant = 0; sextant < 6; sextant++){
        int bp_x = bisecting_points[(2*(sextant+1))%12];
        int bp_y = bisecting_points[(2*(sextant+1) + 1)%12];
        Location delta_1 = delta_to_territory(sextant);
        Location delta_2 = delta_to_territory_pivot(sextant);
        int new_x = bp_x + (delta_1.x) + (delta_2.x);
        int new_y = bp_y + (delta_1.y) + (delta_2.y);
        *(tpb + 2*(sextant)) = new_x;
        *(tpb + 2*(sextant) + 1) = new_y;
        
    }
}


int32_t dot_product_three_loc(Location point_loc, Location start_loc, Location end_loc){
    Location vectorA;
    vectorA.x = point_loc.x - start_loc.x;
    vectorA.y = point_loc.y - start_loc.y;
    Location vectorB;
    vectorB.x = end_loc.x - start_loc.x;
    vectorB.y = end_loc.y - start_loc.y;
    return (int32_t)((int32_t)vectorA.x * (int32_t)vectorB.x + (int32_t)vectorA.y * (int32_t)vectorB.y);
}

uint8_t get_sextant_difference(int curr_sextant, int target_sextant, int direction){
    switch(direction){
        case COUNTER_CLOCKWISE:
            if(target_sextant < curr_sextant)
                target_sextant+=6;
            return (uint8_t)(target_sextant - curr_sextant);
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
    return 0;
}

int32_t cross_product_int_param(int point_loc_x, int point_loc_y, int start_vec_x, int start_vec_y, int end_vec_x, int end_vec_y){
    Location bisecting_vector;
    bisecting_vector.x = end_vec_x - start_vec_x;
    bisecting_vector.y = end_vec_y - start_vec_y;
    Location robot_vector;
    robot_vector.x = point_loc_x - start_vec_x;
    robot_vector.y = point_loc_y - start_vec_y;
    //printf("Returned Cross Product: %ld\n", ((((int32_t)robot_vector.x) * ((int32_t)bisecting_vector.y)) - (((int32_t)robot_vector.y) * ((int32_t)bisecting_vector.x))));
    return ((((int32_t)robot_vector.x) * ((int32_t)bisecting_vector.y)) - (((int32_t)robot_vector.y) * ((int32_t)bisecting_vector.x)));
}

int32_t cross_product_loc_param(Location point_loc, Location start_vec, Location end_vec){
    return cross_product_int_param(point_loc.x, point_loc.y, start_vec.x, start_vec.y, end_vec.x, end_vec.y);
}

uint8_t get_current_sextant(Location point_loc){
    
    //take start_loc->point_loc (so, point_loc-start_loc) x start_loc->end_loc, if >0 then curr_loc right of line, if <0, curr_loc left of line
    //cross product is AxBy - AyBx
    //so, sP.x*se.y - sP.y*se.x
    
    int32_t cross_product_s0e3 = cross_product_int_param(point_loc.x, point_loc.y, bisecting_points[0], bisecting_points[1], bisecting_points[6], bisecting_points[7]);
        
    // WAS (point_loc.x - bp[0])*s0e3.y - (point_loc.y - bp[1])*s0e3.x;
    
    int32_t cross_product_s1e4 = cross_product_int_param(point_loc.x, point_loc.y, bisecting_points[2], bisecting_points[3], bisecting_points[8], bisecting_points[9]);
        
    // WAS (point_loc.x - bp[4])*s1e4.y - (point_loc.y - bp[5])*s1e4.x;
    
    int32_t cross_product_s2e5 = cross_product_int_param(point_loc.x, point_loc.y, bisecting_points[4], bisecting_points[5], bisecting_points[10], bisecting_points[11]);
        
    //WAS (point_loc.x - bp[8])*s2e5.y - (point_loc.y - bp[9])*s2e5.x;
    
    //printf("CP 0-3: %ld, CP 1-4: %ld, CP 2-5: %ld\n", cross_product_s0e3, cross_product_s1e4, cross_product_s2e5);
    
    uint8_t outer_sextant = which_sextant(cross_product_s0e3, cross_product_s1e4, cross_product_s2e5);
    return outer_sextant;
}

float get_cos_theta_general(Location point_loc){
    //vector A = {1, 0}
    //Vector B = point_loc
    if(pythagorean(point_loc.x, point_loc.y) < 1){
        return 0;
    }
    return ((float)point_loc.x)/(pythagorean(point_loc.x, point_loc.y));
}

float get_cos_theta(Location point_loc, uint8_t outer_sextant){
    //assume the center is (0,0)
    
    //dot product formula: A . B = |A||B|cos(theta)
    Location vectorA;
    vectorA.x = bisecting_points[2*outer_sextant];
    vectorA.y = bisecting_points[2*outer_sextant + 1];
    
    Location vectorB;
    vectorB.x = point_loc.x;
    vectorB.y = point_loc.y;
    
    float cos_theta = ((float)((int32_t)vectorA.x * (int32_t)vectorB.x + (int32_t)vectorA.y * (int32_t)vectorB.y)/(float)((pythagorean(vectorA.x, vectorA.y)*pythagorean(vectorB.x, vectorB.y))));
    return cos_theta;
}

float get_min_distance(float cos_theta){
    return INNER_HEX_APATHEM_DIST * 1.0/(cos_theta);
}

float get_max_distance(float cos_theta){
    return OUTER_HEX_APATHEM_DIST * 1.0/(cos_theta);
}

float get_min_distance_loc_param(Location point_loc){
    float cos_theta = get_cos_theta(point_loc, get_current_sextant(point_loc));
    return get_min_distance(cos_theta);
}

float get_max_distance_loc_param(Location point_loc){
    float cos_theta = get_cos_theta(point_loc, get_current_sextant(point_loc));
    return get_max_distance(cos_theta);
}

Location get_bp_for_sextant(uint8_t sextant){
    Location l;
    l.x = bisecting_points[2*sextant];
    l.y = bisecting_points[2*sextant + 1];
    return l;
}


int in_field(Location point_loc){
    
    float cos_theta = get_cos_theta(point_loc, get_current_sextant(point_loc));
    
    float distance = pythagorean(point_loc.x, point_loc.y);
    
    
    if(distance > get_min_distance(cos_theta) && distance < get_max_distance(cos_theta))
        return 1;
    else
        return 0;
}

float calculate_distance_increment(Location start_loc, Location end_loc, uint8_t increments){
    float start_dist = pythagorean(start_loc.x, start_loc.y);
    float end_dist = pythagorean(end_loc.x, end_loc.y);
    float increment_ratio_start = OUTER_HEX_APATHEM_DIST/get_max_distance_loc_param(start_loc);
    float increment_ratio_target = OUTER_HEX_APATHEM_DIST/get_max_distance_loc_param(end_loc);
    return (end_dist*increment_ratio_target - start_dist*increment_ratio_start)/increments;
}

int get_waypoint_direction_five_param(Location curr_loc, Location target_loc, uint8_t curr_sextant, uint8_t target_sextant, float curr_angle){
    if(curr_sextant == target_sextant)
        return 0;
    if(field_state.stage == FIRST_STAGE)
        return COUNTER_CLOCKWISE;
    float curr_angle_field = degrees(atan2(curr_loc.y, curr_loc.x));
    float target_angle_field = degrees(atan2(target_loc.y, target_loc.x));
    float angle_difference = mod_f(target_angle_field - curr_angle_field, 180);
    float angle_difference_pos = angle_difference;
    float angle_difference_neg = angle_difference;
    if(angle_difference < 0)
        angle_difference_pos = angle_difference + 360;
    if(angle_difference > 0)
        angle_difference_neg = angle_difference - 360;
    //slight preference for counter clockwise because convention
    if(mod_f(curr_angle - curr_angle_field, 180) >= 0){
        if(angle_difference_pos < CRITICAL_ANGLE)
            return COUNTER_CLOCKWISE;
        else
            return CLOCKWISE;
    }
    else{
        if(angle_difference_neg > -CRITICAL_ANGLE)
            return CLOCKWISE;
        else
            return COUNTER_CLOCKWISE;
    }
}

int get_waypoint_direction(Location curr_loc, Location target_loc, float curr_angle){
    uint8_t curr_sextant = get_current_sextant(curr_loc);
    uint8_t target_sextant = get_current_sextant(target_loc);
    return get_waypoint_direction_five_param(curr_loc, target_loc, curr_sextant, target_sextant, curr_angle);
}

float bound_waypoint_distance_float_param(float distance_increment, float current_distance){
    float new_distance = current_distance + distance_increment;
    if(new_distance < (INNER_HEX_VERTEX_DIST + ROBOT_WIDTH/2 + 120))
        new_distance = INNER_HEX_VERTEX_DIST + ROBOT_WIDTH/2 + 120;
    if(new_distance > (OUTER_HEX_APATHEM_DIST - ROBOT_WIDTH/2))
        new_distance = OUTER_HEX_APATHEM_DIST - ROBOT_WIDTH/2;
    return new_distance;
}

float bound_waypoint_distance(float distance_increment, Location target_loc_waypoint){
    float new_distance = field_state.distance_increment + pythagorean(field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y);
    if(new_distance < (INNER_HEX_VERTEX_DIST + ROBOT_WIDTH/2 + 120))
        new_distance = INNER_HEX_VERTEX_DIST + ROBOT_WIDTH/2 + 120;
    if(new_distance > (OUTER_HEX_APATHEM_DIST - ROBOT_WIDTH/2))
        new_distance = OUTER_HEX_APATHEM_DIST - ROBOT_WIDTH/2;
    return new_distance;
}

float get_d(uint16_t encoder_value){
    return (((float)encoder_read(LEFT_ENCODER_PORT)) / CLICKS_PER_INCH)*INCHES_TO_PIXELS;
}


Location get_delta(Location curr_loc){
    float dl = get_d(encoder_read(LEFT_ENCODER_PORT));
    float dr = get_d(encoder_read(RIGHT_ENCODER_PORT));
    float curr_angle = degrees(atan2(curr_loc.y, curr_loc.x));
    Location delta;
    float cos_theta = get_cos_theta_general(curr_loc);
    delta.y = (int)(lround((dl*cos_theta + dr*cos_theta)/2.0));
    float sin_theta;
    if(curr_angle > 180 || (curr_angle < 0 && curr_angle > -180))
        sin_theta = 1 * sqrt(1 - (cos_theta * cos_theta));
    else{
        sin_theta = -1 * sqrt(1 - (cos_theta * cos_theta));
    }
    delta.x = (int)(lround((dl*sin_theta) + dr*sin_theta)/2.0);
    return delta;
}

void set_new_destination(Location curr_loc, Location target_loc){
    //printf("STX: %d, STY: %d ", target_loc.x, target_loc.y);
    field_state.target_loc = target_loc;
    uint8_t sextant = get_current_sextant(curr_loc);
    uint8_t target_sextant = get_current_sextant(target_loc);
    //printf("Current sextant: %d, Target Sextant: %d", sextant, target_sextant);
    if(sextant == target_sextant){
        field_state.target_loc_waypoint = field_state.target_loc;
        return;
    }
    int direction = get_waypoint_direction(curr_loc, target_loc, gyro_get_degrees());
    //if(field_state.stage == FIRST_STAGE)
        //direction = COUNTER_CLOCKWISE;
    //printf("Dir: %d ", direction);
    uint8_t sextant_difference = get_sextant_difference(sextant, target_sextant, direction);
    //printf("SD: %d ", sextant_difference);
    field_state.distance_increment = calculate_distance_increment(curr_loc, target_loc, sextant_difference);
    float distance_ratio = OUTER_HEX_APATHEM_DIST/get_max_distance_loc_param(curr_loc);
    float new_distance = bound_waypoint_distance_float_param(field_state.distance_increment, distance_ratio*pythagorean(curr_loc.x, curr_loc.y));
    switch(direction){
        case COUNTER_CLOCKWISE:
            field_state.target_loc_waypoint = get_scaled_loc_int_param(bisecting_points[(2*sextant+2)%12], bisecting_points[(2*sextant+3)%12], field_state.distance_increment + distance_ratio*pythagorean(curr_loc.x, curr_loc.y));
            break;
        case CLOCKWISE:
            field_state.target_loc_waypoint = get_scaled_loc_int_param(bisecting_points[(2*sextant)%12], bisecting_points[(2*sextant+1)%12], field_state.distance_increment + distance_ratio*pythagorean(curr_loc.x, curr_loc.y));
    }
    field_state.target_loc_plane = get_scaled_loc(field_state.target_loc_waypoint, OUTER_HEX_APATHEM_DIST);
    field_state.stored_time = field_state.curr_time;
    //printf("CX: %d, CY: %d, TX: %d, TY: %d, TWX: %d, TWY: %d, CA: %f\n", field_state.curr_loc.x, field_state.curr_loc.y, field_state.target_loc.x, field_state.target_loc.y, field_state.target_loc_waypoint.x, field_state.target_loc_waypoint.y, field_state.curr_angle);

}


Location get_dump_location_robot(uint8_t sextant){
    Location dump_loc;
    dump_loc.x = (int)(cos(radians(60.0*sextant)) * (INNER_HEX_APATHEM_DIST + DUMP_ADDED_DISTANCE));
    dump_loc.y = (int)(sin(radians(60.0*sextant)) * (INNER_HEX_APATHEM_DIST + DUMP_ADDED_DISTANCE));
    return dump_loc;
}

Location get_dump_location(uint8_t sextant){
    Location dump_loc;
    dump_loc.x = (int)(cos(radians(60.0*sextant)) * (INNER_HEX_APATHEM_DIST));
    dump_loc.y = (int)(sin(radians(60.0*sextant)) * (INNER_HEX_APATHEM_DIST));
    return dump_loc;
}

uint8_t pick_dump_sextant(){
    uint8_t returned_sextant = 0;
    uint8_t sextant = get_current_sextant(field_state.curr_loc_plus_delta);
    switch(field_state.color){
        case BLUE:
            if(sextant == 3)
                returned_sextant = 3;
            else if(sextant == 4)
                returned_sextant = 4;
            else if(sextant == 5)
                returned_sextant = 5;
            else{
                int direction_to_3 = get_waypoint_direction(field_state.curr_loc_plus_delta, get_dump_location(3), gyro_get_degrees());
                uint8_t distance_to_3 = get_sextant_difference(sextant, 3, direction_to_3);
                int direction_to_5 = get_waypoint_direction(field_state.curr_loc_plus_delta, get_dump_location(5), gyro_get_degrees());
                uint8_t distance_to_5 = get_sextant_difference(sextant, 5, direction_to_5);
                if(distance_to_3 < distance_to_5)
                    returned_sextant = 3;
                else if(distance_to_5 < distance_to_3)
                    returned_sextant = 5;
                else if(distance_to_5 == distance_to_3){ //technically should never happen
                    returned_sextant = 4; //for paranoia's sake.
                }
                else{
                    returned_sextant = 4;
                }
            }
            break;
            
        case RED:
            if(sextant == 0)
                returned_sextant = 0;
            else if(sextant == 1)
                returned_sextant = 1;
            else if(sextant == 2)
                returned_sextant = 2;
            else{
                int direction_to_0 = get_waypoint_direction(field_state.curr_loc_plus_delta, get_dump_location(0), gyro_get_degrees());
                uint8_t distance_to_0 = get_sextant_difference(sextant, 0, direction_to_0);
                int direction_to_2 = get_waypoint_direction(field_state.curr_loc_plus_delta, get_dump_location(2), gyro_get_degrees());
                uint8_t distance_to_2 = get_sextant_difference(sextant, 2, direction_to_2);
                if(distance_to_0 < distance_to_2)
                    returned_sextant = 0;
                else if(distance_to_2 < distance_to_0)
                    returned_sextant = 2;
                else if(distance_to_0 == distance_to_2){ //technically should never happen
                    returned_sextant = 1; //for paranoia's sake.
                }
                else{
                    returned_sextant = 1;
                }
            }
            break;
    }
    return returned_sextant;
}


void get_your_ass_to_a_toilet(){
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
    uint8_t dump_sextant = pick_dump_sextant();
    Location dump_loc = get_dump_location_robot(dump_sextant);
    set_new_destination(field_state.curr_loc_plus_delta, dump_loc);
    //time_until_dump += 1500*travel_sextants;
    time_during_dump += 100*field_state.balls_held;
}



