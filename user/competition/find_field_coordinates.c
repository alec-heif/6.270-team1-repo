//
//  find_field_coordinates.c
//  
//
//  Created by aheifetz on 1/16/13.
//
//


/*************************************INSTRUCTIONS********************************************
This has two modes of use: if you have the frob knob turned towards the larger value (i.e. away from whatever zero is), it'll constantly print out the robot's current location to the console.
    Mode 1 is useful for finding a particular coordinate, like say you wanted to find (0, 0) or find out how far you can move the robot until you slide off the vps.
 
 Mode 2 is probably more useful for what you guys should do though. In mode 2 (where you have the frob knob turned at or near zero), the robot won't print anything unless you press go, at which point it'll print its current coordinates. So, just place the robot somewhere on the field and press go and it'll print out the current coordinates so you can write them down.
 ************************************END INSTRUCTIONS****************************************/

#include <stdio.h>

void usetup(){
    return;
}

void umain(){
    int go_press_previous = 0;
    
    while(1){
        if(frob_read() < 500){
            if(go_press()){
                if(go_press_previous == 0){
                    printf("X Coordinate: %d, Y Coordinate: %d, Robot Angle: %f \n", game.coords[0].x, game.coords[0].y, (((float)game.coords[0].theta) * 180.0) / 2048);
                    go_press_previous = 1;
                }
            }
            else{
                go_press_previous = 0;
            }

        }
        else{
            printf("X Coordinate: %d, Y Coordinate: %d, Robot Angle: %f \n", game.coords[0].x, game.coords[0].y, (((float)game.coords[0].theta) * 180.0) / 2048);
        }
        pause(50);
    }
}