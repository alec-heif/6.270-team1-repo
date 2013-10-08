//Created by Erwin Hilton
//servo_arm.c

//from limit of servo at top to contact the bar - 115 degrees
//to fully pull the bar - total of 160 degrees from top limit (tolerance)
//to reset - total of 70 degrees from top

//access our score to see if we scored, stop before it damages servo

//distance between robot's wheels = 8.5 inches
//when turning with pivot around center of wheels, makes a circle of radius 4.25 inches. Thus, 90 degree turn is arc length = (8.5 * pi)/4 

// Include headers from OS
#include <stdio.h>
#include <joyos.h>
#include <math.h>
#include <lib/pid.h>
#include <umain.c>

//motor slots on happyboard
#define SERVO_ARM_PORT 0

// usetup is called during the calibration period. 
int usetup (void) {
    return 0;
}

int umain (void) {
    pause(1000);
    set_motors(128, 128);
    pause(500);
    stop_motors();
    pause(500);
    servo_set_pos(1, 200);
    pause(1000);
    int value = 0;
    while(1){
        printf("Servo Val: \n");
        scanf("%d", &value);
        if(value >= 100 && value <= 500){
            printf("Value %d not within range of 100-500. Setting servo to it.\n", value);
            servo_set_pos(1, value);

        }
        else{
            printf("Value %d not within range of 100-500. Setting servo to 200.\n", value);
            servo_set_pos(1, 200);
        }
        pause(1000);
    }
    
    
	//servoArmMove();
	//turn 90 degrees at that motor velocity (spins in place)
    // YOUR CODE GOES HERE

    // Will never return, but the compiler complains without a return
    // statement.
    return 0;
}

