//
//  drivetest1.c
//  
//
//  Created by aheifetz on 1/11/13.
//
//

#include <stdio.h>
#include <joyos.h>

int usetup(){
    return 0;
}

int start_test(char testName[]) {
    printf("\n%s: press Go (or Stop to skip)", testName);
    pause(100);
    return either_click();
}

void test_motors() {
    uint8_t mot;
    uint16_t pos;
    while(1){
        printf("Please choose a motor to test: (0 or 1 please)\n");
        scanf("%d", &mot);
        if (mot >= 0 && mot <= 6){
            printf("Push Go to start testing Motor %d. Push stop at any time to restart the test.\nNote that 0 on the knob corresponds to moving backwards at -255, 255 on knob corresponds to stopped, and 511 on knob corresponds to moving forwards at 255.\n\n", mot);
            break;
        }
        else
            printf("Not a valid motor. Please try again.\n\n");
    }
    go_click();
    printf("Now Testing Motor %d", mot);
    while (!stop_press()) {
        pos = frob_read()/2;
        printf("Motor %d is set to %3d with a current of %dmA\n",mot,pos,motor_get_current_MA(mot));
        motor_set_vel(mot,pos-256);
        pause(50);
    }
    motor_set_vel(mot,0);
}

int umain(void){
    while(1){
        printf("Press Go to start the Test\n\n")
        pause(100);
        if (start_test("Motor Test"))
            test_motors();
    }
}
uint8_t mot = 0;
