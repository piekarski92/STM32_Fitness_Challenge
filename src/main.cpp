#include <mbed.h>
#include <USBSerial.h>
#include "accel.h"
#include "statemachine.h"

InterruptIn btn(USER_BUTTON);
Timeout isrEnd;

volatile int STATE = 0;
volatile bool isrInProgress = false;

void waitOver(){
    isrInProgress = false;
}

void stateIncr(){
    if(isrInProgress){
        return;
    }
    isrInProgress = true;
    STATE++;
    isrEnd.attach(&waitOver,1);
}

int main() {
    mems_startup();
    initSincFunction(0.47,33,1,-0.8);
    btn.rise(&stateIncr);

    wait_ms(500);

    while(1){
        switch(STATE){

            case 1:
                welcomeState();
                break;
            case 2:
                stateMachine_01();
                break;
            case 3:
                transistionState();
                initSincFunction(1.5,90,1,0.58);
                wait_ms(100);
                STATE++;
                break;
            case 4:
                restState();
                break;
            case 5:
                stateMachine_02();
                break;
            case 6:
                transistionState();
                initSincFunction(0.4,24,1,1.1);
                wait_ms(100);
                STATE++;
                break;
            case 7:
                restState();
                break;
            case 8:
                stateMachine_03();
                break;
            case 9:
                transistionState();
                initSincFunction(0.5,35,1.1,4);
                accel_set4G();
                wait_ms(100);
                STATE++;
                break;
            case 10:
                restState();
                break;
            case 11:
                stateMachine_04();
                break;
            case 12:
                goodbyeState();
                break;
            default:
                accel_set2G();
                transistionState();
                initSincFunction(0.47,33,1,-0.8);
                accel_set2G();
                wait_ms(100);
                STATE=1;

        }
    }
    
  }