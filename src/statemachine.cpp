#include <mbed.h>
#include <USBSerial.h>
#include "statemachine.h"
#include "accel.h"
#include "filter.h"

CanonFilter lpFilter;
Timeout isrExerciseRep;

//USBSerial ser;
DigitalOut led_OR(PD_13);
DigitalOut led_GR(PD_12);
DigitalOut led_RD(PD_14);
DigitalOut led_BL(PD_15);

int16_t Xg, Yg, Zg;

volatile bool isrExerciseInProgress = false;

int waitTime = 70;
int BLOCKLEN = 100;
int BUFFER = 5;

double accel_last = 0;
double accelBuffer[100];
double rollingBuffer[10];
double sinc[100];
double corr = 0;
double corr_last = 0;
double deriv = 0;
double xAngle = 0;
double yAngle = 0;
double situpMag = 0;

const double pushupThreshold = 0.23;
const double situpThreshold = -0.4;
const double squatThreshold = 0.7;
const double jackThreshold = 3.2;

double pushupAngleThres = -100;
double situpAngleThres = 50;
double squatAngleThres = 0;

int pushupNumber = 0;
int situpNumber = 0;
int squatNumber = 0;
int jackNumber = 0;

void repWaitTimer(){
    isrExerciseInProgress = false;
}

double nanCheck(double x){
    if(isnan(x)){
        return accel_last;
    }
    else{
        return x;
    }
}

void checkWorkoutGreater(double currentValue, double thres, int *exercise, float waitTime){
    if((currentValue > thres) && !isrExerciseInProgress){
        isrExerciseInProgress = true;
        *exercise = *exercise + 1;
        isrExerciseRep.attach(&repWaitTimer,waitTime);
    }
      
}

void checkWorkoutLess(double currentValue, double thres, int *exercise, float waitTime){
    if((currentValue < thres) && !isrExerciseInProgress){
        isrExerciseInProgress = true;
        *exercise = *exercise + 1;
        isrExerciseRep.attach(&repWaitTimer,waitTime);
    }
      
}

void ledCircularFlashing(){
    led_OR.write(1);
    wait_ms(waitTime);
    led_OR.write(0);
    led_RD.write(1);
    wait_ms(waitTime);
    led_RD.write(0);
    led_BL.write(1);
    wait_ms(waitTime);
    led_BL.write(0);
    led_GR.write(1);
    wait_ms(waitTime);
    led_GR.write(0);
}

void ledCircularFlashingReverse(){
    led_GR.write(1);
    wait_ms(waitTime);
    led_GR.write(0);
    led_BL.write(1);
    wait_ms(waitTime);
    led_BL.write(0);
    led_RD.write(1);
    wait_ms(waitTime);
    led_RD.write(0);
    led_OR.write(1);
    wait_ms(waitTime);
    led_OR.write(0);
}


void ledAllOff(){
    led_OR.write(0);
    led_RD.write(0);
    led_BL.write(0);
    led_GR.write(0);
}

void ledAllOn(){
    led_OR.write(1);
    led_RD.write(1);
    led_BL.write(1);
    led_GR.write(1);   
}

void ledPulseFlash(){
    ledAllOn();
    wait_ms(waitTime);
    ledAllOff();
    wait_ms(waitTime*2);
}

void ledError(){
    led_RD.write(1);
    wait_ms(waitTime);
    led_RD.write(0);
    wait_ms(waitTime);
}

void ledRepCount(int count){
    if(count > 4){
        count = 5;
    }

    switch(count){
        case 0:
            ledAllOff();
            break;
        case 1:
            led_OR.write(1);
            break;
        case 2:
            led_OR.write(1);
            led_RD.write(1);
            break;
        case 3:
            led_OR.write(1);
            led_RD.write(1);
            led_BL.write(1);
            break;
        case 4:
            ledAllOn();
            break;
        case 5:
            ledPulseFlash();
            break;
        default:
            ledError();
            break;
    }
}

void roll(double x[], double buff[], int r){
    
    for(int i = 0 ; i < (BLOCKLEN-r) ; i++){
        x[i] = x[i+r];
    }

    for(int k = 0; k < BUFFER ; k++){
        x[BLOCKLEN-r+k] = buff[k];
    }
}

void correlation(){
    /*
    We will use a normalized correlation equation for our accelerometer data and comparison waveform.
    Correlation is given by:
            corr = SUM{ x[n]*y[n]  }    /   sqrt{ SUM{x^2[n]} * SUM{y^2[n]}  }
    */
    double numerator = 0;
    double denom = 0;
    double denom_x = 0;
    double denom_y = 0;

    for(int i = 0;i < BLOCKLEN;i++){
        numerator += accelBuffer[i]*sinc[i];
        denom_x += pow(accelBuffer[i],2);
        denom_y += pow(sinc[i],2);
    }

    denom = sqrt(denom_x * denom_y);
    denom == 0 ? 1 : denom;

    corr = numerator / denom;
}

double derivative(double x0, double kd){
    double dx = corr_last - x0;
    corr_last = x0;
    return kd*dx;
}

void initSincFunction(double b, double a, double c, double amp){
    /*
    We initiate a sinc function that can be correlated with our accelerometer data
    to extract features from our exercises.
    Values are selected in a way to closely match what a feature looks like.
    */
   double u = 0;

    for(int i = 0; i < BLOCKLEN; i++){
        u = i*b - a;
        if(u == 0){
            // Limit sin(x)/x as x->0 is 1
            sinc[i] = 1;
        }
        else
        {
            sinc[i] = c + amp*(sin(u)/u);
        }
        
        
    }

}

void fillBufferAndFilter(){
    //1. Fill Buffer of accelerometer data
    //Normal Force = sqrt(X^2 + Y^2 + Z^2) found in function getMag
    double val = 0;
    double filt = 0;
    for(int i = 0;i < BUFFER;i++){
        readData(&Xg, &Yg, &Zg);
        val = nanCheck(getMag(Xg,Yg,Zg));
        filt = lpFilter.filterData(val);
        rollingBuffer[i] = filt;
        //ser.printf("%d,%d,%d,%.4f,%.4f\n\r",Xg,Yg,Zg,xAngle,situpMag);
    }
}

double situpCorr(double a, double b){
    double x = a*a;
    double y = b*b;
    return a*b / (x+y);
}

void getAngles(){
    xAngle = getXangle(Yg,Zg);
    yAngle = getYangle(Xg,Yg,Zg);
}

void emptyBuffer(){
    for(int i = 0; i<BLOCKLEN;i++){
        accelBuffer[i] = 0;
    }
}

void welcomeState(){
    /* WELCOME STATE */
    ledCircularFlashing();
}

void stateMachine_01(){
    /* Pushup State */
    fillBufferAndFilter();
    correlation();
    deriv = derivative(corr,20);
    getAngles();
    if(xAngle < pushupAngleThres){
        checkWorkoutGreater(deriv,pushupThreshold,&pushupNumber,1.2);
    }
    ledRepCount(pushupNumber);
    roll(accelBuffer,rollingBuffer,BUFFER);
}

void stateMachine_02(){
    /* Situp State */
    fillBufferAndFilter();
    getAngles();
    situpMag = situpCorr(xAngle,getAxisMag(Xg,Yg,10)); 

    if(xAngle < situpAngleThres){
        checkWorkoutLess(situpMag,situpThreshold,&situpNumber,1.2);
    }
    ledRepCount(situpNumber);
    roll(accelBuffer,rollingBuffer,BUFFER);
}

void stateMachine_03(){
    /* Squat State */
    fillBufferAndFilter();
    correlation();
    deriv = derivative(corr,30);
    getAngles();
    if(xAngle > squatAngleThres){
        checkWorkoutGreater(deriv,squatThreshold,&squatNumber,1.2);
    }
    ledRepCount(squatNumber);
    roll(accelBuffer,rollingBuffer,BUFFER);
}

void stateMachine_04(){
    /* Jumping Jacks State */
    fillBufferAndFilter();
    correlation();
    deriv = derivative(corr,20);
    getAngles();
    checkWorkoutGreater(deriv,jackThreshold,&jackNumber,0.5);
    ledRepCount(jackNumber);
    roll(accelBuffer,rollingBuffer,BUFFER);
}

void transistionState(){
    /* A pass through state that resets the system. */
    lpFilter.resetStates();
    ledAllOff();
    emptyBuffer();
    corr = 0;
    corr_last = 0;
    deriv = 0;
}

void goodbyeState(){
    ledCircularFlashingReverse();
}

void restState(){
    led_BL.write(1);
    wait_ms(waitTime);
    led_BL.write(0);
    wait_ms(waitTime);
}

