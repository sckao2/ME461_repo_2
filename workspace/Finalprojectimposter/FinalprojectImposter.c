//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);//AMBSCK predefine spib interrupt service routing
__interrupt void ADCA_ISR (void);
void setupSpib(void); //SCK predefine setupSPIB
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float uLeft);
void setEPWM2B(float uRight);

// Functions responsible for righting if robot
void setEPWM8A_RCServo(float); // Left lever
void setEPWM8B_RCServo(float); // Right Lever
float rightAngleState1 = 97;
float leftAngleState1 = -99;
float leftAngleState2 = 15;
float rightAngleState2 = -20;

uint32_t hunch = 0;

//lab 6 variable definitions AMBSCK
float leftAngle = 0.0;
float rightAngle = 0.0;//store wheel angles AMBSCK;
float leftDist = 0.0;
float rightDist = 0.0;
float leftDist_1 = 0.0;
float rightDist_1 = 0.0;//store wheel distances and previous distances AMBSCK
float leftAngle_1 = 0.0;
float rightAngle_1 = 0.0;
float leftOmega = 0.0;
float rightOmega = 0.0;
float leftVel = 0.0;
float rightVel = 0.0;
float uLeft = 5.0;
float uRight = 5.0;
float poseX = 0.0;
float poseY = 0.0;
float velX = 0.0;
float velY = 0.0;
float velX_1 = 0.0;
float velY_1 = 0.0;
float Rwh = 0.19460;
float Wr = 0.56759;
float omegaAvg = 0.0;
float phi = 0.0;

float I_L = 0.0;
float e_L = 0.0;
float I_R = 0.0;
float e_R = 0.0;
float I_1_L = 0.0;
float e_1_L = 0.0;
float I_1_R = 0.0;
float e_1_R = 0.0;//AMBSCK memory variables for integrator
float Kp = 3.0;//BALANCING SCK
float Ki = 20.0;//BALANCING SCK
float Kd = 0.08;//BALANCING SCK

float Kp_d = 3.0; //driving sck
float Ki_d = 5.0; //driving sck

float Vref = 0;
float turn = 0.0;
float turn_0 = 0.0;
float Kpturn = 3.0;
float eturn = 0.0;
float Kpwall = 6.0;
float ewall = 0.0;
uint16_t firstRun = 1;
uint16_t stopped = 0;
uint32_t startCycle = 0;

//variable definition for wireless communication
float printLV3 = 0;
float printLV4 = 0;
float printLV5 = 0;
float printLV6 = 0;
float printLV7 = 0;
float printLV8 = 0;
float x = 0;
float y = 0;
float bearing = 0;
extern uint16_t NewLVData;
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t spivalue1 = 0.0;
int16_t spivalue2 = 0.0;
int16_t spivalue3 = 0.0;
int16_t pwmcmd1 = 0;
int16_t pwmcmd2 = 0;
int16_t upcount = 1;
int16_t coolDown= 1; // responsible for delay after switching from state 2 to state 1
int32_t coolDownTime; // responsible for timing the delay based on numTimer1Calls
float v1 = 0.0;
float v2 = 0.0;

//for ex 3 SCK
int16_t dummy= 0; //for reading unneeded 16 bit
int16_t accxr = 0; //accel x
int16_t accyr = 0; //accel y
int16_t acczr = 0; //accel z
int16_t gxr = 0;
int16_t gyr = 0;
int16_t gzr = 0;
float accelx = 0;
float accely = 0;
float accelz = 0;
float gyrox = 0; //gyro x
float gyroy = 0; //gyro y
float gyroz = 0; //gyro z

//AMBSCK Balance Segbot variables
float tilt_value_1 = 0.0;
float gyro_value_1 = 0.0;
float LeftWheel_1 = 0.0;
float RightWheel_1 = 0.0;
float vel_left = 0.0;
float vel_left_1 = 0.0;
float vel_right = 0.0;
float vel_right_1 = 0.0;
float gyro_rate = 0.0;
float gyro_rate_1 = 0.0;
float turnref = 0.0;
//AMBSCK for turn rate control
float turnRate = 0.0;
float turnRate_1 = 0.0;
float turnref_1 = 0.0;
//AMBSCK for speed control
float refSpeed = 0.0;
float intSpeed = 0.0;
float eSpeed = 0.0;
float eSpeed_1 = 0.0;
float intSpeed_1 = 0.0;
float Kp_s = 0.35;
float Ki_s = 1.5;
float uspd = 0.0;

float WhlDiff = 0.0;
float WhlDiff_1 = 0.0;
float vel_WhlDiff = 0.0;
float vel_WhlDiff_1 = 0.0;
float errorDiff = 0.0;
float errorDiff_1 = 0.0;
float errorInt = 0.0;
float errorInt_1 = 0.0;


//AMBSCK from lab 4
uint32_t numADCacalls = 0;
int16_t adca2result = 0;
int16_t adca3result = 0;
float adcina2v = 0.0;
float adcina3v = 0.0;
//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago,
//etc
float xk_1 = 0;
float xk_2 = 0;
float xk_3 = 0;
float xk_4 = 0;
//yk is the filtered value
float yk = 0;
//b is the filter coefficients
//float b[5] = {0.2,0.2,0.2,0.2,0.2}; // 0.2 is 1/5th therefore a 5 point average
#define ARRAYSIZE 79
float xk[ARRAYSIZE]={0};
float joyxk[ARRAYSIZE]={0};
float joyyk[ARRAYSIZE]={0};
float bxk[ARRAYSIZE]={0};
//float b[ARRAYSIZE]={    3.3833240118424500e-02, 2.4012702387971543e-01, 4.5207947200372001e-01, 2.4012702387971543e-01, 3.3833240118424500e-02};
//float b[22]={   -2.3890045153263611e-03,    -3.3150057635348224e-03,    -4.6136191242627002e-03,    -4.1659855521681268e-03,    1.4477422497795286e-03, 1.5489414225159667e-02, 3.9247886844071371e-02, 7.0723964095458614e-02, 1.0453473887246176e-01, 1.3325672639406205e-01, 1.4978314227429904e-01, 1.4978314227429904e-01, 1.3325672639406205e-01, 1.0453473887246176e-01, 7.0723964095458614e-02, 3.9247886844071371e-02, 1.5489414225159667e-02, 1.4477422497795286e-03, -4.1659855521681268e-03,    -4.6136191242627002e-03,    -3.3150057635348224e-03,    -2.3890045153263611e-03};
//float b[32]={   -6.3046914864397922e-04,    -1.8185681242784432e-03,    -2.5619416124584822e-03,    -1.5874939943956356e-03,    2.3695126689747326e-03, 8.3324969783531780e-03, 1.1803612855040625e-02, 6.7592967793297151e-03, -9.1745119977290398e-03,    -2.9730906886035850e-02,    -3.9816452266421651e-02,    -2.2301647638687881e-02,    3.1027965907247105e-02, 1.1114350049251465e-01, 1.9245540210070616e-01, 2.4373020388648489e-01, 2.4373020388648489e-01, 1.9245540210070616e-01, 1.1114350049251465e-01, 3.1027965907247105e-02, -2.2301647638687881e-02,    -3.9816452266421651e-02,    -2.9730906886035850e-02,    -9.1745119977290398e-03,    6.7592967793297151e-03, 1.1803612855040625e-02, 8.3324969783531780e-03, 2.3695126689747326e-03, -1.5874939943956356e-03,    -2.5619416124584822e-03,    -1.8185681242784432e-03,    -6.3046914864397922e-04};
//float b[101]={  -4.1384865093955942e-18,    2.4158288163475484e-05, -1.3320677588450310e-04,    -2.1438469575543533e-04,    1.1898399936030848e-04, 5.3082205069710680e-04, 2.1893290271722703e-04, -7.4768481245699380e-04,    -9.5792023943993328e-04,    4.6168341621969679e-04, 1.8598657706234230e-03, 7.0670080707196015e-04, -2.2492456754091747e-03,    -2.7055293027965603e-03,    1.2307634272343665e-03, 4.6993269406780981e-03, 1.6984303126684232e-03, -5.1577427312871921e-03,    -5.9361687426490355e-03,    2.5904429699616822e-03, 9.5104864390879260e-03, 3.3122378905612003e-03, -9.7118714382866452e-03,    -1.0812123641265282e-02,    4.5715859206989177e-03, 1.6287321385412081e-02, 5.5122975619431172e-03, -1.5726675339333283e-02,    -1.7056081487002734e-02,    7.0329483752597077e-03, 2.4459678182842035e-02, 8.0882772704277336e-03, -2.2565290379886044e-02,    -2.3949227569375457e-02,    9.6706866781569138e-03, 3.2957303117234021e-02, 1.0685317933349465e-02, -2.9243552530078470e-02,    -3.0461077862757931e-02,    1.2077118105660343e-02, 4.0427773465971463e-02, 1.2879264031643200e-02, -3.4645501075422983e-02,    -3.5481182001261206e-02,    1.3834430631479126e-02, 4.5553192553114567e-02, 1.4277570256188015e-02, -3.7792491047513456e-02,    -3.8090059866479127e-02,    1.4617663668474229e-02, 4.7377897417654163e-02, 1.4617663668474229e-02, -3.8090059866479127e-02,    -3.7792491047513456e-02,    1.4277570256188015e-02, 4.5553192553114567e-02, 1.3834430631479126e-02, -3.5481182001261206e-02,    -3.4645501075422983e-02,    1.2879264031643200e-02, 4.0427773465971463e-02, 1.2077118105660343e-02, -3.0461077862757931e-02,    -2.9243552530078470e-02,    1.0685317933349465e-02, 3.2957303117234021e-02, 9.6706866781569138e-03, -2.3949227569375457e-02,    -2.2565290379886044e-02,    8.0882772704277336e-03, 2.4459678182842035e-02, 7.0329483752597077e-03, -1.7056081487002734e-02,    -1.5726675339333283e-02,    5.5122975619431172e-03, 1.6287321385412081e-02, 4.5715859206989177e-03, -1.0812123641265282e-02,    -9.7118714382866452e-03,    3.3122378905612003e-03, 9.5104864390879260e-03, 2.5904429699616822e-03, -5.9361687426490355e-03,    -5.1577427312871921e-03,    1.6984303126684232e-03, 4.6993269406780981e-03, 1.2307634272343665e-03, -2.7055293027965603e-03,    -2.2492456754091747e-03,    7.0670080707196015e-04, 1.8598657706234230e-03, 4.6168341621969679e-04, -9.5792023943993328e-04,    -7.4768481245699380e-04,    2.1893290271722703e-04, 5.3082205069710680e-04, 1.1898399936030848e-04, -2.1438469575543533e-04,    -1.3320677588450310e-04,    2.4158288163475484e-05, -4.1384865093955942e-18};
uint16_t spibcount = 0;

//AMBSCK for kalman filter
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -0.62;//-0.6 with battery, -0.75 with cable CHANGING POINT
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheel = 0;
float RightWheel = 0;
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

float ubal = 0.0;
float K1 = -60.0;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;

// MIC
#define MICARRAYSIZE 79  // define the order of the filter (ARRAYSIZE = order + 1)
#define THRESHOLD 2.4    // Adjust this threshold value based on your requirements for sliding window filter
#define AccelzFallOffPoint 1.7  // the accelz reading to control what pose the robot falls off
__interrupt void ADCB_ISR(void); // RSM: setup interrupt to sample the mic's audio signal.
float slidingWindowFilter(float* buffer, int bufferSize, float newValue); // sliding window filter
int16_t adcb4result = 0; // EX4: define var for ADCINB4
float adcinb4_volt = 0; // EX4: define var for scaled ADCINB4 volts
//float b[32]={   -6.3046914864397922e-04,
//                -1.8185681242784432e-03,
//                -2.5619416124584822e-03,
//                -1.5874939943956356e-03,
//                2.3695126689747326e-03,
//                8.3324969783531780e-03,
//                1.1803612855040625e-02,
//                6.7592967793297151e-03,
//                -9.1745119977290398e-03,
//                -2.9730906886035850e-02,
//                -3.9816452266421651e-02,
//                -2.2301647638687881e-02,
//                3.1027965907247105e-02,
//                1.1114350049251465e-01,
//                1.9245540210070616e-01,
//                2.4373020388648489e-01,
//                2.4373020388648489e-01,
//                1.9245540210070616e-01,
//                1.1114350049251465e-01,
//                3.1027965907247105e-02,
//                -2.2301647638687881e-02,
//                -3.9816452266421651e-02,
//                -2.9730906886035850e-02,
//                -9.1745119977290398e-03,
//                6.7592967793297151e-03,
//                1.1803612855040625e-02,
//                8.3324969783531780e-03,
//                2.3695126689747326e-03,
//                -1.5874939943956356e-03,
//                -2.5619416124584822e-03,
//                -1.8185681242784432e-03,
//                -6.3046914864397922e-04};   // MIC: B coefficients for the LPF for the 31st order with 500Hz cutoff frequency. comment out b[22] around line 237
float b[79]={   2.4557099976303186e-03,
                9.6905814609742210e-04,
                -1.0362089123543512e-03,
                -2.9858809575470263e-03,
                -4.1508683472763035e-03,
                -3.8306485863546492e-03,
                -1.6820536185527194e-03,
                1.9390583094579640e-03,
                5.8476102737113561e-03,
                8.2995302517921577e-03,
                7.6736970666368915e-03,
                3.3311505642549051e-03,
                -3.7629232405354093e-03,
                -1.1058960079007989e-02,
                -1.5249200330573325e-02,
                -1.3677739041744460e-02,
                -5.7575612589032257e-03,
                6.3083256681028621e-03,
                1.7994253391747053e-02,
                2.4103708463083857e-02,
                2.1023488946299164e-02,
                8.6146327378250325e-03,
                -9.1975302038778458e-03,
                -2.5590904755921549e-02,
                -3.3469046275437363e-02,
                -2.8527130889998503e-02,
                -1.1432506554716802e-02,
                1.1946915984194494e-02,
                3.2557421142013127e-02,
                4.1731079582744322e-02,
                3.4879634062432878e-02,
                1.3714285742630979e-02,
                -1.4067012762582006e-02,
                -3.7642993674649863e-02,
                -4.7395172605045832e-02,
                -3.8924097709712786e-02,
                -1.5041955589246264e-02,
                1.5167399671650683e-02,
                3.9906831227999547e-02,
                4.9409372066260178e-02,
                3.9906831227999547e-02,
                1.5167399671650683e-02,
                -1.5041955589246264e-02,
                -3.8924097709712786e-02,
                -4.7395172605045832e-02,
                -3.7642993674649863e-02,
                -1.4067012762582006e-02,
                1.3714285742630979e-02,
                3.4879634062432878e-02,
                4.1731079582744322e-02,
                3.2557421142013127e-02,
                1.1946915984194494e-02,
                -1.1432506554716802e-02,
                -2.8527130889998503e-02,
                -3.3469046275437363e-02,
                -2.5590904755921549e-02,
                -9.1975302038778458e-03,
                8.6146327378250325e-03,
                2.1023488946299164e-02,
                2.4103708463083857e-02,
                1.7994253391747053e-02,
                6.3083256681028621e-03,
                -5.7575612589032257e-03,
                -1.3677739041744460e-02,
                -1.5249200330573325e-02,
                -1.1058960079007989e-02,
                -3.7629232405354093e-03,
                3.3311505642549051e-03,
                7.6736970666368915e-03,
                8.2995302517921577e-03,
                5.8476102737113561e-03,
                1.9390583094579640e-03,
                -1.6820536185527194e-03,
                -3.8306485863546492e-03,
                -4.1508683472763035e-03,
                -2.9858809575470263e-03,
                -1.0362089123543512e-03,
                9.6905814609742210e-04,
                2.4557099976303186e-03}; // RSM-EX4: B coefficients for the 128th order bandpass filter we designed
float xarray_ADCB4[MICARRAYSIZE] = {0}; // MIC: create an x array
float micReadingBuffer[30]; // Sliding window buffer for microphone readings
float micReadingMax = 0;
float micReadingMin = 0;
float deltaMicReading = 0;
float micReading = 0;
float MicLPFThreshold = .5;
float yk_ADCB4 = 0;
int16_t state_scared = 0;
//float accelzBalancePoint = 0.6;
void setDACA(float dacouta0) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta0/3.0*4096.0; // EX1: conversion. perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}
void setDACB(float dacouta1) {
    int16_t DACOutInt = 0;
    DACOutInt = dacouta1/3.0*4096.0; // EX1: conversion. perform scaling of 0 – almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacbRegs.DACVALS.bit.DACVALS = DACOutInt;
}

//FOR STATE MACHINE
uint16_t state = 1;//0 is driving, 1 is balancing, 2 is getting back up

void main(void)
 {
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();
    InitGpio();

    // MIC
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.SPIB_RX_INT = &SPIB_isr; //map interrupt to SPB interrupt function
    PieVectTable.ADCB1_INT = &ADCB_ISR; // MIC: assign ADCB1 to the PieVectTable
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    //-------------------------EPWM2--------------------------------
    //AMBSCK EPWM2 pinmux and initializations for 20Khz carrier frequency
    EPwm2Regs.TBCTL.bit.CTRMODE=0;//set counter mode to count up
    EPwm2Regs.TBCTL.bit.FREE_SOFT=2;//set free soft to free run
    EPwm2Regs.TBCTL.bit.PHSEN=0;//disable phase loading
    EPwm2Regs.TBCTL.bit.CLKDIV=0; //clock divide by 1 to keep clock and counter frequencies the same

    EPwm2Regs.TBCTR=0; //initialize counter value to 0

    EPwm2Regs.TBPRD=2500; //period of wave/period of clock gives clock counts per wave

    EPwm2Regs.CMPA.bit.CMPA=2500;//set to 0 for 0% duty cycle, instantly turns signal low
    EPwm2Regs.CMPB.bit.CMPB=2500;
    EPwm2Regs.AQCTLA.bit.CAU=1;//action when TBCTR=CMPA (falling edge) set signal low
    EPwm2Regs.AQCTLB.bit.CBU=1;//do the same for EPWM2B
    EPwm2Regs.AQCTLA.bit.ZRO=2;//action when TBCTR=0 (beginning of period) set signal high
    EPwm2Regs.AQCTLB.bit.ZRO=2;

    EPwm2Regs.TBPHS.bit.TBPHS =0;

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //2A
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //2B
    init_serialSCIA(&SerialA,115200);

    // Setting up EPWM8 signal for self righting servos
    EPwm8Regs.TBCTL.bit.CTRMODE = 0 ;
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2;
    EPwm8Regs.TBCTL.bit.PHSEN = 0;
    EPwm8Regs.TBCTL.bit.CLKDIV = 4;
    EPwm8Regs.TBCTR = 0;
    EPwm8Regs.TBPRD = 62500;
    EPwm8Regs.CMPA.bit.CMPA = 0;
    EPwm8Regs.CMPB.bit.CMPB = 0;
    EPwm8Regs.AQCTLA.bit.CAU = 1;
    EPwm8Regs.AQCTLA.bit.ZRO = 2;
    EPwm8Regs.AQCTLB.bit.CBU = 1;
    EPwm8Regs.AQCTLB.bit.ZRO = 2;
    EPwm8Regs.TBPHS.bit.TBPHS = 0;
    //setting up pin EPWM8A/B
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1);



    //-----------------------------------------ADCA intializations AMBSCK -----------------
    //copy from lab 4
    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x2; //SOC0 will convert Channel you choose Does not have tobe A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose willtrigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x3; //SOC1 will convert Channel you choose Does not have tobe A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will
    //trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0x4; //SOC0 will convert Channel you choose Does not have tobe B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x11; // EPWM5 ADCSOCA or another trigger you choose will/trigger SOC0
    //AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???; //SOC1 will convert Channel you choose Does not have to
    //be B1
    //AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will
    //trigger SOC1
    //AdcbRegs.ADCSOC2CTL.bit.CHSEL = ???; //SOC2 will convert Channel you choose Does not have to
    //be B2
    //AdcbRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will
    //trigger SOC2
    //AdcbRegs.ADCSOC3CTL.bit.CHSEL = ???; //SOC3 will convert Channel you choose Does not have to
    //be B3
    //AdcbRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA or another trigger you choose will
    //trigger SOC3
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set
    //INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //ADCD
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0x0; // set SOC0 to convert pin D0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC0
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 0x1; //set SOC1 to convert pin D1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD; // EPWM5 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    // MIC: Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0; //load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1; //use ADC VREF as reference voltage
    EDIS;

    //--------------------------------AMBSCK EPWM5 Initializations--------------------
    //copied from lab 4
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as  “trigger”)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1/4/2.5ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

    EPwm7Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm7Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm7Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm7Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as  “trigger”)
    EPwm7Regs.TBCTR = 0x0; // Clear counter
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm7Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm7Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm7Regs.TBPRD = 5000; // Set Period to 1/4/2.5ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm7Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm7Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

    EDIS;
    //--------------------------------------------------------------------
    //AMBSCK set up registers to use SCI
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0x1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 0x1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need toTest
    SpibRegs.SPIFFRX.bit.RXFFIL =0x10; //Interrupt Level to 16 words or more received into FIFO causesinterrupt. This is just the initial setting for the register. Will be changed below

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT6; // AMBSCK spibrx is in group 6
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA,
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT2; // MIC: enable 2nd interrupt source of interrupt 1 for microphone

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //AMBSCK enable spib in pie group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    //AMBSCK enable adca group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    // MIC: enable TINT0 in the PIE: Group 1 interrupt 2 for ADCB1
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;

    //    init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    setupSpib();
    init_eQEPs();//AMBSCK initialize eQEP peripheral

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    // IDLE loop. Just sit and loop forever (optional):
    // IDLE loop. Just sit and loop forever (optional):
    setEPWM8B_RCServo(leftAngleState1);
    setEPWM8A_RCServo(rightAngleState1);

    while(1)
    {
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA,"Left Wheel Angle: %.3f Right Wheel Angle: %.3f\r\n",leftAngle,rightAngle);
            //serial_printf(&SerialA,"Left Wheel Dist: %.3f Right Wheel Dist: %.3f\r\n",leftDist,rightDist);
            //serial_printf(&SerialA,"Accx: %.3f Accy: %.3f Accz: %.3f Gx: %.3f Gy: %.3f Gz: %.3f\r\n",accx,accy,accz,gx,gy,gz);
            //serial_printf(&SerialA,"Left Wheel Vel: %.3f Right Wheel Vel: %.3f\r\n",leftVel,rightVel);
            //serial_printf(&SerialA,"Vref: %.3f Turn: %.3f\r\n",Vref,turn);
            //serial_printf(&SerialA,"Vref: %.3f Turn: %.3f poseX: %.3f poseY: %.3f\r\n",Vref,turn,poseX,poseY);
            serial_printf(&SerialA, "JoyX (adca2): %.3f JoyY (adca3): %.3f accz: %.3f gyrox: %.3f left angle: %.3f right angle %.3f\r\n",adcina2v,adcina3v,accelz,gyrox,leftAngle,rightAngle);

            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {
    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts


    // Insert SWI ISR Code here.......
    //WHEEL SPEEDS AND DISTANCES
    //calculate rates using derivative approximation

    vel_left = 0.6*vel_left_1 + 100.0*(LeftWheel-LeftWheel_1);//wheel speed left


    vel_right = 0.6*vel_right_1 + 100.0*(RightWheel-RightWheel_1);//wheel speed right


    gyro_rate = 0.6*gyro_rate_1 + 100.0*(gyro_value-gyro_value_1);

    WhlDiff = LeftWheel-RightWheel;//dtheta between wheels
    vel_WhlDiff = 0.3333 * vel_WhlDiff_1 + 166.6667 * (WhlDiff - WhlDiff_1);//dtheta dot between wheels
    if(state==1){

        //BALANCING CONTROL
        //AMBSCK calculate control law using rates
        //speed conrol law AMBSCK
        if (coolDown == 0) {
            setEPWM8B_RCServo(leftAngleState1);
            setEPWM8A_RCServo(rightAngleState1);
        }

        if (numTimer1calls >= coolDownTime) {
            coolDown = 0;
        }

        eSpeed = (refSpeed - (vel_left+vel_right)/2.0);

        turnref = turnref + 0.5*(turnRate+turnRate_1)*0.004;
        errorDiff = turnref - WhlDiff;
        if(fabs(turn)>=3.0){
            //errorInt = errorInt; do nothing
        }
        else{
            errorInt = errorInt + 0.5*(errorDiff+errorDiff_1)*0.004;//AMBSCK in lab day 2 come back and check this and variable definitions
        }

        if(fabs(uspd)>=3){

        }
        else{
            intSpeed = intSpeed + 0.5*(eSpeed+eSpeed_1)*0.004;
        }
        ubal = -K1 * tilt_value - K2 * gyro_value - K3 * (vel_left + vel_right) / 2.0 - K4 * gyro_rate;
        uspd = Kp_s * eSpeed + Ki_s * intSpeed;
        //AMBSCK PID control for turn command'
        turn = Kp*errorDiff + Ki*errorInt -Kd*vel_WhlDiff;
        if(fabs(turn)>=4.0){
            if(turn<0){
                turn = -4.0;
            }
            else{
                turn = 4.0;
            }
        }
        if(fabs(uspd)>=4.0){
            if(uspd<0){
                uspd = -4.0;
            }
            else{
                uspd = 4.0;
            }
        }
        //AMBSCK calculate control effort
        uLeft = ubal / 2.0 + turn - uspd;
        uRight = ubal / 2.0 - turn - uspd;

        //AMBSCK write to EPWM2 to control motors
        setEPWM2A(uRight);//AMBSCK 2A commands right side`
        setEPWM2B(-uLeft);

        numSWIcalls++;

        vel_left_1 = vel_left;
        vel_right_1 = vel_right;
        gyro_rate_1 = gyro_rate;
        LeftWheel_1 = LeftWheel;
        RightWheel_1 = RightWheel;
        gyro_value_1 = gyro_value;
        WhlDiff_1 = WhlDiff;
        vel_WhlDiff_1 = vel_WhlDiff;
        errorDiff_1 = errorDiff;
        errorInt_1 = errorInt;

        turnRate_1 = turnRate;
        turnref_1 = turnref;

        eSpeed_1 = eSpeed;
        intSpeed_1 = intSpeed;
    }
    else if(state==0){
        setEPWM8B_RCServo(leftAngleState1);
        setEPWM8A_RCServo(rightAngleState1);
        deltaMicReading = 0;
        micReadingMax = 0;
        micReadingMin = 0;
        stopped = 0;
        //driving control code
        if(firstRun == 1){//SCK first time through the loop, save the current time
            Vref = 0.8;//drive this fast at the start
            startCycle = numTimer1calls;//SCK this is when we started driving, compare to current time to know when to stop.
            firstRun = 0;
        }
        if(firstRun == 0){//if it's not the first time through the cycle
            if(stopped == 0){//if the robot hasn't reached stop time
                if(startCycle+1250 - numTimer1calls < 100){//check if it is stop time
                    //SCK 1250 is how many cycles of timer 1 (period 4000us) this equates to 5 seconds of driving.
                    Vref = 0;//stop driving
                    stopped = 1;//don't do anything else
                    setEPWM2A(0);//AMBSCK 2A commands right side`
                    setEPWM2B(0);
                    firstRun = 1;
                    state = 2;
                }
            }
        }

        if(stopped == 0){//SCK if the robot is not stopped, run control laws to drive, turn, and avoid walls. Use vref for speed, turn for turn, tune avoidance with Kpwall and tune how long with the constant in the loop above.
            leftAngle = readEncLeft();
            rightAngle = readEncRight();

            leftOmega = (leftAngle-leftAngle_1)/0.004;
            rightOmega = (rightAngle-rightAngle_1)/0.004;
            leftAngle_1 = leftAngle;
            rightAngle_1 = rightAngle;
            //AMBSCK store previous values of distance
            leftDist_1 = leftDist;
            rightDist_1 = rightDist;

            leftDist = leftAngle/5.1;//AMBSCK convert angle in rad to distance in ft
            rightDist = rightAngle/5.1;
            //AMBSCK calculate velocity
            leftVel = (leftDist - leftDist_1)/0.004;
            rightVel = (rightDist - rightDist_1)/0.004;

            omegaAvg = 0.5*(leftOmega+rightOmega);
            phi = (Rwh/Wr)*(rightAngle-leftAngle);
            velX = Rwh*omegaAvg*cos(phi);
            velY = Rwh*omegaAvg*sin(phi);
            poseX = poseX + 0.5*(velX+velX_1)*0.004;
            poseY = poseY + 0.5*(velY+velY_1)*0.004;
            velX_1 = velX;
            velY_1 = velY;
            //calculate turn control law
            eturn = turn_0 + leftVel - rightVel;
            //calculate wall avoidance control law
            ewall = joyxk[0]-joyyk[0]; //if the wall is closer then the joy reading is higher. turn=L-R so left should increase turn and right should decrease it.
            //assume x is left, y is right... add this to left and subtract from right
            //calculate control law AMBSCK
            e_L = Vref-leftVel - Kpturn*eturn + Kpwall*ewall;
            I_L=I_1_L+0.004*(e_L+e_1_L);
            uLeft = Kp_d*e_L+Ki_d*I_L;
            if(abs(uLeft) >= 10.0){
                I_L=I_1_L;
            }//prevent integral windup
            //save past states AMBSCK
            e_1_L = e_L;
            I_1_L = I_L;

            //calculate control law AMBSCK
            e_R = Vref-rightVel + Kpturn*eturn - Kpwall*ewall;
            I_R=I_1_R+0.004*(e_R+e_1_R);
            uRight = Kp_d*e_R+Ki_d*I_R;
            if(abs(uRight) >= 10.0){
                I_R=I_1_R;
            }//prevent integral windup
            //save past states AMBSCK
            e_1_R = e_R;
            I_1_R = I_R;


            setEPWM2A(uRight);//AMBSCK 2A commands right side`
            setEPWM2B(-uLeft);


        }
        coolDownTime = numTimer1calls + 2000;
    }
    else if (state==2){
        coolDown = 1;
        setEPWM8B_RCServo(leftAngleState2);
        setEPWM8A_RCServo(rightAngleState2);
        //        state = 1;
        deltaMicReading = 0;
        micReadingMax = 0;
        micReadingMin = 0;
        if(numTimer1calls >= coolDownTime - 1000){
            state = 1;
        }
    }




    DINT;


    //AMBSCK send to labview
    if (NewLVData == 1) {
        NewLVData = 0;
        refSpeed = fromLVvalues[0];
        turnRate = fromLVvalues[1];
        printLV3 = fromLVvalues[2];
        printLV4 = fromLVvalues[3];
        printLV5 = fromLVvalues[4];
        printLV6 = fromLVvalues[5];
        printLV7 = fromLVvalues[6];
        printLV8 = fromLVvalues[7];
    }
    if((numSWIcalls%62) == 0) { // change to the counter variable of you selected 4ms. timer
        DataToLabView.floatData[0] = poseX;
        DataToLabView.floatData[1] = poseY;
        DataToLabView.floatData[2] = phi;
        DataToLabView.floatData[3] = 2.0*((float)numSWIcalls)*.001;
        DataToLabView.floatData[4] = 3.0*((float)numSWIcalls)*.001;
        DataToLabView.floatData[5] = (float)numSWIcalls;
        DataToLabView.floatData[6] = (float)numSWIcalls*4.0;
        DataToLabView.floatData[7] = (float)numSWIcalls*5.0;
        LVsenddata[0] = '*'; // header for LVdata
        LVsenddata[1] = '$';
        for (int i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }
}


// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    numTimer0calls++;
    /*
    if(upcount == 1){
        pwmcmd1 = pwmcmd1 + 10;
        pwmcmd2 = pwmcmd2 + 10;
        if(pwmcmd1 == 3000){
            upcount = 0;
        }
    }
    else{
        pwmcmd1 = pwmcmd1 - 10;
        pwmcmd2 = pwmcmd2 - 10;
        if(pwmcmd1 == 0){
            upcount = 1;
        }
    }
    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }
    //AMBSCK tell SPI to transmit 3 16-bit values over SPI
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 3; //AMBSCK Issue the SPIB_RX_INT when three values are in the RX FIFO
    SpibRegs.SPITXBUF = 0x00DA;
    SpibRegs.SPITXBUF = pwmcmd1; //AMBSCK send first PWM value
    SpibRegs.SPITXBUF = pwmcmd2; // AMBSCK send second PWM value
    if ((CpuTimer0.InterruptCount % 10) == 0) {
        UARTPrint = 1;
    }

     */  //comment out exercise 2 code
    //------AMBSCK comment out old spib transmission that we moved to adca interrupt function-------------
    //GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    //SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    //SpibRegs.SPITXBUF = ((0x8000) | (0x3A00));//SCK start reading from register address 3A, last bit should be 1 to trigger a read
    //SpibRegs.SPITXBUF = 0;//SCK send 0 to enable read accx
    //SpibRegs.SPITXBUF = 0;//accy
    //SpibRegs.SPITXBUF = 0;//accz
    //SpibRegs.SPITXBUF = 0;//temp
    //SpibRegs.SPITXBUF = 0;//gx
    //SpibRegs.SPITXBUF = 0;//gy
    //SpibRegs.SPITXBUF = 0;//gz
    //-----------------------------------------------------------------------------------
    if ((CpuTimer0.InterruptCount % 100) == 0) {
        //        UARTPrint = 1;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    numTimer1calls++;

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    //GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

    //if ((CpuTimer2.InterruptCount % 10) == 0) {
    //UARTPrint = 1;
    //}
}
__interrupt void SPIB_isr(void) {
    /*spivalue1 = SpibRegs.SPIRXBUF; // Read first 16-bit value off RX FIFO. Probably is zero since no chip
    spivalue2 = SpibRegs.SPIRXBUF; // Read second 16-bit value off RX FIFO. this has meaning it is 0 to 4095
    spivalue3 = SpibRegs.SPIRXBUF; // AMBSCK read third value, this has meaning from 0 to 4095

    v1=spivalue2*3.3/4096.0;
    v2=spivalue3*3.3/4096.0;

    GpioDataRegs.GPASET.bit.GPIO9 = 1; // Set GPIO9 high to end Slave Select. Now Scope. Later to deselectDAN28027
    // Later when actually communicating with the DAN28027 do something with the data. Now do nothing.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt*/
    //comment exercise 2 code


    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummy = SpibRegs.SPIRXBUF;
    accxr = SpibRegs.SPIRXBUF;
    accyr = SpibRegs.SPIRXBUF;
    acczr = SpibRegs.SPIRXBUF;
    dummy = SpibRegs.SPIRXBUF;//SCK read the temp value we don't care about
    gxr = SpibRegs.SPIRXBUF;
    gyr = SpibRegs.SPIRXBUF;
    gzr = SpibRegs.SPIRXBUF;

    accelx = accxr*4.0/32767.0;
    accely = accyr*4.0/32767.0;
    accelz = acczr*4.0/32767.0;

    gyrox = gxr*250.0/32767.0;
    gyroy = gyr*250.0/32767.0;
    gyroz = gzr*250.0/32767.0;

    // MIC: use to control the segbot self-balancing point
    if (state_scared == 1) {
        //        accelzBalancePoint = AccelzFallOffPoint;
    }
    else if (state_scared == 0) {
        //        accelzBalancePoint = 0.6;
    }
    if(calibration_state == 0){
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }

    }
    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

}
void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,66 which are also a part of the SPIB setup.
    //SCK begin paste
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0x1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 0x1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need toTest
    SpibRegs.SPIFFRX.bit.RXFFIL =0x10; //Interrupt Level to 16 words or more received into FIFO causesinterrupt. This is just the initial setting for the register. Will be changed below
    //SCK end paste
    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    //-----------------------------------------------------------------------------------------------------------------
    //Step 2.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16-bit transfers, so two registers at a time after the first 16-bit transfer.
    SpibRegs.SPITXBUF = (0x1300 | 0x0000);
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);
    SpibRegs.SPITXBUF = (0x0000 | 0x0013);
    SpibRegs.SPITXBUF = (0x0200 | 0x0000);
    SpibRegs.SPITXBUF = (0x0800 | 0x0006);
    SpibRegs.SPITXBUF = (0x0000 | 0x0000);

    // To address 00x13 write 0x00
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00

    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7);//SCK if called in main, wait in while loop until one value is in RX FIFO
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    int i;
    for(i=1;i<8;i++){
        temp = SpibRegs.SPIRXBUF;//SCK read value even though not needed
    }
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    //Step 3.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    SpibRegs.SPITXBUF = (0x2300 | 0x0000);
    SpibRegs.SPITXBUF = (0x0400 | 0x008C);
    SpibRegs.SPITXBUF = (0x0200 | 0x0088);
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A);


    // To address 00x23 write 0x00
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    for(i=1;i<5;i++){
        temp = SpibRegs.SPIRXBUF;//SCK read value even though not needed
    }
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.


    //Step 4.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);
    // Write to address 0x2A the value 0x81
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x0015); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x00EC); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00E1); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00A0); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0026); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x01DA); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}
void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (-raw*(2*PI/12000));
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(2*PI/12000));
}

//AMBSCK EPWM2 Functions to set duty cycle based on control effort
void setEPWM2A(float uLeft){
    //take value from -10 to 10 and map to value from 0 TBPRD=2500
    if(uLeft>10){
        uLeft=10;
    }
    if(uLeft<-10){
        uLeft=-10;
    }
    uLeft=uLeft+10;//map 0 to 20 to 0 to 2500
    EPwm2Regs.CMPA.bit.CMPA=(uLeft/20.0)*2500.0;
}
void setEPWM2B(float uRight){
    //take value from -10 to 10 and map to value from 0 TBPRD=2500
    if(uRight>10){
        uRight=10;
    }
    if(uRight<-10){
        uRight=-10;
    }
    uRight=uRight+10;//map 0 to 20 to 0 to 2500
    EPwm2Regs.CMPB.bit.CMPB=(uRight/20.0)*2500.0;
}
//AMBSCK ADCA interrupt function to sample joystick
//sck edited function to sample ADCA2 and 3 as IR distance sensor readings. Inverts voltage.
__interrupt void ADCA_ISR (void) {
    numADCacalls++;
    adca2result = AdcaResultRegs.ADCRESULT0;
    adca3result = AdcaResultRegs.ADCRESULT1;
    adcina2v=0;
    adcina3v=0;
    // Here covert ADCINA2 to volts
    joyxk[0] = adca2result/4096.0*5.3;
    joyyk[0] = adca3result/4096.0*5.3;
    //AMBSCK start SPI transmission and reception of acc and gyro
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8;
    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00));//SCK start reading from register address 3A, last bit should be 1 to trigger a read
    SpibRegs.SPITXBUF = 0;//SCK send 0 to enable read accx
    SpibRegs.SPITXBUF = 0;//accy
    SpibRegs.SPITXBUF = 0;//accz
    SpibRegs.SPITXBUF = 0;//temp
    SpibRegs.SPITXBUF = 0;//gx
    SpibRegs.SPITXBUF = 0;//gy
    SpibRegs.SPITXBUF = 0;//gz
    /*
    for (int i=0; i<ARRAYSIZE;i++){
        adcina2v=adcina2v+b[i]*joyxk[i];
    }
    for (int i=0; i<ARRAYSIZE;i++){
        adcina3v=adcina3v+b[i]*joyyk[i];
    }
    // Here write voltages value to DACA
    //setDACA(adcina2v);
    for(int i=0;i<ARRAYSIZE-1;i++){
        joyxk[ARRAYSIZE-1-i]=joyxk[ARRAYSIZE-2-i];
    }
    for(int i=0;i<ARRAYSIZE-1;i++){
        joyyk[ARRAYSIZE-1-i]=joyyk[ARRAYSIZE-2-i];
    }
    if((numADCacalls%100)==0){
        UARTPrint = 1;
    }
     */
    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

// MIC
// Function to implement sliding window filter
float slidingWindowFilter(float* buffer, int bufferSize, float newValue)
{
    //    micReadingMax = max(newValue, micReadingMax);
    // MIC: always get the max mic reading
    if (newValue > micReadingMax) {
        micReadingMax = newValue;
    }
    if (newValue < micReadingMin) {
        micReadingMin = newValue;
    }
    deltaMicReading = micReadingMax - micReadingMin;

    // Shift elements in the buffer
    for (int i = bufferSize - 1; i > 0; --i)
    {
        buffer[i] = buffer[i - 1];
    }

    // Insert the new value at the beginning of the buffer
    buffer[0] = newValue;

    // Calculate the average of the buffer
    float sum = 0;
    for (int i = 0; i < bufferSize; ++i)
    {
        sum += buffer[i];
    }

    return sum / bufferSize;
}





// MIC
// adcb1 pie interrupt
__interrupt void ADCB_ISR (void) {
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;

    adcb4result = AdcbResultRegs.ADCRESULT0;

    adcinb4_volt = adcb4result*3.0/4096.0; // Here covert ADCINB4 to volts


    // MIC: low pass filter  RSM-EX3.
    xarray_ADCB4[0] = adcinb4_volt; // Update the newest input value

    yk_ADCB4 = 0.0; // Calculate the filter output yk
    for (int k = 0; k < ARRAYSIZE; k++) {
        yk_ADCB4 += b[k] * xarray_ADCB4[k];
    }

    for (int k = ARRAYSIZE-1; k > 0; k--) {
        xarray_ADCB4[k] = xarray_ADCB4[k-1];
    }
    //    //    setDACA(adcinb4_volt); // Here write yk to DACB channel
        setDACA(yk_ADCB4 + 1.5);
//    setDACA(adcinb4_volt);

    // MIC: sliding window filter
    //    micReading = slidingWindowFilter(micReadingBuffer, sizeof(micReadingBuffer) / sizeof(micReadingBuffer[0]), adcinb4_volt);
//    micReading = slidingWindowFilter(micReadingBuffer, sizeof(micReadingBuffer) / sizeof(micReadingBuffer[0]), yk_ADCB4);
    micReading = slidingWindowFilter(micReadingBuffer, 30, yk_ADCB4);

    // change "deltaMicReading" to "yk_ADCB4"
    // "THRESHOLD" to "MicLPFThreshold"
    if (yk_ADCB4 > MicLPFThreshold) {
        state_scared = 1;
        state = 0;
        //        setEPWM2A(0);  // MIC: immediately stop the robot
        //        setEPWM2B(0);  // MIC: immediately stop the robot

    }
    else {
        // do nothing
        // state_scared = 0; // if the robot is not scared
    }


    // Print ADCIND0 and ADCIND1’s voltage value to TeraTerm every 100ms
    if ((CpuTimer2.InterruptCount % 100) == 0) {
        UARTPrint = 1;
    }

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
}


// DVG creating RC Servo angle for A
void setEPWM8A_RCServo(float angle) {

    if (angle > 97) {
        angle = 97;
    } else if (angle < -35) {
        angle = -35;
    }

    //    DVG Using the same approach as before with the motor, the CMPA is altered so that the duty cycle corresponds to the effort we wish to see exerted.
    //    This is done by treating the 8% duty cycle as a 0 degree angle and 12% duty cycle as a 90 degree angle. Both those points are used to create a linear
    //    relationship that is used to calculate value needed for CMPA to match the desired duty and angle. Same approach for motor B.
    EPwm8Regs.CMPA.bit.CMPA = (0.08 * EPwm8Regs.TBPRD) + 0.04 * EPwm8Regs.TBPRD * angle / 90;

}

// DVG creating RC Servo angle for B
void setEPWM8B_RCServo(float angle) {

    if (angle > 20) {
        angle = 20;
    } else if (angle < -99) {
        angle = -99;
    }

    EPwm8Regs.CMPB.bit.CMPB = (0.08 * EPwm8Regs.TBPRD) + 0.04 * EPwm8Regs.TBPRD * angle / 90;

}
