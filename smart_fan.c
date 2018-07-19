/*
 * Authors: Patrick Aung, Kevin Ng, Diego Bazan
 * ECE-218: Embedded Microcontroller Projects
 * Final Project: Smart Fan with Automated Speed Control
 * March 8, 2018
 */

/*********** COMPILER DIRECTIVES ***********/
#include "pic24_all.h"                              // textbook library header files
#include "adc.h"                                    // header file for simple adc library
#include "stdio.h"                                  // C library
#define EN_12 _LATB12                               // pin RB11 to write to H-bridge driver
#define PWM_PERIOD 510                              // period of PWM (102 microseconds)
#define S0 0                                        // 0% duty cycle
#define S1 255                                      // 50% duty cycle
#define S2 287                                      // 56.25% duty cycle
#define S3 319                                      // 62.5% duty cycle
#define S4 351                                      // 68.75% duty cycle
#define S5 383                                      // 75% duty cycle
#define S6 414                                      // 81.25% duty cycle
#define S7 446                                      // 87.5% duty cycle
#define S8 478                                      // 93.75% duty cycle
#define S9 510                                      // 100% duty cycle
#define NUM_ROW 4                                   // 4 matrix pad rows
#define NUM_COL 3                                   // 3 matrix pad columns
#define ROW0 _RB15                                  // Row 0 = RB15
#define ROW1 _RB12                                  // Row 1 = RB12
#define ROW2 _RB13                                  // Row 2 = RB13
#define ROW3 _RB14                                  // Row 3 = RB14
#define ROW0_ON (ROW0==0)                           // Row 0 detect signal
#define ROW1_ON (ROW1==0)                           // Row 1 detect signal
#define ROW2_ON (ROW2==0)                           // Row 2 detect signal
#define ROW3_ON (ROW3==0)                           // Row 3 detect signal
#define COL0 _LATB4                                 // Col 0 = RB4
#define COL1 _LATB5                                 // Col 1 = RB5
#define COL2 _LATB3                                 // Col 2 = RB3
#define SLIDER1 _RA2                                // ON/OFF switch
#define SLIDER2 _RA3                                // Auto/Manual switch
#define SLIDER1_ON() (SLIDER1 == 0)                 // Fan ON
#define SLIDER1_OFF() (SLIDER1 == 1)                // Fan OFF
#define SLIDER2_ON() (SLIDER2 == 0)                 // Mannual Mode
#define SLIDER2_OFF() (SLIDER2 == 1)                // Automatic Mode

/*********** GLOBAL VARIABLE AND FUNCTION DEFINITIONS ***********/
uint8_t input;                                      // user character input
uint16_t left;                                      // pin A1 on H-bridge driver (pulse_width input)
uint16_t right;                                     // pin A2 on H-bridge driver (pulse_width input)
uint8_t row;
uint8_t col;
char keypad_table[NUM_ROW][NUM_COL] = {{'1','2','3'}, {'4','5','6'},
                                      {'7','8','9'}, {'*','0','#'}};

/*********** MOTOR FUNCTIONS ***********/
// Configures OC1RS OC2RS registers to be compared with timer2
void configOC(void){
    CONFIG_OC1_TO_RP(RB1_RP);                       // remap pin RB1 to OC1
    CONFIG_OC2_TO_RP(RB2_RP);                       // remap pin RB2 to OC2
    T2CONbits.TON=0;                                // turn off timer2
    OC1R = 0;                                       // resets double buffer
    OC2R = 0;                                       // resets double buffer
    OC1CONbits.OCTSEL=0;                            // sets timer2 as the clock source
    OC2CONbits.OCTSEL=0;                            // sets timer2 as the clock source
    OC1CONbits.OCM=0b110;                           // output Compare x Mode Select bits
    OC2CONbits.OCM=0b110;                           // output Compare x Mode Select bits
}

// Configures timer2 that defines the period of the PWM signal
void configTimer2(void){
    T2CONbits.TON=0;                                // turn timer2 off
    T2CON = 0x0010;                                 // TMR2 off, default idle mode, gate off, 1:8, 32-bit off, internal clock
    PR2 = PWM_PERIOD;                               // sets Preset Reg to PWM_PERIOD
    _T2IF = 0;                                      // clears timer 2 flag (_T2IF)
    T2CONbits.TON=1;                                // turn timer2 on
}

// Interrupt service routine that defines PWM signals
void _ISR _T2Interrupt(void){
    OC2RS = left;                                   // updates OC2RS reg with desired pulse width
    OC1RS = right;                                  // updates OC1RS reg with  desired pulse width
    _T2IF=0;                                        // resets timer 2 interrupt flag
}

/*********** TEMPERATURE SENSOR FUNCTIONS ***********/
// Samples analog input and converts to millivolts
float getAdcMV(void){
    int16_t adcVal = convertADC1();                  // adcvalue in V
    float adcMV = (adcVal/4096.0 * 3.3)*1000;       //convert to float in range 0.0 to VREF
    return adcMV;
}

// Returns Temp in C given adc voltage in mV
float getTempC(float adcMV){
    float V1 = 952.0;                               // mV corresponding to T1 from data sheet
    float V2 = 882.0;                               // mV corresponding to T2 from data sheet
    float T1 = 15.0;                                // lower bound temp in celsius used in linear approx
    float T2= 28.0;                                 // upper bound temp in celsius used in linear approx
    float C = T1 + ((adcMV-V1)/((V2-V1)/(T2-T1)));  //stores computed temperature in celsius
    return C;
}

// Returns temperature in Fahrenheit given temp in Celsius
float getTempF(float tempC){
    float tempF = (tempC*1.8)+32;                   // converts Celsius to Fahrenheit
    return tempF;
}

/*********** MATRIX KEYPAD FUNCTIONS ***********/
// Configure change notification pins of the matrix pad
void configCN(void) {
    _TRISB15 = 1;                                   // Set RB15 as input
    _TRISB14 = 1;                                   // Set RB14 as input
    _TRISB13 = 1;                                   // Set RB13 as input
    _TRISB12 = 1;                                   // Set RB12 as input

    _CN11IE = 1;                                    // Enable CN on RB15
    _CN12IE = 1;                                    // Enable CN on RB14
    _CN13IE = 1;                                    // Enable CN on RB13
    _CN14IE = 1;                                    // Enable CN on RB12

    _CN11PUE = 1;                                   // Internal  pullup on RB15
    _CN12PUE = 1;                                   // Internal  pullup on RB14
    _CN13PUE = 1;                                   // Internal  pullup on RB13
    _CN14PUE = 1;                                   // Internal  pullup on RB12

    _TRISB3 = 0;                                    // Set RB3 as input
    _TRISB4 = 0;                                    // Set RB4 as input
    _TRISB5 = 0;                                    // Set RB5 as input

    _LATB3 = 1;                                     // Enable write to RB3
    _LATB4 = 1;                                     // Enable write to RB4
    _LATB5 = 1;                                     // Enable write to RB5
}

// Interrupt Service Routine for the Matrix Pad
void _ISR _CNInterrupt(){
    DELAY_MS(25);
    do{
        if (ROW0_ON){
           row=0;
        }
        if (ROW1_ON){
            row=1;
        }
        if (ROW2_ON){
           row=2;
        }
        if (ROW3_ON){
            row=3;
        }
    } while(ROW0_ON || ROW1_ON || ROW2_ON || ROW3_ON);
    input = keypad_table[row][col];
    _CNIF=0;                                        // Reset CN Interrupt flag
}

// Enables one matrix column at a time to set the keypad_table array column
void scanKeypad(void){
        col=0;
        COL0=0;
        COL1=1;
        COL2=1;
        DELAY_MS(50);

        col=1;
        COL0=1;
        COL1=0;
        COL2=1;
        DELAY_MS(50);

        col=2;
        COL0=1;
        COL1=1;
        COL2=0;
        DELAY_MS(50);
}

// Controls fan based on matrix input. Displays fan mode and fan speed on terminal
void manual_control_display(void){
    printf("\e[1;1H\e[2J");                         // clear terminal display
    printf("Smart Fan ON: Manual Mode \n\n\r");
    printf("Fan Speed ");
    scanKeypad();                                   // scans the keypad for
    if (input == '1'){
        left=S1;                                    // set to speed 1 (50% duty cycle)
    }
    if (input == '2'){
        left=S2;                                    // set to speed 2 (56.25% duty cycle)
    }
    if (input == '3'){
        left=S3;                                    // set to speed 3 (62.5% duty cycle)
    }
    if (input == '4'){
        left=S4;                                    // set to speed 4 (68.75% duty cycle)
    }
    if (input == '5'){
        left=S5;                                    // set to speed 5 (75% duty cycle)
    }
    if (input == '6'){
        left=S6;                                    // set to speed 6 (81.25% duty cycle)
    }
    if (input == '7'){
        left=S7;                                    // set to speed 7 (87.5% duty cycle)
    }
    if (input == '8'){
        left=S8;                                    // set to speed 8 (93.75% duty cycle)
    }
    if (input == '9'){
        left=S9;                                    // set to speed 9 (100% duty cycle)
    }
    if (input=='1'||input=='2'||input=='3'||input=='4'||
        input=='5'||input=='6'||input=='7'||input=='8'||input=='9'){ // enter only if input is a speed number
        printf("%c \r", input);                     // display speed in rev/s
    }
    DELAY_MS(150);
}

// Controls fan speed based on temperature calculated from temperature sensor reading
// Displays fan mode and fan speed on terminal
void automatic_control_display(){
    float adcMV = getAdcMV();                       // adcMV holds adc voltage in mV
    float Celsius = getTempC(adcMV);                // Celsius holds temp in Celsius
    float Fahrenheit = getTempF(Celsius);           // Fahrenheit holds temp in Fahrenheit
    printf("\e[1;1H\e[2J");                         // clear terminal display
    printf("Smart Fan ON: Automatic Mode \n\n\r");
    printf("Temperature (C) %.2f \r\n", Celsius);
    printf("Temperature (F) %.2f \r\n", Fahrenheit);
    if(Celsius<24.00){
        printf("Fan Speed 0 \r\n\n");
        left=S0;                                    // set to speed 0 (0% duty cycle)
    }
    if (Celsius>=24.0 && Celsius<24.5){
        printf("Fan Speed 1 \r\n\n");
        left=S1;                                    // set to speed 1 (50% duty cycle)
    }
    if (Celsius>=24.5 && Celsius<25.0){
        printf("Fan Speed 2 \r\n\n");
        left=S2;                                    // set to speed 2 (56.25% duty cycle)
    }
    if (Celsius>=25.0 && Celsius<25.5){
        printf("Fan Speed 3 \r\n\n");
        left=S3;                                    // set to speed 3 (62.5% duty cycle)
    }
    if (Celsius>=25.5 && Celsius<26.0){
        printf("Fan Speed 4 \r\n\n");
        left=S4;                                    // set to speed 4 (68.75% duty cycle)
    }
    if (Celsius>=26.0 && Celsius<26.5){
        printf("Fan Speed 5 \r\n\n");
        left=S5;                                    // set to speed 5 (75% duty cycle)
    }
    if (Celsius>=26.5 && Celsius<27.0){
        printf("Fan Speed 6 \r\n\n");
        left=S6;                                    // set to speed 6 (81.25% duty cycle)
    }
    if (Celsius>=27.0 && Celsius<27.5){
        printf("Fan Speed 7 \r\n\n");
        left=S7;                                    // set to speed 7 (87.5% duty cycle)
    }
    if (Celsius>=27.5 && Celsius<28.0){
        printf("Fan Speed 8 \r\n\n");
        left=S8;                                    // set to speed 8 (93.75% duty cycle)
    }
    if (Celsius>=28.0){
        printf("Fan Speed 9 \r\n\n");
        left=S9;                                    // set to speed 9 (100% duty cycle)
    }
    DELAY_MS(1000);
}
/********** MAIN PROGRAM LOOP********************************/
int main ( void ){
    configClock();                                  // sets the clock to 40MHz using FRC and PLL
    configOC();                                     // initialize OC1 register settings
    configTimer2();                                 // initialize timer2 settings
    configUART1(230400);                            // initialize  UART1 with baudrate
    configCN();                                     // configure change notification settings
    CONFIG_RA1_AS_ANALOG();
    configADC1_ManualCH0(RA1_AN, 31, 1);

// Initialize ports and other one-time code
    AD1PCFGL=0xFFFD;
    _TRISB11 = 0;                                   // RB12 as output
    EN_12 = 1;                                      // enable H-bridge driver
    _T2IE = 1;                                      // enable timer 2 interrupt
    _CNIE=1;                                        // enable change notification
    _TRISA2 = 1;                                    // SLIDER1 as input
    _CN30PUE = 1;                                   // enable pullup on SLIDER1 (CN30)
     _TRISA3 = 1;                                   // SLIDER2 as input
    _CN29PUE = 1;                                   // enable pullup on SLIDER2 (CN29)
    left = 0;                                       // set left pwm to 0
    right = 0;                                      // also set right pwm to 0 to stop motor at program start
    input = '0';                                    // set initial matrix input to '0'

/* Main program loop */
	while (1) {
        if (SLIDER1_ON()){
            while(SLIDER2_ON() && SLIDER1_ON()){
                manual_control_display();
            }
            while(SLIDER2_OFF() && SLIDER1_ON()){
                automatic_control_display();
            }
        }
        if (SLIDER1_OFF()){
            printf("\e[1;1H\e[2J");                 // clear terminal display
            left=0;                                 // turn off motor
            printf("Smart Fan OFF \n\r");           // print msg once
            while(SLIDER1_OFF()){
                // do nothing while fan OFF
            }
        }
    }
}
