#include <stdio.h>
#include "NU32.h"
#include <xc.h>

#define flash_time 20 // flash waiting time
#define ang_vel 10
#define BUFFERSIZE 100
static volatile int flash = 1, swipe = 0; // estimate the lighthouse is flash or swipe
static volatile float ang_time_x , ang_time_y; // the time corresponding to the angle of the lighthouse IR rotation 
static volatile int axis = 1; // get axis=1 correspond to x axis position, axis=0 correspond to y axis position

void __ISR(_TIMER_3_VECTOR, IPL5SOFT) Timer3ISR(void) {
	while (axis == 1){  
	if (PORTBbits.RB0 == 1 && flash == 1){ // "start stopwatch" command
			_CP0_SET_COUNT(0);
		    flash = 0;
		    swipe = 1;
		    while (_CP0_GET_COUNT() < flash_time){ ; } // wait for flash time
		}
	    else if (PORTBbits.RB0 == 1 && swipe == 1){ // get the time when IR hits sensor
			ang_time_x = _CP0_GET_COUNT()*25.0;
			swipe = 0;
		}
		else if (PORTBbits.RB0 == 0 & swipe == 0){  //get the time when IR leave the sensor
			ang_time_x = ang_time_x+(_CP0_GET_COUNT()*25.0 - ang_time_x)/2.0; // calculate the angle time 
			flash = 1;
			swipe = 1;
			axis = 0;
		}
	}
	while (axis == 0){
	if (PORTBbits.RB0 == 1 && flash == 1){ // "start stopwatch" command
			_CP0_SET_COUNT(0);
		    flash = 0;
		    swipe = 1;
		    while (_CP0_GET_COUNT() < flash_time){ ; } // wait for flash time
		}
	    else if (PORTBbits.RB0 == 1 && swipe == 1){ // get the time when IR hits sensor
			ang_time_y = _CP0_GET_COUNT()*25.0;
			swipe = 0;
		}
		else if (PORTBbits.RB0 == 0 & swipe == 0){  //get the time when IR leave the sensor
			ang_time_y = ang_time_y+(_CP0_GET_COUNT()*25.0 - ang_time_y)/2.0; // calculate the angle time 
			flash = 1;
			swipe = 1;
			axis = 1;
		}
	}
} 

int main(){
	NU32_Startup();
	__builtin_disable_interrupts();
	PR3 = 3999;                  // period = (PR3+1) * N * 12.5 ns = 50 us, 20 kHz
    TMR3 = 0;                    // initial TMR3 count is 0
    T3CONbits.TCKPS = 0b000;     // Timer3 prescaler N=1 (1:1)
    T3CONbits.ON = 1;            // turn on Timer3
    IPC3bits.T3IP = 5;            // step 4: interrupt priority
    IPC3bits.T3IS = 0;            // step 4: interrupt priority
    IFS0bits.T3IF = 0;            // step 5: clear the int flag
    IEC0bits.T3IE = 1;            // step 6: enable Timer3 

    TRISBbits.TRISB0 = 1; //set B0 as input
    TRISBbits.TRISB1 = 0; //set B1 as output
    flash = 1;  // set PIC32 ready to start stopwatch
    swipe = 0;
    float ang_x, ang_y; // x and y postion of sensor
    char buffer[BUFFERSIZE];
    __builtin_enable_interrupts();
	
	while (1) {	
		switch (axis){
			case 1: // x axis position
			{
				ang_x = ang_time_x * ang_vel;   //calculate x postion 
				sprintf(buffer,"%6.3f\r\n", ang_x);//send data to pc
				NU32_WriteUART3(buffer);
				axis = 0; // set to calculate y position
				break;
			}
			case 0: // y axis position
			{
				ang_y = ang_time_y * ang_vel;   //calculate y position
				sprintf(buffer,"%6.3f\r\n", ang_y);//send data to pc
				NU32_WriteUART3(buffer);
				axis = 1; // set to calculate x position
				break;
			}

		}
	}
	return 0;
}
