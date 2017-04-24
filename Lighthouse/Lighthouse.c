#include <stdio.h>
#include "NU32.h"
#include "rootFinder.h"
#include <xc.h>
#include <string.h>
#include <math.h>

#define FT 15 // middle value of two flash waiting time
#define ang_vel 180 //angle/ms
#define BUFFERSIZE 100


static volatile int count = 0 , j1 = 1 , j2 = 1 , j3 = 1;
static volatile int time[8]; 
// [flash1,sweep11,sweep21,sweep31,flash2,sweep12,sweep22,sweep32] 
//[flash1, sensor1sweep1, senser2sweep1,sensor3sweep1,flash2,sensor1sweep2,sensor2sweep2,sensor3sweep2]
//flash1(vertical) < flash2(horizontal)
//Set INT0 start counting flash time at rising edge 

void clear_stick(int* s){
	int i;
	for(i=0 ; i<8 ; i++){
		s[i] = 0;
	}
}

void __ISR(_EXTERNAL_0_VECTOR, IPL2SOFT) F_EdgeISR(void){ 
		switch (j1){
			case 2:
			{
				time[0] = _CP0_GET_COUNT(); //flash1
				j1++;
				count++;
				break;
			}
			case 5:
			{
				time[4] = _CP0_GET_COUNT(); //flash2
				j1++;
				count++;
				break;
			}
		}
	IFS0bits.INT0IF = 0;
} 

void __ISR(_EXTERNAL_1_VECTOR, IPL3SOFT) RS1_EdgeISR(void){	
			switch (j1){
				case 1:
				{
					clear_stick(time);  //clear time[] 
					_CP0_SET_COUNT(0);
				    j1++; 
				    break;
			    }
			    case 3:
			    {
			    	time[1] = _CP0_GET_COUNT()-time[0]; //sweep11
			    	j1++;
			    	count++;
			    	break;
			    }
			    case 4:
			    {
			    	_CP0_SET_COUNT(0);  
			    	j1++;
			    	break;
			    }
			    case 6:
			    {
			    	time[5] = _CP0_GET_COUNT()-time[4]; //sweep12
			    	j1 = 1;
			    	count++;
			    	break;
			    }
		}
	
	IFS0bits.INT1IF = 0;
}

void __ISR(_EXTERNAL_2_VECTOR, IPL3SOFT) RS2_EdgeISR(void){
		
			switch (j2){
				case 1:
				{
					j2++;
					break;
				}
			    case 2:
			    {
			    	time[2] = _CP0_GET_COUNT()-time[0]; //sweep21
			    	j2++;
			    	count++;
			    	break;
			    }
			    case 3:
			    {
			    	j2++;
			    	break;
			    }
			    case 4:
			    {
			    	time[6] = _CP0_GET_COUNT()-time[4]; //sweep22
			    	j2 = 1;
			    	count++;
			    	break;
			    }
		}
	
	IFS0bits.INT2IF = 0;
}
void __ISR(_EXTERNAL_3_VECTOR, IPL3SOFT) RS3_EdgeISR(void){
		
			switch (j3){
				case 1:
				{
					j3++;
					break;
				}
				case 2:
			    {
			    	time[3] = _CP0_GET_COUNT()-time[0]; //sweep31
			    	j3++;
			    	count++;
			    	break;
			    }
			    case 3:
			    {
			    	j3++;
			    	break;
			    }
			    case 4:
			    {
			    	time[7] = _CP0_GET_COUNT()-time[4]; //sweep32
			    	j3 = 1;
			    	count++;
			    	break;
			    }
		}
	
	IFS0bits.INT3IF = 0;
}
// Initiate flash time INT0 and INT1 
void get_timeinit(void){
	INTCONbits.INT0EP = 0;          // step 3: INT0 triggers on falling edge
    IPC0bits.INT0IP = 2;            // step 4: interrupt priority 2
    IPC0bits.INT0IS = 1;            // step 4: interrupt priority 1
    IFS0bits.INT0IF = 0;            // step 5: clear the int flag
    IEC0bits.INT0IE = 1;            // step 6: enable INT0 by setting IEC0<3>

    INTCONbits.INT1EP = 1;          // step 3: INT1 triggers on rising edge
    IPC1bits.INT1IP = 3;            // step 4: interrupt priority 3
    IPC1bits.INT1IS = 1;            // step 4: interrupt priority 1
    IFS0bits.INT1IF = 0;            // step 5: clear the int flag
    IEC0bits.INT1IE = 1;            // step 6: enable INT1 by setting IEC0<7>

    INTCONbits.INT2EP = 1;          // step 3: INT1 triggers on rising edge
    IPC2bits.INT2IP = 3;            // step 4: interrupt priority 3
    IPC2bits.INT2IS = 2;            // step 4: interrupt priority 2
    IFS0bits.INT2IF = 0;            // step 5: clear the int flag
    IEC0bits.INT2IE = 1;            // step 6: enable INT2 by setting IEC0<11>

    INTCONbits.INT3EP = 1;          // step 3: INT1 triggers on rising edge
    IPC3bits.INT3IP = 3;            // step 4: interrupt priority 3
    IPC3bits.INT3IS = 3;            // step 4: interrupt priority 3
    IFS0bits.INT3IF = 0;            // step 5: clear the int flag
    IEC0bits.INT3IE = 1;            // step 6: enable INT3 by setting IEC0<15>
}

void reset_stick(int *r){   //make sure flash1(vertical) < flash2(horizontal)
	int ii = 0;
	int temp;
	if (r[0] > FT){
		for (ii=0 ; ii<4 ; ii++){
			r[ii] = temp;
			r[ii] = r[ii+4];
			r[ii+4] = temp;
		}
	}
}


// void copy_stick_time(float* t, int* s){
// 	int i;
// 	for (i=0 ; i<8 ; i++){
// 		t[i] = s[i];
// 	}
// }

void multi_constant_arrey(float mul, float* t, int *s){
	int num = 8;
	int i;
	for (i=0 ; i<num ; i++){
		t[i] = (float)s[i]*mul;
	}
}

void coordinate(int *stick, float* a, float* b, float* c, float AB, float BC, float AC){
	
	float t[8];
	float vA,vB,vC,hA,hB,hC,cAB,cAC,cBC;
	float r0[3];  //initial RA RB RC
	float eqs[3];
	float **jacMat=(float**)malloc(3*(sizeof(float*)));
	int i;
	for(i=0;i<3;i++){
		*(jacMat+i)=(float*)malloc(sizeof(float)*3);
	}
	int maxIterVal = 10000; //maxiteration value
	int* maxiter = &maxIterVal;


	reset_stick(stick);
	multi_constant_arrey(0.025*ang_vel,t,stick);  //sticks convert to angle (ms)
	vA = t[1];
	vB = t[2];
	vC = t[3];
	hA = t[5];
	hB = t[6];
	hC = t[7];
	cAB = sin(vA)*cos(hA)*sin(vB)*cos(hB)+sin(vA)*sin(hA)*sin(vB)*sin(hB)+cos(vA)*cos(vB);
	cBC = sin(vB)*cos(hB)*sin(vC)*cos(hC)+sin(vB)*sin(hB)*sin(vC)*sin(hC)+cos(vB)*cos(vC);
	cAC = sin(vA)*cos(hA)*sin(vC)*cos(hC)+sin(vA)*sin(hA)*sin(vC)*sin(hC)+cos(vA)*cos(vC);

	newtonOpt(r0 , maxiter , eqs , jacMat, AB, BC, AC, cAB, cBC, cAC); //calculate R={RA,RB,RC}

	a[0] = r0[0] * sin(vA) * cos(hA);
	a[1] = r0[0] * sin(vA) * sin(hA);
	a[2] = r0[0] * cos(vA);

	b[0] = r0[1] * sin(vB) * cos(hB);
	b[1] = r0[1] * sin(vB) * sin(hB);
	b[2] = r0[1] * cos(vB);

	c[0] = r0[2] * sin(vC) * cos(hC);
	c[1] = r0[2] * sin(vC) * sin(hC);
	c[2] = r0[2] * cos(vC);
}


int main(){
	NU32_Startup();
	__builtin_disable_interrupts();
	get_timeinit();
	float AB=2.0, BC=2.0, AC=4.0;  // Sensor posistion in board
    float A[3],B[3],C[3]; //sensor position
    __builtin_enable_interrupts();
	while (1) {	
		if (count == 8){
			count = 0;
			coordinate(time, A, B, C, AB, BC, AC);
			printf("Ax= %6.3f Ay= %6.3f Az= %6.3f", A[0],A[1],A[2]);
			printf("Bx= %6.3f By= %6.3f Bz= %6.3f", B[0],B[1],B[2]);
			printf("Cx= %6.3f Cy= %6.3f Cz= %6.3f", C[0],C[1],C[2]);
		}
	}
	return 0;
}
