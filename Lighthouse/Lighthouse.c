#include <stdio.h>
#include "NU32.h"
#include "rootFinder.h"
#include <xc.h>
#include <string.h>
#include <math.h>

#define FT 100000 // middle value of two flash waiting time
#define ver 50000  //verify if it is flash time 
#define ang_vel 180 //angle/ms
#define BUFFERSIZE 100


static volatile int count = 0 ,verify = 0, s1 = 0, start  = 0;
static volatile unsigned int time[8]; 
static volatile unsigned int st1, st2, st3;
static volatile char buffer[100];
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

float test = _CP0_GET_COUNT()*0.025;

sprintf(buffer,"%.1f\n\r",test);
			        NU32_WriteUART3(buffer);
	// if(s1 == 0){
	// 	start = 1;
	// 	if(verify == 1){
	// 		if(_CP0_GET_COUNT()>ver){
	// 				time[0] = _CP0_GET_COUNT();
	// 		        sprintf(buffer,"f1 = %d\n\r",time[0]);
	// 		        NU32_WriteUART3(buffer);
	// 				s1 = 1;
	// 				count++;
	// 			}
	// 		}
	// }
	// if (s1 == 2){
	// 	s1 = 3;
	// }
	// if (s1 == 4){
	// 	time[4] = _CP0_GET_COUNT();
	// 		// sprintf(buffer,"f2 = %d\n\r",time[4]);
	// 		// NU32_WriteUART3(buffer);
	// 		s1 = 5;
	// 		count++;
	// }
	// if(s1 == 6){
	// 	s1 = 7;
	// }
	// sprintf(buffer,"%d %d\n\r",time[0],time[4]);
	// 		        NU32_WriteUART3(buffer);
	// // switch (s1){
	// 	case 0:
	// 	{
	// 		start = 1;
	// 		if(verify == 1){
	// 			if(_CP0_GET_COUNT()>ver){
	// 				time[0] = _CP0_GET_COUNT();
	// 		sprintf(buffer,"%d\n\r",time[0]);
	// 		NU32_WriteUART3(buffer);
	// 				s1 = 1;
	// 				count++;
	// 			}
	// 		}
	// 		break;
	// 	}
	// 	case 2:
	// 	{
	// 		s1 = 3;
	// 		break;
	// 	}
	// 	case 4:
	// 	{
	// 		time[4] = _CP0_GET_COUNT();
	// 		// sprintf(buffer,"%d\n\r",time[4]);
	// 		// NU32_WriteUART3(buffer);
	// 		s1 = 5;
	// 		count++;
	// 		break;
	// 	}
	// 	case 6:
	// 	{

	// 		s1 = 7;
	// 		break;
	// 	}
	// }



	// 	switch (j1){
	// 		case 1:
	// 		{
	// 			time[0] = _CP0_GET_COUNT(); //flash1
	// 			j1++;
	// 			count++;
	// 			break;
	// 		}
	// 		case 5:
	// 		{
	// 			time[4] = _CP0_GET_COUNT(); //flash2
	// 			j1++;
	// 			count++;
	// 			break;
	// 		}
	// 	}
	IFS0bits.INT0IF = 0;
} 

void __ISR(_EXTERNAL_1_VECTOR, IPL3SOFT) RS1_EdgeISR(void){	
_CP0_SET_COUNT(0);

	// if (s1 == 0){
	// 	if(start == 1){  
	// 	       _CP0_SET_COUNT(0);
	// 	       verify = 1;
	//         }
	// }
	// if (s1 == 1){
	// 	time[1] = _CP0_GET_COUNT()-time[0];
	// 		s1 = 2;
	// 		count++;
	// }
	// if (s1 == 3){
	// 	_CP0_SET_COUNT(0);
	// 		s1 = 4;
	// }
	// if (s1 == 5){
	// 	time[5] = _CP0_GET_COUNT()-time[4];
	// 		s1 = 6;
	// 		count++;
	// }
	// if (s1 == 7){
	// 	_CP0_SET_COUNT(0);
	// 		s1 = 0;
	// }



	// // switch(s1){
	// 	case 0:
	// 	{
	// 		if(start == 1){  
	// 	       _CP0_SET_COUNT(0);
	// 	       verify = 1;
	//         }
	// 	}
	// 	case 1:
	// 	{
	// 		time[1] = _CP0_GET_COUNT()-time[0];
	// 		s1 = 2;
	// 		count++;
	// 		break;
	// 	}
	// 	case 3:
	// 	{
	// 		_CP0_SET_COUNT(0);
	// 		s1 = 4;
	// 		break;
	// 	}
	// 	case 5:
	// 	{
	// 		time[5] = _CP0_GET_COUNT()-time[4];
	// 		s1 = 6;
	// 		count++;
	// 		break;
	// 	}
	// 	case 7:
	// 	{
	// 		_CP0_SET_COUNT(0);
	// 		s1 = 0;
	// 		break;
	// 	}
	// }

		// 	switch (j1){
		// 		case 1:
		// 		{
		// 			//clear_stick(time);  //clear time[] 
		// 			tnow = _CP0_SET_COUNT();
		// 		    j1++; 
		// 		    break;
		// 	    }
		// 	    case 3:
		// 	    {
		// 	    	time[1] = _CP0_GET_COUNT(); //sweep11
		// 	    	j1++;
		// 	    	count++;
		// 	    	break;
		// 	    }
		// 	    case 4:
		// 	    {
		// 	    	_CP0_SET_COUNT(0);  
		// 	    	j1++;
		// 	    	break;
		// 	    }
		// 	    case 6:
		// 	    {
		// 	    	time[5] = _CP0_GET_COUNT(); //sweep12
		// 	    	j1 = 1;
		// 	    	count++;
		// 	    	break;
		// 	    }
		// }
	
	IFS0bits.INT1IF = 0;
}

void __ISR(_EXTERNAL_2_VECTOR, IPL3SOFT) RS2_EdgeISR(void){
	if(s1 == 1){
		time[2] = _CP0_GET_COUNT()-time[0];
				count++;
	}
	if(s1 == 5){
		time[6] = _CP0_GET_COUNT()-time[4];
				count++;
	}
		// switch(s1){
		// 	case 1:
		// 	{
		// 		time[2] = _CP0_GET_COUNT()-time[0];
		// 		count++;
		// 		break;
		// 	}
		// 	case 5:
		// 	{
		// 		time[6] = _CP0_GET_COUNT()-time[4];
		// 		count++;
		// 		break;
		// 	}
		// }
		// // 	switch (j2){
		// 		case 1:
		// 		{
		// 			j2++;
		// 			break;
		// 		}
		// 	    case 2:
		// 	    {
		// 	    	time[2] = _CP0_GET_COUNT(); //sweep21
		// 	    	j2++;
		// 	    	count++;
		// 	    	break;
		// 	    }
		// 	    case 3:
		// 	    {
		// 	    	j2++;
		// 	    	break;
		// 	    }
		// 	    case 4:
		// 	    {
		// 	    	time[6] = _CP0_GET_COUNT(); //sweep22
		// 	    	j2 = 1;
		// 	    	count++;
		// 	    	break;
		// 	    }
		// }
	
	IFS0bits.INT2IF = 0;
}
void __ISR(_EXTERNAL_3_VECTOR, IPL3SOFT) RS3_EdgeISR(void){
	if(s1 == 1){
		time[3] = _CP0_GET_COUNT()-time[0];
				count++;
	}
	if(s1 == 5){
		time[7] = _CP0_GET_COUNT()-time[4];
				count++;
	}
		// switch(s1){
		// 	case 1:
		// 	{
		// 		time[3] = _CP0_GET_COUNT()-time[0];
		// 		count++;
		// 		break;
		// 	}
		// 	case 5:
		// 	{
		// 		time[7] = _CP0_GET_COUNT()-time[4];
		// 		count++;
		// 		break;
		// 	}
		// }
		// // 	switch (j3){
		// 		case 1:
		// 		{
		// 			j3++;
		// 			break;
		// 		}
		// 		case 2:
		// 	    {
		// 	    	time[3] = _CP0_GET_COUNT(); //sweep31
		// 	    	j3++;
		// 	    	count++;
		// 	    	break;
		// 	    }
		// 	    case 3:
		// 	    {
		// 	    	j3++;
		// 	    	break;
		// 	    }
		// 	    case 4:
		// 	    {
		// 	    	time[7] = _CP0_GET_COUNT(); //sweep32
		// 	    	j3 = 1;
		// 	    	count++;
		// 	    	break;
		// 	    }
		// }
	
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


void copy_stick_time(unsigned int* t, unsigned int* s){
	int i;
	for (i=0 ; i<8 ; i++){
		t[i] = s[i];
	}
}

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
    char buffer[100];
    float Time[8];
    unsigned int stick[8];
    __builtin_enable_interrupts();
	while (1) {	
		// reset_stick(time);
		// multi_constant_arrey(1.0,Time,time);
		// sprintf(buffer,"flash1=%d\r",time[0]);
		// 	NU32_WriteUART3(buffer);

		// 	// copy_stick_time(stick, time);
		// 	// // reset_stick(stick);
		// 	// sprintf(buffer,"%d %d %d %d %d %d %d %d \n\r",stick[0],stick[1],stick[2],stick[3],stick[4],stick[5],stick[6],stick[7]);
		// 	// NU32_WriteUART3(buffer);
		// 	// count = 0;
		// 	// sprintf(buffer,"%d\r\n", adc_counts());
  //  // //      NU32_WriteUART3(buffer);
		// 	// sprintf(buffer,"sticks=%s",time);
		// 	// NU32_WriteUART3(buffer);

		// 	// coordinate(time, A, B, C, AB, BC, AC);
		// 	// printf("Ax= %6.3f Ay= %6.3f Az= %6.3f", A[0],A[1],A[2]);
		// 	// printf("Bx= %6.3f By= %6.3f Bz= %6.3f", B[0],B[1],B[2]);
		// 	// printf("Cx= %6.3f Cy= %6.3f Cz= %6.3f", C[0],C[1],C[2]);
		// }
		;
	}
	return 0;
}
