#ifndef ROOTFINDER__H__
#define ROOTFINDER__H__

int gelim(float **a,float *b,float *x, int n);
void f(float* x, float* eqs, float AB, float BC, float AC, float cAB, float cBC, float cAC);
void jac(float* x, float **jacobian, float cAB, float cBC, float cAC);
void newtonOpt(float* x, int* maxiter, float* eqs, float **jacobian, float AB, float BC, float AC, float cAB, float cBC, float cAC);
 
 #endif