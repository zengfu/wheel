#ifndef _QUATERNION_H
#define _QUATERNION_H


void euler_fromRotation(float* euler,float *r);
void euler_toRotation(float* euler,float *r);
void quat_fromRotation(float* q,float *r);
void quat_toRotation(float* q,float *r);
void quat_fromEuler(float* q,float* euler);
void quat_toEuler(float*q,float* euler);




#endif