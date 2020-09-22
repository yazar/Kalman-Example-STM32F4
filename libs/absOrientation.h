#ifndef ABSORIENTATION_H
#define ABSORIENTATION_H

#ifdef __cplusplus
extern "C" {
#endif

#define PI 3.14159265359

void mutateAbsoluteAcceleration(float a12, float a22, float a31, float a32, float a33,float q0, float q1, float q2, float q3,float ax, float ay, float az, float magneticDeclinationOffset, float *absEast, float *absNorth, float *absUp);

void mutateRelativeAcceleration(float a12, float a22, float a31, float a32, float a33, float spitch, float ax, float ay, float az);

void updateFromSensor(float  qq0, float qq1, float qq2, float qq3, float ax, float ay, float az,float magneticDeclinationOffset, float *absEast, float *absNorth, float *absUp);


void absFarkli(float q0,float q1,float q2,float q3,float ax, float ay, float az, float *absEast, float *absNorth, float *absUp);




#ifdef __cplusplus
}
#endif
#endif // SFE_LSM9DS1_H //