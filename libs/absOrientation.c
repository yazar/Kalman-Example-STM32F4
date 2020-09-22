#include "absOrientation.h"
#include "math.h"



const float TO_RADIANS_COEFF = PI / 180.0;
const float TO_DEGREES_COEFF = 1.0 / TO_RADIANS_COEFF;
const float SENSOR_TO_COMPASS_OFFSET = 180.0;
float magneticDeclinationOffset = 5.36;
float forwardAccelerationValue=0, upwardAccelerationValues=0;

void mutateAbsoluteAcceleration(float a12, float a22, float a31, float a32, float a33,float q0, float q1, float q2, float q3,float ax, float ay, float az, float magneticDeclinationOffset, float *absEast, float *absNorth, float *absUp) {
    float gravity[3];
    gravity[0] = a32;
    gravity[1] = a31;
    gravity[2] = a33;
    float q[4];
    q[0]= q0;  q[1]= q1;   q[2]= q2;   q[3]= q3;
  // I'm computing absolute acceleration differently here from elsewhere because I need to
  // maintain consistency with the quaternions...this is arguably bad with the difference in
  // the other section of this code
    float pureAx = ax;// (ax + gravity[0]); // this might be incorrect...
    float pureAy = ay; //(ay - gravity[1]);
    float pureAz = az;//(az - gravity[2]);

  // this effectively converts a quaternion to a matrix and then applies that rotation to a
  // vector.  Normally I wouldn't do these whacky comments and just move to an aptly named
  // function, but we care about performance a bit 
  float num1 = q[0] * 2.0;
  float num2 = q[1] * 2.0;
  float num3 = q[2] * 2.0;
  float num4 = q[0] * num1;
  float num5 = q[1] * num2;
  float num6 = q[2] * num3;
  float num7 = q[0] * num2;
  float num8 = q[0] * num3;
  float num9 = q[1] * num3;
  float num10 = q[3] * num1;
  float num11 = q[3] * num2;
  float num12 = q[3] * num3;

  float accelerationEast = (1.0 - (num5 + num6)) * pureAx + (num7 - num12) * pureAy + (num8 + num11) * pureAz;
  float accelerationNorth = (num7 + num12) * pureAx + (1.0 - (num4 + num6)) * pureAy + (num9 - num10) * pureAz;
  float accelerationUp = -1 * ((num8 - num11) * pureAx + (num9 + num10) * pureAy + (1.0 - (num4 + num5)) * pureAz);

    float sinMagneticDeclination = sin(magneticDeclinationOffset * TO_RADIANS_COEFF);
  float easternNorthComponent = sinMagneticDeclination * accelerationEast;
  float northernEasternComponent = -sinMagneticDeclination * accelerationNorth;

  accelerationNorth += northernEasternComponent;
  accelerationEast += easternNorthComponent;
	*absEast = accelerationEast;
	*absNorth = accelerationNorth;
	*absUp = accelerationUp;
}
void mutateRelativeAcceleration(float a12, float a22, float a31, float a32, float a33, float spitch, float ax, float ay, float az) {
    float gravity[3];
    gravity[0] = a32;
    gravity[1] = a31;
    gravity[2] = a33;

    float pureForwardAcceleration = -1 * (ay - gravity[1]);
    float pureUpAcceleration = -1 * (az - gravity[2]);
    float pureRightAcceleration = -1 * (ax + gravity[0]);

    float groundForwardAcceleration = (
        cos(spitch * TO_RADIANS_COEFF) * pureForwardAcceleration -
        sin(spitch * TO_RADIANS_COEFF) * pureUpAcceleration
    );
    float groundUpwardAcceleration = (
        cos(spitch * TO_RADIANS_COEFF) * pureUpAcceleration + 
        sin(spitch * TO_RADIANS_COEFF) * pureForwardAcceleration
    ); // SBL pitch yaw and roll rotations aren't commutative, so just leaving out roll entirely
  forwardAccelerationValue = groundForwardAcceleration;
  upwardAccelerationValues = groundUpwardAcceleration;
}

void updateFromSensor(float qq0, float qq1, float qq2, float qq3, float ax, float ay, float az, float magneticDeclinationOffset, float *absEast, float *absNorth, float *absUp) {

    float q[4];
    q[0]= qq0;   q[1]= qq1;   q[2]= qq2;   q[3]= qq3;
		float myroll, mypitch, myyaw;
    float a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    float a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    float a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    float a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    float a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    mypitch = asin(a32) * TO_DEGREES_COEFF;
    myyaw = SENSOR_TO_COMPASS_OFFSET + (atan2f(a12, a22) * TO_DEGREES_COEFF);
    myroll = -atan2(a31, a33) * TO_DEGREES_COEFF;
  float posYaw = myyaw, negYaw = myyaw;
    while(1) {
    if(fabs(posYaw) <= 180.0) {
      myyaw = posYaw;
      break;
    }
    if (fabs(negYaw) <= 180.0) {
      myyaw = negYaw;
      break;
    }
    negYaw -= 360.0;
    posYaw += 360.0;
  }
   
  mutateRelativeAcceleration(a12, a22, a31, a32, a33, mypitch,ax,ay,az);
  mutateAbsoluteAcceleration(a12, a22, a31, a32, a33,q[0],q[1],q[2],q[3],ax,ay,az, magneticDeclinationOffset,absEast, absNorth,absUp);
}

void absFarkli(float q0,float q1,float q2,float q3,float ax, float ay, float az, float *absEast, float *absNorth, float *absUp){
	q1 = -q1;
  q2 = -q2;
  q3 = -q3;
	*absEast = (1-2*(q2*q2 + q3*q3))*ax +   (2*(q1*q2 + q0*q3))*ay +   (2*(q1*q3 - q0*q2))*az;  // rotate linearaccel by quaternion
  *absNorth =   (2*(q1*q2 - q0*q3))*ax + (1-2*(q1*q1 + q3*q3))*ay +   (2*(q2*q3 + q0*q1))*az;
  *absUp =   (2*(q1*q3 + q0*q2))*ax +   (2*(q2*q3 - q0*q1))*ay + (1-2*(q1*q1 + q2*q2))*az;
}



