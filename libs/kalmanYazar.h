
#ifndef KALMANYAZAR_H_
#define KALMANYAZAR_H_

#ifdef __cplusplus
  extern "C" {
#endif


#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "math.h"
#include <stdio.h>
#include "matrix.h"

#define M_PI 3.14159265358979323846

typedef struct GeoPoint {
  float Latitude;
  float Longitude;
}GeoPoint;

typedef struct KalmanFilterFusedPositionAccelerometer {
	

float I[2][2];
	
float H[2][2];

float P[2][2];
	
float Q[2][2];

float R[2][2];

float u[1][1];

float z[2][1];

float A[2][2];

float B[2][1];

float currentState[2][1];


  float currentStateTimestampSeconds;



}KalmanFilterFusedPositionAccelerometer;


void recreateControlMatrix(float deltaSeconds, struct KalmanFilterFusedPositionAccelerometer *k);
void recreateStateTransitionMatrix(float deltaSeconds, struct KalmanFilterFusedPositionAccelerometer *k);
float RadiansToDegrees(float radian);
float DegreesToRadians(float degree);
float geoAngle(float latOrLon);
float LatitudeToMeters(float latitude);
float LongitudeToMeters(float longitude);
GeoPoint GetPointAhead( GeoPoint fromCoordinate, float distanceMeters, float azimuth);
GeoPoint PointPlusDistanceEast(GeoPoint fromCoordinate, float distance);
GeoPoint PointPlusDistanceNorth(GeoPoint fromCoordinate, float distance);
GeoPoint MetersToGeopoint(float latAsMeters, float lonAsMeters);
float  GetDistanceMeters(GeoPoint fromCoordinate , GeoPoint toCoordinate );
void  Predict(float accelerationThisAxis, float timestampNow, struct KalmanFilterFusedPositionAccelerometer *k);
void Update(float position, float velocityThisAxis, float *positionError, float velocityError, struct KalmanFilterFusedPositionAccelerometer *k);
float  GetPredictedPosition (struct KalmanFilterFusedPositionAccelerometer *k);
float  GetPredictedVelocityThisAxis (struct KalmanFilterFusedPositionAccelerometer *k);
void NewKalmanFilterFusedPositionAccelerometer (float initialPosition,
float initialVelocity, float positionStandardDeviation, 
float accelerometerStandardDeviation, float currentTimestampSeconds,
struct KalmanFilterFusedPositionAccelerometer *k
);
float pinv(float Ab);
#ifdef __cplusplus
  }
#endif
#endif  