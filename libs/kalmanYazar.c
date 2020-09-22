#include "kalmanYazar.h"



float RadiansToDegrees(float radian) {
  return (radian * 180.0 / M_PI);
}
float DegreesToRadians(float degree) {
  return (degree * M_PI / 180.0);
}

float geoAngle(float latOrLon) {
  return DegreesToRadians(latOrLon);
}

GeoPoint GetPointAhead( GeoPoint fromCoordinate, float distanceMeters, float azimuth)  {
	const float EARTH_RADIUS = 6371 * 1000.0; // meters
	
  float radiusFraction = distanceMeters / EARTH_RADIUS;

  float bearing = DegreesToRadians(azimuth);

  float lat1 = geoAngle(fromCoordinate.Latitude);
  float lng1 = geoAngle(fromCoordinate.Longitude);

  float lat2_part1 = sin(lat1) * cos(radiusFraction);
  float lat2_part2 = cos(lat1) * sin(radiusFraction) * cos(bearing);

  float lat2 = asin(lat2_part1 + lat2_part2);

  float lng2_part1 = sin(bearing) * sin(radiusFraction) * cos(lat1);
  float lng2_part2 = cos(radiusFraction) - (sin(lat1) * sin(lat2));

  float lng2 = lng1 + atan2(lng2_part1, lng2_part2);
  lng2 = fmod((lng2+3*M_PI),(2*M_PI)) - M_PI;
  GeoPoint g;
  g.Latitude = RadiansToDegrees(lat2);
  g.Longitude = RadiansToDegrees(lng2);
  return g;
}

GeoPoint PointPlusDistanceEast(GeoPoint fromCoordinate, float distance) {
  return GetPointAhead(fromCoordinate, distance, 90.0);
}
GeoPoint PointPlusDistanceNorth(GeoPoint fromCoordinate, float distance) {
  return GetPointAhead(fromCoordinate, distance, 0.0);
}

GeoPoint MetersToGeopoint(float latAsMeters, float lonAsMeters)  {
  GeoPoint point;
	point.Latitude = 0;
  point.Longitude = 0;
  GeoPoint pointEast = PointPlusDistanceEast(point, lonAsMeters);
  GeoPoint pointNorthEast = PointPlusDistanceNorth(pointEast, latAsMeters);
  return pointNorthEast;
}

float  GetDistanceMeters(GeoPoint fromCoordinate , GeoPoint toCoordinate ) {
	const float EARTH_RADIUS = 6371 * 1000.0; // meters
  float deltaLon = geoAngle(toCoordinate.Longitude - fromCoordinate.Longitude);
  float deltaLat = geoAngle(toCoordinate.Latitude - fromCoordinate.Latitude);

  float a = pow(sin(deltaLat/2.0), 2) +(
    cos(geoAngle(fromCoordinate.Latitude))*
      cos(geoAngle(toCoordinate.Latitude))*
      pow(sin(deltaLon/2.0), 2));
  float c = 2 * atan2(sqrt(a), sqrt(1.0-a));
  return EARTH_RADIUS * c;
}


float LatitudeToMeters(float latitude)  {
  GeoPoint g,g2;
  g.Latitude = latitude;
  g.Longitude = 0.0;
  g2.Latitude = 0.0;
  g2.Longitude = 0.0;
  float distance = GetDistanceMeters(g,g2);
  if (latitude < 0) {
    distance *= -1;
  }
  return distance;
}
float LongitudeToMeters(float longitude )  {
  GeoPoint g,g2;
  g.Latitude = 0.0;
  g.Longitude = longitude;
  g2.Latitude = 0.0;
  g2.Longitude = 0.0;
  float distance = GetDistanceMeters(g,g2);
  if (longitude  < 0) {
    distance *= -1;
  }
  return distance;
}

void recreateControlMatrix(float deltaSeconds, struct KalmanFilterFusedPositionAccelerometer *k) {
	float dtSquared = 0.5 * deltaSeconds * deltaSeconds;

	k->B[0][0] = dtSquared;
	k->B[1][0] = deltaSeconds;
  }
void recreateStateTransitionMatrix(float deltaSeconds, struct KalmanFilterFusedPositionAccelerometer *k) {
  k->A[0][0] = 1.0;
  k->A[0][1] = deltaSeconds;

  k->A[1][0] = 0.0;
  k->A[1][1] = 1.0;
}

void  Predict(float accelerationThisAxis, float timestampNow, struct KalmanFilterFusedPositionAccelerometer *k) {
	
  float deltaT = timestampNow - k->currentStateTimestampSeconds;
	//*test = *test+1;	// 4
  recreateControlMatrix(deltaT,k);
	//*test = *test+1; // 5
  recreateStateTransitionMatrix(deltaT,k);
	//*test = *test+1; //6
  k->u[0][0] = accelerationThisAxis;
	//*test = *test+1; //7
	
	float AcurrentState[2][1];
	float Bu[2][1];
	float trans[2][2];
	float tempPMat[2][2];
	float tempPMat2[2][2];
	float tempPMat3[2][2];
	float buCurrentMat[2][1];

	multiplyMatrices21(k->A, k->currentState, AcurrentState);
	multiplyMatrices211(k->B, k->u, Bu);
	addMatrices21(AcurrentState,Bu,buCurrentMat);
	
	k->currentState[0][0] = buCurrentMat[0][0];
	k->currentState[1][0] = buCurrentMat[1][0];
	
	multiplyMatrices22(k->A,k->P,tempPMat);
	transpose22(k->A,trans);
	multiplyMatrices22(tempPMat, trans, tempPMat3);
  addMatrices22(tempPMat3, k->Q, tempPMat2);
	k->P[0][0] = tempPMat2[0][0];
	k->P[0][1] = tempPMat2[0][1];
	k->P[1][0] = tempPMat2[1][0];
	k->P[1][1] = tempPMat2[1][1];
	
  k->currentStateTimestampSeconds = timestampNow;

}


void Update(float position, float velocityThisAxis, float *positionError, float velocityError, struct KalmanFilterFusedPositionAccelerometer *k)
{
  k->z[0][0] = position;
  k->z[1][0] = velocityThisAxis;
  
	if (positionError != NULL) {
    k->R[0][0] = *positionError * *positionError;
	} 
  k->R[1][1] = velocityError * velocityError;
  
	float y[2][1];
	float s[2][2];
	float sInverse[2][2];
	float K[2][2];
	float Ky[2][1];
	float IK[2][2];
	float cStateMat[2][1];
	float pNowMat[2][2];

	subMatrices21(k->z, k->currentState, y);
	addMatrices22(k->P, k->R, s);
	inverse22(s, sInverse,2);
	multiplyMatrices22(k->P, sInverse, K);
	multiplyMatrices21(K, y, Ky);
	addMatrices21(k->currentState, Ky, cStateMat);
	
	k->currentState[0][0] = cStateMat[0][0];
	k->currentState[1][0] = cStateMat[1][0];
	
	multiplyMatrices22(k->I, K, IK);
	
  multiplyMatrices22(IK, k->P, pNowMat);
	k->P[0][0] = pNowMat[0][0];
	k->P[0][1] = pNowMat[0][1];
	k->P[1][0] = pNowMat[1][0];
	k->P[1][1] = pNowMat[1][1];

}

float  GetPredictedPosition (struct KalmanFilterFusedPositionAccelerometer *k)
{
  return (k->currentState[0][0]);
}

float  GetPredictedVelocityThisAxis (struct KalmanFilterFusedPositionAccelerometer *k)
{
  return (k->currentState[1][0]);
}


void NewKalmanFilterFusedPositionAccelerometer (float initialPosition,
float initialVelocity, float positionStandardDeviation, 
float accelerometerStandardDeviation, float currentTimestampSeconds,
struct KalmanFilterFusedPositionAccelerometer *k
)
{
float I[2][2] = {1, 0, 0, 1};
float H[2][2] = {1, 0, 0 ,1};
float P[2][2]= {1, 0, 0, 1};


float Q[2][2];
Q[0][0] = accelerometerStandardDeviation*accelerometerStandardDeviation;
Q[1][1] = accelerometerStandardDeviation*accelerometerStandardDeviation;

float R[2][2];
R[0][0] = positionStandardDeviation*positionStandardDeviation;
R[1][1] = positionStandardDeviation*positionStandardDeviation;

float u[1][1];
float z[2][1];
float A[2][2];
float B[2][1];

float currentState[2][1] = {initialPosition, initialVelocity};



k->I[0][0] = I[0][0];
k->I[0][1] = I[0][1];
k->I[1][0] = I[1][0];
k->I[1][1] = I[1][1];

k->A[0][0] = A[0][0];
k->A[0][1] = A[0][1];
k->A[1][0] = A[1][0];
k->A[1][1] = A[1][1];

k->B[0][0] = B[0][0];
k->B[1][0] = B[1][0];

k->z[0][0] = z[0][0];
k->z[1][0] = z[1][0];

k->u[0][0] = u[0][0];

k->H[0][0] = H[0][0];
k->H[0][1] = H[0][1];
k->H[1][0] = H[1][0];
k->H[1][1] = H[1][1];

k->P[0][0] = P[0][0];
k->P[0][1] = P[0][1];
k->P[1][0] = P[1][0];
k->P[1][1] = P[1][1];

k->Q[0][0] = Q[0][0];
k->Q[0][1] = Q[0][1];
k->Q[1][0] = Q[1][0];
k->Q[1][1] = Q[1][1];

k->R[0][0] = R[0][0];
k->R[0][1] = R[0][1];
k->R[1][0] = R[1][0];
k->R[1][1] = R[1][1];

k->currentState[0][0] = currentState[0][0];
k->currentState[1][0] = currentState[1][0];

k->currentStateTimestampSeconds = currentTimestampSeconds;


}

float pinv(float Ab) {
	int res;
	int res2;
	float AinverseTemp = 1 / Ab;
	float AinverseTemp2;
    AinverseTemp =  (1/Ab) * Ab;
    AinverseTemp2 = 1/AinverseTemp;
    return AinverseTemp2 * (1/Ab);
}
