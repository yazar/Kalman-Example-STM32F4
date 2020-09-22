/*

by Yazar Yazici

	Kullanim:

 ---- Tanimlamalar ----
	#include "gpsParser.h"

float lat, lon, alt;
double lat10,lon10;
uint8_t sec,min,hour;

 ---- Main Tanim ve Fonksiyonlar ----

	initGPS(&huart2);
	
	
	---- While Içerisi ----
	
		GPS_Process(&huart1, &hour, &min, &sec, &lat10, &lon10, &alt, &fix, &angle, &speed);  // okuma

 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "gpsParser.h"

		char watch[200];
	 uint8_t			UTC_Hour;
	 uint8_t			UTC_Min;
	 uint8_t			UTC_Sec;
	 uint16_t		UTC_MicroSec;
	 
	 uint8_t			UTC_Hour_d;
	 uint8_t			UTC_Min_d;
	 uint8_t			UTC_Sec_d;
	 uint16_t		UTC_MicroSec_d;
	 
	 
	 uint8_t total_number_1;
	 uint8_t message_number;
	 uint8_t total_number_2;
	 uint8_t sv;
	 uint8_t elevation_degrees;
	 float trueAngle;
	 uint8_t SNR;		
	 char status;

	 float speed_d;
	
	 float			Latitude;
	 double			LatitudeDecimal;
	 char				NS_Indicator;
	 float			Longitude;
	 double			LongitudeDecimal;
	 char				EW_Indicator;
	
	 uint8_t			PositionFixIndicator;
	 uint8_t			SatellitesUsed;
	 float				HDOP;
	 float				MSL_Altitude;
	 char				MSL_Units;
	 float				Geoid_Separation;
	 char				Geoid_Units;
	
	 uint16_t		AgeofDiffCorr;
	 char				DiffRefStationID[4];
	 char				CheckSum[2];
	 uint8_t		rxBuffer[512];
	 uint16_t	rxIndex;
	 uint8_t		rxTmp;	
	 uint32_t	LastTime;
	 
void initGPS(UART_HandleTypeDef *uartHandler)
{
		HAL_UART_Transmit(uartHandler,(uint8_t*)"$PGCMD,33,1*6C",16,100);
		HAL_UART_Transmit(uartHandler,(uint8_t*)"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28",49,100); 
		HAL_UART_Transmit(uartHandler,(uint8_t*)"$PMTK220,100*2F",16,100);
		HAL_Delay(1000);
}

	double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}

void	GPS_Process(UART_HandleTypeDef *uartHandle,uint8_t *hour1, uint8_t *min1, uint8_t *sec1, double *lat110, double *lon110, float *alt1, uint8_t *fix1, float *northAngle, float *speed1)
{
    HAL_UART_Receive(uartHandle,rxBuffer,200,100);
		char	*str;
	  char  *str_speed; 
		char	*str_head;
	
		str_speed=strstr((char*)rxBuffer,"$GPRMC,");
		str=strstr((char*)rxBuffer,"$GPGGA,");
		
	
	
	
	if(strlen(str_speed) >2)
		{
			sscanf(str_speed,"$GPRMC,%2hhd%2hhd%2hhd.%3hd,%c,%f,%c,%f,%c,%f,%f",&UTC_Hour,&UTC_Min,&UTC_Sec,&UTC_MicroSec,&status,&Latitude,&NS_Indicator,&Longitude,&EW_Indicator,&speed_d,&trueAngle);
								//&UTC_Hour,&UTC_Min,&UTC_Sec,&UTC_MicroSec,&Latitude,&NS_Indicator,&Longitude,&EW_Indicator,&PositionFixIndicator,&SatellitesUsed,&HDOP,&MSL_Altitude,&MSL_Units,&AgeofDiffCorr,DiffRefStationID,CheckSum
			if(NS_Indicator==0)
				NS_Indicator='-';
			if(EW_Indicator==0)
				EW_Indicator='-';
			if(Geoid_Units==0)
				Geoid_Units='-';
			if(MSL_Units==0)
				MSL_Units='-';

			*speed1 = speed_d;			
			*northAngle=trueAngle;

		}
	
		
		if(strlen(str) >2)
		{
			sscanf(str,"$GPGGA,%2hhd%2hhd%2hhd.%3hd,%f,%c,%f,%c,%hhd,%hhd,%f,%f,%c,%hd,%s,*%2s\r\n",&UTC_Hour,&UTC_Min,&UTC_Sec,&UTC_MicroSec,&Latitude,&NS_Indicator,&Longitude,&EW_Indicator,&PositionFixIndicator,&SatellitesUsed,&HDOP,&MSL_Altitude,&MSL_Units,&AgeofDiffCorr,DiffRefStationID,CheckSum);
								//&UTC_Hour,&UTC_Min,&UTC_Sec,&UTC_MicroSec,&Latitude,&NS_Indicator,&Longitude,&EW_Indicator,&PositionFixIndicator,&SatellitesUsed,&HDOP,&MSL_Altitude,&MSL_Units,&AgeofDiffCorr,DiffRefStationID,CheckSum
			if(NS_Indicator==0)
				NS_Indicator='-';
			if(EW_Indicator==0)
				EW_Indicator='-';
			if(Geoid_Units==0)
				Geoid_Units='-';
			if(MSL_Units==0)
				MSL_Units='-';
		if(Latitude > 100 && Longitude > 100){
		LatitudeDecimal=convertDegMinToDecDeg(Latitude);
		LongitudeDecimal=convertDegMinToDecDeg(Longitude);	
		*hour1 = UTC_Hour;
		*min1 = UTC_Min;
		*sec1 = UTC_Sec;
		*fix1 = PositionFixIndicator;
		*lat110 = LatitudeDecimal;
		*lon110 = LongitudeDecimal;
		*alt1 = MSL_Altitude;		
					
		}
		}
		
		memset(rxBuffer,0,sizeof(rxBuffer));
}

float knot2MS(float knotSpeed)
{
   return knotSpeed * 0.514444;  
}
