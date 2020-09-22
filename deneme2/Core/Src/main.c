/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "matrix.h"

#include "bno055.h"
#include "gpsParser.h"
#include "absOrientation.h"
#include "kalmanYazar.h"
#include "matrix.h"
#include "bmp280.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int test = 0;
int zaman = 0, zaman2 = 0;


BMP280_HandleTypedef bmp280;
float pressure, temperature, humidity, altitudeBMP;

const float EARTH_RADIUS = 6371 * 1000.0; // meters
const float ACTUAL_GRAVITY = 9.80665;
double predictedLon = 0;
double predictedLat = 0;
double predictedVE = 0;
double predictedVN = 0;
float yazarPredictedAngle =0;
float speed, angle;

bno055_vector_t v;
bno055_vector_t v2;
float ax =0, ay=0, az=0;
float accelerationEast;
float accelerationNorth;
float accelerationUp;
float q[4];
float magneticDeclination = 5.36;
float roll = 0, pitch = 0, yaw = 0;

float lat, lon, alt;
double lat10 = 0,lon10 = 0;
uint8_t sec,min,hour, fix;
double oldLat =1, oldLon =1;

float  vEast =0;
float vNorth = 0;
float vUp = 0;
double predictedLonMeters = 0;
double predictedLatMeters = 0;
double predictedAlt = 0;
float resultantV = 0;
float timeStamp;

float gpsLatitude_raw;
float gpsLongitude_raw;
float gpsAltitude_raw;
float gpsSpeed_raw;


float latlonStandartDeviation = 2.0; //+ - 1m sapma için
float altitudeStandardDeviation = 3.518522417151836;
  
float accelerometerEastStandardDeviation = ACTUAL_GRAVITY * 0.033436506994600976 ; //0.033436506994600976 idi
float accelerometerNorthStandardDeviation = ACTUAL_GRAVITY * 0.05355371135598354;
float accelerometerUpStandardDeviation = ACTUAL_GRAVITY * 0.2088683796078286;

KalmanFilterFusedPositionAccelerometer longitudeEastKalmanFilter;
KalmanFilterFusedPositionAccelerometer latitudeNorthKalmanFilter;
KalmanFilterFusedPositionAccelerometer altitudeUpKalmanFilter;

int bir = 1;
float h_obs = 0;
float h_hat = 0;
float sigma_baro = 1.0;
float sigma_gps = 10.0;
int k_gps = 0;
float Hb, Qb, Rb, RTemp;


float angleBno = 0;
float h_obsAngle = 0;
float h_hatAngle = 0;
float sigma_baroAngle = 1.0;
float sigma_gpsAngle = 10.0;
int k_gpsAngle = 0;
float HbAngle, QbAngle, RbAngle, RTempAngle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	Rb = 1 *sigma_baro;
	Hb = 1;
	Qb = 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationMode(BNO055_OPERATION_MODE_COMPASS);
		
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_1;
	bmp280.i2c = &hi2c1;
	
	while (!bmp280_init(&bmp280, &bmp280.params)) {}
		
		initGPS(&huart1);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
																			// 33 ms accelerations		112ms gps
		zaman2 = HAL_GetTick();
		v = bno055_getVectorLinearAccel();
		ax = v.x; ay =v.y; az = v.z;
		HAL_Delay(5);
		v =	bno055_getVectorQuaternion();
		q[0] = v.w; q[1] = v.x; q[2] =v.y; q[3] = v.z;
		//updateFromSensor(q[0],q[1],q[2],q[3],ax,ay,az, magneticDeclination, &accelerationEast, &accelerationNorth, &accelerationUp);
		absFarkli(q[0],q[1],q[2],q[3],ax,ay,az, &accelerationEast, &accelerationNorth, &accelerationUp);
		HAL_Delay(5);
		v =	bno055_getVectorEuler();
		angleBno = v.x;
		HAL_Delay(5);
		
		bmp280_read_float(&bmp280, &temperature, &pressure, &humidity, &altitudeBMP);  // okuma
		
		GPS_Process(&huart1, &hour, &min, &sec, &lat10, &lon10, &alt, &fix, &angle, &speed);
		zaman = HAL_GetTick() - zaman2;
		if(fix && lat10 != oldLat && lon10 != oldLon) //GPS isindi ise ve veri geliyor ise
  {
		oldLat = lat10;
    oldLon = lon10;
		gpsLatitude_raw = lat10;
    gpsLongitude_raw = lon10;
    gpsAltitude_raw = alt;
    gpsSpeed_raw = knot2MS(speed);				
    vEast = gpsSpeed_raw * sin(h_hatAngle);	
    vNorth = gpsSpeed_raw * cos(h_hatAngle); 
    vUp = 0;
	}else
  {
    gpsLatitude_raw = 0.0;
    gpsLongitude_raw = 0.0;
    gpsAltitude_raw = 0.0;
    gpsSpeed_raw = 0.0;
    vEast =0;
    vNorth = 0;
    vUp = 0;
  }
	long timeStamp =  HAL_GetTick() / 1000;

	if(!bir) {
	Predict(accelerationNorth ,timeStamp,&latitudeNorthKalmanFilter); 
  Predict(accelerationEast ,timeStamp,&longitudeEastKalmanFilter);		
  Predict(accelerationUp ,timeStamp,&altitudeUpKalmanFilter);
	}
	if(gpsLatitude_raw != 0.0)
{
	if(bir){
			h_hat = 0.5 * (gpsAltitude_raw + altitudeBMP);  
      Qb = 1;
			h_hatAngle = angleBno;  
      QbAngle = 1;
      NewKalmanFilterFusedPositionAccelerometer(gpsLatitude_raw,vNorth,latlonStandartDeviation, 
      accelerometerNorthStandardDeviation,timeStamp,&latitudeNorthKalmanFilter); //Latitude
			
      NewKalmanFilterFusedPositionAccelerometer(gpsLongitude_raw,vEast,latlonStandartDeviation, 
      accelerometerEastStandardDeviation,timeStamp,&longitudeEastKalmanFilter); //Longitude
			
      NewKalmanFilterFusedPositionAccelerometer(gpsAltitude_raw,vUp,altitudeStandardDeviation, 
      accelerometerUpStandardDeviation,timeStamp,&altitudeUpKalmanFilter); //Altitude
			bir = 0;
    }
	float defaultPositionErr = 0; //0 idi
  float velErr = 0.005;
		
	float latitudeAsMeters = LatitudeToMeters(gpsLatitude_raw);
  Update(latitudeAsMeters,vNorth,&defaultPositionErr,velErr,&latitudeNorthKalmanFilter);

  float longitudeAsMeters = LongitudeToMeters(gpsLongitude_raw);
  Update(longitudeAsMeters,vEast,&defaultPositionErr,velErr,&longitudeEastKalmanFilter);

  Update(gpsAltitude_raw,vUp,&defaultPositionErr,velErr,&altitudeUpKalmanFilter);
		
	h_obs = gpsAltitude_raw;
  Rb = 1 * sigma_gps;
  k_gps = k_gps + 1;
		
	h_obsAngle = angle;
  RbAngle = 1 * sigma_gpsAngle;
  k_gpsAngle = k_gpsAngle + 1;
	
}else {
	h_obs = altitudeBMP;
  Rb = 1 * sigma_baro;
	
	h_obsAngle = angleBno;
  RbAngle = 1 * sigma_baroAngle;
}
if(!bir){
		float tmp;
		tmp = Hb*h_hat;
    Qb = pinv(pinv(Qb) + ((Hb * pinv(Rb)) * Hb));
		float tmp2;
		tmp2 = (((Qb * Hb) * pinv(Rb)) * (h_obs - tmp));
		h_hat = (h_hat) + tmp2;															// Altitude Predicted
	
		float tmpAngle;
		tmpAngle = HbAngle*h_hatAngle;
    Qb = pinv(pinv(QbAngle) + ((HbAngle * pinv(RbAngle)) * HbAngle));
		float tmp2Angle;
		tmp2Angle = (((QbAngle * HbAngle) * pinv(RbAngle)) * (h_obsAngle - tmpAngle));
		h_hatAngle = (h_hatAngle) + tmp2Angle;																						// Angle Predicted
	
		predictedLatMeters = GetPredictedPosition(&latitudeNorthKalmanFilter);
    predictedLonMeters = GetPredictedPosition(&longitudeEastKalmanFilter);
    predictedAlt = GetPredictedPosition(&altitudeUpKalmanFilter);
    GeoPoint point =  MetersToGeopoint(predictedLatMeters,predictedLonMeters);

    predictedLon = point.Longitude;
    predictedLat = point.Latitude;
    predictedVE = GetPredictedVelocityThisAxis(&longitudeEastKalmanFilter);
    predictedVN = GetPredictedVelocityThisAxis(&latitudeNorthKalmanFilter);
    resultantV = sqrt(pow(predictedVE, 2) + pow(predictedVN, 2));
    
    yazarPredictedAngle = atan(predictedVN/predictedVE);
	test++;
	
}
		

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
