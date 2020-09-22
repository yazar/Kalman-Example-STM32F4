#include "stm32f4xx_hal.h"

	
void initGPS(UART_HandleTypeDef *uartHandler);
void	GPS_Process(UART_HandleTypeDef *uartHandle,uint8_t *hour1, uint8_t *min1, uint8_t *sec1, double *lat110, double *lon110, float *alt1, uint8_t *fix1, float *northAngle, float *speed1);
double convertDegMinToDecDeg (float degMin);
float knot2MS(float knotSpeed);