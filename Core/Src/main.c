/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  *  lalalalalalallalalallalalalalalallalalalalalal
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mpu6050.h"
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU6050_t mpu6050;

#define PI 3.14159265
const double RAD_TO_DEG = 180/PI;
const double DEG_TO_RAD = PI/180;

double sx, sy, sz;

double kp = 1.9, ki = 1.5, pv = 0, kd = 1.6;

double dis1, dis3, dis2, dis4;

float gx, gy, gz;
float AccX, AccY, AccZ;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
// version request to Pixy
uint8_t versionRequest[] =
{
  0xae,  // first byte of no_checksum_sync (little endian -> least-significant byte first)
  0xc1,  // second byte of no_checksum_sync
  0x0e,  // this is the version request type
  0x00,   // data_length is 0
};
uint8_t getBlocks[] = {
	174,
	193,
	32,
	2,
	7,
	1
};
uint8_t buffer_rx[32];

int ballx;
int bally;
int ballheight;
int ballwidth;

int isBallInView = 0, isInBounds = 1, isPixyChecked = 0, isMPUCollibrated = 0;

#define MAXSPEED 40
#define ZONEDIS_TH 50
#define PIXY_Y_ZERO 114
#define PIXY_X_ZERO 157
#define MAXPIDSPEED 20

uint32_t timcounter = 0;

enum Zones {FAR, CLOSE, BALLIN};

enum Zones zone = FAR;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef struct Motor {
	uint16_t timer;
	uint16_t channel;
	uint16_t in1;
	GPIO_TypeDef *in1_port;
	uint16_t speed;
	int en;
} Motor;

typedef struct SR {
	uint16_t trig_pin;
	GPIO_TypeDef *trig_port;
	uint16_t echo_pin;
	GPIO_TypeDef *echo_port;
} SR;

typedef struct Gyro {
	float x;
	float y;
	float z;
} Gyro;

double deg, yx;

// declaration of Motors
Motor Motor_1 = {1, 2, GPIO_PIN_8, GPIOA, 0, 0};

Motor Motor_2 = {1, 3, GPIO_PIN_4, GPIOA, 0, 0};

Motor Motor_3 = {2, 4, GPIO_PIN_12, GPIOA, 0, 0};

Motor Motor_4 = {1, 4, GPIO_PIN_6, GPIOA, 0, 0};

// declaration of sr04 sensors
SR Sr1 = {GPIO_PIN_14, GPIOB, GPIO_PIN_15, GPIOB};

SR Sr2 = {GPIO_PIN_1, GPIOA, GPIO_PIN_2, GPIOA};

SR Sr3 = {GPIO_PIN_13, GPIOB, GPIO_PIN_12, GPIOB};

SR Sr4 = {GPIO_PIN_0, GPIOA, GPIO_PIN_15, GPIOC};

Gyro Gy = {0, 0, 0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



long int map(long int x, long int in_min, long int in_max, long int out_min, long int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double lerp(double v0, double v1, double t)	 {
	return v0 + t * (v1 - v0);
}

void PWM (Motor *motor, uint32_t speed, int enable) {
	switch(motor->timer) {
		case(1):
			switch(motor->channel){
				case(2):
						TIM1->CCR2 = speed;
						break;
				case(3):
						TIM1->CCR3 = speed;
						break;
				case(4):
						TIM1->CCR4 = speed;
						break;
			}
			break;
		case(2):
			switch(motor->channel) {
				case(4):
					TIM2->CCR4 = speed;
					break;
			}
			break;
	}
	HAL_GPIO_WritePin(GPIOA, motor->in1, enable);
}

void CollibrateMpu6050(int samples) {
	for (RateCalibrationNumber=0; RateCalibrationNumber < samples; RateCalibrationNumber++) {
	    MPU6050_Read_All(&hi2c2, &mpu6050, 0, 0, 0);
	    RateCalibrationRoll+=mpu6050.Gx;
	    RateCalibrationPitch+=mpu6050.Gy;
	    RateCalibrationYaw+=mpu6050.Gz;
	    HAL_Delay(1);
	}
	RateCalibrationRoll/=samples;
	RateCalibrationPitch/=samples;
	RateCalibrationYaw/=samples;

	isMPUCollibrated = 1;
}

void Kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurment) {
	KalmanState = KalmanState + 0.004 * KalmanInput;
	KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
	float KalmanGain = KalmanUncertainty * 1/ (1*KalmanUncertainty + 3 * 3);
	KalmanState = KalmanState + KalmanGain*(KalmanMeasurment - KalmanState);
	KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

	Kalman1DOutput[0] = KalmanState;
	Kalman1DOutput[1] = KalmanUncertainty;
}

void ReadMPU6050() {
	  MPU6050_Read_All(&hi2c2, &mpu6050, RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw);

	  AccX = (float)mpu6050.Accel_X_RAW/16384;
	  AccY = (float)mpu6050.Accel_Y_RAW/16384 + 0.01;
	  AccZ = (float)mpu6050.Accel_Z_RAW/16384;

	  gx = (float)mpu6050.Gyro_X_RAW/131 - RateCalibrationRoll;
	  gy = (float)mpu6050.Gyro_Y_RAW/131 - RateCalibrationPitch;
	  gz = (float)mpu6050.Gyro_Z_RAW/131 - RateCalibrationYaw;

	  sx += gx;
	  sy += gy;
	  sz += gz;

	  Gy.x = sx / 2000;
	  Gy.y = sy / 2000;
	  Gy.z = sz / 2000;
}

void Rotate_to_zero() {
	double e = 0;
	uint32_t u = 0;
	e = 0 - Gy.z;
	int en = e > 0 ? 0 : 1;
	e = abs(e);
	u = abs((int)(kp * e + ki * (e * .1)));
	if (u > MAXPIDSPEED) {
		u = MAXPIDSPEED;
	}

	if (en == 1) {
	}

	if (en == 0) {
	}


	pv = Gy.z;
}

void delay_us(int us) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while (__HAL_TIM_GET_COUNTER(&htim3) < us);
}

double Read_sr(SR *sr) {
	HAL_GPIO_WritePin(sr->trig_port, sr->trig_pin, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(sr->trig_port, sr->trig_pin, GPIO_PIN_SET);
	delay_us(10);
    HAL_GPIO_WritePin(sr->trig_port, sr->trig_pin, GPIO_PIN_RESET);

    int t = 0;
    while(HAL_GPIO_ReadPin(sr->echo_port, sr->echo_pin) != GPIO_PIN_SET);
    while(HAL_GPIO_ReadPin(sr->echo_port, sr->echo_pin) == GPIO_PIN_SET) {
   	   t += 1;
   	   delay_us(1);

   	   if(t > 3000) break;
    }

    return (double)(t*340)/2000;
}


int offsets[] = {-45, -135, 135, 45};

void GotoPoint(double r, double teta, int speed) {
	for (int i = 0; i < 4; ++i) {
		double t = teta + offsets[i];
		double s = sin(t * DEG_TO_RAD) * speed;
		if (s > MAXSPEED) s = MAXSPEED;
		int en = s > 0 ? 1 : 0;
		s = abs(s);
		if (en == 0) {
			switch(i) {
			case 0:
				PWM(&Motor_1, s, en);
				break;
			case 1:
				PWM(&Motor_2, s, en);
				break;
			case 2:
				PWM(&Motor_3, 100 - s, !en);
				break;
			case 3:
				PWM(&Motor_4, s, en);
				break;
			}
		}
		else if (en == 1) {
			switch(i) {
				case 0:
					PWM(&Motor_1, 100 - s, en);
					break;
					case 1:
					PWM(&Motor_2, 100 - s, en);
					break;
				case 2:
					PWM(&Motor_3, s, !en);
					break;
				case 3:
					PWM(&Motor_4, 100 - s, en);
					break;
			}
		}
	}

}

void AllMotorsZero() {
	PWM(&Motor_1, 0, 1);
	PWM(&Motor_2, 0, 1);
	PWM(&Motor_3, 0, 0);
	PWM(&Motor_4, 0, 1);
}

void getBallPosiotion() {
	HAL_SPI_Transmit(&hspi1, &getBlocks, 6, 1000);
	HAL_Delay(1);
	HAL_SPI_Receive(&hspi1, &buffer_rx, 8, 1000); //garbage values
	HAL_SPI_Receive(&hspi1, &buffer_rx, 4, 1000);

	if (buffer_rx[3] != 14) {
		isBallInView = 0;
		return;
	}

	HAL_SPI_Receive(&hspi1, buffer_rx, 14, 1000);

	// x_crop : 83 - 220
	// y_crop : 51 - 186

	ballx = buffer_rx[4] + buffer_rx[5] * 255;
	bally = buffer_rx[6] + buffer_rx[7] * 255;
	ballwidth = buffer_rx[8] + buffer_rx[9] * 255;
	ballheight = buffer_rx[10] + buffer_rx[11] * 255;

	if (!(ballx > 68 && ballx < 229)) {
		isBallInView = 0;
		return;
	}
	if (!(bally > 38 && bally < 197)) {
		isBallInView = 0;
		return;
	}

	isBallInView = 1;

	if (ballx >= PIXY_X_ZERO) {
		ballx -= PIXY_X_ZERO;
	}else {
		ballx = -1 * (PIXY_X_ZERO - ballx);
	}

	if (bally >= PIXY_Y_ZERO) {
		bally = -1 * (PIXY_Y_ZERO - bally);
	}else {
		bally -= PIXY_Y_ZERO;
	}

	ballx *= -1;
	bally *= -1;

	for (int i = 0; i < 26; i++) {
		buffer_rx[i] = 0;
	}
}

double r, teta;

void GetBall(int speed) {
//	double p0x = 0, p0y = 0;
//	double p2x = ballx - 6, p2y = bally - 18;
//	double p1x = p2x / 2 + 20, p1y = p2y / 2 - 10;
//	double px1, py1, px2, py2, x, y, lastpx = 0, lastpy = 0, lastbx, lastby, dy, dx;

	r = abs(sqrt(ballx * ballx + bally * bally));

	if (ballx >= 0) teta = -(atan((double)bally / ballx) * RAD_TO_DEG - 90);
	else if (ballx < 0) teta = -((atan((double)bally/ ballx) + PI)* RAD_TO_DEG - 90);

	if (r >= ZONEDIS_TH) zone = FAR;
	else if (r < ZONEDIS_TH) {
		if (abs(teta) > 15) {
			zone = CLOSE;
		}
		else{
			zone = BALLIN;
		}
	}
	else HAL_Delay(5000);

	switch (zone) {
	case CLOSE:
		if (teta > 0) {
			GotoPoi9nt(r, teta + 35, speed);
		}
		else if(teta < 0) {
			GotoPoint(r, teta - 35, speed);
		}
		break;
	case FAR:
		GotoPoint(r, teta, speed);
		break;
	case BALLIN:
		GotoPoint(10, 0, speed);
		break;
	}

//
//	GotoPoint(10, teta, speed);
//
//	while(ballx >= 6 && bally >= 18);
/*
	for (double t = getBallCurveQ; t < 1.001; t += getBallCurveQ) {
		px1 = lerp(p0x, p1x, t);
		py1 = lerp(p0y, p1y, t);
		px2 = lerp(p1x, p2x, t);
		py2 = lerp(p1y, p2y, t);
		x = lerp(px1, px2, t);
		y = lerp(py1, py2, t);

		dy = y - lastpy;
		dx = x - lastpx;

		if (x >= 0) teta = -(atan(dy / dx) * RAD_TO_DEG - 90);
		else if (x < 0) teta = -((atan(dy / dx) + PI)* RAD_TO_DEG - 90);

		r = sqrt(dx * dx + dy * dy);

		GotoPoint(r, teta, speed);

		m1++;

		while (abs(abs(lastbx - ballx) - abs(dx)) >= 5 && abs(abs(lastby - bally) - abs(dy)) >= 5) {
			if (timcounter % 200 == 0) {
				getBallPosiotion();
			}

			m2++;
		}

		lastbx = ballx;
		lastby = bally;
		lastpx = x;
		lastpy = y;
	}

	AllMotorsZero();
	HAL_Delay(5000);
	*/
}

void BackToGoal() {
	checkBounds();
	if(dis3 >= 20) {
//		while (abs(dis1 - dis2) > 10 && dis3 >= 20) {
//			if (dis1 > dis2) {
//				GotoPoint(10, -135, 20);
//				checkBounds();
//			}
//			else if (dis2 > dis1) {
//				GotoPoint(10, 135, 20);
//				checkBounds();
//			}
//			HAL_Delay(200);
//			getBallPosiotion();
//			if(isBallInView) GetBall(20);
//		}
		while(dis3 >= 20) {
			GotoPoint(10, 180, 20);
			HAL_Delay(200);
			getBallPosiotion();
			checkBounds();
			ReadMPU6050();
			if (floor(Gy.z) > 3) {
				int e = abs(2 - Gy.z);
				int u = kgp * e + kgi * e * .2;
				u = map(u, 0, 50, 0, 20);
				TIM1->CCR2 += u;
				TIM1->CCR3 += u;
			} else if (ceil(Gy.z) < -3) {
				int e = abs(-2	 - Gy.z);
				int u = kgp * e + kgi * e * .2;
				u = map(u, 0, 50, 0, 20);
				TIM1->CCR4 += u;
				TIM2->CCR4 += u;
			}
			//if(isBallInView) GetBall(20);
		}
		HAL_Delay(1000);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (timcounter % 2 == 0 && isMPUCollibrated) {
		ReadMPU6050();
	}

	if (timcounter % 200 == 0 && isPixyChecked) {
		getBallPosiotion();
	}

	timcounter++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_3) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(500);
    }
}

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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  while (MPU6050_Init(&hi2c2) == 1);

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  TIM2->CCR4 = 0;

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  __HAL_TIM_SET_COUNTER(&htim4, 0);

  CollibrateMpu6050(500);

  HAL_Delay(500);

  while(HAL_SPI_Receive(&hspi1, buffer_rx, 1, 100));

  HAL_SPI_Transmit(&hspi1, &versionRequest, 4, 1000);
  HAL_Delay(1);
  HAL_SPI_Receive(&hspi1, &buffer_rx, 8, 1000);  //first 9 garbage data
  HAL_SPI_Receive(&hspi1, &buffer_rx, 22, 1000);

  isPixyChecked = 1;

  HAL_Delay(1000);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if (floor(Gy.z) >= 4 || ceil(Gy.z) <= -4) {
//		  Rotate_to_zero();
//	  }

	  if(isBallInView) {
		  GetBall(35);
	  }
//	  else {
//	  	  AllMotorsZero();
//	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
