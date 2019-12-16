
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#define SPEED_SOUND 0.017
#define CLEAR_PARAM 45
#define SIDE_CLEAR_PARAM 90
#define TURNANGLE 86
#define ANGLE_TOLER 0
#include "dwt_delay.h"
#define FOR_RIGHT 0xC1
#define FOR_LEFT 0xC9
#define BACK_RIGHT 0xC2
#define BACK_LEFT 0xCA
#define BRAKE_RIGHT 0xC4
#define BRAKE_LEFT 0xCC
#define ACCEL_RIGHT 0xC5
#define ACCEL_LEFT 0xCD
#define DIR_RIGHT 1
#define DIR_LEFT 0
#define EB3D_3N_WALL 40
#define ANGLE_OFFSET 2

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId ReadDistForwardHandle;
osThreadId MainMoveHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t local_time, sensor_time;
uint32_t distance;
volatile int angle;
int sensortimef,sensortimer,sensortimel;
char readyf,readyr,readyl;
char readyAngle=0;
int angle1,angle2,angle3,angle4,targetangle;

unsigned long long seconds=0;
unsigned long long milliseconds=0;
char counterEN=0;
int clear(int value);
int readingsf[4];
int readingsr[4];
int readingsl[4];
int readingf;
int readingr;
int readingl;
int abs(int a) {
	return a < 0? -a : a;
}

int diff_angle(int angle_a, int angle_b) {
	int diff = angle_b - angle_a;
	if (diff > 180) {
		diff -= 180;
		diff = -diff;
	} else if (diff < -180) {
		diff += 180;
		diff = -diff;
	}
	return diff; // 0->180 or -180->0
}

void goForward(int speed)
{
	int speedr = speed;
	int speedl=speed;
	//HAL_Delay(delay);
	int diff = diff_angle(angle, targetangle);
	int adiff = abs(diff);
	if(readingl<30||readingr<30)
	{
		if(readingl<30)
		{
			speedr=0;
		}else
		{
			speedl=0;
		}
	}else{
		if (adiff != 0 && adiff<45) {
			if (diff < 0) {
				speedl -= speed*(1-(abs(45-adiff)/45.0f));
			} else {
				speedr -= speed*(1-(abs(45-adiff)/45.0f));
			}
		}else if(adiff != 0)
		{
			if (diff < 0) {
				speedl =0;
			} else {
				speedr =0;
			}
		}
	}
	uint8_t forward[] = {ACCEL_RIGHT, speedr, ACCEL_LEFT,speedl};
	HAL_UART_Transmit(&huart1, forward ,sizeof(forward), 1000);
}

void stop()
{
	uint8_t stop[] = {BRAKE_RIGHT, 0x00, BRAKE_LEFT, 0x00};
	HAL_UART_Transmit(&huart1, stop ,sizeof(stop), 1000);
}
void right(int speed) {
		uint8_t a[] = { FOR_LEFT, speed, BACK_RIGHT, speed};
		HAL_UART_Transmit(&huart1, a ,sizeof(a), 1000);
		while(diff_angle(angle, targetangle) >= ANGLE_TOLER);
		stop();
//	int startAngle=angle;
//	uint8_t a[] = { 0xC2, 0x20, 0xC9, 0x20};
//	HAL_UART_Transmit(&huart1, a ,sizeof(a), 1000);
//	while(1){
//		if(angle>startAngle&&(angle-startAngle>=TURNANGLE))
//				break;
//		if(angle<startAngle&&((360+angle)-startAngle>=TURNANGLE))
//				break;
//	};
//		stop();
}

void left(int speed)
{
		uint8_t a[] = { FOR_RIGHT, speed, BACK_LEFT, speed};
		HAL_UART_Transmit(&huart1, a ,sizeof(a), 1000);
		while(diff_angle(angle, targetangle) <= -ANGLE_TOLER);
		stop();
//	int startAngle=angle;
//		uint8_t a[] = { 0xC1, 0x20, 0xCA, 0x20};
//		HAL_UART_Transmit(&huart1, a ,sizeof(a), 1000);
//		while(1){
//			if(angle<startAngle&&(startAngle-angle>=TURNANGLE))
//				break;
//			if(angle>startAngle&&((360+startAngle)-angle>=TURNANGLE))
//				break;
//		};
//		stop();
	//HAL_Delay(delay);
}


int getReading(int* readings)
{
	int avg=0;
	int count = 0;
	for(int i = 0;i<4;i++) {
			avg+=readings[i];
			count++;
	}
	if(count==0)
		return -1;
	return avg/count;
}
void addReading(int* readings,int reading)
{
	for(int i = 0;i<3;i++)
		readings[i]=readings[i+1];
	readings[3]=reading;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void ReadDist(void const * argument);
void MainMoveFunc(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ReadDistForward */
  osThreadDef(ReadDistForward, ReadDist, osPriorityNormal, 0, 128);
  ReadDistForwardHandle = osThreadCreate(osThread(ReadDistForward), NULL);

  /* definition and creation of MainMove */
  osThreadDef(MainMove, MainMoveFunc, osPriorityNormal, 0, 128);
  MainMoveHandle = osThreadCreate(osThread(MainMove), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	DWT_Init();
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_Delay(3000);
	stop();
	HAL_Delay(1000);
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIG_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ECHO_FOR_Pin */
  GPIO_InitStruct.Pin = ECHO_FOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_FOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin LED_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO_RIGHT_Pin ECHO_LEFT_Pin */
  GPIO_InitStruct.Pin = ECHO_RIGHT_Pin|ECHO_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
 for(;;){
		osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_ReadDist */
/**
* @brief Function implementing the ReadDistForward thread.
* @param argument: Not used
* @retval None
*/
int read=0;
/* USER CODE END Header_ReadDist */
void ReadDist(void const * argument)
{
  /* USER CODE BEGIN ReadDist */
  /* Infinite loop */
	readyr=0;
	readyf=0;
	readyl=0;
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	DWT_Delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
  for(;;)
  {
		if(readyf)
		{
			distance  = sensortimef*SPEED_SOUND;
			addReading(readingsf,distance);
			read=getReading(readingsf);
			if(read!=-1)
				readingf=read;
		}
		if(readyl)
		{
			distance  = sensortimel*SPEED_SOUND;
			addReading(readingsl,distance);
			read=getReading(readingsl);
			if(read!=-1)
					readingl=read;
		}
		if(readyr)
		{
			distance  = sensortimer*SPEED_SOUND;
			addReading(readingsr,distance);
			read=getReading(readingsr);
			if(read!=-1)
					readingr=read;
		}
		if(readyf&&readyl&&readyr)
		{
			readyr=0;
			readyf=0;
			readyl=0;
			HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
			DWT_Delay(10);  // wait for 10 us
			HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
		}
    osDelay(10);
  }
  /* USER CODE END ReadDist */
}

/* USER CODE BEGIN Header_MainMoveFunc */
/**
* @brief Function implementing the MainMove thread.
* @param argument: Not used
* @retval None
*/
int clear(int value){
	return (value >= CLEAR_PARAM);
}
int clearmax(int value,int max){
	return (value >= max);
}
char dir;
int state = 0;
float time = 0;
//enum state {FWD_ALIGNED = 0, TURN, FWD_UNALIGNED, TURN180,UNTURN};
enum status {INIT, FWD_ALIGNED, TURN, FWD_UNALIGNED, STOP};
int comingtoTURNfrom, comingtoSTOPfrom;
char parentState=1;
#define RAMP1 55
#define RAMP2 56
char childState=0;
int startOpen=0;
/* USER CODE END Header_MainMoveFunc */

void MainMoveFunc(void const * argument)
{
  /* USER CODE BEGIN MainMoveFunc */
  /* Infinite loop */
	osDelay(1000);
	while (readyAngle==0)	{
		osDelay(100);
	}
	parentState=1;
	targetangle = angle1;
  for(;;)
  {
			if(parentState==1)
			{
				switch(state){
					case INIT:
						counterEN=0;
						stop();
						osDelay(10);
						if (clear(readingf)){
							state = FWD_ALIGNED;
						}
						break;

					case FWD_ALIGNED:
						
						targetangle = angle1;
						if (clear(readingf)){
							counterEN=1;
							goForward(0x35);
							if(readingl>1000)
							{
								HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
								state=STOP;
							}else
								HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);							
						}
						else{
							counterEN=0;
							stop();
							osDelay(10);
							if (clear(readingl) && readingl >= readingr){
								targetangle = angle4;
								state = TURN;
								comingtoTURNfrom = FWD_ALIGNED;
								dir = DIR_LEFT;
							}
							else if (clear(readingr) && readingr > readingl){
								targetangle = angle2;
								state = TURN;
								comingtoTURNfrom = FWD_ALIGNED;
								dir = DIR_RIGHT;
							}
						}
						break;

					case TURN:
						counterEN=0;
						if (dir == DIR_RIGHT){
							right(0x20);
						}
						else{
							left(0x20);
						}
						if(comingtoTURNfrom == FWD_ALIGNED){
							state = FWD_UNALIGNED;
						}
						else if(comingtoTURNfrom == FWD_UNALIGNED){
							state = FWD_ALIGNED;
						}
						break;
						
					case FWD_UNALIGNED:
						counterEN=0;
						if (dir == DIR_LEFT){
							if (clear(readingf) && !clearmax(readingr, SIDE_CLEAR_PARAM)){
								goForward(0x35);
							}
							else if (clearmax(readingr, SIDE_CLEAR_PARAM)){
								if (clear(readingf)){
									goForward(0x35);
									osDelay(1000);
								}
								targetangle = angle1;
								comingtoTURNfrom = FWD_UNALIGNED;
								dir = DIR_RIGHT;
								state = TURN;
							}
							else {
								stop();
							}
						}
						else if (dir == DIR_RIGHT){
							if (clear(readingf)&& !clearmax(readingl, SIDE_CLEAR_PARAM)){
								goForward(0x35);
							}
							else if (clearmax(readingl, SIDE_CLEAR_PARAM)){
								if (clear(readingf)){
									goForward(0x35);
									osDelay(1000);
								}
								targetangle = angle1;
								comingtoTURNfrom = FWD_UNALIGNED;
								dir=DIR_LEFT;
								state = TURN;
							}
							else{
								//state = STOP;
								stop();
								//comingtoSTOPfrom = FWD_UNALIGNED;
							}
						}
						break;

					case STOP:
						
						counterEN=0;
						targetangle=angle1;
						if (clear(readingf)){
								goForward(0x35);
								osDelay(1000);
							}
						stop();
						targetangle = angle4;
						left(0x20);
						parentState=2;
				    state=INIT;
						int temp=angle2;
						angle2 = angle1;
						angle1 = angle4;
						angle4 = angle3;
						angle3 = temp;
						break;
				}
			}
			
  		else if(parentState==2)
			{
				switch(state)
				{
					case INIT:
						
						targetangle=angle1;
						state=FWD_ALIGNED;
						seconds=0;
						counterEN=0;
						break;
					case FWD_ALIGNED:
						targetangle = angle1;
						if(childState==RAMP1&&seconds>=10)
						{
							if(angle1>=45)
								angle1-=45;
							else
								angle1 = 360+angle1-45;
							angle1=angle1%360;
							angle2=(angle1+90)%360;
							angle3=(angle2+90)%360;
							angle4=(angle3+90)%360;
							
							childState=RAMP2;
							counterEN=0;
							seconds=0;
						}
						else if (clear(readingf)){
							if(childState==RAMP1)
								counterEN=1;
							goForward(0x35);
							if(childState==RAMP2){
								counterEN=1;
								if(seconds>=5){
									if(readingl>3000)
									{
										HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
										//state=STOP;
									}else
										HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);
								}
							}
						}
						else{
							counterEN=0;
							stop();
							osDelay(10);
							if (clear(readingl) && readingl >= readingr){
								targetangle = angle4;
								state = TURN;
								comingtoTURNfrom = FWD_ALIGNED;
								dir = DIR_LEFT;
							}
							else if (clear(readingr) && readingr > readingl){
								targetangle = angle2;
								state = TURN;
								comingtoTURNfrom = FWD_ALIGNED;
								dir = DIR_RIGHT;
							}
						}
						break;

					case TURN:
						if (dir == DIR_RIGHT){
							right(0x20);
						}
						else{
							left(0x20);
						}
						if(comingtoTURNfrom == FWD_ALIGNED){
							state = FWD_UNALIGNED;
						}
						else if(comingtoTURNfrom == FWD_UNALIGNED){
							state = FWD_ALIGNED;
						}
						break;
						
					case FWD_UNALIGNED:
						counterEN=0;
						if (dir == DIR_LEFT){
							if (clear(readingf) && !clearmax(readingr, SIDE_CLEAR_PARAM)){
								goForward(0x35);
							}
							else if (1){
								if (clear(readingf)){
									goForward(0x35);
									osDelay(1000);
								}
								targetangle = angle1;
								comingtoTURNfrom = FWD_UNALIGNED;
								dir = DIR_RIGHT;
								state = TURN;
							}
							else {
								stop();
							}
						}
						else if (dir == DIR_RIGHT){
							if (clear(readingf)&& !clearmax(readingl, SIDE_CLEAR_PARAM)){
								goForward(0x35);
							}
							else if (clearmax(readingl, SIDE_CLEAR_PARAM)){
								targetangle = angle1;
								comingtoTURNfrom = FWD_UNALIGNED;
								dir=DIR_LEFT;
								state = TURN;
							}
							else if(clearmax(readingr, SIDE_CLEAR_PARAM)){
								int temp1=angle1;
								int temp2=angle2;
								angle1 = angle3;
								angle2 = angle4;
								angle3 = temp1;
								angle4 = temp2;
								targetangle=angle1;
								dir = DIR_RIGHT;
								state=TURN;
								childState=RAMP1;
								comingtoTURNfrom = FWD_UNALIGNED;
							}else
							{
								targetangle = angle1;
								comingtoTURNfrom = FWD_UNALIGNED;
								dir=DIR_LEFT;
								state = TURN;
							}
//								else if (clearmax(readingl, SIDE_CLEAR_PARAM)){
//								if (clear(readingf)){
//									goForward(0x35);
//									osDelay(1000);
//								}
//								targetangle = angle1;
//								comingtoTURNfrom = FWD_UNALIGNED;
//								dir=DIR_LEFT;
//								state = TURN;
//							}
							
						}
						break;

					case STOP:
						counterEN=0;
						seconds=0;
						targetangle=angle1;
						if (clear(readingf)){
								goForward(0x35);
								osDelay(1000);
							}
						stop();
						targetangle = angle4;
						left(0x20);
						parentState=1;
				    state=INIT;
						int temp=angle2;
						angle2 = angle1;
						angle1 = angle4;
						angle4 = angle3;
						angle3 = temp;
						break;
				}
			}
			osDelay(10);
	}
  /* USER CODE END MainMoveFunc */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
