/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USER_MAIN_DEBUG
#define COMMAND_BUFFER_SIZE 256 
#define MOTOR_NUM 2
#define FULL_PULSE 100
#define DELTA_PULSE 1
#define MAG_BUFFER_LEN 9
#define CARD_BUFFER_LEN 16
#define huart_info huart1
#define huart_mag huart2
#define huart_rfid huart3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifdef USER_MAIN_DEBUG
#define user_main_printf(format, ...) printf( format "\r\n", ##__VA_ARGS__)
#define user_main_info(format, ...) printf("[\tmain]info:" format "\r\n", ##__VA_ARGS__)
#define user_main_debug(format, ...) printf("[\tmain]debug:" format "\r\n", ##__VA_ARGS__)
#define user_main_error(format, ...) printf("[\tmain]error:" format "\r\n", ##__VA_ARGS__)
#else
#define user_main_printf(format, ...)
#define user_main_info(format, ...)
#define user_main_debug(format, ...)
#define user_main_error(format, ...)
#endif
#define abs(x) ((x)>=0?(x):-(x))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t infoBuffer[1];      // info receive buffer
uint8_t cardBuffer[CARD_BUFFER_LEN];      // card receive buffer
uint8_t magBuffer[MAG_BUFFER_LEN];      // magnetic receive buffer
// command buffer
uint8_t cmdBuffer[COMMAND_BUFFER_SIZE];
uint8_t cmdPtr = 0;
uint8_t cmdCompleted = 0;
// motor variables
int16_t pulse_motor[MOTOR_NUM] = {0};
uint32_t channel_motor[MOTOR_NUM] = {TIM_CHANNEL_1, TIM_CHANNEL_2};
GPIO_TypeDef* motor_sig_port[MOTOR_NUM][2] = {{MOTOR1_SIG1_GPIO_Port, MOTOR1_SIG2_GPIO_Port},
																							{MOTOR2_SIG1_GPIO_Port, MOTOR2_SIG2_GPIO_Port}};
uint16_t motor_sig_pin[MOTOR_NUM][2] = {{MOTOR1_SIG1_Pin, MOTOR1_SIG2_Pin},
																				{MOTOR2_SIG1_Pin, MOTOR2_SIG2_Pin}};
// stop & speed
uint8_t moving_status = 0; // 0 for break; 1 for forward; 2 for backward; 3 for turn left; 4 for turn right;
uint8_t forward_speed = 0;
// site
uint8_t target_site = 0;
// pid
int16_t PID_previous_error = 0;
int16_t PID_integral = 0;
float Kp=0.1;
float Ki=0.01;
float Kd=0.05;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f){
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart_info, temp, 1, 2); // modify huart_info in need
	return ch;
}

int atoi(char* source){
	int length = strlen(source);
	int sum = 0;
	for(int i=0;i<length;i++){
		sum = sum*10 + (source[i]-'0');
	}
	return sum;
}
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	user_main_printf("Welcome to STM32F103!");
	for(uint8_t i=0; i<MOTOR_NUM; ++i){
		HAL_TIM_PWM_Start(&htim2, channel_motor[i]); // enable PWM2 channel
	}
	HAL_TIM_Base_Start_IT(&htim3); // enable timer3 interrupt
	HAL_UART_Receive_IT(&huart_info, infoBuffer, 1); //enable usart1 reveive
	HAL_UART_Receive_IT(&huart_rfid, cardBuffer, CARD_BUFFER_LEN); //enable card usart reveive
	HAL_UART_Receive_IT(&huart_mag, magBuffer, MAG_BUFFER_LEN); //enable mag usart reveive
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// HAL_Delay(50);
		// HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		// HAL_Delay(50);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
// motor
void LogicSwitch(uint8_t motor_id, int8_t logic){
	switch(logic){
		case 1:{ // xuan kong
			HAL_GPIO_WritePin(motor_sig_port[motor_id][0], motor_sig_pin[motor_id][0], GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor_sig_port[motor_id][1], motor_sig_pin[motor_id][1], GPIO_PIN_SET);
			break;
		}
		case 2:{ // zheng zhuan
			HAL_GPIO_WritePin(motor_sig_port[motor_id][0], motor_sig_pin[motor_id][0], GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor_sig_port[motor_id][1], motor_sig_pin[motor_id][1], GPIO_PIN_RESET);
			break;
		}
		case 3:{ // fan zhuan
			HAL_GPIO_WritePin(motor_sig_port[motor_id][0], motor_sig_pin[motor_id][0], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor_sig_port[motor_id][1], motor_sig_pin[motor_id][1], GPIO_PIN_SET);
			break;
		}
		default:{ // sha che
			HAL_GPIO_WritePin(motor_sig_port[motor_id][0], motor_sig_pin[motor_id][0], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor_sig_port[motor_id][1], motor_sig_pin[motor_id][1], GPIO_PIN_RESET);
		}
	}
}
void ModifyPulse(uint8_t motor_id, int16_t pulse){
	pulse = max(DELTA_PULSE-FULL_PULSE, min(pulse, FULL_PULSE-DELTA_PULSE));
	pulse_motor[motor_id] = pulse;
	LogicSwitch(motor_id, pulse>0?2:3);
	user_main_printf("[\thandleCmd]ModifyPulse: %d!", pulse);
	__HAL_TIM_SET_COMPARE(&htim2, channel_motor[motor_id], abs(pulse)); // modify pulse
}
void SpeedUp(uint8_t motor_id, int16_t pulse){
	ModifyPulse(motor_id, pulse_motor[motor_id]+pulse);
}
void SlowDown(uint8_t motor_id, int16_t pulse){
	ModifyPulse(motor_id, pulse_motor[motor_id]-pulse);
}
void MotorHang(){
	for(uint8_t i=0; i<MOTOR_NUM; ++i){
		LogicSwitch(i, 1);
	}
}
void MotorOn(){
	moving_status = 1;
	for(uint8_t i=0; i<MOTOR_NUM; ++i){
		LogicSwitch(i, 2);
	}
}
void MotorOff(){
	moving_status = 0;
	for(uint8_t i=0; i<MOTOR_NUM; ++i){
		LogicSwitch(i, 0);
	}
}
void MotorSpeedUp(){
	for(uint8_t i=0; i<MOTOR_NUM; ++i){
		SpeedUp(i, DELTA_PULSE);
	}
}
void MotorSlowDown(){
	for(uint8_t i=0; i<MOTOR_NUM; ++i){
		SlowDown(i, DELTA_PULSE);
	}
}
void MotorTurnLeft(){
	moving_status = 3;
	SlowDown(0, 5*DELTA_PULSE);
	SpeedUp(1, 5*DELTA_PULSE);
}
void MotorTurnRight(){
	moving_status = 4;
	SpeedUp(0, 5*DELTA_PULSE);
	SlowDown(1, 5*DELTA_PULSE);
}
void MagneticGuide(int8_t* magStatus){
	if(moving_status == 0){ // stop status
		return;
	}
	uint8_t absMag[8] = {0};
	for(uint16_t i=0; i<8; ++i){
		absMag[i] = abs(magStatus[i]);
	}
	user_main_printf("[\thandleNVIC3]: read mag %d %d %d %d %d %d %d %d!", 
									 magStatus[0], magStatus[1], magStatus[2], magStatus[3],
									 magStatus[4], magStatus[5], magStatus[6], magStatus[7]);
	int16_t measured_value = 0;
	for(uint16_t i=0; i<4; ++i){
		measured_value += absMag[i] - absMag[7-i];
	}
	//int16_t PID_error = 0 - measured_value;
	//PID_integral = PID_integral + PID_error*1;
	//int16_t PID_derivative = (PID_error - PID_previous_error) / 1;
	//float PID_output = Kp*PID_error + Ki*PID_integral + Kd*PID_derivative;
	//PID_previous_error = PID_error;
	user_main_printf("[\tGuide]: output %d!", measured_value);
	int16_t forward_thres = 20;
	if(abs(measured_value)<forward_thres){
		moving_status = 1; // forward
		ModifyPulse(0, (int16_t)forward_speed);
		ModifyPulse(1, (int16_t)forward_speed);
	}
	if(measured_value<-1*forward_thres){
		if(moving_status!=3){ // turn left
			moving_status = 3;
			ModifyPulse(0, (int16_t)forward_speed-5*DELTA_PULSE);
			ModifyPulse(1, (int16_t)forward_speed+5*DELTA_PULSE);
		}
	}
	if(measured_value>forward_thres){
		if(moving_status!=4){ // turn right
			moving_status = 4;
			ModifyPulse(0, (int16_t)forward_speed+5*DELTA_PULSE);
			ModifyPulse(1, (int16_t)forward_speed-5*DELTA_PULSE);
		}
	}
}


//清空接收缓冲
void clearCmdBuffer(void)
{
  cmdPtr = 0;
  cmdBuffer[cmdPtr] = 0;
}
void handleCmd(void){
	user_main_printf("[\thandleCmd]Handle Cmd: %s!", cmdBuffer);
	char delim[] = " ";
	char *cmdseg = strtok((char*)cmdBuffer, delim);
	if(cmdseg!=NULL){
		char *idstr = strtok(NULL, delim);
		if(idstr!=NULL){
			uint8_t mid = idstr[0] - '1';
			if(strcmp(cmdseg, "on")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: on!");
				LogicSwitch(mid, 1);
			}
			if(strcmp(cmdseg, "off")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: off!");
				LogicSwitch(mid, 0);
			}
			if(strcmp(cmdseg, "up")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: speed up!");
				SpeedUp(mid, DELTA_PULSE);
			}
			if(strcmp(cmdseg, "down")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: slow down!");
				SlowDown(mid, DELTA_PULSE);
			}
			if(strcmp(cmdseg, "speed")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: set speed!");
				forward_speed = atoi(idstr);
				user_main_printf("[\thandleCmd]speed set to %d!", forward_speed);
			}
		}else{
			if(strcmp(cmdseg, "stop")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: stop!");
				moving_status = 0;
				MotorOff();
			}
			if(strcmp(cmdseg, "start")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: start!");
				moving_status = 1;
				MotorOn();
			}
			if(strcmp(cmdseg, "left")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: left!");
				MotorTurnLeft();
			}
			if(strcmp(cmdseg, "right")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: right!");
				MotorTurnRight();
			}
			if(strcmp(cmdseg, "up")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: up!");
				MotorSpeedUp();
			}
			if(strcmp(cmdseg, "down")==0){
				user_main_printf("[\thandleCmd]Handle Cmd: down!");
				MotorSlowDown();
			}
		}
	}
	clearCmdBuffer();
}
void addCmdBuffer(uint8_t data)
{
	if(data == '\r' ){
		return;
	}
	//如果为回车键，则开始处理串口数据
	if(data == '\n'){
		cmdCompleted = 1;
		handleCmd();
		return;
	}
	// add cmd
	if(cmdPtr < (COMMAND_BUFFER_SIZE - 1)){
		cmdBuffer[cmdPtr] = data;
		cmdBuffer[cmdPtr + 1]=0x00;
		cmdPtr++;
	}
	else{
		cmdBuffer[cmdPtr - 1] = data;
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart_info){
		uint8_t data = infoBuffer[0];
		addCmdBuffer(data);
		HAL_UART_Receive_IT(&huart_info, infoBuffer, 1);
	}
	if(huart == &huart_rfid){
		uint8_t card_id = cardBuffer[15];
		user_main_printf("[\thandleNVIC2]: read card %d!", card_id);
		if(card_id == target_site){
			MotorOff();
		}
		HAL_UART_Receive_IT(&huart_rfid, cardBuffer, CARD_BUFFER_LEN);
	}
	if(huart == &huart_mag){
		// user_main_printf("[\thandleNVIC3]:+++++ mag start +++++!");
		int8_t magStatus[8] = {0};
		for(uint16_t i=0; i<8; ++i){
			magStatus[i] = (int8_t)magBuffer[i];
		}
		
		MagneticGuide(magStatus);
		//user_main_printf("[\thandleNVIC3]: read mag %d %d %d %d %d %d %d %d!", 
		//									magStatus[0], magStatus[1], magStatus[2], magStatus[3],
		//									 magStatus[4], magStatus[5], magStatus[6], magStatus[7]);
		// user_main_printf("[\thandleNVIC3]:----- mag end -----!");
		HAL_UART_Receive_IT(&huart_mag, magBuffer, MAG_BUFFER_LEN);
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart_rfid)
	{
		__HAL_UNLOCK(huart);
		HAL_UART_Receive_IT(&huart_rfid, cardBuffer, CARD_BUFFER_LEN);
	}
	if(huart == &huart_mag)
	{
		__HAL_UNLOCK(huart);
		HAL_UART_Receive_IT(&huart_mag, magBuffer, MAG_BUFFER_LEN);
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == (&htim3))
	{
		if(moving_status!=0){
			uint8_t mag_addr[1] = {0xC1};
			HAL_UART_Transmit(&huart_mag, mag_addr, 1, 2); // send magnetic address
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}
	}
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
