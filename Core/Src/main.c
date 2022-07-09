/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_data = 0;
uint8_t flag = 0;
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart3, &ch, 1, 1000);
	return ch;
}

enum {

	MAXARM1 = 1100,
	MAXARM2 = 1100,
	MAXARM3 = 1100,
	MAXGRIPER = 900,
	MAXROTATION = 1100,

	MIDARM1 = 500,
	MIDARM2 = 600,
	MIDARM3 = 700,
	MIDGRIPER = 800,
	MIDROTATION = 550,

	MINARM1 = 500,
	MINARM2 = 550,
	MINARM3 = 300,
	MINGRIPER = 500,
	MINROTATION = 200,

	INIT_CAPACITY = 50,
	MEMORT_SV = 15
};

typedef struct {

	uint16_t *memory_arm1;
	uint16_t *memory_arm2;
	uint16_t *memory_arm3;
	uint16_t *memory_griper;
	uint16_t *memory_rotation;

} memory_value;

typedef struct {

	uint16_t arm1;
	uint16_t arm2;
	uint16_t arm3;
	uint16_t griper;
	uint16_t rotation;

} CCR_value;

CCR_value ccr;
uint16_t capacity = INIT_CAPACITY;
uint16_t adcvalue[5];
uint16_t my_abs(int16_t value);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void init_malloc(memory_value *memory_arm);
memory_value * resize_malloc(memory_value *memory_arm);
void robot_arm_move();
void robot_arm_memory(memory_value *memory_arm);
void robot_arm_memory_move(memory_value *memory_arm);

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
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart3, &rx_data, 1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);   //PA0 CCR 1  arm1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);   //PB3 CCR 2	arm2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   //PB10 CCR 3 arm3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   //PA3 CCR 4 griper
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   //PA3 CCR 4 griper
	HAL_ADC_Start_DMA(&hadc1,adcvalue,5);       //ADC1 CH10,2,3,4,5 ->

	memory_value memory_arm;
	init_malloc(&memory_arm);

	ccr.arm1 = MIDARM1;
	ccr.arm2 = MIDARM2;
	ccr.arm3 = MIDARM3;
	ccr.griper = MIDGRIPER;
	ccr.rotation = MIDROTATION;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {


		if (flag == 0) {
			robot_arm_move();
		}

		if (flag == 1) {
			robot_arm_memory(&memory_arm);
		}

		if (flag == 2) {
			robot_arm_memory_move(&memory_arm);
			flag =0;
		}

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* USER CODE BEGIN 4 */

void init_malloc(memory_value *memory_arm) {

	memory_arm->memory_arm1 = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (memory_arm->memory_arm1 == NULL) {
		printf("\r\n malloc falied \r\n");
		return;
	}
	memory_arm->memory_arm2 = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (memory_arm->memory_arm2 == NULL) {
		printf("\r\n malloc falied \r\n");
		return;
	}
	memory_arm->memory_arm3 = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (memory_arm->memory_arm3 == NULL) {
		printf("\r\n malloc falied \r\n");
		return;
	}
	memory_arm->memory_griper = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (memory_arm->memory_griper == NULL) {
		printf("\r\n malloc falied \r\n");
		return;
	}
	memory_arm->memory_rotation = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (memory_arm->memory_rotation == NULL) {
		printf("\r\n malloc falied \r\n");
		return;
	}



}

memory_value * resize_malloc(memory_value *memory_arm) {

	uint16_t capacity_tmp = capacity;
	capacity += INIT_CAPACITY;

	uint16_t *arm1_tmp = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (arm1_tmp == NULL) {
		printf("\r\n resize falied \r\n");
		return NULL;
	}
	uint16_t *arm2_tmp = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (arm2_tmp == NULL) {
		printf("\r\n resize falied \r\n");
		return NULL;
	}
	uint16_t *arm3_tmp = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (arm3_tmp == NULL) {
		printf("\r\n resize falied \r\n");
		return NULL;
	}
	uint16_t *griper_tmp = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (griper_tmp == NULL) {
		printf("\r\n resize falied \r\n");
		return NULL;
	}
	uint16_t *rotation_tmp = (uint16_t*) malloc(sizeof(uint16_t) * capacity);
	if (rotation_tmp == NULL) {
		printf("\r\n resize falied \r\n");
		return NULL;
	}

	printf("\r\n malloc resize %d -> %d \r\n", capacity_tmp, capacity);

	for (int i = 0; i < capacity_tmp; i++) {
		arm1_tmp[i] = memory_arm->memory_arm1[i];
		arm2_tmp[i] = memory_arm->memory_arm2[i];
		arm3_tmp[i] = memory_arm->memory_arm3[i];
		griper_tmp[i] = memory_arm->memory_griper[i];
		rotation_tmp[i] = memory_arm->memory_rotation[i];
	}



	for (int i = capacity_tmp; i < capacity; i++) {
		arm1_tmp[i] = 0;
		arm2_tmp[i] = 0;
		arm3_tmp[i] = 0;
		griper_tmp[i] = 0;
		rotation_tmp[i] = 0;
	}

	free(memory_arm->memory_arm1);
	free(memory_arm->memory_arm2);
	free(memory_arm->memory_arm3);
	free(memory_arm->memory_griper);
	free(memory_arm->memory_rotation);

	memory_arm->memory_arm1 = arm1_tmp;
	memory_arm->memory_arm2 = arm2_tmp;
	memory_arm->memory_arm3 = arm3_tmp;
	memory_arm->memory_griper = griper_tmp;
	memory_arm->memory_rotation = rotation_tmp;

	printf("\r\n resize complete \r\n");
	return memory_arm;
}

void robot_arm_move() {


	ccr.arm1 = MINARM1+(adcvalue[0] / 6.83);
	ccr.arm2 = MINARM2+(adcvalue[1]/7.5);
	ccr.arm3 = MINARM3+(adcvalue[2]/ 5.12);
	ccr.griper = MINGRIPER + (adcvalue[3]/ 10.24);
	ccr.rotation = MINROTATION + (adcvalue[4]/ 4.55);

	TIM2->CCR1 = ccr.arm1;
	TIM2->CCR2 = ccr.arm2;
	TIM2->CCR3 = ccr.arm3;
	TIM3->CCR2 = ccr.griper;
	TIM3->CCR1 = ccr.rotation;

	printf("\r\n 1: %4d, 2: %4d, 3 : %4d, 4 : %4d, 5 : %4d \r\n",adcvalue[0],adcvalue[1],adcvalue[2],adcvalue[3],adcvalue[4]);
	printf("\r\n arm1 : %d, arm2 : %d, arm3 : %d, griper4 : %d, rotation : %d  \r\n",
			ccr.arm1, ccr.arm2, ccr.arm3, ccr.griper, ccr.rotation);
	HAL_Delay(10);
}

void robot_arm_memory(memory_value *memory_arm) {


	memset(memory_arm->memory_arm1,0,sizeof(uint16_t)*capacity);
	memset(memory_arm->memory_arm2,0,sizeof(uint16_t)*capacity);
	memset(memory_arm->memory_arm3,0,sizeof(uint16_t)*capacity);
	memset(memory_arm->memory_griper,0,sizeof(uint16_t)*capacity);
	memset(memory_arm->memory_rotation,0,sizeof(uint16_t)*capacity);



	memory_arm->memory_arm1[0] = ccr.arm1;
	memory_arm->memory_arm2[0] = ccr.arm2;
	memory_arm->memory_arm3[0] = ccr.arm3;
	memory_arm->memory_griper[0] = ccr.griper;
	memory_arm->memory_rotation[0] = ccr.rotation;

	int arm1;
	int arm2;
	int arm3;
	int griper;
	int rotation;


	int i = 1;
	while (1) {
		robot_arm_move();
		arm1 = my_abs(memory_arm->memory_arm1[i - 1] - ccr.arm1);
		arm2 = my_abs(memory_arm->memory_arm2[i - 1] - ccr.arm2);
		arm3 = my_abs(memory_arm->memory_arm3[i - 1] - ccr.arm3);
		griper = my_abs(memory_arm->memory_griper[i - 1] - ccr.griper);
		rotation = my_abs(memory_arm->memory_rotation[i - 1] - ccr.rotation);

		if (arm1>MEMORT_SV ||arm2>MEMORT_SV ||arm3>MEMORT_SV ||griper>MEMORT_SV ||rotation>MEMORT_SV ){
			memory_arm->memory_arm1[i] = ccr.arm1;
			memory_arm->memory_arm2[i] = ccr.arm2;
			memory_arm->memory_arm3[i] = ccr.arm3;
			memory_arm->memory_griper[i] = ccr.griper;
			memory_arm->memory_rotation[i] = ccr.rotation;

			i++;
			if (i >= capacity) {
				memory_arm = resize_malloc(memory_arm);
			}
		}
		if (flag == 2) break;

		printf("\r\n i = %d \r\n", i);
		printf("\r\n [memory] arm1:%d, arm2:%d, arm3:%d, griper:%d, rotation:%d \r\n",
				memory_arm->memory_arm1[i-1], memory_arm->memory_arm2[i-1],
				memory_arm->memory_arm3[i-1], memory_arm->memory_griper[i-1],
				memory_arm->memory_rotation[i-1]);

	}
}

void robot_arm_memory_move(memory_value *memory_arm) {

	int i = 0;
	while (i < capacity) {

		if (memory_arm->memory_arm1[i] == 0 || memory_arm->memory_arm2[i] == 0
				|| memory_arm->memory_arm3[i] == 0
				|| memory_arm->memory_griper[i] == 0
				|| memory_arm->memory_rotation[i] == 0) {
			break;
		}
		TIM2->CCR1 = memory_arm->memory_arm1[i];
		TIM2->CCR2 = memory_arm->memory_arm2[i];
		TIM2->CCR3 = memory_arm->memory_arm3[i];
		TIM3->CCR2 = memory_arm->memory_griper[i];
		TIM3->CCR1 = memory_arm->memory_rotation[i];
		HAL_Delay(30);
		printf("\r\n [memory] i = %d,  arm1:%d, arm2:%d, arm3:%d, griper:%d, rotation:%d \r\n",i,
				memory_arm->memory_arm1[i], memory_arm->memory_arm2[i],
				memory_arm->memory_arm3[i], memory_arm->memory_griper[i],
				memory_arm->memory_rotation[i]);
		i++;
	}
	printf("\r\n memory move complete \r\n");
}


uint16_t my_abs(int16_t value){

	if(value<0) return value*-1;
	else return value;
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART3) {
		HAL_UART_Transmit(&huart3, &rx_data, 1, 100);
		HAL_UART_Receive_IT(&huart3, &rx_data, 1);

		switch (rx_data) {

		case '1':
			if (ccr.arm1 >=MAXARM1) {
				ccr.arm1 = MAXARM1;
			} else {
				ccr.arm1 += 10;
			}
			break;

		case '2':
			if (ccr.arm1 <= MINARM2) {
				ccr.arm1 = MINARM2;
			} else {
				ccr.arm1 -= 10;
			}
			break;

		case '5':
			if (ccr.arm2 >= MAXARM2) {
				ccr.arm2 = MAXARM2;
			} else {
				ccr.arm2 += 30;
			}
			break;
		case '4':
			if (ccr.arm2 <= MINARM2) {
				ccr.arm2 = MINARM2;
			} else {
				ccr.arm2 -= 30;
			}
			break;

		case '7':
			if (ccr.arm3 >= MAXARM3) {
				ccr.arm3 = MAXARM3;
			} else {
				ccr.arm3 += 30;
			}
			break;
		case '8':
			if (ccr.arm3 <= MINARM3) {
				ccr.arm3 = MINARM3;
			} else {
				ccr.arm3 -= 30;
			}
			break;

		case '-':
			if (ccr.griper >= MAXGRIPER) {
				ccr.griper = MAXGRIPER;
			} else {
				ccr.griper += 30;
			}
			break;
		case '=':
			if (ccr.griper <= MINGRIPER) {
				ccr.griper = MINGRIPER;
			} else {
				ccr.griper -= 30;
			}
			break;

		case 'a':
			if (ccr.rotation >= MAXROTATION) {
				ccr.rotation = MAXROTATION;
			} else {
				ccr.rotation += 30;
			}
			break;

		case 's':
			if (ccr.rotation <= MINROTATION) {
				ccr.rotation = MINROTATION;
			} else {
				ccr.rotation -= 30;
			}
			break;

		case 'q':
			flag = 0;
			break;
		case 'w':
			flag = 1;
			break;
		case 'e':
			flag = 2;
			break;

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
	while (1) {
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
