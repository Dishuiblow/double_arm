/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"
#include "gpio.h"
#include "spi.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_bsp.h"
#include "vofa.h"
#include "crc16.h"
#include "ws2812.h"
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include "delay.h"
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

extern uint8_t Is_USBget = 0;//

extern int init_motor_data[6] = {0};

extern volatile uint8_t USB_Rcive_Data[117];//!14*8+5=117

//!1-7为左臂
uint8_t motor1_date[8]={0};
uint8_t motor2_date[8]={0};
uint8_t motor3_date[8]={0};
uint8_t motor4_date[8]={0};
uint8_t motor5_date[8]={0};
uint8_t motor6_date[8]={0};
uint8_t motor7_date[8]={0};
//!8-14为右臂
uint8_t motor8_date[8]={0};
uint8_t motor9_date[8]={0};
uint8_t motor10_date[8]={0};
uint8_t motor11_date[8]={0};
uint8_t motor12_date[8]={0};
uint8_t motor13_date[8]={0};
uint8_t motor14_date[8]={0};//右臂夹爪
uint8_t date_to_send[FRAME_DATA_LENGTH] = {0};//data sended to up

uint8_t ControlMode = 0;

uint8_t frame[FRAME_SIZE] = {0};

uint8_t r = 1;
uint8_t g = 100;
uint8_t b = 1;

uint8_t enable_data[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc};
uint8_t disable_data[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfd};

uint8_t motor_origin_data[8] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void flashing(int r, int g, int b);

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
	
	/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
	
	MX_SPI6_Init();
	
  MX_USB_DEVICE_Init();
	
	 /* Initialize all configured peripherals */
	 
  MX_GPIO_Init();
	
  MX_FDCAN1_Init();
	
	MX_FDCAN2_Init();
	
  //MX_TIM3_Init();
  //MX_USART1_UART_Init();
  //MX_TIM4_Init();
	 MX_TIM2_Init();
	 
	 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (475));
	  MX_USART1_UART_Init();
  MX_UART5_Init();
	MX_UART7_Init();
	MX_DMA_Init();
	HAL_Delay(200);
  /* USER CODE BEGIN 2 */
	power(1);
	HAL_Delay(100);
	
	power2(1);
	HAL_Delay(500);

	HAL_GPIO_WritePin(GPIOC, POWER_5V_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_Delay(500);
	
	bsp_fdcan_set_baud(&hfdcan1, CAN_FD_BRS, CAN_BR_5M);
	
	bsp_fdcan_set_baud(&hfdcan2, CAN_FD_BRS, CAN_BR_5M);

	bsp_can_init();
	
	dm_motor_init();
/**
 * @brief 左右臂电机自检
 * 
 */
	fdcanx_send_data(&hfdcan1,0x01,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan1,0x02,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan1,0x03,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan1,0x04,disable_data,8);
	HAL_Delay(100);
 	fdcanx_send_data(&hfdcan1,0x05,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan1,0x06,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x01,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x02,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x03,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x04,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x05,disable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x06,disable_data,8);
	HAL_Delay(100);
	//
	fdcanx_send_data(&hfdcan1,0x01,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan1,0x02,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan1,0x03,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan1,0x04,enable_data,8);
	HAL_Delay(100);
 	fdcanx_send_data(&hfdcan1,0x05,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan1,0x06,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x01,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x02,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x03,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x04,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x05,enable_data,8);
	HAL_Delay(100);
	fdcanx_send_data(&hfdcan2,0x06,enable_data,8);
	HAL_Delay(100);
	//HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE BEGIN 2 */
	
	WS2812_Ctrl(1, 50, 1);
	
	HAL_Delay(100);
	
	Is_USBget = 0;
	
 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		for(int i=0; i<8; i++)
		{
			motor_origin_data[i] = motor[Motor1].ori_date[i];
		}
		Process_Frame();
	
		for(int i=0;i<8;i++)
		{
			motor1_date[i] = USB_Rcive_Data[i];
			motor2_date[i] = USB_Rcive_Data[8+i];
			motor3_date[i] = USB_Rcive_Data[16+i];
			motor4_date[i] = USB_Rcive_Data[24+i];
			motor5_date[i] = USB_Rcive_Data[32+i];
			motor6_date[i] = USB_Rcive_Data[40+i];
			motor7_date[i] = USB_Rcive_Data[48+i];
			motor8_date[i] = USB_Rcive_Data[56+i];
		    motor9_date[i] = USB_Rcive_Data[64+i];
			motor10_date[i]= USB_Rcive_Data[72+i];
			motor11_date[i]= USB_Rcive_Data[80+i];
            motor12_date[i]= USB_Rcive_Data[88+i];
			motor13_date[i]= USB_Rcive_Data[96+i];
			motor14_date[i]= USB_Rcive_Data[104+i];
		}			

		//date_to_send[32] = ControlMode++;
		//if(ControlMode==0x17) ControlMode = 0x00;
		for(int i=0;i<8;i++){
			//左臂 CAN 电机反馈
			date_to_send[0+i]  = motor4340[Motor1].ori_date[i];
			date_to_send[8+i]  = motor4340[Motor2].ori_date[i];
			date_to_send[16+i] = motor4340[Motor3].ori_date[i];
			date_to_send[24+i] = motor[Motor4].ori_date[i];
			date_to_send[32+i] = motor[Motor5].ori_date[i];
			date_to_send[40+i] = motor[Motor6].ori_date[i];
			// 夹爪是PWM控制，无CAN反馈，直接用 0 占位对齐
			date_to_send[48+i] = 0; 
			//右臂 CAN 电机反馈
			date_to_send[56+i] = motor4340[Motor7].ori_date[i];
			date_to_send[64+i] = motor4340[Motor8].ori_date[i];
			date_to_send[72+i] = motor4340[Motor9].ori_date[i];
			date_to_send[80+i] = motor[Motor10].ori_date[i];
			date_to_send[88+i] = motor[Motor11].ori_date[i];
			date_to_send[96+i] = motor[Motor12].ori_date[i];
			//右臂夹爪
			date_to_send[104+i] = 0;
		}

		Frame_Pack(frame, date_to_send);

		if(Is_USBget )
		{
			WS2812_Ctrl(40, 0, 0);
			for(int i=0;i<6;i++)
			{
				 
				if(init_motor_data[i]>50)
				{
					if(i==0)
					{
						for(int i=0;i<8;i++){
						motor4340[Motor1].ori_date[i] = 0;
						}
					}
					if(i==1)
					{
						for(int i=0;i<8;i++){
						motor4340[Motor2].ori_date[i] = 0;
							}
					}
					if(i==2)
					{
						for(int i=0;i<8;i++){
						motor4340[Motor3].ori_date[i] = 0;
							}
					}
					if(i==3)
					{
						for(int i=0;i<8;i++){
						motor[Motor4].ori_date[i] = 0;
							}
					}
					if(i==4)
					{
						for(int i=0;i<8;i++){
						motor[Motor5].ori_date[i] = 0;
							}
					}
					if(i==5)
					{
						for(int i=0;i<8;i++){
						motor[Motor6].ori_date[i] = 0;
							}
					}
 
				}
				else
				{
					init_motor_data[i] +=1;
				}
			}
			

			//CDC_Transmit_FS(frame, FRAME_SIZE);
			vofa_transmit(frame, FRAME_SIZE);
			
			fdcanx_send_data(&hfdcan1, 0x01, motor1_date, 8);
				
			fdcanx_send_data(&hfdcan1, 0x02, motor2_date, 8);
				
			fdcanx_send_data(&hfdcan1, 0x03, motor3_date, 8);
//				
			fdcanx_send_data(&hfdcan1, 0x04, motor4_date, 8);
				
			fdcanx_send_data(&hfdcan1, 0x05, motor5_date, 8);
				
			fdcanx_send_data(&hfdcan1, 0x06, motor6_date, 8);

			fdcanx_send_data(&hfdcan2, 0x01, motor1_date, 8);
				
			fdcanx_send_data(&hfdcan2, 0x02, motor2_date, 8);
				
			fdcanx_send_data(&hfdcan2, 0x03, motor3_date, 8);
//				
			fdcanx_send_data(&hfdcan2, 0x04, motor4_date, 8);
				
			fdcanx_send_data(&hfdcan2, 0x05, motor5_date, 8);
				
			fdcanx_send_data(&hfdcan2, 0x06, motor6_date, 8);
			

				//pwm control servo
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (475-motor7_date[0]));
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (475-motor14_date[0]));
			Is_USBget = 0;
			
		}
		else
		{
			WS2812_Ctrl(0, 1, 0);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;//4;//if it is 4 canfd did not work
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}



//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  /** Supply configuration update enable
//  */
//  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

//  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 2;
//  RCC_OscInitStruct.PLL.PLLN = 40;
//  RCC_OscInitStruct.PLL.PLLP = 1;
//  RCC_OscInitStruct.PLL.PLLQ = 6;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
//  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
//  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}




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
