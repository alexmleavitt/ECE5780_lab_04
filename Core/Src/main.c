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
volatile char recieved;
volatile char input;
volatile int flag;
volatile char read;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Transmit_char(char transmitted);
void Transfer_error(void);
void Transmit_string(char* string);
void Wait_for_input(void);
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
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  //HAL_Init();


  /* Configure the system clock */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR  |= RCC_APB1ENR_USART3EN;
  SystemClock_Config();

	// Configure the leds
	GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18);
	GPIOC->MODER &= ~((1<<13) | (1<<15) | (1<<17) | (1<<19));
	GPIOC->OTYPER &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));
	GPIOC->OSPEEDR &= ~((1<<12) | (1<<14) | (1<<16) | (1<<18));
	GPIOC->PUPDR &= ~((1<<12) | (1<<14) | (1<<16) | (1<<18)
									| (1<<13) | (1<<15) | (1<<17) | (1<<19));
	GPIOC->OSPEEDR &= ~((1<<0) | (1<<1));

	

	GPIOB->MODER |= (1<<23) | (1<<21);
	GPIOB->MODER &= ~((1<<22) | (1<<20));
	GPIOB->OTYPER &= ~((1<<10) | (1<<11));
	GPIOB->OSPEEDR &= ~((1<<20) | (1<<21) | (1<<22) | (1<<23));
	GPIOB->PUPDR &= ~((1<<20) | (1<<21) | (1<<22) | (1<<23));
	GPIOB->AFR[1] |= (1<<14) | (1<<10);
	GPIOB->AFR[1] &= ~((1<<15) | (1<<13) | (1<<12) | (1<<11)
									| (1<<9) | (1<<8));
	
  USART3->BRR = 69;
	USART3->CR1 |= (1<<2) | (1<<3);
	USART3->CR1 |= (1<<0);
	
	USART3->CR1 |= (1<<5);
	NVIC_EnableIRQ(29);
	NVIC_SetPriority(29,1);
  
	GPIOC->ODR &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));	
	//Wait_for_input();
	
  while (1)
  {
//		if (USART3->ISR & (1<<5))
//		{
//			recieved = USART3->RDR;
//			switch(recieved){
//				case 'r':
//				GPIOC->ODR |= (1<<6);	
//				break;
//				case 'b':
//				GPIOC->ODR |= (1<<7);
//				break;
//				case 'g':
//				GPIOC->ODR |= (1<<9);
//				break;
//				case 'o':
//				GPIOC->ODR |= (1<<8);
//				break;
//				default:
//				Transmit_string("error\n");
//			}	
//				
//		}
//		HAL_Delay(200);
//		Transmit_char('a');
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Transmit_char(char transmitted)
{
	while((USART3->ISR & (1<<7)) == 0)
	{
	}
	
	USART3->TDR = transmitted;
}

void Transmit_string(char* string)
{
	int i = 0;
	while(*string != '\0')
	{
		Transmit_char(*string);
		string++;
	}
	Transmit_char('\r');
}

void Transfer_error(void)
{
	Transmit_char('e');
	Transmit_char('r');
	Transmit_char('r');
	Transmit_char('o');
	Transmit_char('r');
	Transmit_char('\n');
	Transmit_char('\r');
	return;
}

void USART3_4_IRQHandler(void)
{
	input = USART3->RDR;
	flag = 1;
	Transmit_string("check\n");
}

void Wait_for_input(void)
{
	int led;
	Transmit_string("CMD? \n");
	while(!flag)
	{
	}
	led = input;
	flag = 0;
	while(!flag)
	{
	}
	char read = input;
			flag = 0;
			switch(read){
				case 0:
				GPIOC->ODR &= ~(1<<led);
				Transmit_string("OFF\n");				
				break;
				case 1:
				GPIOC->ODR |= (1<<led);
				Transmit_string("ON\n");
				break;
				case 2:
				GPIOC->ODR ^= (1<<led);
				Transmit_string("Toggle\n");
				break;
				default:
				Transmit_string("error\n");
				Wait_for_input();
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
