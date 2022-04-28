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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define     LCM_OUT1     	GPIOB->ODR
#define     LCM_OUT0        GPIOC->ODR
#define     LCM_RS         	GPIO_PIN_0        	// PC0
#define     LCM_RW         	GPIO_PIN_1        	// PC1
#define     LCM_E          	GPIO_PIN_2        	// PC2
#define     LCM_D7         	GPIO_PIN_7        	// PB7
#define     LCM_D6        	GPIO_PIN_6        	// PB6
#define     LCM_D5        	GPIO_PIN_5          // PB5
#define     LCM_D4        	GPIO_PIN_4          // PB4
#define     LCM_D3        	GPIO_PIN_3          // PB3
#define     LCM_D2        	GPIO_PIN_2          // PB2
#define     LCM_D1        	GPIO_PIN_1          // PB1
#define     LCM_D0        	GPIO_PIN_0          // PB0
#define     LCM_MASK  (LCM_D7|LCM_D6|LCM_D5|LCM_D4|LCM_D3|LCM_D2|LCM_D1| LCM_D0)
#define     switcho_mask  0xAAAA
#define     switchi_mask  0x0000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t ValRead; // перемення для считывания значения из дисплея
char col;		// номер колонки на дисплее
char addInd;	//адрес позиции курсора (строка +колонка)
char data;	// данные индикатора 0-7
int str;	//
int rs; 	// индекс команда или данные при записи в дисплей
int flenter;// флаг нажати кнопки ввод
int  flVal; // флаг изменения значения позиции курсора
//int flzn; //флаг занятости дисплея В7
int tn=0;//установленная температура нагрева
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Takt(void)
{
	LCM_OUT0 &= ~LCM_E;
	HAL_Delay(10);
	LCM_OUT0 |= LCM_E;
	HAL_Delay(10);
	LCM_OUT0 &= (~LCM_E);
	HAL_Delay(45);
}
//--------------------------------------------------------
void Write_DC(char data, int rs) //функция записии команды или данных
{
	if(rs==1) {
		LCM_OUT0=LCM_RS;
	}
	else {
		LCM_OUT0 &= ~LCM_RS;
	}

	LCM_OUT1 = data;
	Takt();
}
//-----------------------------------------------
void Read_DA (char addInd) // функция записи в память данных
{

	LCM_OUT0 |=LCM_RS;
	LCM_OUT0 |=LCM_RW;
	data=addInd;
	LCM_OUT1 = data;
		Takt();
}

//------------------------------------------------
void Cursor (char col, int str)
{

	if (str ==1) {
	addInd=0x80|col;
	LCM_OUT0 &= ~LCM_RS;
	LCM_OUT1 = addInd;
	Takt();
	}
	else {
		addInd=0x80|0x40|col;
		LCM_OUT0 &= ~LCM_RS;
		LCM_OUT1 = addInd;
		Takt();
	}

}
//-----------------------------------------
void WriteProg(void)
{
  Write_DC(0x01, 0);
  Write_DC(0xA8, 1);//П
  Write_DC(0x70, 1);//р
  Write_DC(0x6F, 1);//о
  Write_DC(0xB4, 1);//г
  Write_DC(0x20, 1);//
  Write_DC(0x4E, 1);//N
  Write_DC(0x3A, 1);//:
  Write_DC(0x5F, 1);//_

  Cursor (0x01, 2);
  Write_DC(0x31, 1);//1
  Cursor (0x03, 2);
  Write_DC(0x32, 1);//2
  Cursor (0x05, 2);
  Write_DC(0x33, 1);//3
}

//------------------------------------------------------
void Dsp1(void)
{
	  Write_DC(0x01, 0);
	  Cursor (0x01, 1);
	  Write_DC(0x74, 1);//t
	  Write_DC(0xBD, 1);//н
	  Write_DC(0x3D, 1);//=
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0


	  Cursor (0x01, 2);
	  Write_DC(0x74, 1);//t
	  Write_DC(0x79, 1);//у
	  Write_DC(0x3D, 1);//=
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0
	  Write_DC(0x30, 1);//0
}
//-------------------------------------------------------
void InitializeLCD(void)
{
    LCM_OUT1 &= ~(LCM_MASK);
    HAL_Delay(100);
    LCM_OUT0 &= ~LCM_RS;
    LCM_OUT0 &= ~LCM_RW;
    LCM_OUT0 &= ~LCM_E;

    Takt();
    Write_DC(0x3C, 0);
    Write_DC(0x3C, 0);
    Write_DC(0x0E, 0);
    Write_DC(0x01, 0);
    Write_DC(0x06, 0);
}
//------------------------------


/*void Ogidan(void) //функция ожидания сброса флага В7
{
	GPIOB->MODER=0x00;
		LCM_OUT0 &=~LCM_RS;
		LCM_OUT0 |=LCM_RW;
		Takt();
	while (flzn==1)
	{
		if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)==0)  {flzn=0;
		}

	}

}*/

/*________________________________
void ReadPos (void)  //чтение смивола в позиции курсора и перезапись его в измененном виде
{

	while ( flVal==1) //пока не нажата кнопка ввод работает цикл по изменению показаний сегмента
		{
		//void Ogidan();

			data=addInd;

		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)==0)  //увеличение значения
		{ HAL_Delay(1000);
			//GPIOB->MODER &= switchi_mask; //установка портаB как вход
			HAL_Delay(40);
			LCM_OUT0 &=~LCM_RS;  //Команда считать данны с ячейки на которой находится курсор
			LCM_OUT0 |=LCM_RW;
			LCM_OUT1 = 0x40|col;
			Takt();
			GPIOB->MODER &= switchi_mask; //установка портаB как вход
			HAL_Delay(40);
			ValRead = GPIOB->IDR;  //считываем в переменную содержание входного регистра
			HAL_Delay(100);
			HAL_Delay(100);

					// ValRead = ValRead+1;

					 GPIOB->MODER |= (0b10 <<0)|(0b10 <<2)|(0b10 <<4)|(0b10 <<6)|(0b10 <<8)|(0b10 <<10)|(0b10 <<12)|(0b10 <<14);// переключаем порт на выход
					 HAL_Delay(40);
					 data=ValRead;				//
					 Write_DC(data, 1); //записываем данные в сигмент на которм стоит курсор, rs уже установлен на 1


		}
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)==0)
		{ HAL_Delay(800);
			LCM_OUT0 |=LCM_RS;
			LCM_OUT0 |=LCM_RW;
			data=addInd;
			Takt();

			GPIOB->MODER &= switchi_mask; //установка портаB как вход
			HAL_Delay(40);

						ValRead= GPIOB->IDR;      //считываем в переменную содержание входного регистра
						HAL_Delay(100);

		ValRead=ValRead-1;
		HAL_Delay(100);
		GPIOB->MODER |=switcho_mask;// переключаем порт на выход
			data=ValRead;				//
			HAL_Delay(100);
			Write_DC(data, 1); //записываем данные в сигмент на которм стоит курсор, rs уже установлен на 1

		}



	if(HAL_GPIO_ReadPin(GPIOC, B1_Pin)==0)
		{
		 HAL_Delay(800);
		flVal=0;
		}
	}
}
*/
//--------------------------------------------
void SvigKursorKnop (void) // перемещение курсора вправо влево предварительно была нажата кнопка butEntr
{
	while( flenter==1)				//Пока не будет нажата РС7 выполняется проверка на сдвиг
	{
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)==0)
		{
			//перемещение влево
			 HAL_Delay(800);
			  Cursor ( col, 1);
			  HAL_Delay(500);
               if (col<7){
            		   col++;}
               else {col=0;}
		}

	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)==0)
		{			//перемещение вправо

		 HAL_Delay(800);
		  Cursor ( col, 2);
					  HAL_Delay(500);
		               if (col<7){
		            		   col++;}
		               else {col=0;}
		}
	if(HAL_GPIO_ReadPin(GPIOC, B1_Pin)==0){			//выход из подпрограммы
		flenter=0;}
	}
}
//_________________________________________________
void ChVal (void)
{
	int tn1=1;
	int tn2=0;
	int tn3=0;
	char tN1;
	char tN2;
	char tN3;
while(flVal==1)
	{

		if(addInd==(0x80|0x06))
			{
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)==0)
				{
				HAL_Delay(1000);
				if(tn1<=9)
					{

					tN1=0x30|tn1;
					Write_DC(tN1, 1);// единицы температуры
					tn1=tn1+1;
					Cursor (0x06, 1);
					}
				else
					{
					tn1=1;
					Write_DC(0x30, 1);
					tn2=tn2+1;
					if(tn2<=9)
						{
						tN2=0x30|tn2;
						Cursor (0x05, 1);
						Write_DC(tN2, 1);// десятки температуры
						Cursor (0x06, 1);
						}
					else
						{
						tn2=1;
						Cursor (0x05, 1);
						Write_DC(0x30, 1);
						tn3=tn3+1;
						if(tn3<=9)
							{

							tN3=0x30|tn3;
							Cursor (0x04, 1);
							Write_DC(tN3, 1);// десятки температуры

							Cursor (0x06, 1);
							}
							else
							{
								tn3=0;
							}
						}
					}


				}
			}
		if(HAL_GPIO_ReadPin(GPIOC, B1_Pin)==0)
		{
			flVal=0;
		}
		}
		tn=100*tn3+10*tn2+tn1;
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
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  InitializeLCD();
   WriteProg();

  /* static uint8_t pData[8]={};

     int i=0;
     int r=0;

     while(1){
     for (;i<8;i++){
       while ((USART3->ISR & USART_ISR_RXNE)==0){}
        pData[i]=USART3->RDR;

     }
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay (100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_Delay (100);


       if ( pData[0]==01){
       r=1;
       i=0;
       uint32_t d=GPIOB->ODR;
      // d=d & 0x000000FF;
       pData[3]=d;

       }

        while ((i<8) & (r==1)){
        if((USART3->ISR & USART_ISR_RXNE)==0)
        {

      	 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      	 //HAL_Delay (100);
        	  	USART3->TDR =pData[i];
        	  i++;

        	HAL_Delay (1);
        }
         }
         r=0;
         i=0;


       }*/

  /* USER CODE END 2 */
   HAL_Delay(5000);
  	  Dsp1();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while  (1)

  {
    /* USER CODE END WHILE */
	  if(HAL_GPIO_ReadPin(GPIOC, B1_Pin)==0)
	  {

		  HAL_Delay(200);
		  flenter=1;

		 SvigKursorKnop ();
		 HAL_Delay(200);

    /* USER CODE BEGIN 3 */
	  }


	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12)==0)
	  {
		  flVal=1;
	  		HAL_Delay(200);
	  		ChVal ();
	  }

  /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
