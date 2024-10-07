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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.14159265358979
const float coxa = 2.5; // coxa length
const float femur = 8.5;// femur length
const float tibia = 12.5;// tibia length
const float xidle = 11; // x in idle state
const float yidle = 12; // y in idle state
const float zidle = 0; // z in idle state
const float R = 5; // length of 1/2 step
const float H = 5; // height of 1 step
// channel cua moi chan
const uint8_t F0 = 15,F1 = 14, F2 = 15;


const uint8_t A0 = 0, A1 = 1, A2 = 2;
const uint8_t B0 = 3, B1 = 4, B2 = 5;
const uint8_t C0 = 6, C1 = 7, C2 = 8;
const uint8_t D0 = 9, D1 = 10, D2 = 11;
const uint8_t E0 = 12, E1 = 13, E2 = 14;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PCA9685_ADDRESS 0x80
// Datasheet link --> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PCA9685_MODE1         0x0         // as in the datasheet page no 10/52
#define PCA9685_PRE_SCALE     0xFE        // as in the datasheet page no 13/52
#define PCA9685_LED0_ON_L     0x6         // as in the datasheet page no 10/52
#define PCA9685_MODE1_SLEEP_BIT      4    // as in the datasheet page no 14/52
#define PCA9685_MODE1_AI_BIT         5    // as in the datasheet page no 14/52
#define PCA9685_MODE1_RESTART_BIT    7    // as in the datasheet page no 14/52

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
  uint8_t readValue;
  // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
  HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  if (Value == 0) readValue &= ~(1 << Bit);
  else readValue |= (1 << Bit);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  HAL_Delay(1);
}


void PCA9685_SetPWMFrequency(uint16_t frequency)
{
  uint8_t prescale;
  if(frequency >= 1526) prescale = 0x03;
  else if(frequency <= 24) prescale = 0xFF;
  //  internal 25 MHz oscillator as in the datasheet page no 1/52
  else prescale = 25000000 / (4096 * frequency);
  // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}


void PCA9685_Init(uint16_t frequency)
{
  PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
  uint8_t registerAddress;
  uint8_t pwm[4];
  registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
  // See example 1 in the datasheet page no 18/52
  pwm[0] = OnTime & 0xFF;
  pwm[1] = OnTime>>8;
  pwm[2] = OffTime & 0xFF;
  pwm[3] = OffTime>>8;
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}


void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
  float Value;
  // 50 Hz servo then 4095 Value --> 20 milliseconds
  // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
  Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
  PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

/* USER CODE BEGIN 0 */

uint32_t begin = 0;
void IK(float x, float y, float z, float *degY, float *degA, float *degB) {
    //
    float H = sqrt(pow(x, 2) + pow(z, 2));
    *degY = asin(z/H) * (float)(180.0 / PI);
    float h_c = H - coxa;
    float L = sqrt(pow(h_c, 2) + pow(y, 2));

    *degB = acos((pow(tibia, 2) + pow(femur, 2) - pow(L, 2)) / (2 * tibia * femur)) * (float)(180.0 / PI);
    float i = acos((pow(L, 2) + pow(femur, 2) - pow(tibia, 2)) / (2 * L * femur)) * (float)(180.0 / PI);
    float i0 = atan2(y, h_c) * (float)(180.0 / PI);  // Correct atan2 usage

    *degA = i - i0;

    *degY = *degY + 90;
    *degA = *degA + 90;
    *degB = 180 - *degB;
}

//

//
//____ attempt : 2
uint32_t deltaTime = 1800; // milisec
short callfreq = 36; // so lan goi ham set servo trong 1 chu ki
float xe = 11, ye = 12, ze = 0;
float xb = 11, yb = 12, zb = 0;
// movin func
void Moving_xz(float alpha)
{
	begin = HAL_GetTick();
	float b0=90,b1=90,b2=90;
	float e0=90,e1=90,e2=90;
	float a0 =90,a1=90,a2=90;
	float f0=90,f1=90,f2=90;
	float c0=90,c1=90,c2=90;
	float d0=90,d1=90,d2=90;
	/* Point_rotate
 	 x' = 11 - z.sin(alpha)
 	 z' = 	 z.cos(alpha)
	*/

	// GROUP 1
	yb = fmin(yidle,H*cos((2*PI/deltaTime)*HAL_GetTick() + PI/2)+yidle); // Y_g1 (B,D,F)
	zb = R*cos((2*PI/deltaTime)*HAL_GetTick()+ PI); // Z_g1 (B,D,F)
	xb = xidle - zb*sin(-alpha*(float)(PI/180.0)); // X_R (chi su dung cho B)
	// GROUP 2
	ye = fmin(yidle,H*cos((2*PI/deltaTime)*HAL_GetTick() - PI/2)+yidle); // Y_g2 (A,C,E)
	ze = R*cos((2*PI/deltaTime)*HAL_GetTick()); // Z_g2 (A,C,E)
	xe = xidle - ze*sin(alpha*(float)(PI/180.0)); // X_L (chi sung dung cho E)

	// voi (A,C,D,F phuong trinh x se dươc tinh rieng luc truyen IK de dieu chinh alpha)
	// Cac lan goi IK duoi đay doi voi chan A,C,D,F se duoc dieu chinh rieng)

	IK(xb,yb,zb*cos(-alpha*(float)(PI/180.0)),&b0,&b1,&b2);
	IK(xe,ye,ze*cos(alpha*(float)(PI/180.0)),&e0,&e1,&e2);

	IK(xidle - ze*sin((-alpha-42)*(float)(PI/180.0)), ye,ze*cos(((-alpha-42))*(float)(PI/180.0)), &a0, &a1,&a2);//fixed
	IK(xidle - ze*sin((-alpha+42)*(float)(PI/180.0)), ye,ze*cos(((-alpha+42))*(float)(PI/180.0)), &c0, &c1,&c2);//fixed

	IK(xidle - zb*sin((alpha-42)*(float)(PI/180.0)),yb,zb*cos(((alpha-42))*(float)(PI/180.0)),&f0,&f1,&f2);//fixed
	IK(xidle - zb*sin((alpha+42)*(float)(PI/180.0)),yb,zb*cos(((alpha+42))*(float)(PI/180.0)),&d0,&d1,&d2);//fixed
//	IK(-(xidle - zb*sin(-(alpha-42)*(float)(PI/180.0))), ye,ze*cos((alpha-42)*(float)(PI/180.0)), &a0, &a1,&a2);
//	IK(xidle - ze*sin((alpha-42)*(float)(PI/180.0)),yb,zb*cos((alpha)*(float)(PI/180.0)),&f0,&f1,&f2);
//	IK(-(xidle - zb*sin(-(alpha+42)*(float)(PI/180.0))), ye,ze*cos((alpha+42)*(float)(PI/180.0)), &c0, &c1,&c2);
//	IK(xidle - ze*sin((alpha+42)*(float)(PI/180.0)),yb,zb*cos((alpha+42)*(float)(PI/180.0)),&d0,&d1,&d2);
	//	IK(xb,yidle-H,zb,&b0,&b1,&b2);
//	IK(6,yidle,4,&e0,&e1,&e2);
	while(HAL_GetTick()-begin < deltaTime/callfreq)
	{
		PCA9685_SetServoAngle(E0, e0);
		PCA9685_SetServoAngle(E1, e1);
		PCA9685_SetServoAngle(E2, e2);
		PCA9685_SetServoAngle(D0, d0);
		PCA9685_SetServoAngle(D1, d1);
		PCA9685_SetServoAngle(D2, d2);
		PCA9685_SetServoAngle(C0, 180-c0);
		PCA9685_SetServoAngle(C1, 180-c1);
		PCA9685_SetServoAngle(C2, 180-c2);



		PCA9685_SetServoAngle(B0, 180-b0);
		PCA9685_SetServoAngle(B1, 180-b1);
		PCA9685_SetServoAngle(B2, 180-b2);
		PCA9685_SetServoAngle(A0, 180-a0);
		PCA9685_SetServoAngle(A1, 180-a1);
		PCA9685_SetServoAngle(A2, 180-a2);

		PCA9685_SetServoAngle(F0, f0);
		//PCA9685_SetServoAngle_1(F1, f1);
		PCA9685_SetServoAngle(0, 90);
	}
}
void L2Pos (uint8_t cxSV, uint8_t fmSV, uint8_t tbSV, float x, float y, float z)
{
	float degY,degA,degB;
	IK(x,y,z,&degY,&degA,&degB);
	begin = HAL_GetTick();
	while(HAL_GetTick()-begin < deltaTime/callfreq)
	{
		PCA9685_SetServoAngle(cxSV, degY);
		PCA9685_SetServoAngle(fmSV, degA);
		PCA9685_SetServoAngle(tbSV, degB);
	}

}
void idle()
{
	  L2Pos(D0,D1,D2,xidle,yidle,zidle);
	  L2Pos(C0,C1,C2,xidle,yidle,zidle);
	  L2Pos(A0,A1,A2,xidle,yidle,zidle);
	  L2Pos(B0,B1,B2,xidle,yidle,zidle);
	  L2Pos(E0,E1,E2,xidle,yidle,zidle);
}

//____SET UP VAR

float angle = 0;


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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init(50); // 50Hz for servo

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //  uint8_t ser0 = 1;
  //  uint32_t begin = HAL_GetTick();
  //  while(HAL_GetTick()-begin < delaytime)
  //  {
  //
  //  }


  // BEGIN SET UP
//  SetUpTIM();
  float setuptime = 10; // (sec)

  // BEGIN SETUP


//  while(HAL_GetTick() < setuptime*1000)
//  while(HAL_GetTick() < (setuptime)*1000)
//  {
//  	// addition setup
////	  L2Pos(A0,A1,A2,xidle,yidle,zidle);
////	  L2Pos(C0,C1,C2,xidle,yidle,zidle);
////	  L2Pos(E0,E1,E2,xidle,yidle,zidle);
////	  PCA9685_SetServoAngle(B2, 45);
////	  PCA9685_SetServoAngle(E1, 90);
////	  PCA9685_SetServoAngle(E2, 90);
//	  Moving_xz(angle);
//
//  }
//  while(HAL_GetTick() < (setuptime*2)*1000)
//  {
//  	// addition setup
////  	Moving_xz(45);
//	 idle();
//  }
//  while(HAL_GetTick() < (setuptime*3)*1000)
//  {
//  	// addition setup
////  	Moving_xz(90);
//	  PCA9685_SetServoAngle(B2, 180);
//  }
//  while(HAL_GetTick() < (setuptime*4)*1000)
//  {
//  	// addition setup
//  	Moving_xz(180);
//  }
  // END SETUP

  while (1)
  {

	  PCA9685_SetServoAngle(0, 90);
//	  idle();
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
