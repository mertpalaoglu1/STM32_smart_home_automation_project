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

#include "liquidcrystal_i2c.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_7

uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
uint8_t TFI = 0;
uint8_t TFD = 0;
char strCopy[15];
uint16_t sayac;
uint16_t mesafe=0;

#define SERVO_ANGLE_0 500  // KAPI için 0 derece için uygun PWM pulse genisligi
#define SERVO_ANGLE_90 1000 // Kapi için 90 derece için uygun PWM pulse genisligi
#define SERVO_ANGLE_180 1299 // Kapi için 180 derece için uygun PWM pulse genisligi

uint16_t waterValue; //su sensörünü okumasi için
uint16_t lightValue;//isik sensörü okumasi için deger atadim
uint16_t gasValue; // gaz sensörü degeri

uint8_t waterFlag=0; //su sensörü flagi
uint8_t gasFlag=0; //gaz flagi
uint8_t ldrFlag=0; // isik flagi
uint8_t mesafeFlag=0; //pir flagi
uint8_t kilitliMi=0; //kapi için kilit.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc2){ //Su sensörü için.
waterValue= HAL_ADC_GetValue(hadc2);
	if(waterValue>150){
		waterFlag=1;
	}else{
		waterFlag=0;
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void kilitFonksiyonu(){
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)){ //kitle tusu
			kilitliMi=1;
			mesafeFlag=0;
		}
		else{ //5 sifir iken açilacak
			kilitliMi=0;
		}
}

void perdeAc(){
	TIM2->CCR2=SERVO_ANGLE_0;
} //manuel açma kapamalar için.

void perdeKapa(){
	 TIM2->CCR2=SERVO_ANGLE_180;
}
void Gas_System(){ //Pir sensörü gelince kapinin kendine ait kontrolu olacak.
    GPIO_PinState digital_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); // Dijital degeri oku
    // Dijital degeri kontrol et
    if (digital_value == 0) // Eger dijital pin HIGH durumdaysa
    {
				gasFlag=1;
				TIM2->CCR1=SERVO_ANGLE_90;
				  HD44780_Clear();
				  HD44780_SetCursor(0,0);
				  HD44780_PrintStr("GAZ KACAGI VAR");
				  HAL_Delay(1000);
				  TIM2->CCR2=SERVO_ANGLE_90;
           // Servo motoru 90 dereceye döndür
        // LED'i yak
        for (int i = 0; i < 40; i++)
        {

        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_6); // ACIL DURUM LED 1'i yak , Buzzer'i aç. , ACIL DURUM LED 2'i yak
            HAL_Delay(500);
        }
    }
    else
    {

    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
				gasFlag=0;
        TIM2->CCR1=SERVO_ANGLE_0; // Servo motoru 0 dereceye döndür
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // ACIL DURUM LED1'i söndür
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // ACIL DURUM LED2'i söndür
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);// Buzzer'i kapat
    }
}

void isikKontrol(){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100); //LDR okudugu veri bozulmasin diye eoc flag kontrolü yapiyoruz.
    lightValue = HAL_ADC_GetValue(&hadc1);

		if(lightValue<1500){ //ara deger bulanacak!
			ldrFlag=0;
		}
		else{
			ldrFlag=1;
		}
}

void perdeKontrol(){
	if(gasFlag==1){ //gaz varsa her türlü açicak.

		perdeAc();
	}
	else if (ldrFlag==1){ //gaz yoksa dis mekandan daha fazla isik varsa kapanacak
		  HD44780_Clear();
		  HD44780_SetCursor(0,0);
		  HD44780_PrintStr("HAVA GUNESLI");
		 HAL_Delay(100);
		perdeKapa();

	}
	else {
		  HD44780_Clear();
		  HD44780_SetCursor(0,0);
		  HD44780_PrintStr("HAVA KAPALI");
		  HAL_Delay(100);
		perdeAc(); //ne gaz varsa ne dis mekan daha isikli ise perde açilacak.

	}
}
/*void pir_kontrol()
	{
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
			pirFlag=1;
		}

	}*/

void kapiKontrol(){
	 if(kilitliMi==0){
			TIM2->CCR1=SERVO_ANGLE_0; // kilit varsa kapı açilmaz.
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0); //DIS ISIK AC.


	  }
	else if(kilitliMi==1 && mesafeFlag==1){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);//DIS ISIK ac.
		TIM2->CCR1=SERVO_ANGLE_90;// Servo motoru 90 dereceye döndür, kapiyi aç
			HAL_Delay(2000);
		TIM2->CCR1=SERVO_ANGLE_0; //kapiyi kapayacak.
		  HD44780_Clear();
		  HD44780_SetCursor(0,0);
	    HD44780_PrintStr("HOSGELDIN SUHA");
		HAL_Delay(3000);


		// 5 sn bekle
		//TIM2->CCR1=SERVO_ANGLE_0; //kapiyi kapat.
		//pirFlag=0;
	}

	 /*  for(int x=0; x<20; x=x+1)
	   {
	     HD44780_ScrollDisplayLeft();  //HD44780_ScrollDisplayRight();
	     HAL_Delay(300);
	   }*/
}

void icIsiklandirma(){ // A6 IÇ ISIK TUSU, B6 IÇ ISIK ÇIKISI ROLESI
	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);//ROLE ÇALISIYOR
		HAL_Delay(10);
	}
	else {
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,0);
	}
}

void suKontrol(){
	HAL_ADC_Start(&hadc2); //Su sensörü ADC'sini baslat.
	HAL_ADC_PollForConversion(&hadc1,100);
	waterValue= HAL_ADC_GetValue(&hadc2);

		if(waterValue>150){
			waterFlag=1;
		}

		else{
			waterFlag=0;
		}
}

void suAlarmKontrol(){
if(waterFlag==1 && gasFlag==0){
				 for (int i = 0; i < 20; i++)
	                {
	                   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_6); // ACIL DURUM LED 1'i yak , Buzzer'i aç. , ACIL DURUM LED 2'i yak
	                    HAL_Delay(1000);
	                }
		}
		else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);//BUZZER susacak.
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);//Acil durum isigi 1 sönecek.
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);//Acil durum isigi 2 sönecek.
			}
}

void microDelay (uint16_t delay){
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

void ekran_calistirma (){
	char snum[16];
	char snum1[16];
	char snum4[16];
	char snum5[16];
      HD44780_Clear();
	  itoa(TCI, snum, 10);
	  itoa(TCD, snum1, 10);
	  itoa(RHI, snum4, 10);
	  itoa(RHD, snum5, 10);
while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0){
	  HD44780_SetCursor(0,0);
	  HD44780_PrintStr(snum);
	  HD44780_SetCursor(2,0);
	  HD44780_PrintStr(".  C");
      HD44780_SetCursor(3,0);
	  HD44780_PrintStr(snum1);
	  HD44780_SetCursor(0,1);
	  HD44780_PrintStr(snum4);
	  HD44780_SetCursor(2,1);
	  HD44780_PrintStr(".  %");
      HD44780_SetCursor(3,1);
	  HD44780_PrintStr(snum5);
	  HAL_Delay(1000);
}
}
void mesafeKontrol(){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);

		while( !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14));
		sayac=0;
		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)){
		sayac++;
		}
		mesafe=sayac/25;
		HAL_Delay(60);

		if(mesafe<50){
		mesafeFlag=1;
		}
		else{
			mesafeFlag=0;
		}
}

/*void ekran_calistirma_2 (){
	char snum[16];
	char snum1[16];
	char snum2[16];
	  HD44780_Clear();
	  itoa(lightValue, snum, 10);
	  itoa(gasValue, snum1, 10);


	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==0){
		  //isikKontrol();
		  HD44780_SetCursor(0,0);
		  HD44780_PrintStr(snum);
		  HD44780_SetCursor(0,1);
		  HD44780_PrintStr(snum1);

	}
}*/

//NEM VE SİCAKLİK OLCUM KODLARİ
uint8_t DHT11_Start (void){
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read (void){
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
HD44780_Init(2);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //  KAPI Timer'i için 1.kanali aktif ediyor.
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // PERDE için timer.
HAL_TIM_Base_Start(&htim1); //sicaklik sensörü timeri


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

			Gas_System();
			isikKontrol();
			perdeKontrol();
			kilitFonksiyonu();
			mesafeKontrol();
			kapiKontrol();
			icIsiklandirma();
			  //suKontrol(); su çalismadigi için kapadim.
				// suAlarmKontrol();

			  if(DHT11_Start())
	    {
	      RHI = DHT11_Read(); // Relative humidity integral
	      RHD = DHT11_Read(); // Relative humidity decimal
	      TCI = DHT11_Read(); // Celsius integral
	      TCD = DHT11_Read(); // Celsius decimal
	      SUM = DHT11_Read(); // Check sum
	      if (RHI + RHD + TCI + TCD == SUM)
	      {
	        // Can use RHI and TCI for any purposes if whole number only needed
	        tCelsius = (float)TCI + (float)(TCD/10.0);
	        tFahrenheit = tCelsius * 9/5 + 32;
	        RH = (float)RHI + (float)(RHD/10.0);
	        // Can use tCelsius, tFahrenheit and RH for any purposes
	        TFI = tFahrenheit;  // Fahrenheit integral
	        TFD = tFahrenheit*10-TFI*10; // Fahrenheit decimal
				}
	        if (TCI <= 27) //  hava sogukken 5 saniye role ardından fan da calisir
	              {

	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);     //  tasdirenc
	        		//HAL_Delay(5000);
	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);    //pervane
	              }
	              else if (TCI >= 29 )
	              {

	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	              }
	              /*else
	              {

	                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
	              }*/
			}
		 if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0){
			 ekran_calistirma();
                   }
		/* if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==0){
			 ekran_calistirma_2 ();
		 }*/


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 143;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC6 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB3 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
