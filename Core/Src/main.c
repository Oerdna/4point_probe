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
  *Author: 		Andrey Tsymbalyuk aka Oerdna, 2021
  *Email: 		tsymbalyuk.andrej@yandex.ru
  *Education: 	Saint Petersburg Electrotechnical University "LETI"
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_PWM_VALUE	0
#define MAX_PWM_VALUE 	900

#define SIZE_OF_RX_BUFFER 	4
#define SIZE_OF_TX_BUFFER 	32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile uint8_t event_TEC;
volatile uint8_t event_RX;
//Communicate
volatile uint8_t str[SIZE_OF_TX_BUFFER] = {0};
volatile uint8_t rx_buffer[SIZE_OF_RX_BUFFER] = {0};
//Multiplexer
const uint16_t selectPins[2] = {S0_R_select_Pin, S1_R_select_Pin};
const uint16_t multpPins[2] = {m20V_Pin, m50V_Pin};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t MAX6675_READ_TERMO(SPI_HandleTypeDef *hspi);
static uint16_t MCP3201_READ(SPI_HandleTypeDef *hspi);
static int computePID(uint16_t input, uint16_t setpoint,
		float kp, float ki, float kd,
		int minOut, int maxOut);
static void selectMuxPin(int pins, uint16_t *s_pins);
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

	//Measurement IV res
	uint16_t spi_mcp1;
	uint16_t spi_mcp2;
	uint8_t mode_multiplexer;
	uint8_t mode_multiplication;

	//Termo and TEC PID
	float kp = 22.84, ki = 2.25, kd = 3.38;
	int pulse_width;
	uint16_t pwm;
	uint16_t spi_termo;
	uint16_t termo_filtered;
	uint16_t termo_target = 120;

	//Event
	event_TEC = 0;
	event_RX = 0;

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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  //Start PWM, channel 1 & 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  //Set zero PWM to both channel
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  //Disable OUTPUT channel Multiplexer's
  HAL_GPIO_WritePin(GPIOB, E_R_select_Pin, GPIO_PIN_SET);

  //Start DAC
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  //Set DAC, 0V out
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

  //Start periodic event for TEC
  HAL_TIM_Base_Start_IT(&htim7);

  //Wait command from PC
  HAL_UART_Receive_DMA (&huart2, rx_buffer, SIZE_OF_RX_BUFFER);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (event_TEC == 2){

		  HAL_TIM_Base_Stop_IT(&htim7);
		  //Start critical section

		  //Reset event
		  event_TEC = 0;

	      HAL_GPIO_WritePin(GPIOA, SSB1_Pin, GPIO_PIN_RESET);
	      spi_termo = MAX6675_READ_TERMO(&hspi1);
	      HAL_GPIO_WritePin(GPIOA, SSB1_Pin, GPIO_PIN_SET);
	      //Low pass filter f_cut = 30 Hz
	      termo_filtered = (1 - 0.87) * termo_filtered + 0.87 * spi_termo;

	      //End critical section
	      HAL_TIM_Base_Start_IT(&htim7);

	      //Compute PID Output PWM
	      pulse_width = computePID(termo_filtered, termo_target,
	    		  kp, ki, kd,
				  -MAX_PWM_VALUE, MAX_PWM_VALUE);

	      //Set PWM value
	      if (pulse_width >= 0){
	    	  pwm = (uint16_t)pulse_width;
	    	  //Check First Frozen!
	    	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	    	  HAL_Delay(20);
	    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
	      }
	      if (pulse_width < 0){
	    	  pwm = (uint16_t)abs(pulse_width);
	    	  //Check First Heat!
	    	  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	    	  HAL_Delay(20);
	    	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm);
	      }
	      /*	Debug section
	      char debug[32] = {0};
	      sprintf(debug, "TC termo: %d\r\n", termo_filtered);
		  HAL_UART_Transmit(&huart2, (char*) debug, sizeof(debug),10);
		  */
	  }
	  // Receive body
	  if (event_RX == 1){
		  event_RX = 0;
	      HAL_UART_Transmit(&huart2, "Rx_OK!\r\n", sizeof("Rx_OK!\r\n"), 10);
	      HAL_Delay(1);

	      /*
	       *	Receive commands are divided into:
	       *
	       *	'1' - Enable|Disable pin PWM TEC,
	       *
	       *	'2' - Send|Set temperature,
	       *
	       *	'3' - Enable|Disable pins for measurement,
	       *
	       *	'4' - Manage measurement,
	       *
	       *	'5' - Take measurement,
	       *
	       *	'other' - Error handler;
	       */

	      switch (rx_buffer[0]){
	      	  case '1':
	      		  /*	Enable|Disable pin PWM TEC;
	      		   *	In this case available command:
	      		   *
	      		   *	'y' - ENABLE PID Termo control,
	      		   *		CALLED by the 2 byte in Receive array,
	      		   *		SET PIN TECswitch_Pin,
	      		   *		RETURN COMPLETE Response - 'TEC_PIN_SET';
	      		   *
	      		   *	'n' - DISABLE PID Termo control,
	      		   *		CALLED by the 2 byte in Receive array,
	      		   *		RESET PIN TECswitch_Pin,
	      		   *		RETURN COMPLETE Response - 'TEC_PIN_RESET';
	      		   *
	      		   *	'r' - READ and SEND PWM value,
	      		   *		CALLED by the 3 byte in Receive array,
	      		   *		RETURN PWM_TEC 'dddd',
		    	   * 		dddd - value of width of PWM in ASCII,
		    	   * 		It CALL immediately after switch Termo control;
	      		   */
		    	  if (rx_buffer[1] == 'y'){
		    		  HAL_GPIO_WritePin(GPIOB, TECswitch_Pin, GPIO_PIN_SET);
		    		  HAL_UART_Transmit(&huart2, "TEC_PIN: SET\r\n", sizeof("TEC_PIN: SET\r\n"),10);
		    	  } else if (rx_buffer[1] == 'n'){
		    		  HAL_GPIO_WritePin(GPIOB, TECswitch_Pin, GPIO_PIN_RESET);
		    		  HAL_UART_Transmit(&huart2, "TEC_PIN: RESET\r\n", sizeof("TEC_PIN: RESET\r\n"),10);
		    	  }
		    	  if (rx_buffer[2] == 'r'){
		    		  memset(str, 0, sizeof(str));
		    		  sprintf(str, "PWM_TEC: %d\r\n", pulse_width);
		    		  HAL_UART_Transmit(&huart2, str, sizeof(str),10);
		    	  }
		    	  break;

	      	  case '2':
		    	  /*	Send|Set temperature;
		    	   * 	In this case available command:
		    	   *
		    	   * 	'r' - READ and SEND termo value,
		    	   * 		CALLED by the 2 byte in Receive array
		    	   * 		RETURN TC_Termo 'ddd',
		    	   * 		ddd - raw temperature from sensor MAX6675 in ASCII,
		    	   * 		CONVERT ddd to celsius: T celsius  = ddd/4;
		    	   *
		    	   * 	'c' - SET termo_target from COM console (aka PuTTy i.e.),
		    	   * 		CALLED by the 2 byte in Receive array
		    	   * 		SET by the last 2 ASCII byte converted to int,
		    	   * 		RECEIVE in CELSIUS M + L BYTE in ASCII,
		    	   * 		RANGE in Celsius: 1 : 95,
		    	   * 		RETURN COMPLETE Response - TC_set,
		    	   * 		Exmpl: receive 0x'2' 'c' '3' '0' - set target 30 celsius;
		    	   *
		    	   * 	'x' - SET termp_targe by any other methods (Python i.e.),
		    	   * 		CALLED by the 2 byte in Receive array
		    	   * 		SET by the uint16_t,
		    	   * 		RECEIVE in RAW TERMO (celsius *4) M + L BYTE,
		    	   * 		RANGE in RAW: 1 : 380,
		    	   * 		RETURN COMPLETE Response - TC_set,
		    	   * 		Exmpl: receive 0x32 78 01 68 - set target 90 celsius;
		    	   */
	      		  memset(str, 0, sizeof(str));
	      		  uint16_t new_termo;
	      		  if (rx_buffer[1] == 'r'){
	      			  sprintf(str, "TC_termo: %d\r\n", termo_filtered);
	      			  HAL_UART_Transmit(&huart2, str, sizeof(str),10);
	      			  break;
	      		  } else if (rx_buffer[1] == 'c'){
	      			  new_termo = atoi(rx_buffer+2);
	      			  new_termo = 4 * new_termo;
	      		  } else if (rx_buffer[1] == 'x'){
	      			  new_termo = ((uint16_t) rx_buffer[2] << 8) | rx_buffer[3];
	      		  }
      			  if (new_termo < 1) {
      				  sprintf(str, "TC_set: FAIL below zero!\r\n");
      			  } else if (new_termo > 380) {
      				  sprintf(str, "TC_set: FAIL extreme heat!\r\n");
      			  } else {
      				termo_target = new_termo;
      				sprintf(str, "TC_set: %d\r\n", termo_target);
      			  }
	      		  HAL_UART_Transmit(&huart2, str, sizeof(str),10);
			      break;

	      	  case '3':
	      		  /*	Enable|Disable pin for measurement;
	      		   *	In this case available command:
	      		   *
	      		   *	'y' - ENABLE Measurement,
	      		   *		CALLED by the 2 byte in Receive array,
	      		   *		RESET PIN E_R_select_Pin,
	      		   *		RETURN COMPLETE Response - '4P_PIN_RESET';
	      		   *
	      		   *	'n' - DISABLE Measurement,
	      		   *		CALLED by the 2 byte in Receive array,
	      		   *		SET PIN E_R_select_Pin,
	      		   *		RETURN COMPLETE Response - '4P_PIN_SET';
	      		   *
	      		   *	'i' - CHANGE Output current source:
	      		   *		CALLED by the 2 byte in Receive array,
	      		   *		SET 2 PINS to configured max current amplifier,
	      		   *		'c' - 3 byte's from console, using ASCII
	      		   *		'x' - 3 byte's uint8_t
	      		   *		4 mode connect serial resistor:
	      		   *			0 - 10 Ohm,
	      		   *			1 - 1kOhm,
	      		   *			2 - 100kOhm,
	      		   *			3 - 1MOhm;
	      		   *
	      		   *	'v' - CHANGE Multiplication:
	      		   *		CALLED by the 2 byte in Receive array,
	      		   *		SET ONE FROM 2 PINS to configured amplifier signal,
	      		   *		'c' - 3 byte's from console, using ASCII
	      		   *		'x' - 3 byte's uint8_t
	      		   *		3 mode multiplication:
	      		   *			0 - x1,
	      		   *			1 - x20,
	      		   *			2 - x50;
	      		   */
	      		memset(str, 0, sizeof(str));
	      		int multi, multpli;
	      		if (rx_buffer[1] == 'y'){	//Enable
	      			HAL_GPIO_WritePin(GPIOB, E_R_select_Pin, GPIO_PIN_RESET);
	      			HAL_UART_Transmit(&huart2, "4P_PIN: RESET\r\n", sizeof("4P_PIN: RESET\r\n"),10);
	      		} else if (rx_buffer[1] == 'n'){	//Disable
	      			HAL_GPIO_WritePin(GPIOB, E_R_select_Pin, GPIO_PIN_SET);
	      			HAL_UART_Transmit(&huart2, "4P_PIN: SET\r\n", sizeof("4P_PIN: SET\r\n"),10);
	      		} else if (rx_buffer[1] == 'i'){	//Change Output
	      			if (rx_buffer[2] == 'c'){
	      				multi = atoi(rx_buffer+3);
	      			} else if (rx_buffer[2] == 'x'){
	      				multi = rx_buffer[3];
	      			}
      				if (!(multi >= 0 && multi <= 3)){
      					sprintf(str, "4P_I_set: FAIL unk output!\r\n");
      				} else {
      					selectMuxPin(multi, selectPins);
      					mode_multiplexer = multi;
    	      			HAL_Delay(1);
      					sprintf(str, "4P_I_set: %d!\r\n", multi);
      				}
      				HAL_UART_Transmit(&huart2, str, sizeof(str),10);
	      		} else if (rx_buffer[1] == 'v'){	//Change Multiplication
	      			if (rx_buffer[2] == 'c'){
	      				multpli = atoi(rx_buffer+3);
	      			} else if (rx_buffer[2] == 'x'){
	      				multpli = rx_buffer[3];
	      			}
      				if (!(multpli >= 0 && multpli <= 2)){
      					sprintf(str, "4P_V_set: FAIL unk Multiplication!\r\n");
      				} else{
      					selectMuxPin(multpli, multpPins);
      					mode_multiplication = multpli;
    	      			HAL_Delay(1);
      					sprintf(str, "4P_V_set: %d!\r\n", multpli);
      				}
      				HAL_UART_Transmit(&huart2, str, sizeof(str),10);
	      		}
	      		break;

	      	  case '4':
	      		  /*	Manage measurement;
	      		   *	In this case available command:
	      		   *
	      		   *	'c' - set current by the DAC from console:
		    	   * 		CALLED by the 2 byte in Receive array
		    	   * 		SET by the last 2 ASCII byte converted to int,
		    	   * 		RECEIVE in Volt M + L BYTE in ASCII,
		    	   * 		RANGE in Volt: 0 : 33,
		    	   * 		RETURN COMPLETE Response - DAC_set,
		    	   * 		Exmpl: receive 0x'2' 'c' '3' '0' - set dac 3V;
		    	   * 			      		   *
	      		   *	'x' - SET current by the DAC uint8_t:
	      		   *		CALLED by the 2 byte in Receive array,
	      		   *		SET by the uint16_t,
	      		   *		RECEIVE in RAW 12 bits,
	      		   *		RANGE in RAW: 0: 4095,
	      		   *		RETURN COMPLETE Response - DAC_set,
	      		   *		Exmpl: receive 0x34 78 04 00 - set dac 1024 ~ 0.82V;
	      		   */
	      		  memset(str, 0, sizeof(str));
	      		  int dac;
	      		  if (rx_buffer[1] == 'c'){
	      			  dac = atoi(rx_buffer+2);
	      			  dac = 124 * dac;
	      		  } else if (rx_buffer[1] == 'x'){
	      			  dac = ((uint32_t) rx_buffer[2] << 8) | rx_buffer[3];
	      		  }
	      		  if (!(dac >= 0 && dac <= 4095)){
	      			  sprintf(str, "DAC_set: FAIL impossible value!\r\n");
	      		  } else{
	      			  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac);
	      			  sprintf(str, "DAC_set: %d\r\n", dac);
	      		  }
	      		  HAL_UART_Transmit(&huart2, str, sizeof(str),10);
	      		  break;

	      	  case '5':
	      		  /*	Take measurement;
	      		   * 	In this case available command:
	      		   *
	      		   * 	'o' - One shot mode:
	      		   * 		Measurement current and voltage from probes,
	      		   * 		RETURN 2 Message,
	      		   * 		1 MESSAGE ADC_Ammeter, and Mode,
	      		   * 		2's MESSAGE ADC_Voltmeter, and Mode,
	      		   *		Calculate: R = (ADC_Voltmeter/ADC_Ammeter)*
	      		   *			*(R_mode_Ammeter/ K_mode_multiplication);
	      		   *
	      		   *	'a' - Measurement multiple dimensions:
	      		   *		More useful algorithm which search optimal
	      		   *		DAC value and Modes for measurement resistance;
	      		   */
	      		  memset(str, 0, sizeof(str));
	      		  if (rx_buffer[1] == 'o'){

					  //Read SPI Ammeter
					  HAL_GPIO_WritePin(GPIOA, SSB3_Pin, GPIO_PIN_RESET);
					  spi_mcp1 = MCP3201_READ(&hspi1);
					  HAL_GPIO_WritePin(GPIOA, SSB3_Pin, GPIO_PIN_SET);

					  //Read SPI Voltmeter
					  HAL_GPIO_WritePin(SSB4_GPIO_Port, SSB4_Pin, GPIO_PIN_RESET);
					  spi_mcp2 = MCP3201_READ(&hspi1);
					  HAL_GPIO_WritePin(SSB4_GPIO_Port, SSB4_Pin, GPIO_PIN_SET);

					  sprintf(str, "ADC_Ammeter: %d, Mode: %d\r\n", spi_mcp1, mode_multiplexer);
					  HAL_UART_Transmit(&huart2, str, sizeof(str),10);
					  sprintf(str, "ADC_Voltmeter: %d, Mode: %d\r\n", spi_mcp2, mode_multiplication);
					  HAL_UART_Transmit(&huart2, str, sizeof(str),10);
	      		  } else if (rx_buffer[1] == 'a'){

	      		  }
	      		  break;

	      	  default:
	      		  /*	Error handler;	*/
	      		  HAL_UART_Transmit(&huart2, "Unk COMMAND\r\n", sizeof("Unk COMMAND\r\n"),10);

	      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 61;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1023;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 61;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1023;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 359;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, SSB3_Pin|SSB1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SSB4_GPIO_Port, SSB4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TECswitch_Pin|S1_R_select_Pin|S0_R_select_Pin|E_R_select_Pin
                          |m20V_Pin|m50V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SSB3_Pin SSB1_Pin */
  GPIO_InitStruct.Pin = SSB3_Pin|SSB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SSB4_Pin TECswitch_Pin */
  GPIO_InitStruct.Pin = SSB4_Pin|TECswitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_R_select_Pin S0_R_select_Pin E_R_select_Pin m20V_Pin
                           m50V_Pin */
  GPIO_InitStruct.Pin = S1_R_select_Pin|S0_R_select_Pin|E_R_select_Pin|m20V_Pin
                          |m50V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static uint16_t MAX6675_READ_TERMO(SPI_HandleTypeDef *hspi){
	//Conversion Time 0.2 s - Impotent
	//buffer
	uint8_t data[2];

	// For blocking tread:
	//while ((HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY));
	HAL_SPI_Receive(hspi, data, 2, 5);

	// convert uint8_t to uint16_t
	uint16_t res = ((uint16_t)data[0] << 8) | data[1];
	if (res & 4) return 0xFFFF;
	res >>= 3;
	return res;
}


static uint16_t MCP3201_READ(SPI_HandleTypeDef *hspi){
	//buffer
	uint8_t data[2];

	// For blocking tread:
	//while ((HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY));
	HAL_SPI_Receive(hspi, data, 2, 5);

	// convert uint8_t to uint16_t
	uint16_t res = ((uint16_t)data[0] << 8) | data[1];
	res >>= 1;
	return res & 0x0FFF;
}

static int computePID(uint16_t input, uint16_t setpoint,
		float kp, float ki, float kd,
		int minOut, int maxOut){

	static float I = 0, prevError = 0;

	// Compute all the working error variables
	float error = setpoint - input;

	// Integer part
	I = I + (prevError + error) / 2;
	if (I < minOut){
		I = (float) minOut;
	}
	if (I > maxOut){
		I = (float) maxOut;
	}

	// Derivative part
	float D = (error - prevError);

	// Remember some variables for next time
	prevError = error;

	// Compute PID Output
	int out_pid = (error * kp + I / ki + D * kd);
	if (out_pid < minOut) return minOut;
	if (out_pid > maxOut) return maxOut;
	return out_pid;
}

static void selectMuxPin(int pins, uint16_t *s_pins)
{
  for (int i=0; i < 2; i++)
  {
    if (pins & (1<<i))
      HAL_GPIO_WritePin(GPIOB, *(s_pins+i), GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(GPIOB, *(s_pins+i), GPIO_PIN_RESET);
  }
}


// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim7 )
  {
	  event_TEC++;
  }
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart){

	event_RX++;
	HAL_UART_Receive_DMA (&huart2, rx_buffer, SIZE_OF_RX_BUFFER);

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
