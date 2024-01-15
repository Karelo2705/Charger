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
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BMS_CAN_ID 0x1806E5F4 // ID of message we send to Charger with target Voltage, Current and charging state
#define CCS_CAN_ID 0x18FF50E5 // ID of message we should receive from Charger with it's current Temperature, Charging Current, Charging Voltage and SoC

#define SOC_THRESHOLD 90
#define CAN_MSG_DLC 8
#define CHARGING_STATE_OFF 1
#define CHARGING_STATE_ON 0

/*Data indexes for message BMS sends to Charger*/
#define MAX_VOLTAGE_HIGH_BYTE_INDEX 0
#define MAX_VOLTAGE_LOW_BYTE_INDEX 1
#define MAX_CURRENT_HIGH_BYTE_INDEX 2
#define MAX_CURRENT_LOW_BYTE_INDEX 3
#define CHARGING_STATE_INDEX 4

/*Data indexes for message BMS receives from Charger*/
#define DATA_VOLTAGE_HIGH_BYTE_INDEX 0
#define DATA_VOLTAGE_LOW_BYTE_INDEX 1
#define DATA_CURRENT_HIGH_BYTE_INDEX 2
#define DATA_CURRENT_LOW_BYTE_INDEX 3
#define DATA_STATUS_FLAGS_INDEX 4

/*Indexes of flags received form Charger*/
#define HARDWARE_FAILURE_FLAG 0
#define TEMPERATURE_OF_CHARGER_FLAG 1
#define INPUT_VOLTAGE_FLAG 2
#define STARTING_STATE_FLAG 3
#define COMMUNICATION_STATE_FLAG 4

/*Definition of flags*/
/*Hardware Failure flags*/
#define NORMAL 0
#define HARDWARE_FAILURE 1
/*Temperature of Charger flags*/
#define NORMAL_TEMPERATURE 0
#define OVER_TEMPERATURE_PROTECTION 1
/*Input Voltage flags*/
#define NORMAL_INPUT_VOLTAGE 0
#define WRONG_INPUT_VOLTAGE 1
/*Starting State flags*/
#define BATTERY_DETECTED 0
#define BATTERY_NOT_DETECTED 1
/*Communication State flags*/
#define NORMAL_COMMUNICATION 0
#define COMMUNACTION_TIMED_OUT 1

#define MAX_TEMPERATURE 40
#define MAX_CHARGING_CURRENT 20
#define MAX_CHARGING_VOLTAGE 50
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef canfilterconfig;
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;

uint32_t TxMailbox;

uint8_t TxData[8];
uint8_t RxData[8];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);

/* USER CODE BEGIN PFP */
void SendCANMessage(unsigned long id, uint8_t* data, int length);
void ReceiveChargerCANMessage(unsigned long id);
int GetChargingState(float soc, float temperature, float chargingCurrent, float chargingVoltage);
void ChargingStateAlgorithm();
void EncodeDataForCAN(uint8_t *dataForCAN);
void DecodeChargerMessage(uint8_t *dataFromCharger);
bool IsHardwareFaluireFlagSet(uint8_t chargerFlags);
bool IsTemperatureFlagSet(uint8_t chargerFlags);
bool IsInputVoltageFlagSet(uint8_t chargerFlags);
bool IsStartingStateFlagSet(uint8_t chargerFlags);
bool IsCommunicationFlagSet(uint8_t chargerFlags);
int calculateChargingCurrent(int temperature, int voltage);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t chargerFlags; //flag byte received from Charger

/*Variables sent to Charger*/
float targetMaxCurrent = 58.2;
float targetMaxVoltage = 320.1;

/*Variables received from Charger */
float chargingCurrent;
float chargingVoltage;

/*Variables received from different sources*/
float soc = 20;
float hottestCellTemperature = 50;
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  ChargingStateAlgorithm();
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SendCANMessage(unsigned long id, uint8_t* data, int length) {
	  TxHeader.IDE = CAN_ID_STD;
	  TxHeader.StdId = id;
	  TxHeader.RTR = CAN_RTR_DATA;
	  TxHeader.DLC = length;
	  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK)
	  	{
	  	   Error_Handler ();
	  	}
}

void ReceiveChargerCANMessage(unsigned long id){
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	  canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
	  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  canfilterconfig.FilterIdHigh = id<<5;
	  canfilterconfig.FilterIdLow = 0;
	  canfilterconfig.FilterMaskIdHigh = id<<5;
	  canfilterconfig.FilterMaskIdLow = 0x0000;
	  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

	  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

	  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

int GetChargingState(float soc, float temperature, float chargingCurrent, float chargingVoltage){
    if (soc < SOC_THRESHOLD) {

        if (temperature > MAX_TEMPERATURE) {
        return CHARGING_STATE_OFF;
        }

        if (chargingCurrent > MAX_CHARGING_CURRENT){
        return CHARGING_STATE_OFF;
        }

        if(chargingVoltage > MAX_CHARGING_VOLTAGE) {
        return CHARGING_STATE_OFF;
        }

        else {
        return CHARGING_STATE_ON;
        }

    }

    else {
        return CHARGING_STATE_OFF;
    }

}

void ChargingStateAlgorithm() {

    ReceiveChargerCANMessage(CCS_CAN_ID);
    DecodeChargerMessage(RxData);

    int chargingState = GetChargingState(soc, hottestCellTemperature, chargingCurrent, chargingVoltage);

    uint8_t CAN_data[CAN_MSG_DLC];

    if(chargingState == CHARGING_STATE_ON){
        CAN_data[CHARGING_STATE_INDEX] = CHARGING_STATE_ON;
        EncodeDataForCAN(CAN_data);
        SendCANMessage(BMS_CAN_ID, CAN_data, CAN_MSG_DLC);
    }

    else{
        CAN_data[CHARGING_STATE_INDEX] = CHARGING_STATE_OFF;
        EncodeDataForCAN(CAN_data);
        SendCANMessage(BMS_CAN_ID, CAN_data, CAN_MSG_DLC);
    }

}

void EncodeDataForCAN(uint8_t *dataForCAN){

    uint16_t voltageInt = (uint16_t)floor((targetMaxVoltage * 10));
    uint16_t currentInt = (uint16_t)floor((targetMaxCurrent * 10));

    uint8_t voltageShort = (uint8_t)voltageInt;
    uint8_t currentShort = (uint8_t)currentInt;


    dataForCAN[MAX_VOLTAGE_HIGH_BYTE_INDEX] = (voltageShort >> 8) & 0xFF;
    dataForCAN[MAX_VOLTAGE_LOW_BYTE_INDEX] = voltageShort & 0xFF;
    dataForCAN[MAX_CURRENT_HIGH_BYTE_INDEX] = (currentShort >> 8) & 0xFF;
    dataForCAN[MAX_CURRENT_LOW_BYTE_INDEX] = currentShort & 0xFF;
}


void DecodeChargerMessage(uint8_t *dataFromCharger){

    uint16_t voltageInt = ((uint16_t)dataFromCharger[DATA_VOLTAGE_HIGH_BYTE_INDEX] << 8) | dataFromCharger[DATA_VOLTAGE_LOW_BYTE_INDEX];
    uint16_t currentInt = ((uint16_t)dataFromCharger[DATA_CURRENT_HIGH_BYTE_INDEX] << 8) | dataFromCharger[DATA_CURRENT_LOW_BYTE_INDEX];

    chargingVoltage = voltageInt * 0.1;
    chargingCurrent = currentInt * 0.1;

    chargerFlags = dataFromCharger[DATA_STATUS_FLAGS_INDEX];
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
