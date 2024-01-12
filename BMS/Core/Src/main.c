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
#define CCS_CAN_ID 0x18FF50E5 // ID of message we should recive from Charger with it's current Temperature, Charging Current, Charging Voltage and SoC

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
/*Hardware Faliure flags*/
#define NORMAL 0
#define HARDWARE_FALIURE 1
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);

/* USER CODE BEGIN PFP */
void SendCANMessage(unsigned long id, uint8_t* data, int length);
uint8_t ReceiveChargerCANMessage(unsigned long id);
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
float soc;
float hottestCellTemperature;
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

  /* USER CODE END 2 */

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

}

uint8_t ReceiveChargerCANMessage(unsigned long id){

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

    receiveChargerCANMessage(CCS_CAN_ID);

    targetMaxCurrent = calculateChargingCurrent(hottestCellTemperature, targetMaxVoltage);

    int chargingState = getChargingState(soc, hottestCellTemperature, chargingCurrent, chargingVoltage);

    uint8_t CAN_data[CAN_MSG_DLC];
    EncodeDataForCAN(CAN_data);

    if(chargingState == CHARGING_STATE_ON){
        CAN_data[CHARGING_STATE_INDEX] = CHARGING_STATE_ON;
        SendCANMessage(BMS_CAN_ID, CAN_data, CAN_MSG_DLC);
    }

    else{
        CAN_data[CHARGING_STATE_INDEX] = CHARGING_STATE_OFF;
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


bool IsHardwareFaluireFlagSet(uint8_t chargerFlags){
    if ((chargerFlags & HARDWARE_FAILURE_FLAG) == HARDWARE_FAILURE_FLAG){
        return HARDWARE_FALIURE; // Hardware faliure flag is set
    }

    return NORMAL; //Hardware faliure flag is not set
}

bool IsTemperatureFlagSet(uint8_t chargerFlags) {
    if ((chargerFlags & TEMPERATURE_OF_CHARGER_FLAG) == TEMPERATURE_OF_CHARGER_FLAG) {
        return OVER_TEMPERATURE_PROTECTION; // Temperature flag is set
    }

    return NORMAL_TEMPERATURE; // Temperature flag is not set
}

bool IsInputVoltageFlagSet(uint8_t chargerFlags) {
    if ((chargerFlags &  INPUT_VOLTAGE_FLAG) == INPUT_VOLTAGE_FLAG) {
        return WRONG_INPUT_VOLTAGE; // Input voltage flag is set
    }

    return NORMAL_INPUT_VOLTAGE; // Input voltage flag is not set
}

bool IsStartingStateFlagSet(uint8_t chargerFlags) {
    if ((chargerFlags & STARTING_STATE_FLAG) == STARTING_STATE_FLAG) {
        return BATTERY_NOT_DETECTED; // Starting state flag is set
    }

    return BATTERY_DETECTED; // Starting state flag is not set
}

bool IsCommunicationFlagSet(uint8_t chargerFlags) {
    if ((chargerFlags & COMMUNICATION_STATE_FLAG)== COMMUNICATION_STATE_FLAG) {
        return COMMUNACTION_TIMED_OUT; // Communication flag is set
    }

    return NORMAL_COMMUNICATION; // Communication flag is not set
}

int calculateChargingCurrent(int temperature, int voltage){
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
