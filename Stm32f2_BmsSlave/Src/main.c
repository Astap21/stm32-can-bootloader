/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "can.h"
#include "crc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#define  WAIT_HOST    0
#define IDLE        1
#define PAGE_PROG    2


// Flash configuration
#define MAIN_PROGRAM_START_ADDRESS              (uint32_t)0x08002000
#define BOOTLOADER_PROGRAM_START_ADDRESS        (uint32_t)0x08000000
#define MAIN_PROGRAM_PAGE_NUMBER                8
#define NUM_OF_PAGES                            (64)

// CAN identifiers
#define DEVICE_CAN_ID                            0x78E
#define CMD_HOST_INIT                            0x01
#define CMD_PAGE_PROG                            0x02
#define CMD_BOOT                                0x03

#define CAN_RESP_OK                              0x01
#define CAN_RESP_ERROR                          0x02

#define FLASH_PAGE_SIZE          ((uint32_t)0x400)
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
/* Private variables ---------------------------------------------------------*/
static CanTxMsgTypeDef        canTxMessage;
static CanRxMsgTypeDef        canRxMessage;
static FLASH_EraseInitTypeDef eraseInitStruct;

uint8_t                       PageBuffer[FLASH_PAGE_SIZE];
volatile int                  PageBufferPtr;
uint8_t                       PageIndex;
int                           PageCRC;

volatile uint8_t              blState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void JumpToApplication()
{
	//объявляем пользовательский тип
	typedef void (*pFunction)(void);
	//и создаём переменную этого типа
	pFunction JumpAddress;
	// извлекаем адрес перехода из вектора Reset
  // и приводим его к пользовательскому типу
  JumpAddress = *(__IO pFunction*)(MAIN_PROGRAM_START_ADDRESS + 4);
  //устанавливаем SP приложения  
  __set_MSP(*(__IO uint32_t*) MAIN_PROGRAM_START_ADDRESS);   
  HAL_DeInit();
	// переход в основную программу
  JumpAddress();
}

void TransmitResponsePacket(uint8_t response)
{
  hcan1.pTxMsg->StdId = DEVICE_CAN_ID;
  hcan1.pTxMsg->DLC = 1;
  hcan1.pTxMsg->Data[0] = response;
  HAL_CAN_Transmit(&hcan1,1);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
  // Skip messages not intended for our device
  if (CanHandle->pRxMsg->StdId != DEVICE_CAN_ID && CanHandle->pRxMsg->ExtId != DEVICE_CAN_ID) {
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
    return;
  }

  if (blState == PAGE_PROG)
  {
    memcpy(&PageBuffer[PageBufferPtr],
      CanHandle->pRxMsg->Data,
      CanHandle->pRxMsg->DLC);
    PageBufferPtr += CanHandle->pRxMsg->DLC;

    if (PageBufferPtr == FLASH_PAGE_SIZE) {
      HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
      HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
      HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
      HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);

      HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
      uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)PageBuffer, FLASH_PAGE_SIZE / 4);

      //if (crc == PageCRC && PageIndex <= NUM_OF_PAGES)
			if (PageIndex <= NUM_OF_PAGES)
      {
        HAL_FLASH_Unlock();

        uint32_t PageError = 0;

        eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
        eraseInitStruct.Sector = BOOTLOADER_PROGRAM_START_ADDRESS + PageIndex * FLASH_PAGE_SIZE;
        eraseInitStruct.NbSectors = 1;

        HAL_FLASHEx_Erase(&eraseInitStruct, &PageError);

        for (int i = 0; i < FLASH_PAGE_SIZE; i += 4)
        {
          HAL_FLASH_Program(TYPEPROGRAM_WORD, BOOTLOADER_PROGRAM_START_ADDRESS + PageIndex * FLASH_PAGE_SIZE + i, *(uint32_t*)&PageBuffer[i]);
        }

        HAL_FLASH_Lock();

        TransmitResponsePacket(CAN_RESP_OK);
      }
      else
      {
        TransmitResponsePacket(CAN_RESP_ERROR);
      }

      blState = IDLE;

			HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
			HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
			HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
			HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    }

    HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
    return;
  }

  switch(CanHandle->pRxMsg->Data[0])
  {
    case CMD_HOST_INIT:
      blState = IDLE;
      TransmitResponsePacket(CAN_RESP_OK);
      break;
    case CMD_PAGE_PROG:
      if (blState == IDLE) {
				// очищает страницу
        memset(PageBuffer, 0, FLASH_PAGE_SIZE);
				//контрольная сумма
        memcpy(&PageCRC, &CanHandle->pRxMsg->Data[2], sizeof(int));
				// номер страницы
        PageIndex = CanHandle->pRxMsg->Data[1] + MAIN_PROGRAM_PAGE_NUMBER;
        blState = PAGE_PROG;
        PageBufferPtr = 0;
      } else {
        // Should never get here
      }
      break;
    case CMD_BOOT:
      TransmitResponsePacket(CAN_RESP_OK);
      JumpToApplication();
      break;
    default:
      break;
  }

  HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  hcan1.pTxMsg = &canTxMessage;
  hcan1.pRxMsg = &canRxMessage;

  CAN_FilterConfTypeDef canFilterConfig;
  canFilterConfig.FilterNumber = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = 0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.BankNumber = 1;
  HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);

  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_Delay(5000);
	
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  // Timed out waiting for host
  if (blState == WAIT_HOST) {
    JumpToApplication();
  }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
