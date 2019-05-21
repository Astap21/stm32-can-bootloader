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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "flashRead.h"

#define  WAIT_HOST    0
#define IDLE        1
#define PAGE_PROG    2


// Flash configuration
#define MAIN_PROGRAM_START_ADDRESS              (uint32_t)0x08003000
#define BOOTLOADER_PROGRAM_START_ADDRESS        (uint32_t)0x08000000
#define MAIN_PROGRAM_PAGE_NUMBER                8
#define NUM_OF_PAGES                            (64)

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
uint32_t                      PageCRC;
uint32_t                      CalculatedCrc;
volatile uint8_t              blState;

uint8_t                       slaveId = 0;
uint32_t                      slaveIdAdrressInFlash = 0x08002FFC;
uint32_t                      deviceCanId = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/*
  Name  : CRC-32
  Poly  : 0x04C11DB7    x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11
                       + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1
  Init  : 0xFFFFFFFF
  Revert: true
  XorOut: 0xFFFFFFFF
  Check : 0xCBF43926 ("123456789")
  MaxLen: 268 435 455 байт (2 147 483 647 бит) - обнаружение
   одинарных, двойных, пакетных и всех нечетных ошибок
*/
uint32_t Crc32(const uint8_t *buf, uint32_t len);
void JumpToApplication(void);
void TransmitResponsePacket(uint8_t response);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  slaveId = (uint8_t)flash_read(slaveIdAdrressInFlash);
	if (slaveId == 0xFF || slaveId > 0xF) slaveId = 0;
	deviceCanId = 0x780 + slaveId;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();

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
  HAL_Delay(500);
	
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
  hcan1.pTxMsg->ExtId = deviceCanId;
  hcan1.pTxMsg->DLC = 8;
	hcan1.pTxMsg->IDE = CAN_ID_EXT;
  hcan1.pTxMsg->Data[0] = response;
  HAL_CAN_Transmit(&hcan1,1);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
  // Skip messages not intended for our device
  if (CanHandle->pRxMsg->StdId != deviceCanId && CanHandle->pRxMsg->ExtId != deviceCanId) {
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
      CalculatedCrc = Crc32(PageBuffer, FLASH_PAGE_SIZE);
      if (CalculatedCrc == PageCRC && PageIndex <= NUM_OF_PAGES)
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
const uint32_t Crc32Table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
    0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
    0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
    0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
    0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
    0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
    0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
    0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
    0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
    0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
    0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
    0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
    0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
    0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
    0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
    0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
    0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

uint32_t Crc32(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    while (len>0){
			len--;
      crc = (crc >> 8) ^ Crc32Table[(crc ^ *buf++) & 0xFF];
		}
		crc = crc ^ 0xFFFFFFFF;
    return crc;
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
