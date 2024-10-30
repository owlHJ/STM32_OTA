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
#include "crc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_PATTERN 		"\r\n:"
#define START_PATTERN_LEN 	3
#define CHUNK_SIZE 			16
#define NUM_CHUNKS 			1024
#define BUFFER_SIZE			256
#define UART_BUFFER_SIZE 1024
#define HEX_DATA_BUFFER_SIZE 1024

#define SOURCE_START_ADDRESS 0x08000000  // 원본 주소 시작 (0x08000000)
#define TARGET_START_ADDRESS 0x08010000  // 복사할 주소 시작 (0x08020000)
#define STMF32F1_MAX_ADDRESS 0x0801FFFF  // 복사할 주소 시작 (0x08020000)
#define PAGE_SIZE            0x400       // F103의 페이지 크기 (1KB)
#define NUMBER_OF_PAGES      30         // 복사할 페이지 수 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_buffer[UART_BUFFER_SIZE];
uint8_t hex_data_buffer[HEX_DATA_BUFFER_SIZE];
uint32_t data_index = 0;
uint32_t hex_data_index = 0;
uint32_t hex_body_index = 0;
bool is_body = false;  // 본문 시작 여부를 추적하는 플래그

uint8_t data_buffer[NUM_CHUNKS][CHUNK_SIZE]; // Buffer for storing received chunks
uint32_t flash_address = 0x08020000;
uint8_t chunk_index = 0, byte_index = 0;
uint8_t data_receiving = 0;


uint32_t current_flash_address = TARGET_START_ADDRESS;
uint8_t match_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void send_command(const char* command);
void receive_response(char* buffer, uint16_t length);
void flash_write(uint32_t address, uint8_t *data, uint16_t length);
void process_uart_data(void) ;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UART ?��?��?��?���? ?��?��?��?�� ?�� 줄씩 ?��?��?���?
void receive_data_via_uart()
{
    uint8_t received_byte;

    // Loop to receive bytes and fill chunks
    while (1)
    {
        // Receive one byte from UART
        HAL_UART_Receive(&huart1, &received_byte, 1, HAL_MAX_DELAY);

        // Store the received byte in the current chunk
        data_buffer[chunk_index][byte_index] = received_byte;
        byte_index++;

        // Check if the chunk is filled
        if (byte_index >= CHUNK_SIZE)
        {
            byte_index = 0;       // Reset byte index for the next chunk
            chunk_index++;        // Move to the next chunk

            // Check if we have filled all chunks
            if (chunk_index >= NUM_CHUNKS)
            {
                chunk_index = 0;  // Reset chunk index to overwrite old data
            }
        }
    }
}

void write_chunk_to_flash(uint32_t address, uint8_t *chunk, uint8_t length) {
    HAL_FLASH_Unlock();
    for (uint8_t i = 0; i < length; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i * 2, chunk[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return;
        }
    }
    HAL_FLASH_Lock();
}
void Flash_CopyPages(uint32_t srcAddress, uint32_t dstAddress, uint32_t pageSize, uint32_t numberOfPages) {
    HAL_FLASH_Unlock();  // 플래시 메모리 언락

    // 먼저 전체 저장할 공간을 지우기 위한 설정
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = dstAddress;
    EraseInitStruct.NbPages = numberOfPages;

    // 전체 대상 메모리 지우기
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        // 오류 처리
        HAL_FLASH_Lock();
        return;
    }

    // 페이지 단위로 원본에서 대상 메모리로 데이터 복사
    for (uint32_t i = 0; i < numberOfPages; i++) {
        uint32_t srcPageAddress = srcAddress + i * pageSize;
        uint32_t dstPageAddress = dstAddress + i * pageSize;

        // 원본에서 대상 메모리로 데이터 복사
        for (uint32_t offset = 0; offset < pageSize; offset += 4) {
            uint32_t data = *(__IO uint32_t *)(srcPageAddress + offset);  // 원본 데이터 읽기

            // 데이터를 대상 메모리에 쓰기
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dstPageAddress + offset, data) != HAL_OK) {
                // 오류 처리
                HAL_FLASH_Lock();
                return;
            }
        }
    }

    HAL_FLASH_Lock();  // 플래시 메모리 락
}
void Flash_EraseMemory(uint32_t start_address, uint32_t numberOfPages) {
    HAL_FLASH_Unlock();  // 플래시 메모리 언락

    // 플래시 메모리 지우기 설정
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = start_address;
    EraseInitStruct.NbPages = numberOfPages;

    // 전체 메모리 지우기
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        // 오류 처리
        HAL_FLASH_Lock();
        return;
    }

    HAL_FLASH_Lock();  // 플래시 메모리 락
}

// 청크 데이터를 플래시 메모리에 저장하는 함수
void Flash_WriteChunk(uint32_t *chunk_data, uint32_t chunk_size) {
    HAL_FLASH_Unlock();  // 플래시 메모리 언락

    // 청크 데이터 단위로 플래시에 기록
    for (uint32_t i = 0; i < chunk_size; i += 4) {
        uint32_t data = chunk_data[i / 4];  // 4바이트 단위로 데이터 읽기

        // 플래시에 데이터 쓰기
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_flash_address, data) != HAL_OK) {
            // 오류 처리
            HAL_FLASH_Lock();
            return;
        }

        // 다음 플래시 주소로 이동
        current_flash_address += 4;
    }

    HAL_FLASH_Lock();  // 플래시 메모리 락
}
void ProcessChunkData(uint8_t *chunk_data, uint32_t chunk_size) {
    // 청크 데이터를 플래시에 쓰기
    Flash_WriteChunk((uint32_t *)chunk_data, chunk_size);
}
void StartFirmwareUpdate() {
    // 1. 플래시 메모리 지우기 (시작 시 1회)
    Flash_EraseMemory(TARGET_START_ADDRESS, NUMBER_OF_PAGES);
}


//********************
void Flash_WriteData(uint8_t *data, uint32_t size) {
    HAL_FLASH_Unlock();

    for (uint32_t i = 0; i < size; i += 4) {
        uint32_t word = (data[i + 3] << 24) | (data[i + 2] << 16) | (data[i + 1] << 8) | data[i];
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_flash_address, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return;
        }
        current_flash_address += 4;
    }

    HAL_FLASH_Lock();
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, &uart_buffer[data_index], 1);

  send_command("AT\r\n");
  HAL_Delay(2000);
  StartFirmwareUpdate();
  send_command("AT+CWQAP\r\n");
  HAL_Delay(1000); // Disconnect any previous connection

  send_command("AT+RST\r\n");
  HAL_Delay(2000); // Reset the module

  // Connect to Wi-Fi network
  send_command("AT+CWJAP=\"Atom2_4GHz\",\"atom1234!\"\r\n");
  HAL_Delay(5000);  // Adjust delay if connection takes longer

  // Confirm IP address
  send_command("AT+CIFSR\r\n");
  HAL_Delay(1000); // Request IP address

  // Start TCP connection to Flask server
  send_command("AT+CIPSTART=\"TCP\",\"192.168.0.60\",5000\r\n");
  HAL_Delay(2000);

  // Send GET request to /test endpoint
  send_command("AT+CIPSEND=55\r\n"); // Adjust the byte length based on request string
  HAL_Delay(1000);

  // Send HTTP GET request
  send_command("GET /download/firmware HTTP/1.1\r\nHost: 192.168.0.60\r\n\r\n");
  HAL_Delay(20000); // Wait for server response

  // 플래시 페이지 복사 실행
 // Flash_CopyPages(SOURCE_START_ADDRESS, TARGET_START_ADDRESS, PAGE_SIZE, NUMBER_OF_PAGES);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //receive_response(data_buffer, sizeof(data_buffer));
	 // flash_write(flash_address, data_buffer, sizeof(data_buffer));
	  //flash_address += sizeof(data_buffer);
	 // process_uart_data();
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

/* USER CODE BEGIN 4 */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart_buffer[data_index++] = uart_buffer[0];

        // Wrap data_index in a circular buffer manner if needed
        if (data_index >= BUFFER_SIZE) {
            data_index = 0;
        }

        // Re-enable UART interrupt for the next byte
        HAL_UART_Receive_IT(&huart1, &uart_buffer[data_index], 1);
    }
}
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static const char start_pattern[] = ":020000040800F2";
    if (huart->Instance == USART1) {
        uint8_t received_byte = uart_buffer[data_index++];

        // 본문 시작 확인
        if (!is_body) {
            if (received_byte == start_pattern[match_index]) {
                match_index++;
                if (match_index == sizeof(start_pattern) - 1) {
                    is_body = true;      // 본문 시작 플래그 설정
                    data_index = 0;      // 데이터 인덱스 초기화
                    match_index = 0;     // 매칭 인덱스 초기화
                    hex_data_index = 0;  // hex_data_buffer 인덱스 초기화
                }
            } else {
                match_index = 0;  // 매칭 실패 시 다시 처음부터 매칭 시도
            }
        } else {
            // 본문 데이터 읽기
            if (received_byte == '\n' || received_byte == '\r') {
                // 데이터 라인 끝에 도달했을 때 처리
                if (hex_data_index > 0) {
                    Flash_WriteData(hex_data_buffer, hex_data_index);
                    hex_data_index = 0;
                }
            } else if (received_byte == ':' && hex_data_index == 0) {
                // 새 라인 시작 시 첫 번째 ':' 문자 무시
                // 주소 오프셋을 무시하고 데이터만 저장하기 위해 별도 처리
                data_index = 0;
            } else if (hex_data_index >= 9) {
                // ":10"과 주소 오프셋 이후의 실제 데이터만 저장
                hex_data_buffer[hex_data_index - 9] = received_byte;
                hex_data_index++;
            }
        }

        // 다음 바이트 수신 준비
        HAL_UART_Receive_IT(&huart1, &uart_buffer[data_index], 1);
    }
}
void process_uart_data(void) {
    static uint8_t pattern_index = 0;

    for (uint16_t i = 0; i < data_index; i++) {
        uint8_t received_byte = uart_buffer[i];

        if (!data_receiving) {
            // Pattern matching logic
            if (received_byte == START_PATTERN[pattern_index]) {
                pattern_index++;
                if (pattern_index == START_PATTERN_LEN) {
                    data_receiving = 1;
                    pattern_index = 0;
                    byte_index = 0;
                }
            } else {
                pattern_index = 0;
            }
        } else {
            // Store valid hex data
          //  hex_data_buffer[chunk_index][byte_index++] = received_byte;

            // If chunk is full, write it to flash and reset indices
            if (byte_index >= CHUNK_SIZE) {
                //write_chunk_to_flash(flash_address, hex_data_buffer[chunk_index], CHUNK_SIZE);
                //flash_address += CHUNK_SIZE;
                byte_index = 0;
                chunk_index = (chunk_index + 1) % NUM_CHUNKS;
            }
        }
    }
    // Reset data_index to start fresh on the next callback
    data_index = 0;
}

void send_command(const char* command)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);
}

// Function to receive response from ESP8266
void receive_response(char* buffer, uint16_t length)
{
    HAL_UART_Receive(&huart1, (uint8_t*)buffer, length, HAL_MAX_DELAY);
}

// Function to write received data to flash memory
void flash_write(uint32_t address, uint8_t *data, uint16_t length)
{
    HAL_FLASH_Unlock();
    for (uint16_t i = 0; i < length; i += 4)
    {
        uint32_t word = *(uint32_t*)(&data[i]);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word);
    }
    HAL_FLASH_Lock();
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
