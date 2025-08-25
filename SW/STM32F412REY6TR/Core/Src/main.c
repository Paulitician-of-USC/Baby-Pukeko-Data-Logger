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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "h3lis331dl.h"
#include "lsm6dso32x.h"
#include "MS560702BA03-50/MS5607SPI.h"
#include "MT29F1G01ABAFDWB/drv_spi_flash.h"
#include "usbd_cdc_if.h"
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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN PV */
uint8_t stateValue = 0;
uint8_t flight = 0;
uint64_t PacketCount = 0;
uint32_t timestamp = 0;

extern uint8_t UserRxBufferFS[];  // The buffer used by the USB driver

extern volatile uint8_t usb_data_received;  // Flag set when data arrives
uint8_t usb_rx_buffer[APP_RX_DATA_SIZE];  // Copy buffer for parsing

// External flash write buffer (make sure it's large enough!)
uint8_t flash_tx_buffer[128];
char 	usb_tx_buffer[128];

extern struct MS5607Readings readings;
struct MS5607 MS5;
spi_flash_t flash;
struct LSM6DSO32 LSM6;
struct H3LIS331dl H3L;
LogPacket Current_Data;

Parameters params;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

uint8_t H3L_Init(SPI_HandleTypeDef *, GPIO_TypeDef *, uint16_t);
uint8_t LSM_Init(SPI_HandleTypeDef *, GPIO_TypeDef *, uint16_t);
void LSM6DSO32_ReadAndConvert(void);
void Log_Data(void);
void Update_Data(void);
uint32_t get_logging_interval_us(STATE_MACHINE state);
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
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(500);
  printf("Baby Pukeko V1.0.1\n\r");

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); 	HAL_Delay(500);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);HAL_Delay(500);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); 	HAL_Delay(500);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  stateValue = STATE_INIT;
  uint8_t init = 1;
  init  = MT2_Init(&hspi3, MT2_NSS_GPIO_Port, MT2_NSS_Pin);
  if(!init) {printf("Flash Init Failed!\n\r");}
  HAL_Delay(10);
  init = MS5607_Init(&hspi1, MS_NSS_GPIO_Port, MS_NSS_Pin);
  if(!init) {printf("MS5607 Init Failed!\n\r");}
  HAL_Delay(10);
  init = H3L_Init(&hspi2, H3_NSS_GPIO_Port, H3_NSS_Pin);
  if(!init) {printf("H3L Init Failed!\n\r");}
  HAL_Delay(10);
  init = LSM_Init(&hspi2, LSM_NSS_GPIO_Port, LSM_NSS_Pin);
  if(!init) {printf("LSM Init Failed!\n\r"); }
  HAL_Delay(10);
//  if(!init) {Error_Handler();}

  printf("Initialization successful!\n\r");
  stateValue = STATE_INIT_SUCCESS;

  Update_Data();
  snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
						 "%lu,%u,%u,%.2,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f %.2f %.2f\r\n",
						 (unsigned long)
						 Current_Data.time_ms, Current_Data.packet_num, Current_Data.flag,
						 Current_Data.temp_c,  Current_Data.pressure_pa,Current_Data.altitude_m,
						 Current_Data.Hacc_x,  Current_Data.Hacc_y,     Current_Data.Hacc_z,
						 Current_Data.Lacc_x,  Current_Data.Lacc_y,     Current_Data.Lacc_z,
						 Current_Data.gyro_x,  Current_Data.gyro_y,     Current_Data.gyro_z);

  printf(usb_tx_buffer);

//  HAL_Delay(60000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while(stateValue == STATE_INIT_SUCCESS) {

		  printf("Waiting for USB Response...\n\r");

		  uint32_t start_tick = HAL_GetTick(); // Record starting time (ms)
		  uint32_t timeout_ms = 60000;          // 30 seconds

		  usb_data_received = 0; // Clear flag

		  while ((HAL_GetTick() - start_tick) < timeout_ms) {
			  if (usb_data_received) {
				  usb_data_received = 0;
				  if (strcmp((char*)usb_rx_buffer, "CONFIG") == 0)
				  {stateValue = STATE_ARM; break;}

				  else if (strcmp((char*)usb_rx_buffer, "READ") == 0)
				  {stateValue = STATE_READ_LOG; break;}

				  else {printf("Unkown Message Recieved!\n\r"); break;}
			  }
		  }
		  if(stateValue == STATE_READ_LOG || STATE_MOD_CONFIG) {
			  break;
		  }
		  else{
			  printf("No Config Recieved! Arming....\n\r");
			  stateValue = STATE_ARM;
			  break;
		  }
	  }
//	  if(stateValue == STATE_MOD_CONFIG) {
//		  //currently will not support Configuration
//		  stateValue = STATE_ARM;
//		  break;
//	  }

	  if(stateValue == STATE_READ_LOG) {
		  stateValue = STATE_ARM; break;
//		  HAL_Delay(5000);
//
//			//uint8_t *data_start = flash->buf_out + PAGE_DATA_OFFSET;
//
//			printf("Time, packetCount, State, Temp (C), Pressure (Pa), Alt (m), HX (G), HY, HZ, LX, LY, LZ, GX, GY, GZ \n ");
//			for (int i = 0; i < ENTRIES_PER_PAGE; i++) {
//			//	Current_Data = (LogPacket *)(data_start + i * LOG_ENTRY_SIZE);
//
//			uint8_t current_flight = Current_Data.flight_num;
//			// (1) Detect end of valid data
//			if (current_flight == 0xFF || current_flight == 0x00) {
//				printf("EoF\r\n");
//				break;
//			}
//
//			// (2) Detect new flight number
//			if (Current_Data.flight_num != current_flight) {
//				current_flight = Current_Data.flight_num;
//				snprintf(flash_tx_buffer, sizeof(flash_tx_buffer),
//						 "\r\nNew Flight Detected: Flight #%u", current_flight);
//				printf(flash_tx_buffer);
//			}
//
//			snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
//					 "%lu,%u,%u,%.2,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f %.2f %.2f\r\n",
//					 (unsigned long)
//					 Current_Data.time_ms, Current_Data.packet_num, Current_Data.flag,
//					 Current_Data.temp_c,  Current_Data.pressure_pa,Current_Data.altitude_m,
//					 Current_Data.Hacc_x,  Current_Data.Hacc_y,     Current_Data.Hacc_z,
//					 Current_Data.Lacc_x,  Current_Data.Lacc_y,     Current_Data.Lacc_z,
//					 Current_Data.gyro_x,  Current_Data.gyro_y,     Current_Data.gyro_z);
//
//			printf(usb_tx_buffer);
//		  }
	  }

	  while(stateValue == STATE_ARM) {
		get_logging_interval_us(stateValue);
		Update_Data();

		static uint16_t above_threshold_counter = 0;
		// Calculate Acceleration magnitude
		float magnitude = sqrtf(Current_Data.Lacc_x * 	Current_Data.Lacc_x +
								Current_Data.Lacc_y *  	Current_Data.Lacc_y +
								Current_Data.Lacc_z * 	Current_Data.Lacc_z);

		// Check against threshold
		if (magnitude > params.G_THRESHOLD){
		 above_threshold_counter++;
		 if (above_threshold_counter >= params.THRESHOLD_COUNT){
			 above_threshold_counter = 0;  // reset after detection
			 stateValue = STATE_BOOST;
			 flight++; //Start logging a new flight
			 break;}
		}
		else
		{
		 above_threshold_counter = 0;
		}

		//Log_Data(&flash, flight, PacketCount, timestamp, &MS5, &LSM6, &H3L, stateValue);
		snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
							 "%lu,%u,%u,%.2,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f %.2f %.2f\r\n",
							 (unsigned long)
							 Current_Data.time_ms, Current_Data.packet_num, Current_Data.flag,
							 Current_Data.temp_c,  Current_Data.pressure_pa,Current_Data.altitude_m,
							 Current_Data.Hacc_x,  Current_Data.Hacc_y,     Current_Data.Hacc_z,
							 Current_Data.Lacc_x,  Current_Data.Lacc_y,     Current_Data.Lacc_z,
							 Current_Data.gyro_x,  Current_Data.gyro_y,     Current_Data.gyro_z);

		printf(usb_tx_buffer);
	  }

	  while(stateValue == STATE_BOOST) {
		get_logging_interval_us(stateValue);
		Update_Data();
		Log_Data();
	  }

	  while(stateValue == STATE_BURNOUT) {
		Update_Data();
		Log_Data();
	  }

	  while(stateValue == STATE_APOGEE) {
		  Update_Data();
		  Log_Data();
	  }

	  while(stateValue == STATE_LANDED) {

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MT2_NSS_Pin|MS_NSS_Pin|H3_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LSM_NSS_GPIO_Port, LSM_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : MT2_NSS_Pin MS_NSS_Pin H3_NSS_Pin */
  GPIO_InitStruct.Pin = MT2_NSS_Pin|MS_NSS_Pin|H3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM_NSS_Pin */
  GPIO_InitStruct.Pin = LSM_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LSM_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin PC14 */
  GPIO_InitStruct.Pin = LED_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



uint8_t LSM_Init(SPI_HandleTypeDef *hspix, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	LSM6.hspix=hspix;
	LSM6.GPIOx=GPIOx;
	LSM6.GPIO_Pin=GPIO_Pin;

	uint8_t tx[2], rx[2];
	uint8_t whoami;

	// === Step 1: Read WHO_AM_I ===
	tx[0] = LSM6DSO32_WHO_AM_I | 0x80;
	tx[1] = 0x00;

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(hspix, tx, rx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

	whoami = rx[1];
	if (whoami != LSM6DSO32_ID) {
		printf("LSM STATUS: whoami Failed! Expected 0x%02X got 0x%02X\n", LSM6DSO32_ID, whoami);
		return 0;
	} // Device not found

	// === Step 2: Set accelerometer to ±32g, ODR = 6.66 kHz ===
	// CTRL1_XL = 0x7C => ODR_XL = 1111 (6.66kHz), FS_XL = 11 (±32g)
	tx[0] = LSM6DSO32_CTRL1_XL | 0x40;
	tx[1] = 0x7C;

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspix, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

	// === Step 3: Set gyroscope to ±2000 dps, ODR = 6.66 kHz ===
	// CTRL2_G = 0x7C => ODR_G = 1111 (6.66kHz), FS_G = 11 (±2000 dps)
	tx[0] = LSM6DSO32_CTRL2_G | 0x40;
	tx[1] = 0x7C;

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspix, tx, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

	return 1; // Success
}

// Main logging function
void Log_Data() {
    uint8_t* ptr = flash_tx_buffer;
    flash.buf_out = ptr;
    memcpy(ptr, &Current_Data.flight_num,sizeof(uint8_t));ptr += sizeof(uint8_t);

    // Write metadata
    memcpy(ptr, &Current_Data.time_ms, 		sizeof(uint32_t));ptr += sizeof(uint32_t);
    memcpy(ptr, &Current_Data.packet_num, 	sizeof(uint16_t));ptr += sizeof(uint16_t);
    memcpy(ptr, &Current_Data.flag, 		sizeof(uint8_t));ptr += sizeof(uint8_t);
    memcpy(ptr, &Current_Data.temp_c, 		sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.pressure_pa, 	sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.altitude_m, 	sizeof(float));ptr += sizeof(float);

    // Write LSM6D data
    memcpy(ptr, &Current_Data.Lacc_x, 		sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.Lacc_y, 		sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.Lacc_z, 		sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.gyro_x, 		sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.gyro_x, 		sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.gyro_z, 		sizeof(float));ptr += sizeof(float);

    // Write H3L data
    memcpy(ptr, &Current_Data.Hacc_x, 		sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.Hacc_y, 		sizeof(float));ptr += sizeof(float);
    memcpy(ptr, &Current_Data.Hacc_z, 		sizeof(float));ptr += sizeof(float);

    // Send data to flash
    cmd_send_to_flash(&flash, sizeof(ptr));
    Current_Data.packet_num++;
}

void LSM6DSO32_ReadAndConvert()
{
    uint8_t tx[13] = {0}; // 1 address + 12 data
    uint8_t rx[13] = {0};

    tx[0] = LSM6DSO32_OUTX_L_G | 0x80 | 0x40;

    HAL_GPIO_WritePin(LSM6.GPIOx, LSM6.GPIO_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(LSM6.hspix, tx, rx, 13, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LSM6.GPIOx, LSM6.GPIO_Pin, GPIO_PIN_SET);

    // Unpack raw values
    int16_t gx = (int16_t)(rx[2]  << 8 | rx[1]);
    int16_t gy = (int16_t)(rx[4]  << 8 | rx[3]);
    int16_t gz = (int16_t)(rx[6]  << 8 | rx[5]);

    int16_t ax = (int16_t)(rx[8]  << 8 | rx[7]);
    int16_t ay = (int16_t)(rx[10] << 8 | rx[9]);
    int16_t az = (int16_t)(rx[12] << 8 | rx[11]);

    // Convert to physical units
    Current_Data.gyro_x = lsm6dso32_from_fs2000_to_mdps(gx);
    Current_Data.gyro_y = lsm6dso32_from_fs2000_to_mdps(gy);
    Current_Data.gyro_z = lsm6dso32_from_fs2000_to_mdps(gz);

    Current_Data.Lacc_x = lsm6dso32_from_fs32_to_mg(ax);
    Current_Data.Lacc_y = lsm6dso32_from_fs32_to_mg(ay);
    Current_Data.Lacc_z = lsm6dso32_from_fs32_to_mg(az);
}


uint32_t get_logging_interval_us(STATE_MACHINE state) {
    switch (state) {
        case STATE_ARM:     				   return 20000;  // 5 Hz
        case (STATE_BOOST || STATE_BURNOUT):   return 1000;    // 1000 Hz
        case STATE_APOGEE:    				   return 100000;  // 10 Hz
        default:             				   return 100000;  // 10 Hz default for safety
    }
}

void Update_Data() {
	Current_Data.time_ms = HAL_GetTick();
	Current_Data.flag = stateValue;
	// Pull Data from LSM6
	LSM6DSO32_ReadAndConvert();

	// Pull Data from MS5607
	MS5607Update();
	Current_Data.temp_c = MS5.altitude;
	Current_Data.pressure_pa = MS5.pressure;
	Current_Data.altitude_m = MS5.temperature;

	// Pull Data from H3L and Convert
	readAxes();
	Current_Data.Hacc_x = H3L.xaxis * 0.049f;
	Current_Data.Hacc_y = H3L.yaxis * 0.049f;
	Current_Data.Hacc_z = H3L.zaxis * 0.049f;
}

int _write(int file, char *data, int len)
{
    CDC_Transmit_FS((uint8_t*)data, len);
    return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	printf("Error!");
	HAL_Delay(250);
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
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
