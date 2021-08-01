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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"

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
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart3;

#define c_uart	&huart3

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* Re-implement function fputc: Refer to: */
/* https://developer.arm.com/documentation/dui0475/c/the-arm-c-and-c---libraries/redefining-low-level-library-functions-to-enable-direct-use-of-high-level-library-functions */
/* Private function prototypes -----------------------------------------------*/
struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};
/*send text over SWV*/
int fputc(int ch, FILE *f) {
    ITM_SendChar(ch);//send method for SWV
    return(ch);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define BL_RX_LEN 	200
uint8_t bl_rx_buffer[BL_RX_LEN];

uint8_t supported_command[] = {
				BL_GET_VER,
				BL_GET_HELP,
				BL_GET_CID,
				BL_GET_RDP_STATUS,
				BL_GO_TO_ADDR,
				BL_FLASH_ERASE,
				BL_MEM_WRITE,
				BL_EN_R_W_PROTECT,
				BL_MEM_READ,
				BL_READ_SECTOR_STATUS,
				BL_OTP_READ,
				BL_DIS_R_W_PROTECT
};

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
  MX_CRC_Init();
  MX_USART3_UART_Init();
	
  /* CUSTOM BOOTLOADER CODE BEGIN 2 */
	// User pressed button to confirm using bootloader command
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
		printf("BL_DEBUG_MSG: Button is pressed .. going to BL mode\r\n");
		// continue in bootloader mode
		bootloader_uart_read_data();
	}
	// User not pressed: Confirm move to user application
	else {
		printf("BL_DEBUG_MSG: Button is not pressed .. executing User app\r\n");
		bootloader_jump_to_user_app();
	}
  /* CUSTOM BOOTLOADER CODE END 2 */
}

void bootloader_uart_read_data(void) {
	uint8_t rcv_len = 0;
	
	while(1) {
		// Default Frame: [Lengh::1Byte] [Command:1Byte] [...] [CRC:4Byte]
		memset(bl_rx_buffer, 0, 200);
		// Here we read ad decode the commands coming from host
		// First read only one byte from the host, which is the "length" field of the command
		HAL_UART_Receive(c_uart, bl_rx_buffer, 1, HAL_MAX_DELAY);
		rcv_len = bl_rx_buffer[0];
		// Next to receive all following data, using the rcv_len
		HAL_UART_Receive(c_uart, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);
		// Swtich command
		switch(bl_rx_buffer[1]) {
			case BL_GET_VER:
				bootloader_handle_getver_cmd(bl_rx_buffer);
				break;
			case BL_GET_HELP:
				bootloader_handle_gethelp_cmd(bl_rx_buffer);
				break;
			case BL_GET_CID:
				bootloader_handle_getcid_cmd(bl_rx_buffer);
				break;
			case BL_GET_RDP_STATUS:
				bootloader_handle_getrdp_cmd(bl_rx_buffer);
				break;
			case BL_GO_TO_ADDR:
				bootloader_handle_go_cmd(bl_rx_buffer);
				break;
			case BL_FLASH_ERASE:
				bootloader_handle_flash_erase_cmd(bl_rx_buffer);
				break;
			case BL_MEM_WRITE:
				bootloader_handle_mem_write_cmd(bl_rx_buffer);
				break;
			case BL_EN_R_W_PROTECT:
				bootloader_handle_enb_rw_cmd(bl_rx_buffer);
				break;
			case BL_MEM_READ:
				bootloader_handle_mem_read_cmd(bl_rx_buffer);
				break;
			case BL_READ_SECTOR_STATUS:
				bootloader_handle_read_flash_cmd(bl_rx_buffer);
				break;
			case BL_OTP_READ:
				bootloader_handle_otp_read_cmd(bl_rx_buffer);
				break;
			case BL_DIS_R_W_PROTECT:
				bootloader_handle_dis_rw_cmd(bl_rx_buffer);
				break;
			default:
				printf("BL_DEBUG_MSG: Invaid command code recieved from host\r\n");
				break;
		}
		
	}
}

/*
	Code to jump to user application
  Here we use Sector 2 of FASH_SECTOR2_BASE_ADDRESS
  is where the user application stored
*/
void bootloader_jump_to_user_app(void) {
	// Function pointer to hold the address of the reset handler of the user app
	void (*app_reset_handler)(void);
	
	printf("BL_DEBUG_MSG: Bootloader jump to user application\r\n");
	
	// 1. Configure the MSP by reading the value from the baseline address of the Flash sector 2
	uint32_t msp_value = *(volatile uint32_t*)FLASH_SECTOR2_BASE_ADDRESS;
	printf("BL_DEBUG_MSG: MSP Value is %#x \r\n", msp_value);
	// Using CMSIS API set MSP
	__set_MSP(msp_value);
	
	/*
	2. Not fetch the reset handler address of the user application
	from the location FLASH_SECTOR2_BASE_ADDRESS + 4
	*/
	uint32_t resethandler_value = *(volatile uint32_t*) (FLASH_SECTOR2_BASE_ADDRESS + 4);
	
	app_reset_handler = (void*) resethandler_value;
	
	printf("BL_DEBUG_MSG: App resethandler addr is %#x \r\n", (uint32_t)app_reset_handler);
	
	// 3. Change VTOR register and Jump to reset handler of the user application
	SCB->VTOR = FLASH_SECTOR2_BASE_ADDRESS;
	app_reset_handler();
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
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

/************************** Implement of Bootloader Handle command ************/

/* Common helper function */
// ACK and NACK function
void bootloader_send_ack(uint8_t follow_len) {
	// Here we send 2 byte.. First is ACK, Second is len value
	uint8_t ack_buf[2];
	ack_buf[0] = ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(c_uart, ack_buf, 2, HAL_MAX_DELAY);
}
void bootloader_send_nack() {
	uint8_t nack = NACK;
	HAL_UART_Transmit(c_uart, &nack, 1, HAL_MAX_DELAY);
}

// Function verify crc data
uint8_t bootloader_verify_crc(uint8_t* pBuffer, uint32_t len, uint32_t host_crc) {
	uint32_t uwCRCValue = 0xff;
	
	/* Reset CRC Calculation Unit (hcrc->Instance->INIT is
  *  written in hcrc->Instance->DR) */
  __HAL_CRC_DR_RESET(&hcrc);
	
	// Calculation CRC(Without reset CRC value in loop): HAL_CRC_Accumulate
	for ( int i=0; i<len; i++) {
		uint32_t data = pBuffer[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &data, 1);
	}
	
	if ( uwCRCValue == host_crc )
		return VERIFY_CRC_SUCCESS; // return success
	else
		return VERIFY_CRC_FAIL;	// return failed
}

// Function to write data into UART3
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len) {
	HAL_UART_Transmit(c_uart, pBuffer, len, HAL_MAX_DELAY);
}



// Return bootloader version
uint8_t get_bootloader_version() {
	return (uint8_t) BL_VERSION;
}
/* Helper function to handle: BL_GET_VER command */
void bootloader_handle_getver_cmd(uint8_t *pBuffer){
	printf("BL_DEBUG_MSG: Bootloader get Version BL\r\n");
	uint8_t bl_version;
	
	// Total length of packet from host
	uint32_t command_packet_len = pBuffer[0] + 1;
	
	// Get CRC value from host
	uint32_t host_crc = *(uint32_t *) (pBuffer + command_packet_len - 4);
	
	// 1. Verify checksum(CRC)
	if (!bootloader_verify_crc(pBuffer, command_packet_len - 4, host_crc)) {
		printf("BL_DEBUG_MSG: Checksum is success\r\n");
		// Send Ack
		bootloader_send_ack(sizeof(bl_version));
		bl_version = get_bootloader_version();
		printf("BL_DEBUG_MSG: Bootloader version: %d %x\r\n", bl_version, bl_version);
		bootloader_uart_write_data(&bl_version, 1);
	}
	else {
		printf("BL_DEBUG_MSG: Checksum is failed\r\n");
		// Send NAck
		bootloader_send_nack();
	}
}

/* Helper function to handle: BL_GET_HELP command */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer){
	printf("BL_DEBUG_MSG: Bootloader get help cmd\r\n");
	
	// Total length of packet from host
	uint32_t command_packet_len = pBuffer[0] + 1;
	
	// Get CRC value from host
	uint32_t host_crc = *(uint32_t *) (pBuffer + command_packet_len - 4);
	
	// 1. Verify checksum(CRC)
	if (!bootloader_verify_crc(pBuffer, command_packet_len - 4, host_crc)) {
		printf("BL_DEBUG_MSG: Checksum is success\r\n");
		// Send Ack
		bootloader_send_ack(sizeof(supported_command));
		bootloader_uart_write_data(supported_command, sizeof(supported_command));
	}
	else {
		printf("BL_DEBUG_MSG: Checksum is failed\r\n");
		// Send NAck
		bootloader_send_nack();
	}
}

// Get chip identifier number function
uint16_t get_mcu_chip_id(void) {
	return ((uint16_t)(DBGMCU->IDCODE) & 0xFFFU);
}

/* Helper function to handle: BL_GET_CID command */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer){
	printf("BL_DEBUG_MSG: Bootloader get CID of MCU\r\n");
	uint16_t get_cid;
	
	// Total length of packet from host
	uint32_t command_packet_len = pBuffer[0] + 1;
	
	// Get CRC value from host
	uint32_t host_crc = *(uint32_t *) (pBuffer + command_packet_len - 4);
	
	// 1. Verify checksum(CRC)
	if (!bootloader_verify_crc(pBuffer, command_packet_len - 4, host_crc)) {
		printf("BL_DEBUG_MSG: Checksum is success\r\n");
		// Send Ack
		get_cid = get_mcu_chip_id();
		bootloader_send_ack(sizeof(get_cid));
		bootloader_uart_write_data((uint8_t*)&get_cid, sizeof(get_cid));
	}
	else {
		printf("BL_DEBUG_MSG: Checksum is failed\r\n");
		// Send NAck
		bootloader_send_nack();
	}
	
}

// Get flash rdp level function
uint8_t get_flash_rdp_level() {
	return (uint8_t)(*(uint16_t*)(0x1FFFC000U)>>8);
}

/* Helper function to handle: BL_GET_RDP_STATUS command */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer){
	printf("BL_DEBUG_MSG: Bootloader get flash rdp level\r\n");
	uint8_t get_rdp;
	
	// Total len
	uint32_t command_packet_len = pBuffer[0] + 1;
	
	// Get host_crc which sent by host pc
	uint32_t host_crc = *(uint32_t *) (pBuffer + command_packet_len - 4);
	
	if (!bootloader_verify_crc(pBuffer, command_packet_len - 4, host_crc)) {
		printf("BL_DEBUG_MSG: Checksum success\r\n");
		get_rdp = get_flash_rdp_level();
		bootloader_send_ack(sizeof(get_rdp));
		bootloader_uart_write_data(&get_rdp, sizeof(get_rdp));
	}
	else {
		printf("BL_DEBUG_MSG: Checksum failed\r\n");
		bootloader_send_nack();
	}
	
}

bool verify_address(uint32_t addr) {
	// Which address which we can jump to?
	// Refer to "main.h"
	if (addr >= SRAM1_BASE && addr <= SRAM1_END)
		return ADDRESS_VALID;
	else if (addr >= SRAM2_BASE && addr <= SRAM2_END)
		return ADDRESS_VALID;
	else if (addr >= FLASH_BASE && addr <= FLASH_END) {
		return ADDRESS_VALID;
	}
	else if (addr >= BKPSRAM_BASE && addr <= BKPSRAM_END)
		return ADDRESS_VALID;
	else
		return ADDRESS_INVALID;
}

/* Helper function to handle: BL_GO_TO_ADDR command */
void bootloader_handle_go_cmd(uint8_t *pBuffer){
	printf("BL_DEBUG_MSG: Bootloader go to address\r\n");
	uint8_t flag = ADDRESS_INVALID;
	uint32_t go_addr = 0;
	
	// Total length of command packet
	uint32_t command_packet_len = pBuffer[0] + 1;
	
	// Get value of host_crc
	uint32_t host_crc = *(uint32_t*) (pBuffer + command_packet_len - 4);
	
	// Verify CRC data
	if (!bootloader_verify_crc(pBuffer, command_packet_len - 4, host_crc)) {
		printf("BL_DEBUG_MSG: BL checksum success\r\n");
		// Send ACK
		bootloader_send_ack(sizeof(flag));
		// Obtain data
		go_addr = *(uint32_t*)&pBuffer[2];
		printf("BL_DEBUG_MSG: Go Address %#x\r\n", go_addr);
		// Verify address
		if (!verify_address(go_addr)) {
			printf("BL_DEBUG_MSG: Go Address %#x ::: Valid\r\n", go_addr);
			// Printf to host to confirm vaild address
			flag = ADDRESS_VALID;
			bootloader_uart_write_data(&flag, 1);
			
			//Setting if using FLASH sector to programming need to setup VTOR
			if ( FLASH_SECTOR0_BASE_ADDRESS <= go_addr && \
				   go_addr < FLASH_SECTOR1_BASE_ADDRESS)
				SCB->VTOR = FLASH_SECTOR0_BASE_ADDRESS;
			else if ( FLASH_SECTOR1_BASE_ADDRESS <= go_addr && \
				   go_addr < FLASH_SECTOR2_BASE_ADDRESS)
				SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;
			else if ( FLASH_SECTOR2_BASE_ADDRESS <= go_addr && \
				   go_addr < FLASH_SECTOR3_BASE_ADDRESS)
				SCB->VTOR = FLASH_SECTOR2_BASE_ADDRESS;
			else if ( FLASH_SECTOR3_BASE_ADDRESS <= go_addr && \
				   go_addr < FLASH_SECTOR4_BASE_ADDRESS)
				SCB->VTOR = FLASH_SECTOR3_BASE_ADDRESS;
			else if ( FLASH_SECTOR4_BASE_ADDRESS <= go_addr && \
				   go_addr < FLASH_SECTOR5_BASE_ADDRESS)
				SCB->VTOR = FLASH_SECTOR4_BASE_ADDRESS;
			else if ( FLASH_SECTOR5_BASE_ADDRESS <= go_addr && \
				   go_addr < FLASH_SECTOR6_BASE_ADDRESS)
				SCB->VTOR = FLASH_SECTOR5_BASE_ADDRESS;
			else if ( FLASH_SECTOR6_BASE_ADDRESS <= go_addr && \
				   go_addr < FLASH_SECTOR7_BASE_ADDRESS)
				SCB->VTOR = FLASH_SECTOR6_BASE_ADDRESS;
			else if ( FLASH_SECTOR7_BASE_ADDRESS <= go_addr && \
				   go_addr < FLASH_SECTOREND_BASE_ADDRESS)
				SCB->VTOR = FLASH_SECTOR7_BASE_ADDRESS;
			
			// Assume that: Address area of host is correct.
			// Please aware about T bit in Arm cortex:
			// T bit = 0: Using Thumb instruction
			// T bit = 1: Using ARM instruction
			// But ARM cortex just know ARM instruction. T bit = 0 will trigger erro Fault Handler.
			go_addr += 1; // CPU will using the first bit to set T bit(Important point)
			void (*jump_address) (void) = (void*)go_addr;
			
			// Jump to that address by call
			jump_address();
		}
		// Address invaild
		else {
			printf("BL_DEBUG_MSG: Go Address %#x ::: Invalid\r\n", go_addr);
			bootloader_uart_write_data(&flag, 1);
		}
		
	}
	// CRC failed
	else {
		printf("BL_DEBUG_MSG: BL checksum failed\r\n");
		bootloader_send_nack();
	}
}

uint8_t execute_flash_erase(uint8_t Sector, uint8_t NumOfSectors) {
	FLASH_EraseInitTypeDef hflash;
	HAL_StatusTypeDef status = HAL_ERROR;
	uint8_t RemainingSectors;
	uint32_t SectorError;
	
	if ( Sector > 8)
		status = HAL_ERROR;
	
	if (Sector == 0xff || Sector < 8 ) {
		if ( Sector == 0xff ) {
			hflash.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else {
			RemainingSectors = 8 - Sector;
			if ( NumOfSectors > RemainingSectors ) {
				NumOfSectors = RemainingSectors;
			}
			hflash.TypeErase = FLASH_TYPEERASE_SECTORS;
			hflash.Sector = Sector;
			hflash.NbSectors = NumOfSectors;
		}
		hflash.Banks = FLASH_BANK_1;
		hflash.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		
		HAL_FLASH_Unlock();
		status = HAL_FLASHEx_Erase(&hflash, &SectorError);		
		HAL_FLASH_Lock();
	}
	
	return status;
}

/* Helper function to handle: BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer){
	printf("BL_DEBUG_MSG: Bootloader call Flash erase cmd\r\n");
	uint8_t erase_status;
	
	//Store total len of command packet
	uint32_t total_len = pBuffer[0] + 1;
	
	// Get value of host_crc
	uint32_t host_crc = *(uint32_t*)(pBuffer + total_len - 4);
	
	// Check CRC
	if (!bootloader_verify_crc(pBuffer, total_len - 4, host_crc)) {
		printf("BL_DEBUG_MSG: checksum success\r\n");
		// Send to host ack
		bootloader_send_ack(1);
		
		// Print status of flash sector:
		printf("BL_DBG_MSG: Sector: %d and No_of_Sectors: %d\r\n", 
			pBuffer[2], pBuffer[3]);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		erase_status = execute_flash_erase(pBuffer[2], pBuffer[3]);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		
		printf("BL_DBG_MSG: flase erase status: %#x\r\n", erase_status);
		bootloader_uart_write_data(&erase_status, 1);
	}
	else {
		printf("BL_DEBUG_MSG: checksum failed\r\n");
		bootloader_send_nack();
	}
}

HAL_StatusTypeDef execute_mem_write(uint32_t address, uint8_t* pData, uint8_t len) {
	HAL_StatusTypeDef status;
	
	HAL_FLASH_Unlock();
	
	for (int i=0; i<len; i++) {
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, pData[i]);
		if (status != HAL_OK)
			break;
	}
	
	HAL_FLASH_Lock();
	return status;
}

/* Helper function to handle: BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer){
	printf("BL_DBG_MSG: Bootloader mem write\r\n");
	
	uint8_t write_status = ADDRESS_INVALID;
	uint8_t payload_len = pBuffer[6];
	uint32_t address = *(uint32_t*)(&pBuffer[2]);
	
	// get total lenght of command packet
	uint32_t Command_Packet_len = pBuffer[0] + 1;
	
	// Get value of host_crc
	uint32_t host_crc = *(uint32_t*)(pBuffer + Command_Packet_len - 4);
	if ( !bootloader_verify_crc(pBuffer, Command_Packet_len -4, host_crc)) {
		printf("BL_DBG_MSG: Bootloader checksum success\r\n");
		bootloader_send_ack(1);
		printf("BL_DBG_MSG: Bootloader write mem %#x\r\n", address);
		if ( !verify_address(address) ) {
			printf("BL_DBG_MSG: Address Valid\r\n");
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			write_status = execute_mem_write(address, &pBuffer[7], payload_len);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			
			bootloader_uart_write_data(&write_status, 1);
		}
		else {
			printf("BL_DBG_MSG: Address Invaid\r\n");
			write_status = ADDRESS_INVALID;
			bootloader_uart_write_data(&write_status, 1);
		}
	}
	else {
		printf("BL_DBG_MSG: Bootloader checksum failed\r\n");
		bootloader_send_nack();
	}
}

uint8_t execute_en_dis_writeprotection(uint8_t mask_sector, uint8_t pro_mode, uint8_t dis_mode) {
	HAL_StatusTypeDef status;
	uint32_t* pOPTCR = (uint32_t*)OPTCR_BYTE0_ADDRESS;
	// STM32F407 just support 1 protection mode: Write protection refer to HW UM
	if (pro_mode == 1) {
		// Unlock flash OB
		HAL_FLASH_OB_Unlock();
		
		// Wait until BSY flag not set
		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) == SET);
		
		if (dis_mode == 0) {
			*pOPTCR &= ~(mask_sector << 16);
		}
		else {
			*pOPTCR |= (0xffU << 16);
		}
		
		// Write 1 to OPT Start bit
		*pOPTCR |= 0x02U;
		
		// Wait until BSY flag not set
		while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) == SET);
		
		status = HAL_OK;
		
		// Lock flash OB
		HAL_FLASH_OB_Lock();
	}
	else 
		status = HAL_ERROR;
	
	return status;
}

/* Helper function to handle: BL_EN_R_W_PROTECT command */
void bootloader_handle_enb_rw_cmd(uint8_t *pBuffer){
	printf("BL_DBG_MSG: Bootloader enable protect flash\r\n");
	uint8_t status;
	
	// Get length of command packet
	uint32_t command_packet_len = pBuffer[0] + 1;
	
	// Get value of receive_crc
	uint32_t receive_crc = *(uint32_t*)(pBuffer + command_packet_len - 4);
	
	if (!bootloader_verify_crc(pBuffer, command_packet_len - 4, receive_crc)) {
		printf("BL_DBG_MSG: Bootloader checksum success\r\n");
		bootloader_send_ack(1);
		status = execute_en_dis_writeprotection(pBuffer[2], pBuffer[3], 0);
		bootloader_uart_write_data(&status, 1);
	}
	else {
		printf("BL_DBG_MSG: Bootloader checksum failed\r\n");
		bootloader_send_nack();
	}
}

/* Helper function to handle: BL_DIS_R_W_PROTECT command */
void bootloader_handle_dis_rw_cmd(uint8_t *pBuffer){
	printf("BL_DBG_MSG: Bootloader disable protect flash\r\n");
	uint8_t status;
	
	// Get length of command packet
	uint32_t command_packet_len = pBuffer[0] + 1;
	
	// Get value of receive_crc
	uint32_t receive_crc = *(uint32_t*)(pBuffer + command_packet_len - 4);
	
	if (!bootloader_verify_crc(pBuffer, command_packet_len - 4, receive_crc)) {
		printf("BL_DBG_MSG: Bootloader checksum success\r\n");
		bootloader_send_ack(1);
		status = execute_en_dis_writeprotection(0, 1, 1);
		bootloader_uart_write_data(&status, 1);
	}
	else {
		printf("BL_DBG_MSG: Bootloader checksum failed\r\n");
		bootloader_send_nack();
	}
}

/* Helper function to handle: BL_MEM_READ command */
void bootloader_handle_mem_read_cmd(uint8_t *pBuffer){
	
}

/* Helper function to handle: BL_READ_SECTOR_STATUS command */
void bootloader_handle_read_flash_cmd(uint8_t *pBuffer){
	
}

/* Helper function to handle: BL_OTP_READ command */
void bootloader_handle_otp_read_cmd(uint8_t *pBuffer){
	
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
