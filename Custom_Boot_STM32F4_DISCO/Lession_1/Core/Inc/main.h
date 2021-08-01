/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* BOOTLOADER VERSION */
#define BL_VERSION 		0x01

/* ACK and NACK bytes */
#define ACK		0xA5
#define NACK	0x7F

/* CRC state */
#define VERIFY_CRC_SUCCESS	0
#define VERIFY_CRC_FAIL			1

#define FLASH_SECTOR0_BASE_ADDRESS 			0x8000000U
#define FLASH_SECTOR1_BASE_ADDRESS 			0x8004000U
#define FLASH_SECTOR2_BASE_ADDRESS			0x8008000U
#define FLASH_SECTOR3_BASE_ADDRESS 			0x800C000U
#define FLASH_SECTOR4_BASE_ADDRESS 			0x8010000U
#define FLASH_SECTOR5_BASE_ADDRESS 			0x8020000U
#define FLASH_SECTOR6_BASE_ADDRESS 			0x8040000U
#define FLASH_SECTOR7_BASE_ADDRESS 			0x8060000U
#define FLASH_SECTOREND_BASE_ADDRESS 		0x8080000U

// Bootloader function prototypes
void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_send_ack(uint8_t follow_len);
void bootloader_send_nack(void);
uint8_t bootloader_verify_crc(uint8_t* pBuffer, uint32_t len, uint32_t host_crc);
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len);

uint8_t get_bootloader_version(void);
void bootloader_handle_getver_cmd(uint8_t *);
void bootloader_handle_gethelp_cmd(uint8_t *);
void bootloader_handle_getcid_cmd(uint8_t *);
void bootloader_handle_getrdp_cmd(uint8_t *);
void bootloader_handle_go_cmd(uint8_t *);
void bootloader_handle_flash_erase_cmd(uint8_t *);
void bootloader_handle_mem_write_cmd(uint8_t *);
void bootloader_handle_enb_rw_cmd(uint8_t *);
void bootloader_handle_mem_read_cmd(uint8_t *);
void bootloader_handle_read_flash_cmd(uint8_t *);
void bootloader_handle_otp_read_cmd(uint8_t *);
void bootloader_handle_dis_rw_cmd(uint8_t *);

#define	BL_GET_VER										0x51
#define	BL_GET_HELP										0x52
#define	BL_GET_CID										0x53
#define	BL_GET_RDP_STATUS							0x54
#define	BL_GO_TO_ADDR									0x55
#define	BL_FLASH_ERASE								0x56
#define	BL_MEM_WRITE									0x57
#define	BL_EN_R_W_PROTECT							0x58
#define	BL_MEM_READ										0x59
#define	BL_READ_SECTOR_STATUS					0x5A
#define	BL_OTP_READ										0x5B
#define	BL_DIS_R_W_PROTECT						0x5C


/* Declear using to function with mem of ST */
#define ADDRESS_VALID									0x00
#define ADDRESS_INVALID								0x01

#define KByte													1024
#define MByte													KByte*KByte

// List section can be go to
#define SRAM1_SIZE										112*KByte
#define SRAM1_END											(SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE										16*KByte
#define SRAM2_END											(SRAM2_BASE + SRAM2_SIZE)
#define BKPSRAM_SIZE									4*KByte
#define BKPSRAM_END										(BKPSRAM_BASE + BKPSRAM_SIZE)





#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
