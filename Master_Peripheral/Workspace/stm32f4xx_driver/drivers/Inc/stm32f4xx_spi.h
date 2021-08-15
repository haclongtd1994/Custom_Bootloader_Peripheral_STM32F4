/*
 * stm32f4xx_spi.h
 *
 *  Created on: Aug 10, 2021
 *      Author: phant
 */

#ifndef INC_STM32F4XX_SPI_H_
#define INC_STM32F4XX_SPI_H_

#include "stm32f4xx.h"

/* --------------------- Bits definition for SPI ---------------------------- */
/*
 * CR1
 */
#define SPIBITs_CPHA							0
#define SPIBITs_CPOL							1
#define SPIBITs_MSTR							2
#define SPIBITs_BR								3
#define SPIBITs_SPE								6
#define SPIBITs_LSBFIRST						7
#define SPIBITs_SSI								8
#define SPIBITs_SSM								9
#define SPIBITs_RXONLY							10
#define SPIBITs_DFF								11
#define SPIBITs_CRCNEXT							12
#define SPIBITs_CRCEN							13
#define SPIBITs_BIDIOE							14
#define SPIBITs_BIDIMODE						15

/*
 * CR2
 */
#define SPIBITs_RXDMAEN							0
#define SPIBITs_TXDMAEN							1
#define SPIBITs_SSOE							2
#define SPIBITs_FRF								4
#define SPIBITs_ERRIE							5
#define SPIBITs_RXNEIE							6
#define SPIBITs_TXEIE							7

/*
 * SR
 */
#define SPIBITs_RXNE							0
#define SPIBITs_TXE								1
#define SPIBITs_CHSIDE							2
#define SPIBITs_UDR								3
#define SPIBITs_CRCERR							4
#define SPIBITs_MODF							5
#define SPIBITs_OVR								6
#define SPIBITs_BSY								7
#define SPIBITs_FRE								8

/*
 * Flag Status register
 */
#define SPI_FLAG_TXE							(1 << SPIBITs_TXE)
#define SPI_FLAG_BSY							(1 << SPIBITs_BSY)


/* --------------------- Macros using by this driver ------------------------ */
/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE				0
#define SPI_DEVICE_MODE_MASTER				1

/*
 * @SPI_BusConfig
 */
#define SPI_BUSCONFIG_FD					1
#define SPI_BUSCONFIG_HD					2
#define SPI_BUSCONFIG_SD_RXONLY				3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_DEVIDE_2					0
#define SPI_SCLK_DEVIDE_4					1
#define SPI_SCLK_DEVIDE_8					2
#define SPI_SCLK_DEVIDE_16					3
#define SPI_SCLK_DEVIDE_32					4
#define SPI_SCLK_DEVIDE_64					5
#define SPI_SCLK_DEVIDE_128					6
#define SPI_SCLK_DEVIDE_256					7

/*
 * @SPI_DataFormat
 */
#define SPI_DFF_8_BIT						0
#define SPI_DFF_16_BIT						1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW						0
#define SPI_CPOL_HIGH						1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_FIRST_CAPTURE				0
#define SPI_CPHA_SECOND_CAPTURE				1

/*
 * @SPI_SSM
 */
#define SPI_SSM_HW							0
#define SPI_SSM_SW							1


typedef struct Stag_SPI_Config_t {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DataFormat;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct Stag_SPI_Handler_t {
	SPI_RegDef_t* pSPIx;			/* This is base address of SPIx */
	SPI_Config_t SPIConfig;		/* Structure store config information */
} SPI_Handler_t;

/*
 * APIs support by this driver
 */
// Peripheral Clock Setup
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t En_Dis);

// Init and DeInit
void SPI_Init(SPI_Handler_t* pSPIHandler);
void SPI_DeInit(SPI_RegDef_t* pSPIx);

// Data Send & Receive
void SPI_EnableSPI(SPI_RegDef_t* pSPIx, uint8_t En_Dis);
void SPI_EnableNSSPin(SPI_RegDef_t* pSPIx, uint8_t En_Dis);
void SPI_EnableSSOEPin(SPI_RegDef_t* pSPIx, uint8_t En_Dis);
uint8_t GetStatusFlag(SPI_RegDef_t* pSPIx, uint32_t Mask);
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len);

// IRQ Configuration and IRQ Handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t Priority, uint8_t En_Dis);
void SPI_IRQHandling(SPI_Handler_t* pSPIHandler);


#endif /* INC_STM32F4XX_SPI_H_ */
