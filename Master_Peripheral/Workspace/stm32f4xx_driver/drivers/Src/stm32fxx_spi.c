/*
 * stm32fxx_spi.c
 *
 *  Created on: Aug 10, 2021
 *      Author: phant
 */
#include "stm32f4xx_spi.h"

/*******************************************************************************
 * @fn						- SPI_PeriClockControl
 *
 * @brief					- Using to setting clock for SPI
 *
 * @pSPIx					- Base address of SPI peripheral
 * @En_Dis					- Select ENABLE or DISABLE clock
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_PeriClockControl(SPI_RegDef_t* pSPIx, uint8_t En_Dis) {
	if (En_Dis == ENABLE){
		if (pSPIx == SPI1)
			SPI1_PCLK_EN();
		else if (pSPIx == SPI2)
			SPI2_PCLK_EN();
		else if (pSPIx == SPI3)
			SPI3_PCLK_EN();
	}
	else {
		if (pSPIx == SPI1)
			SPI1_PCLK_DIS();
		else if (pSPIx == SPI2)
			SPI2_PCLK_DIS();
		else if (pSPIx == SPI3)
			SPI3_PCLK_DIS();
	}
}

/*******************************************************************************
 * @fn						- SPI_Init
 *
 * @brief					- Using to initialize for SPI registers
 *
 * @pSPIHandler				- Handler of SPI peripherals
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_Init(SPI_Handler_t* pSPIHandler) {
	__IO uint32_t tempreg = 0;

	// Setting Device mode: Master or Slave
	tempreg |= pSPIHandler->SPIConfig.SPI_DeviceMode << SPIBITs_MSTR;

	// Setting Bus Configuration
	if (pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_FD) {
		// BIDI Mode should be 0: 2-line unidirectional data
		tempreg &= ~(1 << SPIBITs_BIDIMODE);
	}
	else if (pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_HD) {
		// BIDE Mode should be 1: 1-line bidirectional data
		tempreg |= (1 << SPIBITs_BIDIMODE);
	}
	if (pSPIHandler->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_SD_RXONLY) {
		// BIDI Mode should be 0: 2-line unidirectional data
		tempreg &= ~(1 << SPIBITs_BIDIMODE);
		// RXONLY should be 1: Setting Receive mode only
		tempreg |= (1 << SPIBITs_RXONLY);
	}

	// Setting Speed
	tempreg |= pSPIHandler->SPIConfig.SPI_SclkSpeed << SPIBITs_BR;

	// Setting DFF
	tempreg |= pSPIHandler->SPIConfig.SPI_DataFormat << SPIBITs_DFF;

	// Setting SPI_CPOL
	tempreg |= pSPIHandler->SPIConfig.SPI_CPOL << SPIBITs_CPOL;

	// Setting SPI_CPHA
	tempreg |= pSPIHandler->SPIConfig.SPI_CPHA << SPIBITs_CPHA;

	// Setting SPI_SSM
	tempreg |= pSPIHandler->SPIConfig.SPI_SSM << SPIBITs_SSM;

	pSPIHandler->pSPIx->CR1 = tempreg;
}

/*******************************************************************************
 * @fn						- SPI_DeInit
 *
 * @brief					- Using to de-initialize for SPI registers
 *
 * @pSPIx					- Base address of SPI peripheral
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_DeInit(SPI_RegDef_t* pSPIx) {
	if (pSPIx == SPI1)
		SPI1_RESET();
	else if (pSPIx == SPI2)
		SPI2_RESET();
	else if (pSPIx == SPI3)
		SPI3_RESET();
}

/*******************************************************************************
 * @fn						- SPI_EnableSPI
 *
 * @brief					- Enable SPI peripheral
 *
 * @pSPIx					- Base address of SPI peripheral
 * @En_Dis					- En/Dis peripheral
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_EnableSPI(SPI_RegDef_t* pSPIx, uint8_t En_Dis){
	if (En_Dis == ENABLE) {
		//Enable SPI peripheral
		pSPIx->CR1 |= (1 << SPIBITs_SPE);
	}
	else {
		//Disable SPI peripheral
		pSPIx->CR1 &= ~(1 << SPIBITs_SPE);
	}
}

/*******************************************************************************
 * @fn						- SPI_EnableNSSPin
 *
 * @brief					- Enable NSS PIN
 *
 * @pSPIx					- Base address of SPI peripheral
 * @En_Dis					- En/Dis peripheral
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_EnableNSSPin(SPI_RegDef_t* pSPIx, uint8_t En_Dis){
	if (En_Dis == ENABLE) {
		//Enable SPI peripheral
		pSPIx->CR1 |= (1 << SPIBITs_SSI);
	}
	else {
		//Disable SPI peripheral
		pSPIx->CR1 &= ~(1 << SPIBITs_SSI);
	}
}

/*******************************************************************************
 * @fn						- SPI_EnableSSOEPin
 *
 * @brief					- Enable SSOE PIN
 *
 * @pSPIx					- Base address of SPI peripheral
 * @En_Dis					- En/Dis peripheral
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_EnableSSOEPin(SPI_RegDef_t* pSPIx, uint8_t En_Dis){
	if (En_Dis == ENABLE) {
		//Enable SPI peripheral
		pSPIx->CR2 |= (1 << SPIBITs_SSOE);
	}
	else {
		//Disable SPI peripheral
		pSPIx->CR2 &= ~(1 << SPIBITs_SSOE);
	}
}

/*******************************************************************************
 * @fn						- GetStatusFlag
 *
 * @brief					- Get status of SPI
 *
 * @pSPIx					- Base address of SPI peripheral
 * @Mask					- Mask of flag
 *
 * @return					- True/False
 *
 * @note					- None
 */
uint8_t GetStatusFlag(SPI_RegDef_t* pSPIx, uint32_t Mask) {
	if (pSPIx->SR & Mask)
		return 1;
	return 0;
}

/*******************************************************************************
 * @fn						- SPI_SendData
 *
 * @brief					- Read value of PIN
 *
 * @pSPIx					- Base address of SPI peripheral
 * @pTxBuffer				- Pointer to data TX
 * @len						- Length of data
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_SendData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len){
	while (len > 0){
		// Wait TX buffer empty: TXE bit
		while (GetStatusFlag(pSPIx, SPI_FLAG_TXE) == 0);

		// Check DFF bit to send data
		if ((pSPIx->CR1 & (1 << SPIBITs_DFF)) == 0) {
			// Send 8 bit
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
		else {
			// Send 16 bit
			pSPIx->DR = *((uint16_t* )pTxBuffer);
			len--;
			len--;
			(uint16_t* )pTxBuffer++;
		}
	}
}

/*******************************************************************************
 * @fn						- SPI_ReceiveData
 *
 * @brief					- Read value of PIN
 *
 * @pSPIx					- Base address of SPI peripheral
 * @pTxBuffer				- Pointer to data RX
 * @len						- Length of data
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t len){

}



/*******************************************************************************
 * @fn						- GPIO_IRQConfig
 *
 * @brief					- Configuration IRQ of GPIO (EXTI)
 *
 * @IRQNumber				- IRQ Number
 * @Priority				- Priority
 * @En_Dis					- Select ENABLE or DISABLE clock
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t Priority, uint8_t En_Dis) {

}

/*******************************************************************************
 * @fn						- GPIO_IRQHandling
 *
 * @brief					- The function handling for GPIOx
 *
 * @SPI_Handler_t			- Handler of SPI peripheral
 *
 * @return					- None
 *
 * @note					- None
 */
void SPI_IRQHandling(SPI_Handler_t* pSPIHandler) {

}


