/*
 * 007_SPI_TXOnly_Arduino.c
 *
 *  Created on: Aug 15, 2021
 *      Author: phant
 */

#include "stdint.h"
#include "string.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"

GPIO_Handler_t 	SpiGPIOHandler;
SPI_Handler_t 	SpiHanlder;
GPIO_Handler_t 	Btn;

void Init_GPIO_SPI(void){
	// Enable clock before setting
	GPIO_PeriClockControl(GPIOB, ENABLE);

	SpiGPIOHandler.pGPIOx = GPIOB;
	SpiGPIOHandler.pPinConfig.GPIO_PinAltFunMode = GPIO_AF_05;
	SpiGPIOHandler.pPinConfig.GPIO_PinMode = GPIO_MODE_ALT;
	SpiGPIOHandler.pPinConfig.GPIO_PinOType = GPIO_OTYPE_PUSH_PULL;
	SpiGPIOHandler.pPinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SpiGPIOHandler.pPinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;

	// Setting pin MOSI
	SpiGPIOHandler.pPinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SpiGPIOHandler);

	// Setting pin SCK
	SpiGPIOHandler.pPinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SpiGPIOHandler);

	// Setting pin NSS
	SpiGPIOHandler.pPinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SpiGPIOHandler);
}

void SPI_configuration(void){
	//Enable clock for SPI before setting
	SPI_PeriClockControl(SPI2, ENABLE);

	SpiHanlder.pSPIx = SPI2;
	SpiHanlder.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FD;
	SpiHanlder.SPIConfig.SPI_CPHA = SPI_CPHA_FIRST_CAPTURE;
	SpiHanlder.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SpiHanlder.SPIConfig.SPI_DataFormat = SPI_DFF_8_BIT;
	SpiHanlder.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SpiHanlder.SPIConfig.SPI_SSM = SPI_SSM_HW;
	SpiHanlder.SPIConfig.SPI_SclkSpeed = SPI_SCLK_DEVIDE_8;

	SPI_Init(&SpiHanlder);
}

void GPIO_Btn_Init(void){
	// Enable clock before setting
	GPIO_PeriClockControl(GPIOA, ENABLE);

	Btn.pGPIOx = GPIOA;
	Btn.pPinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	Btn.pPinConfig.GPIO_PinNumber = GPIO_PIN_00;
	Btn.pPinConfig.GPIO_PinOType = GPIO_OTYPE_PUSH_PULL;
	Btn.pPinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Btn.pPinConfig.GPIO_PinSpeed = GPIO_SPEED_VERYHIGH;

	GPIO_Init(&Btn);
}

void delay(){
	for (uint32_t i = 0; i <= 500000/2; i++);
}

int main(){
	char* user_data="Hello\r\n";
	/* ------------------------ Setting SPI --------------------------------- */
	// Initialize clock/pin GPIO of SPI
	Init_GPIO_SPI();
	// Initialize SPI peripheral
	SPI_configuration();

	// Configuration SSOE bit if setting SS HW
	/*
	 * SSOE = ENABLE: SS Pin Low when SPE active, otherwise.
	 */
	SPI_EnableSSOEPin(SPI2, ENABLE);
	/* ------------------------ Setting Btn --------------------------------- */
	// Initialize GPIO button
	GPIO_Btn_Init();

	while(1) {
		// Wait until user press button
		while (!GPIO_ReadFromInputPin(&Btn, GPIO_PIN_00));
		delay();

		/* ------------------- Start a transmittion ------------------------- */
		// Enable SPI
		SPI_EnableSPI(SPI2, ENABLE);

		// Send length of data to ARDUINO slave
		uint8_t data_len = strlen(user_data);
		SPI_SendData(SPI2, &data_len, 1);

		// Send data to ARDUINO slave
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// Confirm SPI is not busy
		while(GetStatusFlag(SPI2, SPI_FLAG_BSY));

		// Disable SPI
		SPI_EnableSPI(SPI2, DISABLE);
		/* ---------------------- End a transmittion ------------------------ */
	}
	return 0;
}
