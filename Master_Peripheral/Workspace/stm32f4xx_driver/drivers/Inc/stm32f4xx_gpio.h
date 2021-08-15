/*
 * stm32f4xx_gpio.h
 *
 *  Created on: Aug 7, 2021
 *      Author: phant
 */

#ifndef INC_STM32F4XX_GPIO_H_
#define INC_STM32F4XX_GPIO_H_

#include "stm32f4xx.h"

/* --------------------- Macros using by this driver ------------------------ */
/*
 * @GPIO_PIN
 */
#define GPIO_PIN_00							0
#define GPIO_PIN_01							1
#define GPIO_PIN_02							2
#define GPIO_PIN_03							3
#define GPIO_PIN_04							4
#define GPIO_PIN_05							5
#define GPIO_PIN_06							6
#define GPIO_PIN_07							7
#define GPIO_PIN_08							8
#define GPIO_PIN_09							9
#define GPIO_PIN_10							10
#define GPIO_PIN_11							11
#define GPIO_PIN_12							12
#define GPIO_PIN_13							13
#define GPIO_PIN_14							14
#define GPIO_PIN_15							15

/*
 * @GPIO_MODES
 */
#define GPIO_MODE_INPUT						0
#define GPIO_MODE_OUTPUT					1
#define GPIO_MODE_ALT						2
#define GPIO_MODE_ANOLOG					3
#define GPIO_MODE_INPUT_RT					4
#define GPIO_MODE_INPUT_FT					5
#define GPIO_MODE_INPUT_RFT					6

/*
 * @GPIO_OTYPES
 */
#define GPIO_OTYPE_PUSH_PULL				0
#define GPIO_OTYPE_OPEN_DRAIN				1

/*
 * @GPIO_SPEED
 */
#define GPIO_SPEED_LOW						0
#define GPIO_SPEED_MEDIUM					1
#define GPIO_SPEED_HIGH						2
#define GPIO_SPEED_VERYHIGH					3

/*
 * @GPIO_PU_PD
 */
#define GPIO_NO_PUPD						0
#define GPIO_PU								1
#define GPIO_PD								2

/*
 * @GPIO_ALT_FUNC
 */
#define GPIO_AF_00							0
#define GPIO_AF_01							1
#define GPIO_AF_02							2
#define GPIO_AF_03							3
#define GPIO_AF_04							4
#define GPIO_AF_05							5
#define GPIO_AF_06							6
#define GPIO_AF_07							7
#define GPIO_AF_08							8
#define GPIO_AF_09							9
#define GPIO_AF_10							10
#define GPIO_AF_11							11
#define GPIO_AF_12							12
#define GPIO_AF_13							13
#define GPIO_AF_14							14
#define GPIO_AF_15							15


/*
 * Structure support by this Driver
 */
typedef struct Stag_GPIO_PinConfig_t {
	uint8_t GPIO_PinNumber;			/*!< Refer to @GPIO_PIN >!*/
	uint8_t GPIO_PinMode;			/*!< Refer to @GPIO_MODES >!*/
	uint8_t GPIO_PinSpeed;			/*!< Refer to @GPIO_SPEED >!*/
	uint8_t GPIO_PinPuPdControl;	/*!< Refer to @GPIO_PU_PD >!*/
	uint8_t GPIO_PinOType;			/*!< Refer to @GPIO_OTYPES >!*/
	uint8_t GPIO_PinAltFunMode;		/*!< Refer to @GPIO_ALT_FUNC >!*/
} GPIO_PinConfig_t;



typedef struct Stag_GPIO_Handler_t {
	GPIO_RegDef_t* 		pGPIOx;
	GPIO_PinConfig_t	pPinConfig;
} GPIO_Handler_t;

/*
 * APIs support by this driver
 */
// Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t En_Dis);

// Init and DeInit
void GPIO_Init(GPIO_Handler_t* pGPIOHandler);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

// Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_Handler_t* pGPIOHandler, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Handler_t* pGPIOHandler);
void GPIO_WriteToOutputPin(GPIO_Handler_t* pGPIOHandler, uint8_t PinNumber, \
		uint8_t Level);
void GPIO_WriteToOutputPort(GPIO_Handler_t* pGPIOHandler, uint8_t Level);
void GPIO_ToggleOutputPin(GPIO_Handler_t* pGPIOHandler, uint8_t PinNumber);

// IRQ Configuration and IRQ Handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t Priority, uint8_t En_Dis);
void GPIO_IRQHandling(uint8_t PinNumber);

/*
 * Global variable
 */
extern GPIO_Handler_t GPIOLED, GPIOButton;


#endif /* INC_STM32F4XX_GPIO_H_ */
