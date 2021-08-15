/*
 * stm32f4xx_gpio.c
 *
 *  Created on: Aug 7, 2021
 *      Author: phant
 */

#include "stm32f4xx_gpio.h"

/*******************************************************************************
 * @fn						- GPIO_PeriClockControl
 *
 * @brief					- Using to setting clock for GPIO
 *
 * @pGPIOx					- Base address of GPIO peripheral
 * @En_Dis					- Select ENABLE or DISABLE clock
 *
 * @return					- None
 *
 * @note					- None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t En_Dis) {
	if (En_Dis == ENABLE){
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
		else if (pGPIOx == GPIOI)
			GPIOI_PCLK_EN();
	}
	else {
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_DIS();
		else if (pGPIOx == GPIOB)
			GPIOB_PCLK_DIS();
		else if (pGPIOx == GPIOC)
			GPIOC_PCLK_DIS();
		else if (pGPIOx == GPIOD)
			GPIOD_PCLK_DIS();
		else if (pGPIOx == GPIOE)
			GPIOE_PCLK_DIS();
		else if (pGPIOx == GPIOF)
			GPIOF_PCLK_DIS();
		else if (pGPIOx == GPIOG)
			GPIOG_PCLK_DIS();
		else if (pGPIOx == GPIOH)
			GPIOH_PCLK_DIS();
		else if (pGPIOx == GPIOI)
			GPIOI_PCLK_DIS();
	}
}

/*******************************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- Using to initialize for GPIO registers
 *
 * @pGPIOHandler			- Handler of GPIO peripherals
 *
 * @return					- None
 *
 * @note					- None
 */
void GPIO_Init(GPIO_Handler_t* pGPIOHandler) {
	__IO uint32_t reg_value = 0;

	// 1. Setting mode
	// Check interrupts of non-interrupt
	if (pGPIOHandler->pPinConfig.GPIO_PinMode <= GPIO_MODE_ANOLOG) {
		// non-interrupt
		reg_value = (pGPIOHandler->pPinConfig.GPIO_PinMode << \
				(2 * pGPIOHandler->pPinConfig.GPIO_PinNumber));
		pGPIOHandler->pGPIOx->MODER &= ~(0x3 << \
				pGPIOHandler->pPinConfig.GPIO_PinNumber); //Clear old setting
		pGPIOHandler->pGPIOx->MODER |= reg_value;
	}
	else {
		// Setting EXTI_RTSR and EXTI_FTSR corresponding with EXT lines
		if (pGPIOHandler->pPinConfig.GPIO_PinMode == GPIO_MODE_INPUT_RT) {
			EXTI->RTSR |= (1 << pGPIOHandler->pPinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandler->pPinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandler->pPinConfig.GPIO_PinMode == GPIO_MODE_INPUT_FT) {
			EXTI->FTSR |= (1 << pGPIOHandler->pPinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandler->pPinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandler->pPinConfig.GPIO_PinMode == GPIO_MODE_INPUT_RFT) {
			EXTI->FTSR |= (1 << pGPIOHandler->pPinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandler->pPinConfig.GPIO_PinNumber);
		}

		// 2. Configuration mask bit 23 interrupt lines
		EXTI->IMR |= (1 << pGPIOHandler->pPinConfig.GPIO_PinNumber);

		// 3. Select which port connect to EXTIx line
		SYSCFG_PCLK_EN();
		reg_value = (pGPIOHandler->pPinConfig.GPIO_PinNumber);
		uint8_t portcode = GPIO_PORT_DECODE(pGPIOHandler->pGPIOx)
		SYSCFG->EXTICR[reg_value/4] = ( portcode << (4 * (reg_value % 4)));

	}

	// 2. Setting speed
	reg_value = (pGPIOHandler->pPinConfig.GPIO_PinSpeed << \
			(2 * pGPIOHandler->pPinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->OSPEEDR &= ~(0x3 << \
			pGPIOHandler->pPinConfig.GPIO_PinNumber);  //Clear old setting
	pGPIOHandler->pGPIOx->OSPEEDR |= reg_value;

	// 3. Setting PU PD
	reg_value = (pGPIOHandler->pPinConfig.GPIO_PinPuPdControl << \
			(2 * pGPIOHandler->pPinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->PUPDR &= ~(0x3 << \
			pGPIOHandler->pPinConfig.GPIO_PinNumber); //Clear old setting
	pGPIOHandler->pGPIOx->PUPDR |= reg_value;

	// 4. Setting OTYPE
	reg_value = (pGPIOHandler->pPinConfig.GPIO_PinOType << \
			pGPIOHandler->pPinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIOx->OTYPER &= ~(0x1 << \
			pGPIOHandler->pPinConfig.GPIO_PinNumber); //Clear old setting
	pGPIOHandler->pGPIOx->OTYPER |= reg_value;

	// 5. Check Mode Alternate or not :: setting Alternate function
	if (pGPIOHandler->pPinConfig.GPIO_PinMode == GPIO_MODE_ALT) {
		__IO uint64_t ALT_Reg_Value = 0;
		ALT_Reg_Value = ((uint64_t)pGPIOHandler->pPinConfig.GPIO_PinAltFunMode << \
				(4 * (uint64_t)pGPIOHandler->pPinConfig.GPIO_PinNumber));
		pGPIOHandler->pGPIOx->AFR.BothReg &= ~(uint64_t)(0xf << \
				(4 * (uint64_t)pGPIOHandler->pPinConfig.GPIO_PinNumber)); //Clear old setting
		pGPIOHandler->pGPIOx->AFR.BothReg |= ALT_Reg_Value;
	}
	else {
		// Do nothing
	}
}

/*******************************************************************************
 * @fn						- GPIO_DeInit
 *
 * @brief					- Using to de-initialize for GPIO registers
 *
 * @pGPIOx					- Base address of GPIO peripheral
 *
 * @return					- None
 *
 * @note					- None
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx) {
	if (pGPIOx == GPIOA)
		GPIOA_RESET();
	else if (pGPIOx == GPIOB)
		GPIOB_RESET();
	else if (pGPIOx == GPIOC)
		GPIOC_RESET();
	else if (pGPIOx == GPIOD)
		GPIOD_RESET();
	else if (pGPIOx == GPIOE)
		GPIOE_RESET();
	else if (pGPIOx == GPIOF)
		GPIOF_RESET();
	else if (pGPIOx == GPIOG)
		GPIOG_RESET();
	else if (pGPIOx == GPIOH)
		GPIOH_RESET();
	else if (pGPIOx == GPIOI)
		GPIOI_RESET();
}

/*******************************************************************************
 * @fn						- GPIO_ReadFromInputPin
 *
 * @brief					- Read value of PIN
 *
 * @pGPIOHandler			- Handler of GPIO peripherals
 * @PinNumber				- Select Pin Number of PORT
 *
 * @return					- GPIO_PIN_SET or GPIO_PIN_RESET macros
 *
 * @note					- None
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Handler_t* pGPIOHandler, uint8_t PinNumber) {
	__IO uint8_t temp;
	temp = (uint8_t)((pGPIOHandler->pGPIOx->IDR >> PinNumber) & 0x0001);
	return temp;
}

/*******************************************************************************
 * @fn						- GPIO_ReadFromInputPort
 *
 * @brief					- Read value of PORT
 *
 * @pGPIOHandler			- Handler of GPIO peripherals
 *
 * @return					- Value of INPUT register
 *
 * @note					- None
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Handler_t* pGPIOHandler) {
	__IO uint16_t temp;
	temp = (uint16_t)(pGPIOHandler->pGPIOx->IDR);
	return temp;
}

/*******************************************************************************
 * @fn						- GPIO_WriteToOutputPin
 *
 * @brief					- Write Level to PIN
 *
 * @pGPIOHandler			- Handler of GPIO peripherals
 * @PinNumber				- Select Pin Number of PORT
 * @Level					- GPIO_PIN_SET or GPIO_PIN_RESET macros
 *
 * @return					- None
 *
 * @note					- None
 */
void GPIO_WriteToOutputPin(GPIO_Handler_t* pGPIOHandler, uint8_t PinNumber, \
		uint8_t Level) {
	if (Level == GPIO_PIN_SET) {
		pGPIOHandler->pGPIOx->ODR |= (1 << PinNumber);
	}
	else {
		pGPIOHandler->pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*******************************************************************************
 * @fn						- GPIO_WriteToOutputPort
 *
 * @brief					- Write Level to PORT
 *
 * @pGPIOHandler			- Handler of GPIO peripherals
 * @Level					- GPIO_PIN_SET or GPIO_PIN_RESET macros
 *
 * @return					- None
 *
 * @note					- None
 */
void GPIO_WriteToOutputPort(GPIO_Handler_t* pGPIOHandler, uint8_t Level) {
	pGPIOHandler->pGPIOx->ODR = Level;
}

/*******************************************************************************
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					- Flip Value of Pin
 *
 * @pGPIOHandler			- Handler of GPIO peripherals
 * @PinNumber				- Select Pin Number of PORT
 *
 * @return					- None
 *
 * @note					- None
 */
void GPIO_ToggleOutputPin(GPIO_Handler_t* pGPIOHandler, uint8_t PinNumber) {
	pGPIOHandler->pGPIOx->ODR ^= (1 << PinNumber);
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
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t Priority, uint8_t En_Dis) {
	if (En_Dis == ENABLE) {
		if (IRQNumber < 32) {
			NVIC_ISER0 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber < 64) {
			NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber < 96) {
			NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}

		uint8_t reg_num = IRQNumber / 4;
		uint8_t pos_start = (IRQNumber % 4) * 8;
		uint8_t shift_amound = pos_start + (8 - PRI_BITS_IMPLEMENTED);

		*(NVIC_IPR + reg_num) = (Priority << shift_amound);
	}
	else {
		if (IRQNumber < 32) {
			NVIC_ICER0 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber < 64) {
			NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber < 96) {
			NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}

/*******************************************************************************
 * @fn						- GPIO_IRQHandling
 *
 * @brief					- The function handling for GPIOx
 *
 * @PinNumber				- Select Pin Number of PORT
 *
 * @return					- None
 *
 * @note					- None
 */
void GPIO_IRQHandling(uint8_t PinNumber) {
	if (EXTI->PR &= (1 << PinNumber)){
		GPIO_ToggleOutputPin(&GPIOLED, GPIO_PIN_08);
		EXTI->PR |= (1 << PinNumber);
	}
}




