/*
 * stm32f4xx.h
 *
 *  Created on: Aug 7, 2021
 *      Author: phant
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stdint.h>

/*
 * Simple macros using coding
 */
#define __IO								volatile
#define ENABLE								1
#define DISABLE								0
#define SET									ENABLE
#define RESET								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET

/*
 * Processor Registers:
 */
// NVIC:
#define PRI_BITS_IMPLEMENTED				4
// Interrupt Set-Enable Registers
#define NVIC_ISER0							*(uint32_t* )0xE000E100
#define NVIC_ISER1							*(uint32_t* )0xE000E104
#define NVIC_ISER2							*(uint32_t* )0xE000E108


// Interrupt Clear-Enable Registers
#define NVIC_ICER0							*(uint32_t* )0xE000E180
#define NVIC_ICER1							*(uint32_t* )0xE000E184
#define NVIC_ICER2							*(uint32_t* )0xE000E188

// Interrupt Priority Registers
#define NVIC_IPR							(uint32_t* )0xE000E400

/* -------------------------------------------------------------------------- */

/*
 * Base address of FLASH and SRAM memory
 */
#define FLASH_BASE							0x08000000U
#define SRAM1_BASE							0x20000000U
#define SRAM2_BASE							0x2001C000U
#define SRAM								SRAM1_BASEADDRESS
#define ROM									0x1FFF0000U


/*
 * AHBx and APBx bus peripheral base address
 */
#define PERIPH_BASE							0x40000000U
#define APB1_PERIPH_BASE					PERIPH_BASE
#define APB2_PERIPH_BASE					0x40010000U
#define AHB1_PERIPH_BASE					0x40020000U
#define AHB2_PERIPH_BASE					0x50000000U

/*
 * Base address of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASE							(AHB1_PERIPH_BASE + 0x0000)
#define GPIOB_BASE							(AHB1_PERIPH_BASE + 0x0400)
#define GPIOC_BASE							(AHB1_PERIPH_BASE + 0x0800)
#define GPIOD_BASE							(AHB1_PERIPH_BASE + 0x0C00)
#define GPIOE_BASE							(AHB1_PERIPH_BASE + 0x1000)
#define GPIOF_BASE							(AHB1_PERIPH_BASE + 0x1400)
#define GPIOG_BASE							(AHB1_PERIPH_BASE + 0x1800)
#define GPIOH_BASE							(AHB1_PERIPH_BASE + 0x1C00)
#define GPIOI_BASE							(AHB1_PERIPH_BASE + 0x2000)

/*
 * Base address of peripherals which are hanging on APB1 bus
 */
#define RCC_BASE							(AHB1_PERIPH_BASE + 0x3800)
#define I2C1_BASE							(APB1_PERIPH_BASE + 0x5400)
#define I2C2_BASE							(APB1_PERIPH_BASE + 0x5800)
#define I2C3_BASE							(APB1_PERIPH_BASE + 0x5C00)
#define SPI2_BASE							(APB1_PERIPH_BASE + 0x3800)
#define SPI3_BASE							(APB1_PERIPH_BASE + 0x3C00)
#define USART2_BASE							(APB1_PERIPH_BASE + 0x4400)
#define USART3_BASE							(APB1_PERIPH_BASE + 0x4800)
#define UART4_BASE							(APB1_PERIPH_BASE + 0x4C00)
#define UART5_BASE							(APB1_PERIPH_BASE + 0x5000)

/*
 * Base address of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASE							(APB2_PERIPH_BASE + 0x3000)
#define USART1_BASE							(APB2_PERIPH_BASE + 0x1000)
#define USART6_BASE							(APB2_PERIPH_BASE + 0x1400)
#define EXTI_BASE							(APB2_PERIPH_BASE + 0x3C00)
#define SYSCFG_BASE							(APB2_PERIPH_BASE + 0x3800)

/*
 * GPIO definition structures
 */
typedef union Utag_AFR_Reg {
	uint64_t BothReg;
	struct {
		uint32_t Low_Reg;
		uint32_t High_Reg;
	};
} AFR_Reg;

typedef struct Stag_GPIO_RegDef_t {
	__IO uint32_t MODER;		/* GPIO port mode register,						Offset: 0x00 */
	__IO uint32_t OTYPER;		/* GPIO port output type register,				Offset: 0x04 */
	__IO uint32_t OSPEEDR;		/* GPIO port output speed register,				Offset: 0x08 */
	__IO uint32_t PUPDR;		/* GPIO port pull-up/pull-down register,		Offset: 0x0C */
	__IO uint32_t IDR;			/* GPIO port input data register,				Offset: 0x10 */
	__IO uint32_t ODR;			/* GPIO port output data register,				Offset: 0x14 */
	__IO uint32_t BSRR;			/* GPIO port bit set/reset register,			Offset: 0x18 */
	__IO uint32_t LCKR;			/* GPIO port configuration lock register,		Offset: 0x1C */
	__IO AFR_Reg  AFR;			/* GPIO alternate function low/high register,	Offset: 0x20 */
} GPIO_RegDef_t;

/*
 * RCC definition structures
 */
typedef struct Stag_RCC_RegDef_t {
	__IO uint32_t CR;			/* RCC clock control register,					Offset: 0x00 */
	__IO uint32_t PLLCFGR;		/* RCC PLL configuration register,				Offset: 0x04 */
	__IO uint32_t CFGR;			/* RCC clock configuration register,			Offset: 0x08 */
	__IO uint32_t CIR;			/* RCC clock interrupt register,				Offset: 0x0C */
	__IO uint32_t AHB1RSTR;		/* RCC AHB1 peripheral reset register,			Offset: 0x10 */
	__IO uint32_t AHB2RSTR;		/* RCC AHB2 peripheral reset register,			Offset: 0x14 */
	__IO uint32_t AHB3RSTR;		/* RCC AHB3 peripheral reset register,			Offset: 0x18 */
	uint32_t Reserved;			/* Reserved,									Offset: 0x1C */
	__IO uint32_t APB1RSTR;		/* RCC APB1 peripheral reset register,			Offset: 0x20 */
	__IO uint32_t APB2RSTR;		/* RCC APB2 peripheral reset register,			Offset: 0x24 */
	uint32_t Reserved1;			/* Reserved,									Offset: 0x28 */
	uint32_t Reserved2;			/* Reserved,									Offset: 0x2C */
	__IO uint32_t AHB1ENR;		/* RCC AHB1 peripheral clock enable register,	Offset: 0x30 */
	__IO uint32_t AHB2ENR;		/* RCC AHB2 peripheral clock enable register,	Offset: 0x34 */
	__IO uint32_t AHB3ENR;		/* RCC AHB3 peripheral clock enable register,	Offset: 0x38 */
	uint32_t Reserved3;			/* Reserved,									Offset: 0x3C */
	__IO uint32_t APB1ENR;		/* RCC APB1 peripheral clock enable register,	Offset: 0x40 */
	__IO uint32_t APB2ENR;		/* RCC APB2 peripheral clock enable register,	Offset: 0x44 */
	uint32_t Reserved4;			/* Reserved,									Offset: 0x48 */
	uint32_t Reserved5;			/* Reserved,									Offset: 0x4C */
	__IO uint32_t AHB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register,	Offset: 0x50 */
	__IO uint32_t AHB2LPENR;	/* RCC AHB2 peripheral clock enable in low power mode register,	Offset: 0x54 */
	__IO uint32_t AHB3LPENR;	/* RCC AHB3 peripheral clock enable in low power mode register,	Offset: 0x58 */
	uint32_t Reserved6;			/* Reserved,									Offset: 0x5C */
	__IO uint32_t APB1LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register,	Offset: 0x60 */
	__IO uint32_t APB2LPENR;	/* RCC AHB1 peripheral clock enable in low power mode register,	Offset: 0x64 */
	uint32_t Reserved7;			/* Reserved,									Offset: 0x68 */
	uint32_t Reserved8;			/* Reserved,									Offset: 0x6C */
	__IO uint32_t BDCR;			/* RCC Backup domain control register,			Offset: 0x70 */
	__IO uint32_t CSR;			/* RCC clock control & status register,			Offset: 0x74 */
	uint32_t Reserved9;			/* Reserved,									Offset: 0x78 */
	uint32_t Reserved10;		/* Reserved,									Offset: 0x7C */
	__IO uint32_t SSCGR;		/* RCC spread spectrum clock generation register,				Offset: 0x80 */
	__IO uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register,			Offset: 0x8C */
} RCC_RegDef_t;

/*
 * EXTI structure
 */
typedef struct Stag_EXTI_RegDef_t {
	__IO uint32_t IMR;		/* Interrupt mask register,							Offset: 0x00 */
	__IO uint32_t EMR;		/* Event mask register,								Offset: 0x04 */
	__IO uint32_t RTSR;		/* Rising trigger selection register,				Offset: 0x08 */
	__IO uint32_t FTSR;		/* Falling trigger selection register,				Offset: 0x0C */
	__IO uint32_t SWIER;	/* Software interrupt event register,				Offset: 0x10 */
	__IO uint32_t PR;		/* Pending register,								Offset: 0x14 */
} EXTI_RegDef_t;

/*
 * EXTI structure
 */
typedef struct Stag_SYSCFG_RegDef_t {
	__IO uint32_t MEMRMP;		/* SYSCFG memory remap register,				Offset: 0x00 */
	__IO uint32_t PMC;			/* SYSCFG peripheral mode configuration,		Offset: 0x04 */
	__IO uint32_t EXTICR[4];	/* SYSCFG external interrupt configuration,		Offset: 0x08 */
	uint32_t Reserved;			/* Reserved,									Offset: 0x18 */
	uint32_t Reserved0;			/* Reserved,									Offset: 0x1C */
	__IO uint32_t CMPCR;		/* Compensation cell control register,			Offset: 0x20 */
} SYSCFG_RegDef_t;

/*
 * SPI structure
 */
typedef struct Stag_SPI_RegDef_t {
	__IO uint32_t CR1;			/* SPI control register 1,						Offset: 0x00 */
	__IO uint32_t CR2;			/* SPI control register 2,						Offset: 0x04 */
	__IO uint32_t SR;			/* SPI status register,							Offset: 0x08 */
	__IO uint32_t DR;			/* SPI data register,							Offset: 0x0C */
	__IO uint32_t CRCPR;		/* SPI CRC polynomial register,					Offset: 0x10 */
	__IO uint32_t RXCRCR;		/* SPI RX CRC register,							Offset: 0x14 */
	__IO uint32_t TXCRCR;		/* SPI TX CRC register,							Offset: 0x18 */
	__IO uint32_t I2SCFGR;		/* SPI_I2S configuration register,				Offset: 0x1C */
	__IO uint32_t I2SPR;		/* SPI_I2S prescaler register,					Offset: 0x20 */
}SPI_RegDef_t;


// RCC definition
#define RCC									((RCC_RegDef_t* ) RCC_BASE)

// Peripherals definitions
#define GPIOA								((GPIO_RegDef_t* )GPIOA_BASE)
#define GPIOB								((GPIO_RegDef_t* )GPIOB_BASE)
#define GPIOC								((GPIO_RegDef_t* )GPIOC_BASE)
#define GPIOD								((GPIO_RegDef_t* )GPIOD_BASE)
#define GPIOE								((GPIO_RegDef_t* )GPIOE_BASE)
#define GPIOF								((GPIO_RegDef_t* )GPIOF_BASE)
#define GPIOG								((GPIO_RegDef_t* )GPIOG_BASE)
#define GPIOH								((GPIO_RegDef_t* )GPIOH_BASE)
#define GPIOI								((GPIO_RegDef_t* )GPIOI_BASE)

// EXTI definition
#define EXTI								((EXTI_RegDef_t* )EXTI_BASE)

// SYSCFG definition
#define SYSCFG								((SYSCFG_RegDef_t* )SYSCFG_BASE)

// SPI definition
#define SPI1								((SPI_RegDef_t* )SPI1_BASE)
#define SPI2								((SPI_RegDef_t* )SPI2_BASE)
#define SPI3								((SPI_RegDef_t* )SPI3_BASE)


/*
 * Select PORT CODE each PORT
 */
#define GPIO_PORT_DECODE(x)					(x == GPIOA)?0:\
											(x == GPIOB)?1:\
											(x == GPIOC)?2:\
											(x == GPIOD)?3:\
											(x == GPIOE)?4:\
											(x == GPIOF)?5:\
											(x == GPIOG)?6:\
											(x == GPIOH)?7:\
											(x == GPIOI)?8:0;

/*
 * IRQ number in NVIC
 */
// EXTI
#define IRQ_NO_EXTI0						6
#define IRQ_NO_EXTI1						7
#define IRQ_NO_EXTI2						8
#define IRQ_NO_EXTI3						9
#define IRQ_NO_EXTI4						10
#define IRQ_NO_EXTI9_5						23
#define IRQ_NO_EXTI15_10					40



/*
 * Clock Enable Macros for Peripherals
 */
// GPIOx
#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()						(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()						(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()						(RCC->AHB1ENR |= (1<<8))

// I2Cx
#define I2C1_PCLK_EN()						(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()						(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()						(RCC->APB1ENR |= (1<<23))

// SPIx
#define SPI1_PCLK_EN()						(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()						(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()						(RCC->APB1ENR |= (1<<15))

// UARTx
#define USART1_PCLK_EN()					(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()					(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()					(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()						(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()						(RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()					(RCC->APB2ENR |= (1<<5))

// SYSCFG
#define SYSCFG_PCLK_EN()					(RCC->APB2ENR |= (1<<14))

/*
 * Clock Disable Macros for Peripherals
 */
// GPIOx
#define GPIOA_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DIS()					(RCC->AHB1ENR &= ~(1<<8))

// I2Cx
#define I2C1_PCLK_DIS()						(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DIS()						(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DIS()						(RCC->APB1ENR &= ~(1<<23))

// SPIx
#define SPI1_PCLK_DIS()						(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DIS()						(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DIS()						(RCC->APB1ENR &= ~(1<<15))

// UARTx
#define USART1_PCLK_DIS()					(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DIS()					(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DIS()					(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DIS()					(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DIS()					(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCLK_DIS()					(RCC->APB2ENR &= ~(1<<5))

// SYSCFG
#define SYSCFG_PCLK_DIS()					(RCC->APB2ENR &= ~(1<<14))

/*
 * Reset Peripherals
 */
// GPIOx
#define GPIOA_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 0));   \
												(RCC->AHB1RSTR &= ~(0x1 << 0)); \
											} while(0)
#define GPIOB_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 1));   \
												(RCC->AHB1RSTR &= ~(0x1 << 1)); \
											} while(0)
#define GPIOC_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 2));   \
												(RCC->AHB1RSTR &= ~(0x1 << 2)); \
											} while(0)
#define GPIOD_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 3));   \
												(RCC->AHB1RSTR &= ~(0x1 << 3)); \
											} while(0)
#define GPIOE_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 4));   \
												(RCC->AHB1RSTR &= ~(0x1 << 4)); \
											} while(0)
#define GPIOF_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 5));   \
												(RCC->AHB1RSTR &= ~(0x1 << 5)); \
											} while(0)
#define GPIOG_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 6));   \
												(RCC->AHB1RSTR &= ~(0x1 << 6)); \
											} while(0)
#define GPIOH_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 7));   \
												(RCC->AHB1RSTR &= ~(0x1 << 7)); \
											} while(0)
#define GPIOI_RESET()						do{ \
												(RCC->AHB1RSTR |= (1 << 8));   \
												(RCC->AHB1RSTR &= ~(0x1 << 8)); \
											} while(0)

// SPI
#define SPI1_RESET()						do{ \
												(RCC->APB2RSTR |= (1 << 12));   \
												(RCC->APB2RSTR &= ~(0x1 << 12)); \
											} while(0)
#define SPI2_RESET()						do{ \
												(RCC->APB1RSTR |= (1 << 14));   \
												(RCC->APB1RSTR &= ~(0x1 << 14)); \
											} while(0)
#define SPI3_RESET()						do{ \
												(RCC->APB1RSTR |= (1 << 15));   \
												(RCC->APB1RSTR &= ~(0x1 << 15)); \
											} while(0)


/*
 * Global variable
 */


#endif /* INC_STM32F4XX_H_ */
