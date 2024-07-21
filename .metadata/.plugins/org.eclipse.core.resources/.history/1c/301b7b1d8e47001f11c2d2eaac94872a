/*
 * stm32f407xx.h
 *
 *  Created on: Jun 25, 2024
 *      Author: ADMIN
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>

#define __vo volatile

/***********************START: Processor Specific Details *************/

/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */

#define NVIC_ISER0 ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 ((__vo uint32_t*)0xE000E10C)


/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */

#define NVIC_ICER0 ((__vo uint32_t*)0xE00E180)
#define NVIC_ICER1 ((__vo uint32_t*)0xE00E184)
#define NVIC_ICER2 ((__vo uint32_t*)0xE00E188)
#define NVIC_ICER3 ((__vo uint32_t*)0xE00E18C)


/*
 * ARM Cortex Mx Processor Priority Register Addresses
 */

#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)


/*
 * ARM Cortex Mx Processor Number Of Priority Bits Implemented In Priority Register
 */

#define NO_PR_BITS_IMPLEMENTED 		4

/* base addresses of flash and SRAM memories*/

#define FLASH_BASEADDR              0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR				0x20001C00U
#define ROM							0x1FFF0000U
#define SRAM						SRAM1_BASEADDR


/* AHBx and APBx Bus Peripheral base addresses */

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define	APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U


/* Base addresses of peripherals which are hanging on AHB1 bus */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)


/* Base addresses of peripherals which are hanging on APB1 bus */

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)


/* Base addresses of peripherals which are hanging on APB2 bus */

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)


/* peripheral register definition structure for GPIO */

typedef struct
{
	__vo uint32_t MODER;  			/*GPIO port mode register 							- offset: 0x00*/
	__vo uint32_t OTYPER;			/*GPIO port output type register 					- offset: 0x04*/
	__vo uint32_t OSPEEDR;			/*GPIO port output speed register 					- offset: 0x08*/
	__vo uint32_t PUPDR;			/*GPIO port pull-up/pull-down register 				- offset: 0x0C*/
	__vo uint32_t IDR;				/*GPIO port input data register 					- offset: 0x10*/
	__vo uint32_t ODR;				/*GPIO port output data register 					- offset: 0x14*/
	__vo uint32_t BSRR;				/*GPIO port bit set/reset register 					- offset: 0x18*/
	__vo uint32_t LCKR;				/*GPIO port configuration lock register 			- offset: 0x1C*/
	__vo uint32_t AFR[2];			/*AFT[0] : GPIO alternate function low register 	- offset: 0x20 :
									  AFT[1] : GPIO alternate function high register 	- offset: 0x24*/
}GPIO_RegDef_t;


/* peripheral register definition structure for RCC */

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;

}RCC_RegDef_t;


/* peripheral register definition structure for SPI */

typedef struct
{
	__vo uint32_t CR1;  			/* 						- offset:  */
	__vo uint32_t CR2;				/*  					- offset:  */
	__vo uint32_t SR;				/*  					- offset:  */
	__vo uint32_t DR;				/* 						- offset:  */
	__vo uint32_t CRCPR;			/* 						- offset:  */
	__vo uint32_t RXCRCR;			/* 						- offset:  */
	__vo uint32_t TXCRCR;			/* 						- offset:  */
	__vo uint32_t I2FCFGR;			/* 						- offset:  */
	__vo uint32_t I2SPR;			/* 						- offset:  */
}SPI_RegDef_t;


/* peripheral register definition structure for I2C */

typedef struct
{
	__vo uint32_t CR1;  			/* 						- offset:  */
	__vo uint32_t CR2;				/*  					- offset:  */
	__vo uint32_t OAR1;				/*  					- offset:  */
	__vo uint32_t OAR2;				/* 						- offset:  */
	__vo uint32_t DR;				/* 						- offset:  */
	__vo uint32_t SR1;				/* 						- offset:  */
	__vo uint32_t SR2;				/* 						- offset:  */
	__vo uint32_t CCR;				/* 						- offset:  */
	__vo uint32_t TRISE;			/* 						- offset:  */
	__vo uint32_t FLTR;				/* 						- offset:  */
}I2C_RegDef_t;


/* peripheral register definition structure for USART */

typedef struct
{
	__vo uint32_t SR;  			    /* 						- offset:  */
	__vo uint32_t DR;				/*  					- offset:  */
	__vo uint32_t BRR;				/*  					- offset:  */
	__vo uint32_t CR1;				/* 						- offset:  */
	__vo uint32_t CR2;				/* 						- offset:  */
	__vo uint32_t CR3;				/* 						- offset:  */
	__vo uint32_t USART_GTPR;		/* 						- offset:  */

}USART_RegDef_t;


/* peripheral register definition structure for EXTI */

typedef struct
{
	__vo uint32_t IMR;  			/* 						- offset: 0x00 */
	__vo uint32_t EMR;				/*  					- offset: 0x04 */
	__vo uint32_t RTSR;				/*  					- offset: 0x08 */
	__vo uint32_t FTSR;				/* 						- offset: 0x0C */
	__vo uint32_t SWIER;			/* 						- offset: 0x10 */
	__vo uint32_t PR;				/* 						- offset: 0x14 */
}EXTI_RegDef_t;


/* peripheral register definition structure for SYSCFG */

typedef struct
{
	__vo uint32_t MEMRMP;  			/* 						- offset: 0x00 */
	__vo uint32_t PMC;				/*  					- offset: 0x04 */
	__vo uint32_t EXTICR[4];		/*  					- offset: 0x08 - 0x14 */
	uint32_t RESERVED1[2];			/* 						- offset: 0x1C */
	__vo uint32_t CMPCR;			/* 						- offset: 0x20 */
	uint32_t RESERVED2[2];			/* 						- offset: 0x24 - 0x28 */
	__vo uint32_t CFGR;				/* 						- offset: 0x2C */
}SYSCFG_RegDef_t;


/* peripheral definitions Macro*/

#define GPIOA  			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI  			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4			((USART_RegDef_t*)UART4_BASEADDR)
#define UART5			((USART_RegDef_t*)UART5_BASEADDR)
#define USART6			((USART_RegDef_t*)USART6_BASEADDR)


/* Clock Enable Macros for GPIOx peripherals */

#define GPIOA_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 7 ))
#define GPIOI_PCLK_EN()    ( RCC->AHB1ENR |= ( 1 << 8 ))


/* Clock Enable Macros for I2Cx peripherals */

#define I2C1_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 21 ))
#define I2C2_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 22 ))
#define I2C3_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 23 ))


/* Clock Enable Macros for SPIx peripherals */

#define SPI1_PCLK_EN()	  		( RCC->APB2ENR |= ( 1 << 12 ))
#define SPI2_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 14 ))
#define SPI3_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 15 ))
#define SPI4_PCLK_EN()    		( RCC->APB2ENR |= ( 1 << 13 ))


/* Clock Enable Macros for USARTx peripherals */

#define USART1_PCLK_EN()	  	( RCC->APB2ENR |= ( 1 << 4 ))
#define USART2_PCLK_EN()	  	( RCC->APB1ENR |= ( 1 << 17 ))
#define USART3_PCLK_EN()	  	( RCC->APB1ENR |= ( 1 << 18 ))
#define UART4_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 19 ))
#define UART5_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 20 ))
#define USART6_PCLK_EN()	  	( RCC->APB2ENR |= ( 1 << 5 ))
#define UART7_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 30 ))
#define UART8_PCLK_EN()	  		( RCC->APB1ENR |= ( 1 << 31 ))


/* Clock Enable Macros for SYSCFG peripherals */

#define SYSCFG_PLCK_EN()		( RCC->APB2ENR |= (1 << 14))


/* Clock Disable Macros for GPIOx peripherals */

#define GPIOA_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 7 ))
#define GPIOI_PCLK_DI()    ( RCC->AHB1ENR &= ~( 1 << 8 ))


/* Clock Disable Macros for I2Cx peripherals */

#define I2C1_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 21 ))
#define I2C2_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 22 ))
#define I2C3_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 23 ))


/* Clock Disable Macros for SPIx peripherals */

#define SPI1_PCLK_DI()	  		( RCC->APB2ENR &= ~( 1 << 12 ))
#define SPI2_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 14 ))
#define SPI3_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 15 ))
#define SPI4_PCLK_DI()    		( RCC->APB2ENR &= ~( 1 << 13 ))


/* Clock Disable Macros for USARTx peripherals */

#define USART1_PCLK_DI()	  	( RCC->APB2ENR &= ~( 1 << 4 ))
#define USART2_PCLK_DI()	  	( RCC->APB1ENR &= ~( 1 << 17 ))
#define USART3_PCLK_DI()	  	( RCC->APB1ENR &= ~( 1 << 18 ))
#define UART4_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 19 ))
#define UART5_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 20 ))
#define USART6_PCLK_DI()	  	( RCC->APB2ENR &= ~( 1 << 5 ))
#define UART7_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 30 ))
#define UART8_PCLK_DI()	  		( RCC->APB1ENR &= ~( 1 << 31 ))


/* Macros to reset GPIOx peripherals */

#define GPIOA_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 0)); ( RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 1)); ( RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 2)); ( RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 3)); ( RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 4)); ( RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 5)); ( RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 6)); ( RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 7)); ( RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()		do{( RCC->AHB1RSTR |= (1 << 8)); ( RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)	( 	(x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
										(x == GPIOE)?4:\
										(x == GPIOF)?5:\
										(x == GPIOG)?6:\
										(x == GPIOH)?7:\
										(x == GPIOI)?8:0	)

/*
 *	IRQ(Interrupt Request) Numbers of STM32F407x MCU
 *	NOTE : update these macros with valid value according to your MCU
 */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define NVIC_IRQ_PRIO0			0
#define NVIC_IRQ_PRIO1			1
#define NVIC_IRQ_PRIO2			2
#define NVIC_IRQ_PRIO3			3
#define NVIC_IRQ_PRIO4			4
#define NVIC_IRQ_PRIO5			5
#define NVIC_IRQ_PRIO6			6
#define NVIC_IRQ_PRIO7			7
#define NVIC_IRQ_PRIO8			8
#define NVIC_IRQ_PRIO9			9
#define NVIC_IRQ_PRIO10			10
#define NVIC_IRQ_PRIO11			11
#define NVIC_IRQ_PRIO12			12
#define NVIC_IRQ_PRIO13			13
#define NVIC_IRQ_PRIO14			14
#define NVIC_IRQ_PRIO15			15

//Some generic macros

#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET



/*
 ***********	Bit Position definitions of SPI peripheral **********
 *
 */

/* Bit Position definitions SPI_CR1 */

#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15


/* Bit Position definitions SPI_CR2 */

#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7


/* Bit Position definitions SPI_SR */

#define SPI_SR_RXNE					0
#define SPI_SR_TXE					1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8


/*
 ***********	Bit Position definitions of I2C peripheral **********
 *
 */


/* Bit Position definitions I2C_CR1 */

#define I2C_CR1_PE					0
#define I2C_CR1_NOSTRETCH			7
#define I2C_CR1_START				8
#define I2C_CR1_STOP				9
#define I2C_CR1_ACK					10
#define I2C_CR1_SWRST				15


/* Bit Position definitions I2C_OAR1 */

#define I2C_OAR1_ADD0				0
#define I2C_OAR1_ADD71				1
#define I2C_OAR1_ADD98				8
#define I2C_OAR1_ADDMODE			15


/* Bit Position definitions I2C_CR2 */

#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN				8
#define I2C_CR2_ITEVTEN				9
#define I2C_CR2_ITBUFEN				10


/* Bit Position definitions I2C_SR1 */

#define I2C_SR1_SB					0
#define I2C_SR1_ADDR				1
#define I2C_SR1_BTF					2
#define I2C_SR1_ADD10				3
#define I2C_SR1_STOPF				4
#define I2C_SR1_RXNE				6
#define I2C_SR1_TXE					7
#define I2C_SR1_BERR				8
#define I2C_SR1_ARLO				9
#define I2C_SR1_AF					10
#define I2C_SR1_OVR					11
#define I2C_SR1_TIMEOUT				14


/* Bit Position definitions I2C_SR2 */

#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY				1
#define I2C_SR2_TRA					2
#define I2C_SR2_GENCALL				4
#define I2C_SR2_DUALF				7


/* Bit Position definitions I2C_CCR */

#define I2C_CCR_CCR					0
#define I2C_CCR_DUTY				14
#define I2C_CCR_FS					15


/*
 ***********	Bit Position definitions of USART peripheral **********
 *
 */


/* Bit Position definitions USART_SR */

#define USART_SR_PE					0
#define USART_SR_FE					1
#define USART_SR_NF					2
#define USART_SR_ORE				3
#define USART_SR_IDLE				4
#define USART_SR_RXNE				5
#define USART_SR_TC					6
#define USART_SR_TXE				7
#define USART_SR_LBD				8
#define USART_SR_CTS				9


/* Bit Position definitions USART_CR1 */

#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15


/* Bit Position definitions USART_CR2 */

#define USART_CR2_ADD				0
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14


/* Bit Position definitions USART_CR3 */

#define USART_CR3_EIE				0
#define USART_CR3_IREN				1
#define USART_CR3_IRLP				2
#define USART_CR3_HDSEL				3
#define USART_CR3_NACK				4
#define USART_CR3_SCEN				5
#define USART_CR3_DMAR				6
#define USART_CR3_DMAT				7
#define USART_CR3_RTSE				8
#define USART_CR3_CTSE				9
#define USART_CR3_CTSIE				10
#define USART_CR3_ONEBIT			11




#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
