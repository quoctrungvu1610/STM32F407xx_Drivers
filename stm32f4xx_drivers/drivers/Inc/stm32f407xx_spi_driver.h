/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jun 29, 2024
 *      Author: ADMIN
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/* Configuration structure for SPIx peripheral */

typedef struct
{
	uint8_t SPI_DeviceMode; // Slave or Master
	uint8_t SPI_BusConfig;	// Full-Duplex, Half-Duplex, Simplex
	uint8_t SPI_SclkSpeed;	//
	uint8_t SPI_DFF;		// 8 or 16 bit Data Frame
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/* Handle structure for SPIx peripheral */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0


/*
 * @SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD					1    //Full-Duplex
#define SPI_BUS_CONFIG_HD					2	 //Half-Duplex
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY		3    //Simplex Tx-Only
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3	 //Simplex Rx-Only


/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2					0    //fpclk divided by 2
#define SPI_SCLK_SPEED_DIV4					1	 //fpclk divided by 4
#define SPI_SCLK_SPEED_DIV8					2    //fpclk divided by 8
#define SPI_SCLK_SPEED_DIV16				3	 //fpclk divided by 16
#define SPI_SCLK_SPEED_DIV32				4	 //fpclk divided by 32
#define SPI_SCLK_SPEED_DIV64				5	 //fpclk divided by 64
#define SPI_SCLK_SPEED_DIV128				6	 //fpclk divided by 128
#define SPI_SCLK_SPEED_DIV256				7	 //fpclk divided by 256


/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS 						0
#define SPI_DFF_16BITS 						1


/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH 						1
#define SPI_CPOL_LOW						0


/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH 						1
#define SPI_CPHA_LOW						0


/*
 * @SPI_SSM
 */

#define SPI_SSM_EN 							1
#define SPI_SSM_DI							0


/*
 * SPI Related status flags definitions
 */

#define SPI_TXE_FLAG						( 1 << SPI_SR_TXE )
#define SPI_RXNE_FLAG						( 1 << SPI_SR_RXNE )
#define SPI_BUSY_FLAG						( 1 << SPI_SR_BSY )



/*-----API support by this driver-----*/


/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


/*
 * IRQ configuration and ISR handling
 */
void SPI(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
