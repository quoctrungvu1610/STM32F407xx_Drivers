/*
 * 006spi_tx_testing.c
 *
 *  Created on: Jul 1, 2024
 *      Author: ADMIN
 */

/*
 *	Test the SPI_SendData API to sent the string "Hello World" and use bellow configurations
 *	1. SPI-2 Master mode
 *	2. SCLK = max possible
 *	3. DFF = 0 and DFF = 1
 */

/*
 *	PB14 -> SPI2_MISO
 *	PB15 -> SPI2_MOSI
 *	PB13 -> SPI2_SCLK
 *	PB12 -> SPI2_NSS
 *	Alt function mode : 5
 */

#include "stm32f407xx.h"
#include <string.h>

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;// generates sclk of 8mHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;// Software slave management enabled for NSS pin

	SPI_Init(&SPI2Handle);
}

int main(void)
{
	char user_data[] = "Hello World";
	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	// This make NSS Signal internally high and avoid  MODF error

	SPI_SSIConfig(SPI2, ENABLE);

	// Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_SendData(SPI2,(uint8_t*)user_data, strlen(user_data));

	//let confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;

}
