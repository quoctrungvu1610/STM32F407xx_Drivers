# Drivers for STM32F407xx MCUs
## This driver provides several APIs for STM32F407xx series microcontrollers:
* GPIO Configuration
* SPI Communication Protocol
* I2C Communication Protocol
* USART Communication Protocol

Drivers and Examples can be found at ***stm32f4xx_drivers/drivers*** and ***stm32f4xx_drivers/Src***

## Examples 

### Examples can be found at ***stm32f4xx_drivers/Src***
### 1. Example 008spi_cmd_handling.c
This example is used to communicate between Stm32f407VG and Arduino UNO using the SPI protocol
#### Connection diagrams:
| STM32F407xx | Logic Level Converter |         Arduino         |
|:-----------:|:---------------------:|:-----------------------:|
|    5V Pin   |         HV Pin        |                         |
|    3V Pin   |         LV Pin        |                         |
|     GND     |       GND - GND       |           GND           |
|   PB15 Pin  |       LV1 - HV1       |          Pin 11         |
|   PB13 Pin  |       LV2 - HV2       |          Pin 13         |
|   PB14 Pin  |       LV3 - HV3       |          Pin 12         |
|             |                       | Pin 10 (Connect to GND) |

#### How to test:
1. Build and Upload the sketch from ***stm32f4xx_drivers/Src/008spi_cmd_handling.c*** to STM32F407xx board.
2. Build and Upload the sketch from ***Arduino/spi/002SPISlaveCmdHandling.ino*** to Arduino UNO board.
3. Connect a LED to pin 9 of Arduino UNO Board. 
4. Open the Serial Monitor on Arduino IDE, then press the User Button on STM23F407xx board.

### 2. Example 009i2c_master_tx_testing.c
This example is used to communicate between Stm32f407VG and Arduino UNO using the I2C protocol
#### Connection diagrams:
|  STM32F407xx  | Logic Level Converter | Arduino |
|:-------------:|:---------------------:|:-------:|
|     5V Pin    |         HV Pin        |         |
|     3V Pin    |         LV Pin        |         |
|      GND      |       GND - GND       |   GND   |
| PB6 Pin (SCL) |       LV1 - HV1       |  Pin A5 |
| PB9 Pin (SDA) |       LV2 - HV2       |  Pin A4 |

#### How to test:
1. Build and Upload the sketch from ***stm32f4xx_drivers/Src/009i2c_master_tx_testing.c*** to STM32F407xx series.
2. Build and Upload the sketch from ***Arduino/i2c/001I2CSlaveRxString.ino*** to Arduino UNO board.
4. Open the Serial Monitor on Arduino IDE, then press the User Button on STM23F407xx board.

### 3. Example 015uart_tx.c
This example is used to communicate between Stm32f407VG and Arduino UNO using the UART protocol
#### Connection diagrams:
|  STM32F407xx | Logic Level Converter |   Arduino  |
|:------------:|:---------------------:|:----------:|
|    5V Pin    |         HV Pin        |            |
|    3V Pin    |         LV Pin        |            |
|      GND     |       GND - GND       |     GND    |
| PA2 Pin (Tx) |       LV1 - HV1       | Pin 0 (Rx) |
| PA3 Pin (Rx) |       LV2 - HV2       | Pin 1 (Tx) |

#### How to test:
1. Build and Upload the sketch from ***stm32f4xx_drivers/Src/015uart_tx.c.c*** to STM32F407xx series.
2. Build and Upload the sketch from ***Arduino/uart/001UARTRxString.ino*** to Arduino UNO board.
4. Open the Serial Monitor on Arduino IDE, then press the User Button on STM23F407xx board.
