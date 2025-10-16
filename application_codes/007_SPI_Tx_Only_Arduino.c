/*
 * 007_SPI_Tx_Only_Arduino.c
 *
 *  Created on: Oct 3, 2025
 *      Author: venkatesaperumal.r
 */


/*
 * SPI_send_data_testing.c
 *
 *  Created on: Sep 30, 2025
 *      Author: venkatesaperumal.r
 */

//  SPI1_SCK -->  PA5
//  SPI1_MOSI --> PA7

#include"stm32f407xx.h"
#include<string.h>



void SPI1_GPIOInits(void){

	GPIO_Handle_t SPIPins;
	// SPIPins1;

	SPIPins.pGPIOx=GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;



	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

//	//MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
//
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}




void SPI1_Inits(void){

	SPI_Handle_t SPI1Handle;
	SPI1Handle.pSPIx=SPI1;
	SPI1Handle.SPIConfig.SPI_BusConfig=SPI_BUS_CONFIG_FD;
	SPI1Handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI1Handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;
	SPI1Handle.SPIConfig.SPI_DFF=SPI_DFF_8BITS;
	SPI1Handle.SPIConfig.SPI_CPOL=SPI_CPOL_HIGH;
	SPI1Handle.SPIConfig.SPI_CPHA=SPI_CPHA_HIGH;
	SPI1Handle.SPIConfig.SPI_SSM=SPI_SSM_DI;
	SPI_Init(&SPI1Handle);

}

void GPIO_ButtonInit(void){

	   GPIO_Handle_t gpioButton;
	   gpioButton.pGPIOx=GPIOA;
	   gpioButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	   gpioButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	   gpioButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	   gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	   GPIO_Init(&gpioButton);

}

int main(void){

	char test_data[]="SPI3";

	GPIO_ButtonInit();

	// GPIO pins as SPI2 pins

	SPI1_GPIOInits();

	// SPI2 configurations

	SPI1_Inits();

	// SSOE Enable

	SPI_SSOEConfig(SPI1,ENABLE);


	while(1){

		while( !GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// SPI Peripheral Enable

		SPI_PeripheralControl(SPI1,ENABLE);

		// Length of Data

		uint8_t dataLength=strlen(test_data);
		SPI_SendData(SPI1,&dataLength,1);

		// Send Data

		SPI_SendData(SPI1,(uint8_t*)test_data,strlen(test_data));

		// Confirm SPI is not Busy

		while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

		// SPI Peripheral Disable

		SPI_PeripheralControl(SPI1,DISABLE);

	}

	return 0;

}
