/*
 * 009spi_message_rcv_it.c
 *
 *  Created on: Oct 20, 2025
 *      Author: venkatesaperumal.r
 */

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"

extern void initialise_monitor_handles(void);


SPI_Handle_t SPI2handle;

#define MAX_LEN 500

char rcvBuff[MAX_LEN];

volatile char ReadByte;

volatile uint8_t rcvStop=0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable=0;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

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

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}


/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */

void Slave_GPIO_InterruptPinInit(void){

	GPIO_Handle_t SPIInitPin;
	memset(&SPIInitPin,0,sizeof(SPIInitPin));

	SPIInitPin.pGPIOx=GPIOD;
	SPIInitPin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_6;
	SPIInitPin.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT;
	SPIInitPin.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	SPIInitPin.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PIN_PU;

	GPIO_Init(&SPIInitPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
}


int main(void){

	initialise_monitor_handles();

	uint8_t dummy=0xff;

	printf("Dummy data\n");

	Slave_GPIO_InterruptPinInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	printf("SSOE Enabled\n");

	SPI_IRQInterruptConfig(IRQ_NO_SPI2,ENABLE);

	printf("Interrupt Enabled\n");

	while(1){

		rcvStop=0;

		while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

		printf("data available now\n");

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,DISABLE);

		SPI_PeripheralControl(SPI2, ENABLE);

		printf("Peripheral Enabled\n");

		while(!rcvStop){

		 /* fetch the data from the SPI peripheral byte by byte in interrupt mode */

			while(SPI_SendDataIT(&SPI2handle,&dummy,1)==SPI_BUSY_IN_TX);
			//printf("send data complete\n");
			while(SPI_ReceiveDataIT(&SPI2handle,&ReadByte,1)==SPI_BUSY_IN_RX);
			//printf("receive data complete\n");

		}


		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Received Data = %s\n",rcvBuff);

		dataAvailable=0;

		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

	}
	return 0;
}


/* Runs when a data byte is received from the peripheral over SPI*/

void SPI2_IRQHandler(void){

	SPI_IRQHandling(&SPI2handle);

}

/* Slave data available interrupt handler */

void EXTI9_5_IRQHandler(void){

	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable=1;
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){

	static uint32_t i=0;

	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */

	if(AppEv==SPI_EVENT_RX_CMPLT){

		rcvBuff[i++]=ReadByte;
		if(ReadByte=='\0' || (i==MAX_LEN)){
			rcvStop=1;
			rcvBuff[i-1]='\0';
			i=0;
		}
	}

}


