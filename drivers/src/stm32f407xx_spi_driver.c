/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Sep 29, 2025
 *      Author: venkatesaperumal.r
 */

#include<stdint.h>
#include"stm32f407xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
		else if(pSPIx == SPI5){
			SPI5_PCLK_EN();
		}
		else if(pSPIx == SPI6){
			SPI6_PCLK_EN();
		}
	}
	else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
		else if(pSPIx == SPI5){
			SPI5_PCLK_DI();
		}
		else if(pSPIx == SPI6){
			SPI6_PCLK_DI();
		}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle){

	    // 1. Enable SPI peripheral clock
	    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	    uint32_t tempreg = 0;

	    // 2. Configure device mode (Master/Slave)
	    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	    // 3. Configure bus type
	    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	    {
	        tempreg &= ~(1 << SPI_CR1_BIDIMODE);   // full-duplex
	    }
	    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	    {
	        tempreg |= (1 << SPI_CR1_BIDIMODE);    // half-duplex
	    }
	    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	    {
	        tempreg &= ~(1 << SPI_CR1_BIDIMODE);   // unidirectional receive only
	        tempreg |= (1 << SPI_CR1_RXONLY);
	    }

	    // 4. Configure clock speed
	    tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	    // 5. Configure data frame format
	    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	    // 6. Configure CPOL and CPHA
	    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	    // 7. Configure software slave management
	    if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_EN)
	    {
	        tempreg |= (1 << SPI_CR1_SSM);  // Enable SSM
	    }
	    else
	    {
	        tempreg &= ~(1 << SPI_CR1_SSM);
	        // For hardware NSS, make sure NSS pin is high externally
	    }

	    // 8. Write configuration to CR1
	    pSPIHandle->pSPIx->CR1 = tempreg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        // wait until TXE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // check DFF
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16-bit DFF
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len -= 2;
            pTxBuffer += 2;
        }
        else
        {
            // 8-bit DFF
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }

}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len){

    while(Len > 0)
    {
        // wait until RXNE is set
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

        // check DFF
        if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
        {
            // 16-bit DFF
            *((uint16_t*)pRxBuffer)=pSPIx->DR;
            Len -= 2;
            pRxBuffer += 2;
        }
        else
        {
            // 8-bit DFF
            *pRxBuffer=pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }

}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		if(IRQNumber <= 31){

			*NVIC_ISER0 |= (1<<IRQNumber);

		}
		else if(IRQNumber > 31   && IRQNumber < 64){

			*NVIC_ISER1 |= (1<<IRQNumber%32);
		}

		else if(IRQNumber >= 64    && IRQNumber < 96){

			*NVIC_ISER2 |= (1<<IRQNumber%32);
		}

	}
	else{
		if(IRQNumber <= 31){

			*NVIC_ICER0 |= (1<<IRQNumber);

		}
		else if(IRQNumber > 31   && IRQNumber < 64){

			*NVIC_ICER1 |= (1<<IRQNumber%32);
		}

		else if(IRQNumber >= 64    && IRQNumber < 96){

			*NVIC_ICER2 |= (1<<IRQNumber%32);
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if(state!=SPI_BUSY_IN_TX){

		// 1. Save the TX Buffer address and its len
		pSPIHandle->pTxBuffer=pTxBuffer;
		pSPIHandle->TxLen=Len;

		// 2. Mark SPI state as BUSY in transmission state
		// so no other code can take over same SPI peripheral
		// until transmission over
		pSPIHandle->TxState=SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt when
		// TXE flag is set
		pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_TXEIE);
	}
	// 4. Data transmission handled by ISR code

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RxState;

	if(state!=SPI_BUSY_IN_RX){

		// 1. Save the RX Buffer address and its len
		pSPIHandle->pRxBuffer=pRxBuffer;
		pSPIHandle->RxLen=Len;

		// 2. Mark SPI state as BUSY in receive state
		// so no other code can take over same SPI peripheral
		// until reception over
		pSPIHandle->RxState=SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt when
		// RXNE flag is set
		pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_RXNEIE);
	}
	// 4. Data transmission handled by ISR code

	return state;
}

// helper function

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen){

		// TxLen is zero, so close the SPI communication and
		// inform the application.

		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
	   *((uint16_t*)pSPIHandle->pRxBuffer)=(uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen-=2;
		(uint16_t*)pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit DFF
	    *pSPIHandle->pRxBuffer=(uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen){

		// RxLen is zero, so close the SPI communication and
		// inform the application.

		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;

	if(pSPIHandle->TxState!=SPI_BUSY_IN_TX){

		temp=pSPIHandle->pSPIx->DR;
		temp=pSPIHandle->pSPIx->SR;

	}
	(void)temp;

	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}


void SPI_IRQHandling(SPI_Handle_t *pHandle){

	uint8_t temp1, temp2;

	//TXE

	temp1=pHandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2=pHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);

	if(temp1 && temp2){

		spi_txe_interrupt_handle(pHandle);
	}

	//RXNE

	temp1=pHandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2=pHandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);

	if(temp1 && temp2){

		spi_rxne_interrupt_handle(pHandle);
	}

	//OVR

	temp1=pHandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2=pHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);

	if(temp1 && temp2){

		spi_ovr_err_interrupt_handle(pHandle);
	}


}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer=NULL;
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState=SPI_READY;

}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer=NULL;
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState=SPI_READY;

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){

	uint8_t temp;
	temp=pSPIx->DR;
	temp=pSPIx->SR;
	(void)temp;
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= 1<<SPI_CR1_SSI;
	}
	else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR2 |= 1<<SPI_CR2_SSOE;
	}
	else{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){

	// weak implementation, user application may override this function.
}


