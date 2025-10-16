/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Oct 8, 2025
 *      Author: venkatesaperumal.r
 */

#include"stm32f407xx_rcc_driver.h"

#include"stm32f407xx_i2c_driver.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr=SlaveAddr<<1;
	SlaveAddr&=~(1);  // slave address + write(0) bit
	pI2Cx->DR=SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr=SlaveAddr<<1;
	SlaveAddr|=(1);  // slave address + read(1) bit
	pI2Cx->DR=SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){

	uint32_t dummy_read;

	//check for device mode

	if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL)){

		//device is in master mode
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize==1){

				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR Flag
				dummy_read=pI2CHandle->pI2Cx->SR1;
				dummy_read=pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else{
			//clear the ADDR Flag
			dummy_read=pI2CHandle->pI2Cx->SR1;
			dummy_read=pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}

	}
	else{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read=pI2CHandle->pI2Cx->SR1;
		dummy_read=pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}

}

static void I2C_MasterHandlerTXEInterrupt(I2C_Handle_t *pI2CHandle){

	if(pI2CHandle->TxLen>0){

		// 1. load the data into DR
		pI2CHandle->pI2Cx->DR=*(pI2CHandle->pTxBuffer);

		// 2. decrement the len
		pI2CHandle->TxLen--;

		// 3. increment the buffer address
		pI2CHandle->pTxBuffer++;

	}

}

static void I2C_MasterHandlerRXNEInterrupt(I2C_Handle_t *pI2CHandle){

	// data reception

	if(pI2CHandle->RxSize==1){
		*pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize>1){
		if(pI2CHandle->RxLen==2){

			// clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		}

		// read the DR
		*pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen==0){

		//close the I2C data reception and notify the application

		//1. generate the stop condition
		if(pI2CHandle->Sr==I2C_DISABLE_SR)
		  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2. close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){

	// Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){

	// Implement the code to disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		if(pI2Cx==I2C1){

			I2C1_PCLK_EN();
		}
		else if(pI2Cx==I2C2){

			I2C2_PCLK_EN();
		}
		if(pI2Cx==I2C3){

			I2C3_PCLK_EN();
		}
	}
	else{
		if(pI2Cx==I2C1){

			I2C1_PCLK_DI();
		}
		else if(pI2Cx==I2C2){

			I2C2_PCLK_DI();
		}
		if(pI2Cx==I2C3){

			I2C3_PCLK_DI();
		}
	}
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}
	else{

		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi){

	if(EnOrDi == I2C_ACK_ENABLE){
		pI2Cx->CR1|=(1<<I2C_CR1_ACK);
	}
	else{
		pI2Cx->CR1&=~(1<<I2C_CR1_ACK);
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg=0;

	// clock enable
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	// ack bit
	tempreg|=pI2CHandle->I2C_Config.I2C_ACKControl<<10;
	pI2CHandle->pI2Cx->CR1=tempreg;

	// FREQ bit
	tempreg=0;
	tempreg|=RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR1=(tempreg&0x3F);

	// device own address
	tempreg=0;
	tempreg|=pI2CHandle->I2C_Config.I2C_DeviceAddress<<1;
	tempreg|=(1<<14);
	pI2CHandle->pI2Cx->OAR1=tempreg;

	// CCR value

	uint16_t ccr_value=0;
	tempreg=0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed<=I2C_SCL_SPEED_SM){

		// Standard mode
		ccr_value=(RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg|=(ccr_value & 0xFFF);
	}

	else{

		// Fast Mode
		tempreg|=(1<<15);
		tempreg|=(pI2CHandle->I2C_Config.I2C_FMDutyCycle<<14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle==I2C_FM_DUTY_2){

			ccr_value=(RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed));

		}
		else{

			ccr_value=(RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}

		tempreg|=(ccr_value&0xFFF);
		pI2CHandle->pI2Cx->CCR=tempreg;

		// TRISE
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed<=I2C_SCL_SPEED_SM){

			tempreg=(RCC_GetPCLK1Value()/1000000U)+1;
		}
		else{
			tempreg=((RCC_GetPCLK1Value()*300)/1000000000U)+1;
		}

		pI2CHandle->pI2Cx->TRISE=(tempreg&0x3F);

	}

}


void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}

}



uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName){

	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){

	// 1. Generate start condition

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Check start completed by checking SB flag in SR1 reg

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG));

	// 3. Send slave address (7 bit) + R/W bit

	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	// 4. Check address phase completed by checking ADDR flag

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG));

	// 5. Clear the ADDR flag

	I2C_ClearADDRFlag(pI2CHandle);

	// 6. Send data until its length becomes zero

	while(Len>0){

		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
		pI2CHandle->pI2Cx->DR=*pTxBuffer;
		Len--;
	}

	// 7. TXE and BTF set to 1 before generating stop condition

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_BTF_FLAG));


	// 8. Generate stop condition
	if(Sr==I2C_DISABLE_SR)
	  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){

	// 1. Generate start condition

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Check start completed by checking SB flag in SR1 reg

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG));

	// 3. Send slave address (7 bit) + R/W bit

	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	// 4. Check address phase completed by checking ADDR flag

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG));

	if(Len==1){

		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		I2C_ClearADDRFlag(pI2CHandle);

		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG));

		if(Sr==I2C_DISABLE_SR)
		  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// read the data
		*pRxBuffer=pI2CHandle->pI2Cx->DR;

	}

	if(Len>1){

		I2C_ClearADDRFlag(pI2CHandle);

		// read the data
		for(uint32_t i=Len;i>0;i--){

			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG));

			if(i==2){

				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				if(Sr==I2C_DISABLE_SR)
				  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			*pRxBuffer=pI2CHandle->pI2Cx->DR;

			pRxBuffer++;
		}

	}

	// re-enable acking

	if(pI2CHandle->I2C_Config.I2C_ACKControl==I2C_ACK_ENABLE){

		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer ;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer ;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize=Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){

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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);

}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint32_t temp1, temp2, temp3;

	temp1=pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITEVTEN);
	temp2=pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITBUFEN);

	temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_SB);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode

	if(temp1 && temp3){

		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}
	}

	temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_ADDR);

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address

	if(temp1 && temp3){

		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_BTF);

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event

	if(temp1 && temp3){

		//BTF flag is set

		if(pI2CHandle->TxRxState==I2C_BUSY_IN_TX){

			//make sure that TXE is also set

			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TxE)){

				//BTF, TXE = 1

				if(pI2CHandle->TxLen==0){

				  //1. generate the STOP condition

					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

				  //2. reset all the member elements of the handle structure

					I2C_CloseSendData(pI2CHandle);

				  //3. notify the application about transmission complete

					I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);

				}
			}

		}
		else if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX);
	}

	temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_STOPF);

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	//The below code block will not be executed by the master since STOPF will not set in master mode

	if(temp1 && temp3){

		//STOPF flag is set
		//Clear the STOPF  1) read SR1 2) Write to CR1 )

		pI2CHandle->pI2Cx->CR1|=0x0000;

		//Notify the application about transmission complete

		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);

	}

	temp3=pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TxE);

	//5. Handle For interrupt generated by TXE event

	if(temp1 && temp2 && temp3){

		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL)){

			//TXE flag is set
			//We have to do the data transmission

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){

				I2C_MasterHandlerTXEInterrupt(pI2CHandle);
			}
		}

	}

	//6. Handle For interrupt generated by RXNE event

	if(temp1 && temp2 && temp3){

		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL)){

			//RXNE flag is set

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

				I2C_MasterHandlerRXNEInterrupt(pI2CHandle);
			}
		}

	}

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

	uint32_t temp1, temp2;

	//Know the status of ITERREN control bit in the CR2
	temp2=(pI2CHandle->pI2Cx->CR2) & (1<<I2C_CR2_ITERREN);

	//check for bus error
	temp1=(pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_BERR);

	if(temp1 && temp2){

		// clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_BERR);

		// notify the application about error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	//check for arbitration lost error
	temp1=(pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_ARLO);

	if(temp1 && temp2){

		// clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_ARLO);

		// notify the application about error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

	//check for ACK failure error
	temp1=(pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_AF);

	if(temp1 && temp2){

		// clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_AF);

		// notify the application about error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

	//check for overrun/underrun error
	temp1=(pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_OVR);

	if(temp1 && temp2){

		// clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_OVR);

		// notify the application about error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

	//check for time out  error
	temp1=(pI2CHandle->pI2Cx->SR1) & (1<<I2C_SR1_TIMEOUT);

	if(temp1 && temp2){

		// clear the bus error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1<<I2C_SR1_TIMEOUT);

		// notify the application about error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

















