/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Sep 22, 2025
 *      Author: venkatesaperumal.r
 */

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx.h"

/*
 * @fn						- GPIO_PeriClockControl
 *
 * @brief					- This function enables or disables clock for GPIO Port
 *
 * @param[in]				- base address of GPIO
 *
 * @param[in]				- Enable or Disable macros
 *
 * @return					- none
 *
 * @Note					- none
 *
*/


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}

	}
	else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}


/*
 * @fn						- GPIO_Init
 *
 * @brief					- This function initializes the GPIO Port
 *
 * @param[in]				- base address of GPIO

 * @return					- none
 *
 * @Note					- none
 *
*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp=0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//  mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
		pGPIOHandle->pGPIOx->MODER|=temp; //set
	}
	else{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){

			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){

			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RFT){

			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
	    SYSCFG->EXTICR[temp1] = portCode << (temp2*4);

		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp=0;

	// speed

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR|=temp;

	temp=0;

	// pull up or pull down

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->PUPDR|=temp;

	temp=0;

	// optype

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER|=temp;

	//alt function

    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN){
    	uint8_t temp1,temp2;
    	temp1=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
    	temp2=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8;
    	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
    	pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
    }

}


/*
 * @fn						- GPIO_DeInit
 *
 * @brief					- This function de-initializes the GPIO Port
 *
 * @param[in]				- base address of GPIO

 * @return					- none
 *
 * @Note					- none
 *
*/


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}

}


/*
 * @fn						- GPIO_ReadFromInputPin
 *
 * @brief					- This function reads the value from GPIO pin
 *
 * @param[in]				- base address of GPIO
 *
 * @param[in]				- Value of Pin Number

 * @return					- pin value (0 or 1)
 *
 * @Note					- none
 *
*/


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value=(uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;
}



/*
 * @fn						- GPIO_ReadFromInputPort
 *
 * @brief					- This function is used to read the GPIO Port
 *
 * @param[in]				- base address of GPIO

 * @return					- port value
 *
 * @Note					- none
 *
*/



uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;
}


/*
 * @fn						- GPIO_WriteToOutputPin

 * @brief					- This function enables to write the output to GPIO pin
 *
 * @param[in]				- base address of GPIO
 *
 * @param[in]				- Value of Pin Number
 *
 * @param[in]				- Value to write to output pins
 *
 * @return					- none
 *
 * @Note					- none
 *
*/



void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value==GPIO_PIN_SET){
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}



/*
 * @fn						- GPIO_WriteToOutputPort
 *
 * @brief					- This function is used to write to GPIO Port
 *
 * @param[in]				- base address of GPIO
 *
 * @param[in]				- Value of GPIO Port
 *
 * @return					- none
 *
 * @Note					- none
 *
*/


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;
}




/*
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					- This function is used to toggle the output pin of GPIO
 *
 * @param[in]				- base address of GPIO
 *
 * @param[in]				- Value of Pin Number
 *
 * @return					- none
 *
 * @Note					- none
 *
*/


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1<<PinNumber);
}


/*
 * @fn						- GPIO_IRQConfig
 *
 * @brief					- This function is used to configure the Interrupt Request
 *
 * @param[in]				- Value of Pin Number
 *
 * @param[in]				- IRQPriority Value
 *
 * @param[in]				- Enable or Disable Value
 *
 * @return					- none
 *
 * @Note					- none
 *
*/




void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){


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

			*NVIC_ICER2 |= (1<<IRQNumber%64);
		}
	}

}

/*
 * @fn						- GPIO_IRQHandling
 *
 * @brief					- This function is used to handle the interrupt
 *
 * @param[in]				- Pin Number
 *
 * @return					- none
 *
 * @Note					- none
 *
*/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber){

	if(EXTI->PR & (1<<PinNumber)){
		EXTI->PR |= (1<<PinNumber);
	}
}
//400 + 3 ==>  412      ----> NVIC correct ipr
//400 + 12  ==> 430     ----> NVIC wrong ipr
