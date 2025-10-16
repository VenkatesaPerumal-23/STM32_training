#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

void delay(void){
    for(uint32_t i=0; i<500000; i++);
}

int main(void)
{
    GPIO_Handle_t gpioLed, gpioButton, gpioLed1, gpioButton1;

    memset(&gpioLed,0,sizeof(gpioLed));
    memset(&gpioButton,0,sizeof(gpioButton));
    memset(&gpioLed1,0,sizeof(gpioLed1));
    memset(&gpioButton1,0,sizeof(gpioButton1));

    // LED config -> PD12
    gpioLed.pGPIOx = GPIOD;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


    // LED config -> PD13
    gpioLed1.pGPIOx = GPIOD;
    gpioLed1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    gpioLed1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLed1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioLed1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&gpioLed);

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&gpioLed1);

    // Button config -> interrupt

    gpioButton.pGPIOx = GPIOB;
    gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOB, ENABLE);
    GPIO_Init(&gpioButton);


    // Button config -> polling

    gpioButton1.pGPIOx = GPIOD;
    gpioButton1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    gpioButton1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpioButton1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioButton1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // enable pull-up

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&gpioButton1);



    // Map EXTI5 line to Port B
    SYSCFG_PCLK_EN();

    // Enable IRQ in NVIC
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI2, ENABLE);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI2, NVIC_IRQ_PRI15);

    while(1){
    	if(GPIO_ReadFromInputPin(GPIOD,GPIO_PIN_NO_6)==1){
    		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_13);
    		delay();
    	}
    }
}

// ISR
void EXTI2_IRQHandler(void)
{
    delay();
    GPIO_IRQHandling(GPIO_PIN_NO_2);             // clear pending bit
    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
    delay();// toggle LED
}
