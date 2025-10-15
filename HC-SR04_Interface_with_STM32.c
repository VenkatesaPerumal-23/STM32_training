#include "stm32f407xx.h"
#include <stdint.h>
#include <stdio.h>

extern void initialise_monitor_handles(void); // for semihosting

void delay_in_microseconds(int us)
{
    volatile int i;
    while(us--)
    {
        for(i = 0; i < 16; i++);
    }
}

int main(void)
{
    GPIO_Handle_t trig, echo;
    initialise_monitor_handles(); // semihosting setup

    printf("Ultrasonic Sensor Test Starting...\n");

    // --- Trigger pin PB9 ---
    trig.pGPIOx = GPIOB;
    trig.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    trig.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    trig.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    trig.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    trig.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&trig);

    // --- Echo pin PB6 ---
    echo.pGPIOx = GPIOB;
    echo.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    echo.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    echo.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    echo.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&echo);

    while(1)
    {
        volatile unsigned long count = 0;
        float distance;

        // 1. Send 10 µs trigger pulse
        GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_9, 1);
        delay_in_microseconds(10);
        GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_9, 0);

        // 2. Wait for Echo to go HIGH
        while(!GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_6));

        // 3. Count duration while Echo is HIGH
        while(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_6))
        {
            count++;
        }

        // 4. Convert count to distance (approximate)
        distance = (count * 0.0343f) / 2.0f;

        // 5. Print distance through semihosting
        printf("Distance = %.2f cm\n", distance);

        // delay for next measurement (~60 ms)
        delay_in_microseconds(60000);
    }
}
