/*
 * uart f407vgt6
 *
 *  Created on: Jan 13, 2022
 *      Author: Linh
*/

#include <stdint.h>

typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
}GPIO_Config_t;

typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_Config_t;

typedef struct
{
	volatile uint32_t CSR;
	volatile uint32_t RVR;
	volatile uint32_t CVR;
	volatile uint32_t CALIB;
}SYSTICK_Config_t;


USART_Config_t *pUSART2 = (USART_Config_t *)0x40004400;
GPIO_Config_t *pGPIOA = (USART_Config_t *)0x40020000;
SYSTICK_Config_t *Systick = (USART_Config_t *)0xE000E010;

/*RCC*/
uint32_t *pAHB1ENR = (uint32_t *)(0x40023800 + 0x30);
uint32_t *pAPB1ENR = (uint32_t *)(0x40023800 + 0x40);

void UART_Config();
void GPIO_Config();
uint8_t USART2_ReceiveData();
void USART2_SendData(uint16_t Data);
void Systick_Delay_ms(uint32_t u32Delay);

uint8_t u8Buffer_Transmit[5] = "abcrt";
uint16_t u16Buffer_Transmit[5] = {5,6,7,8,9};
uint8_t u8Buffer_Receive[10] = {};
uint8_t Rx=0, Tx=0;

main()
{
	GPIO_Config();
	UART_Config();
	
	
				/*Transmit with no interrupt*/
//	while(1)
//	{
//		for (int i=0; i<5; i++)
//		{
//			/*send data*/
//			pUSART2->DR = (uint8_t)u8Buffer_Transmit[i];
//			/*wait*/
//			while( ((pUSART2->SR >> 7) & 0x01 ) == 0 );
//		}
//		while(1);
//		Systick_Delay_ms(10);
//	}
		
				/*transmit with interrupt  -  chua duoc*/
//	while(1)
//	{
//		pUSART2->DR = (uint8_t)u8Buffer_Transmit[Tx];
//		Systick_Delay_ms(100);
//	}

				/*receive with interrupt*/
		while(1)
		{
		}
	
}

void UART_Config()
{
	uint32_t temp;
	
	/*enable clock usart2*/
	*pAPB1ENR |= 1<<17;
	
	/*baudrate 9600*/
	pUSART2->BRR = 0x683;
	/*1 start bit, 8 data bit, n stop bit*/
	pUSART2->CR1 &= ~(1<<12);
	/*Bit 10 PCE: Parity control disable*/
	pUSART2->CR1 &= ~(1<<10);
	/*Odd Parity*/
	//pUSART2->CR1 |= 1<<9;
	/*oversampling by 16*/
	pUSART2->CR1 &= ~(1<<15);
	/*Bit 3 TE: Transmitter enable*/
	pUSART2->CR1 |= 1<<3;
	/*Bit 2 RE: Receiver enable*/
	pUSART2->CR1 |= 1<<2;
	/*Bits 13:12 = 00: 1.5 STOP bits*/
	pUSART2->CR2 |= (3<<12);
	/*00 - hardware flow control disabled*/	
	pUSART2->CR3 &= ~(3<<8);
	/*  An USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_SR registe*/
	pUSART2->CR1 |= 1<<5;
		
	
/*
*	NVIC config
*/
	/*enable USART2 interrupt*/
	uint32_t *pISER0 = (uint32_t *)0xE000E100;
	*(pISER0 + (uint8_t)(38/32)) |= 1 << (38 - 32);

	
	/*Bit 13 UE: USART enable*/
	pUSART2->CR1 |= 1<<13;
}


void USART2_IRQHandler()
{
	if(( pUSART2->SR >> 5 & 0x01 ) == 1);
	{
		u8Buffer_Receive[Rx] = (uint8_t)pUSART2->DR;
		if(u8Buffer_Receive[Rx] == '\n') Rx=0;
		else Rx++;
	}
	/*clear flag*/
	pUSART2->SR &= ~(1<<5);
}


void GPIO_Config()
{
	uint32_t temp;
	
	/*enable clock gpioa*/
	*pAHB1ENR |= 0x01;
/*
*PA2 - TX
*/
	temp = (uint32_t)2 << (2*2);
	pGPIOA->MODER &= ~((uint32_t)3 << (2*2));
	pGPIOA->MODER |= temp;						//alternate function
	
	temp = (uint32_t)0b0111 << (4*2);
	pGPIOA->AFRL &= ~((uint32_t)0b1111 << (4*2));
	pGPIOA->AFRL |= temp;							//AF7
	
	pGPIOA->OTYPER &= ~(1<<2);				//output PP
	
	temp = ((uint32_t)2 << (2*2));
	pGPIOA->OSPEEDR &= ~((uint32_t)3 << (2*2));
	pGPIOA->OSPEEDR |= temp;					//high speed
	
	temp = (uint32_t)1 << (2*2);
	pGPIOA->PUPDR &= ~( (uint32_t)3 << (2*2) );
	pGPIOA->PUPDR |= temp;						//pull up
	
/*
*PA3 - RX
*/
	temp = (uint32_t) 2 << (2*3);
	pGPIOA->MODER &= ~((uint32_t)3 << (2*3));
	pGPIOA->MODER |= temp;						//alternate function
	
	temp = (uint32_t)0b0111 << (4*3);
	pGPIOA->AFRL &= ~((uint32_t)0b1111 << (4*3));
	pGPIOA->AFRL |= temp;							//AF7
	
	pGPIOA->OTYPER &= ~(1<<3);				//output PP
	
	temp = ((uint32_t)2 << (2*3));
	pGPIOA->OSPEEDR &= ~((uint32_t)3 << (2*3));
	pGPIOA->OSPEEDR |= temp;					//high speed
	
	temp = (uint32_t)1 << (2*3);
	pGPIOA->PUPDR &= ~( (uint32_t)3 << (2*3) );
	pGPIOA->PUPDR |= temp;						//pull up
}

void Systick_Delay_ms(uint32_t u32Delay)
{
	while(u32Delay)
	{
		/*Cortex System timer clock max 168/8 MHz*/
		Systick->RVR = 21000-1;
		Systick->CVR = 0;
		
		/*no exception*/
		/*clear counter flag*/
		/*enable counter*/
		/*processor clock - 72M*/
		Systick->CSR = 0x05;
	
		while(((Systick->CSR) & (1<<16)) == 0)
		{
			
		}
		--u32Delay;
	}
}


