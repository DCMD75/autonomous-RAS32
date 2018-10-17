#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "stdio.h"
#include "LidarX4.h"

static __IO uint32_t usTicks;

void SysTick_Handler()
{
	if(usTicks)
	{
		usTicks--;
	}
}

void DelayUs(uint32_t us)
{
	usTicks = us;
	while(usTicks);
}

void DelayMs(uint32_t ms)
{
	while(ms--)
	{
		DelayUs(1000);
	}
}

void PCWrite(char* pdata)
{
	while(*pdata)
	{
		while( ! USART_GetFlagStatus(USART2, USART_FLAG_TXE) );
		USART_SendData(USART2, *pdata++);
	}
}

void PCWritePacket(char* data)
{		
	char output[20];

	sprintf(output, "PH: %u\t", ( data[1]<<8 ) + data[0]);
	PCWrite(output);
	
	sprintf(output, "CT: %u\t", data[2]);
	PCWrite(output);
	
	sprintf(output, "LSN: %u\t", data[3]);
	PCWrite(output);
	
	uint16_t value = (uint16_t)( (data[4]<<8) | data[5]);
	sprintf(output, "FSA: %u\t", value);
	PCWrite(output);
	
	value = (uint16_t)( (data[7]<<8) | data[6]);
	sprintf(output, "LSA: %u\t", value);
	PCWrite(output);
	
	value = (uint16_t)( (data[9]<<8) | data[8]);
	sprintf(output, "CS: %u\t", value);
	PCWrite(output);
	
	for (int i = 0; i < data[3]; ++i)
	{			
		value = (uint16_t)( (data[(2*i+1)+8]<<8) | data[(2*i)+8]);
		sprintf(output, "S%u: %u\t", i, value);
		PCWrite(output);
	}
	PCWrite("\n");
}

char PCRead()
{
	while( ! USART_GetFlagStatus(USART2, USART_FLAG_RXNE));
	USART_ReceiveData(USART2);
	return 'a';
}

void init(){
	/* Init clock */
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000000);
	
	/* Start RCC */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* Init variables */
	USART_InitTypeDef USARTInitStructure;
	GPIO_InitTypeDef GPIOInitStructure;
	
	/* Init USART 2 PC */
	USARTInitStructure.USART_BaudRate = 115200;
	USARTInitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTInitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USARTInitStructure.USART_Parity = USART_Parity_No;
	USARTInitStructure.USART_StopBits = USART_StopBits_1;
	USARTInitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USARTInitStructure);
	USART_Cmd(USART2, ENABLE);
	/* Init USART 2 PIN*/
	//TX
	GPIOInitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIOInitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIOInitStructure);
	//RX A3
	GPIOInitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOInitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIOInitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIOInitStructure);
}

int main()
{
	/* Init PC communication USART2 pin TX A2, RX A3 */
	init();
	/* Init lidar USART3 pin TX B10, RX B11*/
	lidarInit();

	/**
	 * Create an array to receive the lidar data and 
	 * set the length to the maximum size of a lidar packet.
	 */
	char data[LIDAR_MAX_MSG_SIZE];
	
	/* Send message to PC to signal the beginning of the programme */
	PCWrite("Start\n");
	/* Tell the lidar to start scanning */
	lidarSend(START_SCAN);
	
	/* Read lidar data and send it to PC for infinity */
	while(1)
	{
		lidarReadPacket(data);
		PCWritePacket(data);
	}
}
