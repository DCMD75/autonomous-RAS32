/**
 * YDLidar X4 driver
 */
 
#include "LidarX4.h"

#include "string.h"

#include "stm32f10x_usart.h"

/**
 * Initalize the communication with the lidar 
 * by configuring the pin B10 and B11 and the 
 * usart3.
 * RX B11 green
 * TX B10 yellow
 */
void lidarInit(){
	// Local init typdef structure to initalize the usart and pin
	USART_InitTypeDef USARTInitStructure;
	GPIO_InitTypeDef GPIOInitStructure;
	
	// Start the rcc for usart3 and GPIOB
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/** 
	 * The usart3 is configured with the Lidar baud rate 
   * which is 128000 and the other communitcation parameters.
	 */
	USARTInitStructure.USART_BaudRate = 128000;
	USARTInitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTInitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USARTInitStructure.USART_Parity = USART_Parity_No;
	USARTInitStructure.USART_StopBits = USART_StopBits_1;
	USARTInitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USARTInitStructure);
	// Start the USART3
	USART_Cmd(USART3, ENABLE);
  
	/**
	 * Initalize the GPIO corresponding to the use of the 
	 * USART3. Which are :
	 * TX pin B10 (should be wired with the yellow wire)
	 * RX pin B11 (should be wired with the green wire)
	 */
	GPIOInitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIOInitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIOInitStructure);
	GPIOInitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOInitStructure.GPIO_Pin = GPIO_Pin_11;	
	GPIOInitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIOInitStructure);
}

/**
 * Send a command to the lidar could be
 * A command is made of the start 0xA5 and the command 0xYY
 */
void lidarSend(uint16_t command){
	/* Use in the for loop, represents the index of the array */
	uint8_t i;
	/* Store the data to be send to the lidar, byte in real 
	 but uint16_t are needed for the send function */
	uint16_t data[2];
	/* The header of the message should be this value */
	data[0] = 0xA5;
	/* Then add the command you want to send */
	data[1] = command;
	
	/* For all index in the array */
	for( i = 0; i < 2; ++i)
	{
		/* Wait for the transmit buffer to be empty */
		while( ! USART_GetFlagStatus(USART3, USART_FLAG_TXE) );
		/* Send data byte with USART */
		USART_SendData(USART3, data[i]);
	}
}

void lidarReadPacket(char* data){
	/* Flag set when the header of a packet is found */
	int flag = 1;
	/* Index of array */
	uint8_t i;
	/* */
	int CT;
	/* */
	int LSN;
	/* Collect the entier packet from lidar (header, content, ...) */
	//char data[LIDAR_MAX_MSG_SIZE];
	
	/* Clear all previous data in the array */
	memset(data, 0, LIDAR_MAX_MSG_SIZE);
	
	/* Get the start of the packet */
	while( flag ){
		data[0] = data[1];
		
		while( ! USART_GetFlagStatus(USART3, USART_FLAG_RXNE) );
		data[1] = USART_ReceiveData(USART3);
		
		if( (data[0] == (char)0xaa) && (data[1] == 0x55) ){
			flag = 0;
		}
	}
	
	/* Get the packet length CT and type LSN */
	for( i = 2; i < 4; ++i){
		/* Wait for data to come in */
		while( ! USART_GetFlagStatus(USART3, USART_FLAG_RXNE) );
		/* Read the data from usart3 */
		data[i] = USART_ReceiveData(USART3);
	}

	/* Get data according to length and type */
	CT = data[2];
	LSN = data[3];
	
	/* If CT =0 it means that it's a point cloud packet */
	if(CT == 0x00){
		/**
		 * Read the number of packet given by LSN with the 
		 * folowing principle:
		 * LSN is the number of packet, a packet is 2 byte long 
		 * and we also have FSA, LSA and CS packet.
		 * so there are 2*(LSN + 3) byte to read.
		 */
		for (i = 4; i < ( ( LSN + 3 ) << 1 ) ; ++i){ 
			while( ! USART_GetFlagStatus(USART3, USART_FLAG_RXNE) );
			data[i] = USART_ReceiveData(USART3);
		}
	}
}
