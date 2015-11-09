#include <stdint.h>
#include <stdbool.h>
#include <stm32l1xx.h>
#include <stdio.h>
/*
#if 1
#pragma import(__use_no_semihosting)             
//               
struct __FILE 
{ 
	int handle; 
	
}; 
 
FILE __stdout;       
  
_sys_exit(int x) 
{ 
	x = x; 
} 
//
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 
//end
*/
//////////////////////////////////////////////////////////////////

volatile char StringLoop[] = "The quick brown fox jumps over the lazy dog\r\n";

unsigned char UART_CRYST_TX_BUF[128]={0};
unsigned char UART_CRYST_RX_BUF[128]={0};
unsigned char UART_CRYST_TX_LENGTH;

static unsigned char  fac_us=0;
static unsigned short fac_ms=0;

extern void SWO_Enable( void );

void delay_init(unsigned char SYSCLK)
{
 SysTick->VAL=0X00000000;
 SysTick->CTRL&=0xfffffffb;
 fac_us=SYSCLK/8;      
 fac_ms=(unsigned short)fac_us*1000;
SysTick->CTRL&=0XFFFFFFFE;
SysTick->VAL=0X00000000;
 
}            
void delay_ms(unsigned short nms)
{    
 SysTick->LOAD=(unsigned long)nms*fac_ms;
 SysTick->CTRL|=0x01;
 while(!(SysTick->CTRL&(1<<16)));
 SysTick->CTRL&=0XFFFFFFFE;
 SysTick->VAL=0X00000000; 
}   
void delay_us(unsigned long Nus)
{ 
 SysTick->LOAD=Nus*fac_us;
 SysTick->CTRL|=0x01;
 while(!(SysTick->CTRL&(1<<16)));
 SysTick->CTRL=0X00000000;
 SysTick->VAL=0X00000000;
} 
 
int csac_uart_send(unsigned char byte)
{
	while (!(USART1->SR & USART_FLAG_TXE));
	USART1->DR = byte;
	return 0;
}
int csac_uart_read()
{
	int ch = -1;
	if (USART1->SR & USART_FLAG_RXNE)
	{
		ch = USART1->DR & 0xff;
	}
	
	return ch;
}

int gps_uart_send(unsigned char byte)
{
	while (!(USART2->SR & USART_FLAG_TXE));
	USART2->DR = byte;
	return 0;
}
int gps_uart_read()
{
	int ch = -1;
	if (USART2->SR & USART_FLAG_RXNE)
	{
		ch = USART2->DR & 0xff;
	}
	
	return ch;
}


int daq_uart_send(unsigned char byte)
{
	while (!(USART3->SR & USART_FLAG_TXE));
	USART3->DR = byte;
	return 0;
}
int daq_uart_read()
{
	int ch = -1;
	if (USART3->SR & USART_FLAG_RXNE)
	{
		ch = USART3->DR & 0xff;
	}
	
	return ch;
}

void csac_uart_wait_rcv()
{
	while(!(USART1->SR & USART_FLAG_RXNE))
		delay_ms(1);
}
void gps_uart_wait_rcv()
{
	while(!(USART2->SR & USART_FLAG_RXNE))
		delay_ms(1);
}
void daq_uart_wait_rcv()
{
	while(!(USART3->SR & USART_FLAG_RXNE))
		delay_ms(1);
}

/*----CSAC UART1 PA9 TX PA10 RX------*/
#define UART1_GPIO_TX			GPIO_Pin_9
#define UART1_GPIO_TX_SOURCE	GPIO_PinSource9
#define UART1_GPIO_RX			GPIO_Pin_10
#define UART1_GPIO_RX_SOURCE	GPIO_PinSource10
#define UART1_GPIO_AF			GPIO_AF_USART1
#define UART1_GPIO				GPIOA
/*----GPS UART2 PA2 TX PA3 RX-------*/
#define UART2_GPIO_TX			GPIO_Pin_2
#define UART2_GPIO_TX_SOURCE	GPIO_PinSource2
#define UART2_GPIO_RX			GPIO_Pin_3
#define UART2_GPIO_RX_SOURCE	GPIO_PinSource3
#define UART2_GPIO_AF			GPIO_AF_USART2
#define UART2_GPIO				GPIOA
/*----DAQ UART3 PB10 TX PB11 RX-----*/
#define UART3_GPIO_TX			GPIO_Pin_10
#define UART3_GPIO_TX_SOURCE	GPIO_PinSource10
#define UART3_GPIO_RX			GPIO_Pin_11
#define UART3_GPIO_RX_SOURCE	GPIO_PinSource11
#define UART3_GPIO_AF			GPIO_AF_USART3
#define UART3_GPIO				GPIOB

/*-----CSAC UART CONFIG  UART1-----*/
int CSAC_Uart_Config()
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* Enable USART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);   //usart1 apb2

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(UART1_GPIO, UART1_GPIO_TX_SOURCE, UART1_GPIO_AF);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(UART1_GPIO, UART1_GPIO_RX_SOURCE, UART1_GPIO_AF);

	/* Configure USART Tx, Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX | UART1_GPIO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 57600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
    
	/* Enable USART */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//NVIC_EnableIRQ(USART1_IRQn);
	
	USART_Cmd(USART1, ENABLE);
	return 0;
}
void CSAC_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*-----GPS UART CONFIG  UART2-----*/
int GPS_Uart_Config()
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(UART2_GPIO, UART2_GPIO_TX_SOURCE, UART2_GPIO_AF);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(UART2_GPIO, UART2_GPIO_RX_SOURCE, UART2_GPIO_AF);

	/* Configure USART Tx, Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = UART2_GPIO_TX | UART2_GPIO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART2_GPIO, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
    
	/* Enable USART */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//NVIC_EnableIRQ(USART2_IRQn);
	USART_Cmd(USART2, ENABLE);
	return 0;
}
void GPS_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*-----DAQ UART CONFIG  UART3-----*/
int DAQ_Uart_Config()
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	/* Enable USART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Connect PXx to USARTx_Tx */
	GPIO_PinAFConfig(UART3_GPIO, UART3_GPIO_TX_SOURCE, UART3_GPIO_AF);

	/* Connect PXx to USARTx_Rx */
	GPIO_PinAFConfig(UART3_GPIO, UART3_GPIO_RX_SOURCE, UART3_GPIO_AF);

	/* Configure USART Tx, Rx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = UART3_GPIO_TX | UART3_GPIO_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART3_GPIO, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	/* Enable USART */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//NVIC_EnableIRQ(USART3_IRQn);
	USART_Cmd(USART3, ENABLE);
	return 0;
}
void DAQ_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void UART_CRYST_STATUS()
{
	unsigned char iIndex = 0;
	UART_CRYST_TX_BUF[iIndex++] = '!';
	UART_CRYST_TX_BUF[iIndex++] = '^';
	UART_CRYST_TX_BUF[iIndex++] = 0x0D;
	UART_CRYST_TX_BUF[iIndex++] = 0x0A;
	UART_CRYST_TX_LENGTH = iIndex;
}

/*void USART3_IRQHandler(void)  
{  
		USART_SendData(USART3, 0x49);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
}  
*/
int main(void)
{
	unsigned char i,j;
	//unsigned char rxdata[10]={0};
	//static int tx_index = 0;  
	//static int rx_index = 0;
	delay_init(32);
	SWO_Enable();
	printf("in main\n");
	
	//GPS_NVIC_Config();
	//GPS_Uart_Config();
	
	//DAQ_Uart_Config();
	//DAQ_NVIC_Config();
	CSAC_Uart_Config();
	
	UART_CRYST_STATUS();
	//csac_uart_wait_rcv();
	for(i=0;i<UART_CRYST_TX_LENGTH;i++)
	{
		csac_uart_send(UART_CRYST_TX_BUF[i]);
	}
	

	while(1)
	{
		/*
		for(i=0;i<UART_CRYST_TX_LENGTH;i++)
		{
			csac_uart_send(UART_CRYST_TX_BUF[i]);
			//USART_SendData(USART1, UART_CRYST_TX_BUF[i]);
			//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		}
		*/
		
		//gps_uart_wait_rcv();
		//gps_uart_send((unsigned char)gps_uart_read());
		
		//csac_uart_wait_rcv();
		//csac_uart_send(0x55);
		//USART_SendData(USART1, UART_CRYST_TX_BUF[2]);
		//while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
		
		//for(i=0;i<UART_CRYST_TX_LENGTH;i++)
		//{
		//	csac_uart_send(UART_CRYST_TX_BUF[i]);
		//}
		
		
		for(j=0;j<128;j++)
		{
			UART_CRYST_RX_BUF[j]=csac_uart_read();
		}
		
		//daq_uart_wait_rcv();
		//daq_uart_send((unsigned char)daq_uart_read());
		for(i = 0;i < 128; i++)
		printf("CSAC %c\n",UART_CRYST_RX_BUF[i]);
		//csac_uart_wait_rcv();
		//csac_uart_send((unsigned char)csac_uart_read());
	}
}
