/**
  ******************************************************************************
  * @file    main.c 
  * @author  Chen Bing
  * @version V1.1.0
  * @date    08-Nov.-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f2xx.h"

#include "MODBUS.h"
#include "MBSlave.h"

static __IO uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nTime);



/* Private define ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg TxMessage;
CanRxMsg RxMessage;

uint8_t KeyNumber = 0x0;

/* Private function prototypes -----------------------------------------------*/
void NVIC_Config(void);
void CAN1_Config(void);
void CAN2_Config(void);
void Init_RxMes(CanRxMsg *RxMessage);
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
uint16_t PrescalerValue = 0;

/* Private function prototypes -----------------------------------------------*/
void TIM_Config(void);

//static void TIM_Configuration(void);
static void USART3_Config(void); 
static void SPI_Config(void);

uint8	ADUBuff[MAX_ADU_LENGTH];  
SLAVE_INFORMATION 	 SlaveDevice = {
	0x04,					// 地址
	0x00,					// 链路层协议
	1152,					// 波特率 = BaudRate * 100	
	0,						// 奇偶校验
	2,						// 停止位
	ADUBuff					// 主机请求从帧指针
};
//uint32 T15TC_Value;
//uint32 T35TC_Value;
uint8 mb_handle_event_flag;

void StartCountT15(void)
{
  TIM_SetCounter(TIM2,0x0000);
  TIM_Cmd(TIM2, ENABLE);
}

//********************************************************************************************************
// 函数名称：StartCountT35
// 输入参数：无
// 输出参数：无
// 功能描述：使能T3.5计数。T3.5的时间值为UART传输3.5个字符的时间。
//			 注意：该函数只是使能计时，不允许在函数中产生任何延时。
//			       如果T3.5时间结束执行T35EndHandle函数。
//********************************************************************************************************
void StartCountT35(void)
{
  TIM_SetCounter(TIM3,0x0000);
  TIM_Cmd(TIM3, ENABLE);
}
void SendResponse(uint8 *buff,uint16 len)
{
	
  uint16 i;
  GPIO_SetBits (GPIOC, GPIO_Pin_4); 
	Delay(1);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
  for(i=0;i<len;i++)
  {
    USART_SendData(USART3,buff[i]);
	  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
  }
	Delay(1);
	GPIO_ResetBits (GPIOC, GPIO_Pin_4);
}

uint8 MB_GetDiscrete(uint16 Address)
{
  uint8 i;
  if(Address<4)
    i=1;
  else
    i=0;
  return i;
}
uint8 MB_GetCoils(uint16 Address)
{
  uint8 i;
  if(Address<4)
    i=1;
  else
    i=0;
  return i;
}
uint8 MB_SetCoil(uint16 Address,uint8 CoilValue)
{	
	return TRUE;
}

/***************************************************
** 配置输入寄存器的参数。
**
****************************************************/
uint16 InputRegBuffer[END_INPUT_REG_ADDR]={0,0};
//**************************************************
// 函数名称：MB_GetInputRegValue
// 输入参数：Address，寄存器地址
// 输出参数：返回寄存器值
// 功能描述：读输入寄存器值函数，访函数由用户编写
//**************************************************
uint16 MB_GetInputRegValue(uint16 Address)
{
	return InputRegBuffer[Address];

}
//**************************************************
// 函数名称：MB_SetInputRegValue
// 输入参数：Address，寄存器地址
//			 Value，设置输入寄存器的值
// 输出参数：无
// 功能描述：读输入寄存器值函数，访函数由用户编写
//**************************************************
void MB_SetInputRegValue(uint16 Address,uint16 Value)
{
//	InputRegBuffer[Address] = Value;
}
uint16 HoldRegBuffer[8];

uint16 MB_GetRegValue(uint16 Address)
{
  	//return InputRegBuffer[Address];
	  return HoldRegBuffer[Address];
	//return REG_RWs[Address*2+1]*256+REG_RWs[Address*2];
}
void MB_SetRegValue(uint16 Address,uint16 Value)
{
	  HoldRegBuffer[Address] = Value;
  //InputRegBuffer[Address] = Value;
//  REG_RWs[Address*2+1]=(u8)(Value/256);
 // REG_RWs[Address*2]=(u8)(Value%256);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure1;
	GPIO_InitTypeDef  GPIO_InitStructure2;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
	
  /* NVIC Configuration */
  NVIC_Config();
  
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Configure the GPIO_LED and RS485_DE pin */
  GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure1.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure1.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure1);

  GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure2);

 /* Enable the GPIO_INPUT Clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* Configure the GPIO_Input pin */
  GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure2);

/* USART3 configuration */
  USART3_Config();
	
	if (SysTick_Config(SystemCoreClock / 1000))
  { 
    /* Capture error */ 
    while (1);
  }
	TIM_Config();  
	GPIO_ResetBits (GPIOC, GPIO_Pin_4); 
  /* TIM2 enable counter */
  //TIM_Cmd(TIM2, ENABLE);
	//TIM_Cmd(TIM2, ENABLE);
  MBSlaveIni(&SlaveDevice);
  mb_handle_event_flag=0;

/* SPI2 configuration ---------------------------------------*/
  SPI_Config();

	/* CAN configuration */
  CAN1_Config();
	CAN2_Config();
	GPIO_SetBits (GPIOC, GPIO_Pin_0); // RUN_LED OFF
	GPIO_SetBits (GPIOC, GPIO_Pin_1); //COM_LED OFF
  
	HoldRegBuffer[0] = 199;
  HoldRegBuffer[1] = 100;
	HoldRegBuffer[2] = 1510;
	HoldRegBuffer[3] = 1320;
	HoldRegBuffer[4] = 1460;
  
	while(1)
  {

		IdleModbus();
		
  }
	
}
/**
  * @brief  Configures the CAN1.
  * @param  None
  * @retval None
  */
void CAN1_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN1_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
  GPIO_PinAFConfig(CAN1_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN | CAN1_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN1_GPIO_PORT, &GPIO_InitStructure);

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN1);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  /* CAN Baudrate = 1MBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 2;
  /* CAN Baudrate = 500KBps (CAN clocked at 30 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 4;
  /* CAN Baudrate = 250KBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 8;
  /* CAN Baudrate = 125KBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 16;
  
	
	CAN_Init(CAN1, &CAN_InitStructure);

  /* CAN filter init */
/*
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
*/  
  
  /* Enable FIFO 0 message pending Interrupt */
//  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/**
  * @brief  Configures the CAN2.
  * @param  None
  * @retval None
  */
void CAN2_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN2_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN2_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
  GPIO_PinAFConfig(CAN2_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN | CAN2_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN2_GPIO_PORT, &GPIO_InitStructure);

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CAN2);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  /* CAN Baudrate = 1MBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 2;
  /* CAN Baudrate = 500KBps (CAN clocked at 30 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  CAN_InitStructure.CAN_Prescaler = 4;
  /* CAN Baudrate = 250KBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 8;
  /* CAN Baudrate = 125KBps (CAN clocked at 30 MHz) */
  //CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  //CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
  //CAN_InitStructure.CAN_Prescaler = 16;
  
	CAN_Init(CAN2, &CAN_InitStructure);

  /* CAN filter init */

  /*
  CAN_FilterInitStructure.CAN_FilterNumber = 14;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  */
  
  /* Enable FIFO 0 message pending Interrupt */
  //CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}


/**
  * @brief  Configures the NVIC.
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;

    /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	

	
	
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	
	/* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  

}

/**
  * @brief  Initializes the Rx Message.
  * @param  RxMessage: pointer to the message to initialize
  * @retval None
  */
void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t i = 0;

  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (i = 0;i < 8;i++)
  {
    RxMessage->Data[i] = 0x00;
  }
}


static void USART3_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
	/* USART IOs configuration ***********************************/
  /* Enable GPIOC clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* Connect PC10 to USART3_Tx */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
  /* Connect PC11 to USART3_Rx*/
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
  /* Configure USART3_Tx and USART3_Rx as alternate function */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* USART configuration ***************************************/
  /* USART3 configured as follow:
  - BaudRate = 115200 baud
  - Word Length = 8 Bits
  - One Stop Bit
  - No parity
  - Hardware flow control disabled (RTS and CTS signals)
  - Receive and transmit enabled
  */
  /* Enable USART3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
  
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);   
	
	/* Enable USART3 */
  USART_Cmd(USART3, ENABLE);

}

static void SPI_Config(void)
{
	
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

	/* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Enable the GPIOB clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Connect PB13 to SPI2_SCK */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);

	/* Connect PB14 to SPI2_MISO */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);

	/* Connect PB15 to SPI2_MOSI */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* SPI configuration *****************************************/
  SPI_InitStructure.SPI_Direction =
  SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
}	
	
/**
  * @brief  Configure the TIM IRQ Handler.
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
	/* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

    /* Compute the prescaler value */
  //PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 6000000) - 1;
  
	PrescalerValue = 59;//1us
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 750;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

   TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);


  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 5750;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);

   TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  /* TIM Interrupts enable */
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);


}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
