/****************************************Copyright (c)**************************************************
**                               广州周立功单片机发展有限公司
**                                     研    究    所
**                                        产品一部 
**
**                                 http://www.zlgmcu.com
**
**--------------文件信息--------------------------------------------------------------------------------
**文   件   名: MBSlave.c
**创   建   人: 周立山
**最后修改日期: 2005年7月29日
**描        述: MODBUS RTU 协议栈
**
**--------------历史版本信息----------------------------------------------------------------------------
** 创建人: 周立山
** 版  本: 1.0a
** 日　期: 2005年7月29日
** 描　述: 原始版本
**
**------------------------------------------------------------------------------------------------------
** 修改人: 
** 版  本: 
** 日　期: 
** 描　述: 
**
**--------------当前版本修订------------------------------------------------------------------------------
** 修改人: 
** 日　期: 
** 描　述: 
**
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
#include	"modbus_config.h"
#include	"modbus.h"
#include    "MBSlave.h"


#ifdef UCOSII
	OS_EVENT *mb_handle_event_ptr;
#endif
extern uint8 mb_handle_event_flag;


extern uint16 Modbus_CRC16(uint8 *Buff_addr,uint16 len);

extern SendResponse(uint8 *buff,uint16 len);

// 功能代码处理函数数组
extern AT_CODE FUNCTION_ARRAY	FCodeHandleArray[MAX_FUNCTION];			  


//********************************************************************************************************
// 函数名称：MBSlaveIni
// 输入参数：DevicInfPtr，从机描述信息数据结构指针
// 输出参数：无
// 功能描述：MODBUS从机初始化
//********************************************************************************************************
void MBSlaveIni(SLAVE_INFORMATION *DevicInfPtr)
{
	
	ADUData.Address = DevicInfPtr->Address;
	ADUData.ADUBuffPtr = DevicInfPtr->RequestPtr;
	ADUData.ADULength = 0;

//	ADUData.RedundancyCheck = 0;
	ADUData.EndT15 = FALSE;
	ADUData.FrameOK = FALSE;
	
	PDUData.PDUBuffPtr = NULL;
//	PDUData.Response.ExceptionCode = 0;

#ifdef UCOSII
	mb_handle_event_ptr = OSSemCreate(0);			// MODBUS处理信号量
#endif
}


//********************************************************************************************************
// 函数名称：FunctionHandle
// 输入参数：无
// 输出参数：无
// 功能描述：功能请求处理
//********************************************************************************************************
void FunctionHandle(void)
{
	uint8 i;
	PDU_RESPONSE Response;
	for(i=0;i<MAX_FUNCTION;i++)
	{
		if(FCodeHandleArray[i].Code==PDUData.FunctionCode)
		{
			Response.PDUDataPtr = PDUData.PDUBuffPtr;
			Response.PDUByteLength = PDUData.PDULength;

			FCodeHandleArray[i].Function( &Response );		// 执行处理返回数据结构指针
			
		//	PDUData.PDUBuffPtr = Response.PDUDataPtr;
			PDUData.PDULength = Response.PDUByteLength;
			PDUData.ExceptionCode = Response.ExceptionCode;
			return;
		}	
	}
	PDUData.ExceptionCode = EXCEPTION_CODE_1;	//	功能代码无效
}

void PackPDU2ADU(void)
{
	if(PDUData.ExceptionCode)	// 请求处理失败
	{
		*PDUData.PDUBuffPtr = PDUData.FunctionCode|0x80;
		*(PDUData.PDUBuffPtr+1) = PDUData.ExceptionCode;
		PDUData.PDULength = 2;
	}
	ADUData.ADULength = PDUData.PDULength + 3;		// ADU = Adress + PDU + CRC/LRC = 1 + PDU + 2;
	
}

//********************************************************************************************************
// 函数名称：TransfersReplyHandle
// 输入参数：无
// 输出参数：无
// 功能描述：传输应答帧处理
//********************************************************************************************************
void TransfersReplyHandle(void)
{
	uint16	CRC_OR_LRC;
	*ADUData.ADUBuffPtr = ADUData.Address;	  								// 设备地址
	CRC_OR_LRC = Modbus_CRC16(ADUData.ADUBuffPtr,ADUData.ADULength-2);		// 计算CRC (ADU中CRC前的数据)
	*(ADUData.ADUBuffPtr+ADUData.ADULength-1) = (uint8)(CRC_OR_LRC/0x100);	// CRC Hi or LRC Hi 
	*(ADUData.ADUBuffPtr+ADUData.ADULength-2) = (uint8)CRC_OR_LRC;			// CRC Hi or LRC Hi 

	SendResponse(ADUData.ADUBuffPtr,ADUData.ADULength);
}


//********************************************************************************************************
// 函数名称：FunctionHandle
// 输入参数：无
// 输出参数：无
// 功能描述：MODBUS服务函数
//********************************************************************************************************
void IdleModbus(void)
{
	uint8 err;

	// 检验有效帧
//	OSSemPend(mb_handle_event_ptr,0,&err);

    if(mb_handle_event_flag==1)
	  mb_handle_event_flag=0;
	else
	  return;
	if(PDUData.FrameOK==FALSE)		// 接收帧未结束返回
		return;
	PDUData.FrameOK = FALSE;
	 
	FunctionHandle();				// 功能代码处理函数
	PackPDU2ADU();					// 将PDU帧封装成ADU帧
	TransfersReplyHandle();			// 传输应答帧
	// 准备收到下一帧

//	ADUData.ADUBuffPtr = SlaveDevice.RequestPtr;
	ADUData.ADULength = 0;
	PDUData.PDUBuffPtr = NULL;
}








