/****************************************Copyright (c)**************************************************
**                               广州周立功单片机发展有限公司
**                                     研    究    所
**                                        产品一部 
**
**                                 http://www.zlgmcu.com
**
**--------------文件信息--------------------------------------------------------------------------------
**文   件   名: MBSlave.h
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
#ifndef __MBSLAVE_H
#define __MBSLAVE_H
#include    "modbus.h"
/***************************************************
** 使能MODBUS功能代码
****************************************************/
#define	READ_COILS_EN		1	// 读线圈
#define	READ_DIS_INPUT_EN	1	// 读离散量输入
#define	READ_HOLD_REG_EN	1	// 读保持寄存器
#define	READ_INPUT_REG_EN	1	// 读输入寄存器
#define	WRITE_SING_COIL_EN	1	// 写单个线圈
#define	WRITE_SING_REG_EN	1	// 写单个寄存器
#define	WRITE_MULT_COIL_EN	0	// 写多个线圈
#define	WRITE_MULT_REG_EN	1	// 写多个寄存器
#define	MASK_WRITE_REG_EN	0	// 处理屏蔽寄存器指令
#define	READ_WRITE_REG_EN	0	// 读写多个寄存器


/***************************************************
** 配置线圈的参数。定义线圈的数量
****************************************************/
#define END_COILS_ADDR			8
/***************************************************
** 配置离散输入的参数。定义离散输入量的数量
****************************************************/
#define END_DISC_INPUT_ADDR		8
/***************************************************
** 配置输入寄存器的参数。定义输入寄存器的数量
****************************************************/
#define END_INPUT_REG_ADDR		20
/***************************************************
** 配置保持寄存器的参数。定义保持寄存器数量
****************************************************/
#define END_HOLDING_REG_ADDR	20				

#define	MAX_FUNCTION		(READ_COILS_EN     	+\
							 READ_DIS_INPUT_EN	+\
							 READ_HOLD_REG_EN	+\
							 READ_INPUT_REG_EN	+\
							 WRITE_SING_COIL_EN	+\
							 WRITE_SING_REG_EN	+\
							 WRITE_MULT_COIL_EN	+\
							 WRITE_MULT_REG_EN	+\
							 MASK_WRITE_REG_EN	+\
							 READ_WRITE_REG_EN	+\
							 1)

/* 从机描述 */
typedef struct __SLAVE_INFORMATION{
	uint8   Address;		  			// 地址
	uint8	LineProtocol;	  			// 链路层协议
	uint16	BaudRate;	  				// 波特率 = BaudRate * 100	

	uint8	Parity;						// 奇偶校验
	uint8	StopBit;					// 停止位

	uint8*	RequestPtr;					// 主机请求从帧指针
}SLAVE_INFORMATION;

// 从机描述信息
extern SLAVE_INFORMATION SlaveDevice;

// 功能代码结构
typedef struct __FUNCTION_ARRAY{
	uint8	 Code;
	void	 (*Function)(PDU_RESPONSE *Response);
}FUNCTION_ARRAY;



void MBSlaveIni(SLAVE_INFORMATION *DevicInfPtr);
void IdleModbus(void);

#endif