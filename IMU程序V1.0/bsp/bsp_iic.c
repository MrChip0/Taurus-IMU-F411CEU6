/**
	* @file bsp_iic.c
	* @version 1.0
	* @date 2019.11.26
  *
  * @brief  软件IIC驱动
  *
  *	@author YY(Part of codes reference ALIENTEK)
  *
  */

#include "bsp_iic.h"

/**********************************************************************************************************
*函 数 名: IIC_Soft_Init
*功能说明: 软件IIC初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IIC_Soft_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); 
	
	GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10;
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
	GPIO_Initure.Pull=GPIO_PULLUP;          
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	IIC_SDA(GPIO_PIN_SET);
	IIC_SCL(GPIO_PIN_SET);  
}

/**********************************************************************************************************
*函 数 名: IIC_Start
*功能说明: 产生IIC起始信号
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     					//SDA线输出
	IIC_SDA(GPIO_PIN_SET);	  	  
	IIC_SCL(GPIO_PIN_SET);
	delay_us(4);
 	IIC_SDA(GPIO_PIN_RESET);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL(GPIO_PIN_RESET);//钳住I2C总线，准备发送或接收数据 
}	  

/**********************************************************************************************************
*函 数 名: IIC_Stop
*功能说明: 产生IIC停止信号
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();							//SDA线输出
	IIC_SCL(GPIO_PIN_RESET);
	IIC_SDA(GPIO_PIN_RESET);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL(GPIO_PIN_SET); 
	IIC_SDA(GPIO_PIN_SET);	//发送I2C总线结束信号
	delay_us(4);							   	
}

/**********************************************************************************************************
*函 数 名: IIC_Wait_Ack
*功能说明: 等待应答信号到来
*形    参: 无
*返 回 值: 1：接收应答失败
					 0：接收应答成功
**********************************************************************************************************/
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      					 //SDA设置为输入  
	IIC_SDA(GPIO_PIN_SET);	delay_us(1);	   
	IIC_SCL(GPIO_PIN_SET);	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(GPIO_PIN_RESET);   
	return 0;  
} 

/**********************************************************************************************************
*函 数 名: IIC_Ack
*功能说明: 产生ACK应答
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL(GPIO_PIN_RESET);
	SDA_OUT();
	IIC_SDA(GPIO_PIN_RESET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET);
}

/**********************************************************************************************************
*函 数 名: IIC_NAck
*功能说明: 不产生ACK应答	
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL(GPIO_PIN_RESET);
	SDA_OUT();
	IIC_SDA(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET);
}		

/**********************************************************************************************************
*函 数 名: IIC_Send_Byte
*功能说明: IIC发送一个字节	
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/		  
void IIC_Send_Byte(uint8_t txd)
{                        
	uint8_t t;   
	SDA_OUT(); 	    
	IIC_SCL(GPIO_PIN_RESET);	//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		if(txd&0x80)
			IIC_SDA(GPIO_PIN_SET);
		else
			IIC_SDA(GPIO_PIN_RESET);
		txd<<=1; 	  
		delay_us(2);  
		IIC_SCL(GPIO_PIN_SET);
		delay_us(2); 
		IIC_SCL(GPIO_PIN_RESET);	
		delay_us(2);
	}	 
} 	

/**********************************************************************************************************
*函 数 名: IIC_Read_Byte
*功能说明: IIC读取一个字节	
*形    参: 无
*返 回 值: 无 
**********************************************************************************************************/	
uint8_t IIC_Read_Byte(void)
{
	uint8_t receive=0;
	SDA_IN();									 //SDA设置为输入
  for(uint8_t i=0;i<8;i++)
	{
		receive<<=1;
		IIC_SCL(GPIO_PIN_RESET); 
		delay_us(2);
		IIC_SCL(GPIO_PIN_SET);
		if(READ_SDA)
			receive++;  
		delay_us(1);
  }	
	return receive;
}

/**********************************************************************************************************
*函 数 名: IIC_Reg_Write
*功能说明: 单个寄存器写入(用八位地址)
*形    参: 从机地址 寄存器地址 写入数据
*返 回 值: 写入状态
**********************************************************************************************************/
bool IIC_Reg_Write(uint8_t SlaveAddress,uint8_t Reg_Address,uint8_t Reg_data)
{
	IIC_Start();
	IIC_Send_Byte(SlaveAddress+0);	//从机地址＋Write
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return false;
	}
	IIC_Send_Byte(Reg_Address);
	IIC_Wait_Ack();
	IIC_Send_Byte(Reg_data);
	IIC_Wait_Ack();
	IIC_Stop();
	return true;
}

/**********************************************************************************************************
*函 数 名: IIC_Reg_Read
*功能说明: 单个寄存器读取(用八位地址)
*形    参: 从机地址 寄存器地址
*返 回 值: 数据
**********************************************************************************************************/
uint8_t IIC_Reg_Read(uint8_t SlaveAddress,uint8_t Reg_Address)
{
	uint8_t REG_data;
	IIC_Start();
	IIC_Send_Byte(SlaveAddress+0);	//从机地址＋Write
	if(IIC_Wait_Ack())
	{
		IIC_Stop();
		return false;
	}
	IIC_Send_Byte(Reg_Address);
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(SlaveAddress+1);	//从机地址＋Read
	IIC_Wait_Ack();
	
	REG_data = IIC_Read_Byte();
	
	IIC_NAck();
	IIC_Stop();
	
	return REG_data;
}




