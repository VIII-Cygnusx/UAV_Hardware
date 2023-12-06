/*******************************************************************
* @brief 此包专门为NRF24L01适配，随调随用
* @warning 此程序只能对频指定设备，随机设备对频功能尚未实现
* @param NRF24L01_SPI_LINE 为NRF24L01的SPI总线，需要把你定义的spi总线重定义到这里
* @param NRF_CS_GPIO_Port 为工作发送使能端口	低电平工作，反之
* @param NRF_CE_GPIO_Port 为芯片使能端口		低电平使能芯片,反之
* @date 2023/10/25
* @version 1.0.1
******************************************************************/

#include<nrf24l01.h>

const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0xE1,0xE2,0xE3,0xE4,0xe5}; //发送地址
//以防cubemx乱调参数，这里要专门为NRF24L01的spi做初始化
//硬件初始化
void NRF24L01_init(void){
	  while(NRF24L01_Check())printf("ERROR\n");
  	  NRF24L01_RX_Mode();//配置为接收模式
	  printf("SUCCED");
} 
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{
    u8 Rxdata;
    HAL_SPI_TransmitReceive(&NRF24L01_SPI_LINE,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //返回收到的数据		
}
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
    Clr_NRF24L01_CS;                 //使能SPI传输
  	status =SPI1_ReadWriteByte(reg);//发送寄存器号    
  	SPI1_ReadWriteByte(value);      //写入寄存器的值
    Set_NRF24L01_CS;                //禁止SPI传输	   
  	return(status);       		    //返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
   	Clr_NRF24L01_CS;             //使能SPI传输		
  	SPI1_ReadWriteByte(reg);    //发送寄存器号
  	reg_val=SPI1_ReadWriteByte(0XFF);//读取寄存器内容
  	Set_NRF24L01_CS;             //禁止SPI传输		    
  	return(reg_val);            //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,read=0xff;	       
  	Clr_NRF24L01_CS;            //使能SPI传输
  	status=SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	 
	HAL_SPI_TransmitReceive(&NRF24L01_SPI_LINE,&read,pBuf,len,HAL_MAX_DELAY);
  	Set_NRF24L01_CS;            //关闭SPI传输
  	return status;             //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status;	    
	Clr_NRF24L01_CS;             //使能SPI传输
  	SPI1_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
	HAL_SPI_TransmitReceive(&NRF24L01_SPI_LINE,pBuf,&status,len,HAL_MAX_DELAY);
  	Set_NRF24L01_CS;             //关闭SPI传输
  	return status;              //返回读到的状态值
}	

u8 NRF24L01_Check(void){
    u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
  
	sta=NRF24L01_Read_Reg(STATUS);          //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);  //清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}	
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void NRF24L01_RX_Mode(void)
{
	Clr_NRF24L01_CE;//片选NRF24L01芯片	  low
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x00);       //使能通道0的自动应答    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);   //使能通道0的接收地址  	 
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,1);	        //设置RF通信频率		  
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07);    //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);     //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	Set_NRF24L01_CE; //CE为高,进入接收模式   high
}	
uint8_t RC_rxData[32];


//通道解析
void RC_Analy(void)  
{
/*             Receive  and check RC data                               */	
	if(NRF24L01_RxPacket(RC_rxData)==SUCCESS)
	{ 	
		uint8_t CheckSum=0;
		for(int i=0;i<31;i++)CheckSum +=  RC_rxData[i];
		if(RC_rxData[31]==CheckSum && RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)  //如果接收到的遥控数据正确
		{
			  	Remote.roll = ((uint16_t)RC_rxData[4]<<8) | RC_rxData[5];  //通道1
				Remote.roll = LIMIT(Remote.roll,1000,2000);
				Remote.pitch = ((uint16_t)RC_rxData[6]<<8) | RC_rxData[7];  //通道2
				Remote.pitch = LIMIT(Remote.pitch,1000,2000);
				Remote.thr = 	((uint16_t)RC_rxData[8]<<8) | RC_rxData[9];   //通道3 
				Remote.thr = 	LIMIT(Remote.thr,1000,2000);
				Remote.yaw =  ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //通道4
				Remote.yaw =  LIMIT(Remote.yaw,1000,2000);
				Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //通道5  左上角按键都属于通道5  
				Remote.AUX1 =  LIMIT(Remote.AUX1,1000,2000);
				Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //通道6  右上角按键都属于通道6 
				Remote.AUX2 =  LIMIT(Remote.AUX2,1000,2000);
				Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //通道7  左下边按键都属于通道7 
				Remote.AUX3 =  LIMIT(Remote.AUX3,1000,2000);
				Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //通道8  右下边按键都属于通道6  
				Remote.AUX4 = LIMIT(Remote.AUX4,1000,4000);	
		}
  }

}