#include "MT6701.h"




/*******************************************
驱动芯片：MT6701
驱动方式：SPI1

使用引脚： 
          PA4 ---CS    GPIO初始化即可
	      PA5 ---SCK   以下为CUBEMAX初始化
		  PA6 ---MISO
		  PA7 ---MISI  未使用该引脚，不连接芯片




********************************************/


/*******************************************
函数：uint8_t SPIx_ReadWriteByte(uint8_t TxData)
函数功能：SPI读写函数
SPI1读写一个字节
该SPI写无意义，TxData可以任意给值，只是为了触发读取

*******************************************/
//static uint8_t SPIx_ReadWriteByte(void)
//{

//    uint8_t Rxdata;
//	//HAL_SPI_TransmitReceive_DMA   看看能不能换成这个，这个有点问题
//	//HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, HAL_MAX_DELAY);
//    HAL_SPI_Receive(&hspi1,  &Rxdata, 1, HAL_MAX_DELAY);
//    return Rxdata;
//}


/*******************************************
函数：void MT6701_Read(uint8_t* pBuffer)
函数功能：完整读取MT6701所有数据
数据包括：
		0-13：14位角度数据[13:0]
		14-17：4位磁场状态数据[3:0]    
		18-23：6位CRC校验码[5:0]
*******************************************/



static void MT6701_Read(uint8_t *p)
{
    MT6701_CS_ON();
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t[3]){0xFF,0xFF,0xFF}, p, 3, 10); // 0=不超时
    MT6701_CS_OFF();
}

uint16_t MT6701_ReadRaw(void)   //14位原始数据
{
    uint8_t data[3];
    uint16_t angle_u16;

    MT6701_Read(data);
    angle_u16 = (uint16_t)(data[1] >> 2);    
    angle_u16 |= ((uint16_t)data[0] << 6);                //该操作后得到14位原始角度数据
	return angle_u16;
}

float MT6701_ReadAngle(void)
{
    float angle_f = 0.0f;
    uint16_t angle_u16;
    angle_u16=MT6701_ReadRaw();                //该操作后得到14位原始角度数据
    angle_f = (float)angle_u16 * (360.0f / 16384.0f);    //角度换算
    return angle_f;
}

//读取磁编码器归一化弧度值:(0-6.28)
float MT6701_ReadRad(void)
{
	return (float)MT6701_ReadRaw()*(2*PI/16384.0f);                //该操作后得到弧度值
	
}

//累计磁编码器弧度角度:(0-∞)
//float MT6701_SumRads(void)
//{
//	float val = MT6701_ReadRad();
//	float d_angle = val -

//}

//angle_raw返回14位角度数据：0-16384，angle为转换后的真实角度值：0-360，field_status,两位磁场数据[1:0]
void MT6701_Read_ALL(uint16_t*angle_raw, float*angle, uint8_t*field_status)
{
    float angle_f = 0.0f;
    uint8_t data[3];
    uint16_t angle_u16;
    uint8_t status;
	
    MT6701_Read(data);                  //读取24位数据
	
    angle_u16 = (uint16_t)(data[1] >> 2); 
    angle_u16 |= ((uint16_t)data[0] << 6);  //该操作后得到14位原始角度数据
	
    status = (data[2] >> 6);
    status |= (data[1] & 0x03) << 2;        //该操作过后得到4位Mg磁场数据
	
    if(angle_raw != NULL) {
        *angle_raw = angle_u16;
    }

    if(angle != NULL) {

        angle_f = (float)angle_u16 * (360.0f / 16384.0f);
        *angle = angle_f;
    }
    if(field_status != NULL) {
        *field_status = status & 0x03;    //得到低两位，Mg磁场数据[1:0],0为正常，1磁场过强，2磁场太弱
    }
}


