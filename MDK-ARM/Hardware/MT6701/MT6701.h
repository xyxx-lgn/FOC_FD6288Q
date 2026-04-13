#ifndef __MT6701_H
#define __MT6701_H		


//头文件包含
#include "spi.h"
#include "ALL_H.h"


#define MT6701_CS_ON()      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET)
#define MT6701_CS_OFF()     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET)
 

 
//函数声明
static uint8_t SPIx_ReadWriteByte(void);
static void MT6701_Read(uint8_t* pBuffer);   //读出24位原始数据
uint16_t MT6701_ReadRaw(void);   //14位原始数据
float MT6701_ReadAngle(void);    //角度数据
float MT6701_ReadRad(void);       //弧度值
//angle_raw返回14位角度数据：0-16384，angle为转换后的真实角度值：0-360，field_status,两位磁场数据[1:0]
void MT6701_Read_ALL(uint16_t*angle_raw, float*angle, uint8_t*field_status);
//void MT6701_Close_Loop(void);
#endif
