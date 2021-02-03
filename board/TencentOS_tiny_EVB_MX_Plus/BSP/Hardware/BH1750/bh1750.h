#ifndef __BH1750_H__
#define __BH1750_H__
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define I2C_OWN_ADDRESS  0x0A

#define BH1750_Addr 0x46
#define BH1750_ON   0x01
#define BH1750_CON  0x10
#define BH1750_ONE  0x20
#define BH1750_RSET 0x07


/***************************************************************
* 函数名称: BH1750_Data_TypeDef
* 说    明: BH1750结构体
* 参    数: 无
* 返 回 值: 无
***************************************************************/
typedef struct
{
    char Lux[5];  //光强
}BH1750_Data_TypeDef;


void Init_BH1750(void);      //IO初始化，
void Start_BH1750(void);     //上电，设置清除数据寄存器
//void Read_BH1750(void);    //连续的读取内部寄存器数据
float Convert_BH1750(void);

#endif /* __BH1750_H__ */

