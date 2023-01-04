#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "stm32f1xx_hal_spi.h" 

extern SPI_HandleTypeDef hspi2;

#if OLED_MODE==1
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	DATAOUT(dat);	    
	if(cmd)
	  OLED_DC_Set();
	else 
	  OLED_DC_Clr();		   
	OLED_CS_Clr();
	OLED_WR_Clr();	 
	OLED_WR_Set();
	OLED_CS_Set();	  
	OLED_DC_Set();	 
} 	    	    
#else
void OLED_WR_Byte(u8 dat,u8 cmd)
{	  
	if(cmd)
	  OLED_DC_Set();
	else 
	  OLED_DC_Clr();		
  
	OLED_CS_Clr();
	HAL_SPI_Transmit(&hspi2 ,&dat, 1, 1000);
	OLED_CS_Set();
	OLED_DC_Set();   	  
} 
#endif

void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte((((x)&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte(((x)&0x0f)|0x01,OLED_CMD); 
}   	  

void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC??
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
 
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC??
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 

void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);
		OLED_WR_Byte (0x02,OLED_CMD);
		OLED_WR_Byte (0x10,OLED_CMD);
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	}
}

void OLED_Init(void)
{ 	 
	OLED_RES_Clr();
	HAL_Delay(200);
	OLED_RES_Set();
	
  OLED_WR_Byte(0xAE,OLED_CMD); /*display off*/ 
  OLED_WR_Byte(0x02,OLED_CMD); /*set lower column address*/ 
  OLED_WR_Byte(0x10,OLED_CMD); /*set higher column address*/
	OLED_WR_Byte(0xB0,OLED_CMD); /*set page address*/ 
	OLED_WR_Byte(0x40,OLED_CMD); /*set display start lines*/ 
	OLED_WR_Byte(0x81,OLED_CMD); /*contract control*/ 
	OLED_WR_Byte(0x88,OLED_CMD); /*4d*/ 
	OLED_WR_Byte(0x82,OLED_CMD); /* iref resistor set and adjust ISEG*/ 
	OLED_WR_Byte(0x00,OLED_CMD); 
	OLED_WR_Byte(0xA1,OLED_CMD); /*set segment remap 0xA0*/ 
	OLED_WR_Byte(0xA2,OLED_CMD); /*set seg pads hardware configuration*/ 
	OLED_WR_Byte(0xA4,OLED_CMD); /*Disable Entire Display On (0xA4/0xA5)*/ 
	OLED_WR_Byte(0xA6,OLED_CMD); /*normal / reverse*/ 
	OLED_WR_Byte(0xA8,OLED_CMD); /*multiplex ratio*/ 
	OLED_WR_Byte(0x3F,OLED_CMD); /*duty = 1/64*/ 
	OLED_WR_Byte(0xC8,OLED_CMD); /*Com scan direction 0XC0*/
	OLED_WR_Byte(0xD3,OLED_CMD); /*set display offset*/ 
	OLED_WR_Byte(0x00,OLED_CMD); /* */ 
	OLED_WR_Byte(0xD5,OLED_CMD); /*set osc division*/ 
	OLED_WR_Byte(0xa0,OLED_CMD); 
	OLED_WR_Byte(0xD9,OLED_CMD); /*set pre-charge period*/ 
	OLED_WR_Byte(0x22,OLED_CMD); 
	OLED_WR_Byte(0xdb,OLED_CMD); /*set vcomh*/ 
	OLED_WR_Byte(0x40,OLED_CMD); 
	OLED_WR_Byte(0x31,OLED_CMD); /* Set pump 7.4v */ 
	OLED_WR_Byte(0xad,OLED_CMD); /*set charge pump enable*/ 
	OLED_WR_Byte(0x8b,OLED_CMD); /*Set DC-DC enable (0x8a=disable; 0x8b=enable) */ 
	OLED_Clear();
	OLED_WR_Byte(0xAF,OLED_CMD);	
}  