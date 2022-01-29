
#ifndef INC_OLED_H_
#define INC_OLED_H_ 

#include "main.h"
#include "stm32f4xx_hal.h"

#define OLED_RST_Clr() HAL_GPIO_WritePin(GPIOE,OLED_RST_Pin,GPIO_PIN_RESET) // RST = 0
#define OLED_RST_Set() HAL_GPIO_WritePin(GPIOE,OLED_RST_Pin,GPIO_PIN_SET) // RST = 1

#define OLED_RS_Clr() HAL_GPIO_WritePin(GPIOE,OLED_DC_Pin,GPIO_PIN_RESET) // DC = 0
#define OLED_RS_Set() HAL_GPIO_WritePin(GPIOE,OLED_DC_Pin,GPIO_PIN_SET) // DC = 1

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(GPIOE,OLED_SCL_Pin,GPIO_PIN_RESET) // SCL = 0
#define OLED_SCLK_Set() HAL_GPIO_WritePin(GPIOE,OLED_SCL_Pin,GPIO_PIN_SET) // SCL = 1

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(GPIOE,OLED_SDA_Pin,GPIO_PIN_RESET) // SDA = 0
#define OLED_SDIN_Set() HAL_GPIO_WritePin(GPIOE,OLED_SDA_Pin,GPIO_PIN_SET) // SDA = 1
#define OLED_CMD 0 // Write Command
#define OLED_DATA 1 // Write Data

#define CHAR_H 12
#define CHAR_W 8
#define SCREEN_W 128
#define SCREEN_H 60
#define MAX_CHAR_POSX 120
#define MAX_CHAR_POSY 60

//OLED Control Functions
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);					   
							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);					

//void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);	
void OLED_ShowChar(uint8_t x,uint8_t y,char chr,uint8_t size,uint8_t mode);	
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const char *p);	 		
  
uint32_t oled_pow(uint8_t m,uint8_t n);
//void OLED_Showword(uint8_t x,uint8_t y,uint8_t *num,uint8_t mode);

#endif
