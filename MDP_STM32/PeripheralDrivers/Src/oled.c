#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"

uint8_t OLED_GRAM[128][8];

void OLED_Refresh_Gram(void)
{
	uint8_t i,n;
	for(i=0;i<CHAR_W;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);
		OLED_WR_Byte (0x00,OLED_CMD);
		OLED_WR_Byte (0x10,OLED_CMD);
		for(n=0;n<SCREEN_W;n++) OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}

void OLED_WR_Byte(uint8_t dat,uint8_t cmd)
{
	uint8_t i;
    if(cmd)  OLED_RS_Set(); //CHANGE: OLED_RS_H
    else OLED_RS_Clr(); //CHANGE: OLED_RS_L

	for(i=0;i<8;i++)
	{
		OLED_SCLK_Clr();//CHANGE: OLED_SCLK_L
		if(dat&0x80)OLED_SDIN_Set(); //CHANGE: OLED_SDIN_H
		else OLED_SDIN_Clr(); //CHANGE: OLED_SDIN_L
		OLED_SCLK_Set(); //CHANGE: OLED_SCLK_H
		dat<<=1;
	}
	OLED_RS_Set();  //CHANGE: OLED_RS_H
}

void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);
	OLED_WR_Byte(0X14,OLED_CMD);
	OLED_WR_Byte(0XAF,OLED_CMD);
}

void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);
	OLED_WR_Byte(0X10,OLED_CMD);
	OLED_WR_Byte(0XAE,OLED_CMD);
}

void OLED_Clear(void)
{
	uint8_t i,n;
	for(i=0;i<CHAR_W;i++)
	for(n=0;n<SCREEN_W;n++)
	OLED_GRAM[n][i]=0x00;
	OLED_Refresh_Gram();
}

void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
	uint8_t pos,bx,temp=0;
	if(x>=SCREEN_W||y>=SCREEN_H)return;
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;
}

void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot)
{
	uint8_t x,y;
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
	}
	OLED_Refresh_Gram();
}

void OLED_ShowChar(uint8_t x,uint8_t y,char chr,uint8_t size,uint8_t mode)
{
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' ';
    for(t=0;t<size;t++)
    {
		//CHANGE: if(size==16)//temp=oled_asc2_1206[chr][t];
		//else
	//	if (size ==12) temp=oled_asc2_1206[chr][t];
	//	else temp=oled_asc2_1608[chr][t];
	temp=oled_asc2_1206[chr][t];
    for(t1=0;t1<CHAR_W;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}
    }
	OLED_Refresh_Gram();
}

uint32_t oled_pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}
//
//x,y :
//len :
//size:
//mode:	0,;1,
//num:(0~4294967295);
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1;
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1);
	}
}

void OLED_ShowString(uint8_t x,uint8_t y,const char *p)
{
    while(*p!='\0')
    {
        if(x>MAX_CHAR_POSX){x=0;y+=CHAR_H;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        //CHANGE: OLED_ShowChar(x,y,*p,16,1);
		OLED_ShowChar(x,y,*p,CHAR_H,1);
        x+=CHAR_W;
        p++;
    }
}

/*
void OLED_Showword(uint8_t x,uint8_t y,uint8_t *num,uint8_t mode)
{
	uint8_t t,t1,t2;
	uint8_t temp;
	uint8_t y0=y;
//	t=*num;
	for(t=0;t<50;t++)
	{
		if((*num==word[t].Index[0])&&(*(num+1)==word[t].Index[1]))//
		{
			for(t1=0;t1<32;t1++)		//
			{
				temp=word[t].Msk[t1];
				for(t2=0;t2<8;t2++)	//
				{
					if(temp&0x80)			//
						OLED_DrawPoint(x,y,1);
					else
						OLED_DrawPoint(x,y,0);
					temp<<=1;
					y++	;				//
//					if(y>=127) {return ;}		//
					if((y-y0)==16)		//
					{
						y=y0;
						x++;
						if(x>=127) return;
						break;
					}
				}
			}//

		}
	}
		OLED_Refresh_Gram();
}
*/

//SSD1306
void OLED_Init(void)
{
	//ADD: 
	HAL_PWR_EnableBkUpAccess(); // Enable access to the RTC and Backup Register
	//ADD:
	__HAL_RCC_LSE_CONFIG(RCC_LSE_OFF); 	//turn OFF the LSE oscillator, LSERDY flag goes low after 6 LSE oscillator clock cycles
																			//LSE oscillator switch off to let PC13 PC14 PC15 be IO

	HAL_PWR_DisableBkUpAccess();

	//CHANGE: OLED_RST_L;			  		//
	OLED_RST_Clr();
	//CHANGE: LL_mDelay(100);
	HAL_Delay(100);
	//CHANGE: OLED_RST_H;
	OLED_RST_Set();

	OLED_WR_Byte(0xAE,OLED_CMD); // Off Display

	OLED_WR_Byte(0xD5,OLED_CMD); // Set Oscillator Division
	OLED_WR_Byte(80,OLED_CMD);   //[3:0]: divide ratio of the DCLK, [7:4], set the oscillator frequency. Reset
	OLED_WR_Byte(0xA8,OLED_CMD); // multiple ratio
	OLED_WR_Byte(0X3F,OLED_CMD); //duty = 0x3F(1/64)
	OLED_WR_Byte(0xD3,OLED_CMD); //set display offset
	OLED_WR_Byte(0X00,OLED_CMD); //0

	OLED_WR_Byte(0x40,OLED_CMD);//set display start line [5:0]- from 0-63. RESET

	OLED_WR_Byte(0x8D,OLED_CMD);//Set charge pump
	OLED_WR_Byte(0x14,OLED_CMD);//Enable Charge pump. //  bit2，
	OLED_WR_Byte(0x20,OLED_CMD);//Set Memory Addressing Mode
	OLED_WR_Byte(0x02,OLED_CMD);//Page Addressing Mode (RESET) //[1:0],;;1
	OLED_WR_Byte(0xA1,OLED_CMD);//Set segment ??,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD);//Set COM Output Scan Direction //; bit3[N-;N:
	OLED_WR_Byte(0xDA,OLED_CMD);//Set COM Pins
	OLED_WR_Byte(0x12,OLED_CMD);//[5:4] setting

	OLED_WR_Byte(0x81,OLED_CMD);//Contrast Control
	OLED_WR_Byte(0xEF,OLED_CMD);//1~256; Default: 0x7F
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-charge Period
	OLED_WR_Byte(0xf1,OLED_CMD);//[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD);//
	OLED_WR_Byte(0x30,OLED_CMD);//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD);//Enable display outputs according to the ?? content //;bit0:1,;0,
	OLED_WR_Byte(0xA6,OLED_CMD);//Set normal display //;bit0:1,;0,
	OLED_WR_Byte(0xAF,OLED_CMD);//Display ON
	//REMOVE: LL_mDelay(100);
	OLED_Clear();
}
