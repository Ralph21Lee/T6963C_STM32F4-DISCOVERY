/**************************************************************************************************
  * file    drv_t6963c_lcdc.c
  * author  Ralph Lee
  * version V1.0
  * date    19-July-2017
  * brief   The is a drver code for driving a LCM with T6963C.
***************************************************************************************************/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"

#include "drv_t6963c_lcdc.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

// Two GPIOs are needed to control LCM. One is for font selection and the other is reset control.
#define     LCM_SIG_NO  2       // 2 GPIOs for a LCM

//--------  GPIOs defintion for LCM0 -----
// port for font selection
#define     LCM0_FS_GPIO_PORT           GPIOA
#define     LCM0_FS_GPIO_PIN            GPIO_PIN_5
#define     LCM0_FS_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define     LCM0_FS_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define     LCM0_FS_6X8                 GPIO_PIN_SET
#define     LCM0_FS_8X8                 GPIO_PIN_RESET

// port for reset control
#define     LCM0_RST_GPIO_PORT          GPIOA
#define     LCM0_RST_PIN                GPIO_PIN_9
#define     LCM0_RST_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()    
#define     LCM0_RST_GPIO_CLK_DISABLE() __GPIOA_CLK_ENABLE()    

#define     SRAM_BANK1_ADDR             0x60000000
#define     SRAM_BANK2_ADDR             0x64000000
#define     SRAM_BANK3_ADDR             0x68000000
#define     SRAM_BANK4_ADDR             0x6c000000

#define LCM0_GPIO_CLK_ENABLE(__INDEX__)   (((__INDEX__) == 0) ? LCM0_FS_GPIO_CLK_ENABLE()  : LCM0_RST_GPIO_CLK_ENABLE())

#define LCM0_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? LCM0_FS_GPIO_CLK_DISABLE() : LCM0_RST_GPIO_CLK_DISABLE())

#define SRAM_MEMORY_WIDTH           FMC_NORSRAM_MEM_BUS_WIDTH_8

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

/*
GPIO_TypeDef* LCM0_GPIO_PORT[LCM_SIG_NO] = {
                                LCM0_FS_GPIO_PORT,
                                LCM0_RST_GPIO_PORT,
                                };
uint16_t LCM0_GPIO_PIN[LCM_SIG_NO] = {
                                LCM0_FS_GPIO_PIN,
                                LCM0_RST_PIN,
                                };
static LCDC_t  LCM0 = {
        (volatile uint8_t*)(SRAM_BANK2_ADDR + 0),   // pointer to data port of LCDC
        (volatile uint8_t*)(SRAM_BANK2_ADDR + 1),   // pointer to command port of LCDC
        (volatile uint8_t*)(SRAM_BANK2_ADDR + 1),   // pointer to status port of LCDC

        LCD_FONT_WIDTH,                     // font selection
        LCD_MAX_VERTICAL_DOTS,              // max_vertical_dot;
        LCD_MAX_HORIZONTAL_DOTS,            // max_horizontal_dot;
        LCD_BYTES_PER_ROW,                  // bytes_per_row;
        LCD_GRAPHIC_SIZE,                   // graphic_size;
        LCD_TEXT_SIZE,                      // text area size in a page
        (LCD_TEXT_SIZE + LCD_GRAPHIC_SIZE), // page_size: a page including text and graphic area
        LCD_TEXT_HOME,                      // text_home_addr;
        LCD_GRAPHIC_HOME,                   // graphic_home_addr;
        LCD_EXTERNAL_CG_HOME,               // external_CG_home_addr;
        LCD_OFFSET_REGISTER,                // offset_register;

        //----- GPIO related feilds that connected to LCD
        LCM0_GPIO_PORT,                     // pointer to GPIO address list used for controlling LCD
        LCM0_GPIO_PIN,                      // pointer to GPIO pin number list
};
*/
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      EXTERNAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/



                            
//#define LCM_STATUS_ADDR (SRAM_BANK_ADDR + 1)    // same as command address and different direction
//static uint16_t current_page_text_addr;
//static uint16_t current_page_graphic_addr;

// -------------------------------------------------------------------------------

//  setup the GPIO pin connected to LCD
void lcd_pin_setup(LCDC_t* h_lcdc, LcdPin_Typedef lcd_pin)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    LCM0_GPIO_CLK_ENABLE(lcd_pin);

    /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin = h_lcdc->GPIO_PIN[lcd_pin];
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

    HAL_GPIO_Init(h_lcdc->GPIO_PORT[lcd_pin], &GPIO_InitStruct);
    HAL_GPIO_WritePin(h_lcdc->GPIO_PORT[lcd_pin], h_lcdc->GPIO_PIN[lcd_pin], GPIO_PIN_SET);
}

//  setup every pins that connected to LCD
void lcd_setup(LCDC_t* h_lcdc)
{
    lcd_pin_setup(h_lcdc, LCM_FS);
    lcd_pin_setup(h_lcdc, LCM_RST);
}

//  function that set/clr Reset signal of LCD
void lcd_Reset(LCDC_t* h_lcdc, GPIO_PinState state)
{
    HAL_GPIO_WritePin(h_lcdc->GPIO_PORT[LCM_RST], h_lcdc->GPIO_PIN[LCM_RST], state);
}

void lcd_font_select(LCDC_t* h_lcdc, font_width_t font_sel)
{
    if (font_sel == width_6_dots)
    {
        HAL_GPIO_WritePin(h_lcdc->GPIO_PORT[LCM_FS], h_lcdc->GPIO_PIN[LCM_FS], GPIO_PIN_SET);
    }
    else //(font_sel == FONT_WIDTH_8_DOTS)
    {
        HAL_GPIO_WritePin(h_lcdc->GPIO_PORT[LCM_FS], h_lcdc->GPIO_PIN[LCM_FS], GPIO_PIN_RESET);
    }
}

inline uint8_t lcd_read_status(LCDC_t* h_lcdc)  // get LCD display status byte
{
    uint8_t glcd_status;

    glcd_status = *(h_lcdc->pStatus);
    
    return(glcd_status);
}


inline uint8_t lcd_read_data(LCDC_t* h_lcdc)
{
    while((*(h_lcdc->pStatus) & 0x03) != 0x03);   // wait for LCDC ready
    return(*h_lcdc->pData);
}


inline void lcd_write_data(LCDC_t* h_lcdc, uint8_t data)
{
    while((*(h_lcdc->pStatus) & 0x03) != 0x03);   // wait for LCDC ready
    *h_lcdc->pData = data;
    __no_operation();
}


inline void lcd_write_cmd(LCDC_t* h_lcdc, uint8_t cmd)
{
    while((*(h_lcdc->pStatus) & 0x03) != 0x03);   // wait for LCDC ready
    *h_lcdc->pCmd= cmd;
    __no_operation();
}


inline void lcd_set_address(LCDC_t* h_lcdc, uint16_t addr){
    lcd_write_data(h_lcdc, addr & 0xFF);
    lcd_write_data(h_lcdc, addr >> 8);
    lcd_write_cmd(h_lcdc, T6963_SET_ADDRESS_POINTER);
}

inline void lcd_set_graphic_home_addr(LCDC_t* h_lcdc)
{
	lcd_write_data(h_lcdc, h_lcdc->graphic_home_addr  & 0xff);
	lcd_write_data(h_lcdc, h_lcdc->graphic_home_addr  >> 8 );
    lcd_write_cmd(h_lcdc, T6963_SET_GRAPHIC_HOME_ADDRESS);
}

inline void lcd_set_graphic_area(LCDC_t* h_lcdc)
{
    lcd_write_data(h_lcdc, h_lcdc->bytes_per_row & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->bytes_per_row >> 8);
    lcd_write_cmd(h_lcdc, T6963_SET_GRAPHIC_AREA);
}

inline void lcd_set_text_home_addr(LCDC_t* h_lcdc)
{
	lcd_write_data(h_lcdc, h_lcdc->text_home_addr & 0xff);
	lcd_write_data(h_lcdc, h_lcdc->text_home_addr >> 8 );
    lcd_write_cmd(h_lcdc, T6963_SET_TEXT_HOME_ADDRESS);
}

inline void lcd_set_text_area(LCDC_t* h_lcdc)
{
    lcd_write_data(h_lcdc, h_lcdc->bytes_per_row & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->bytes_per_row >> 8);
    lcd_write_cmd(h_lcdc, T6963_SET_TEXT_AREA);
}

inline void lcd_set_offset_register(LCDC_t* h_lcdc)
{
    lcd_write_data(h_lcdc, h_lcdc->offset_register& 0xff);
    lcd_write_data(h_lcdc, 0);
    lcd_write_cmd(h_lcdc, T6963_SET_OFFSET_REGISTER);
}

void lcd_clear_graph(LCDC_t* h_lcdc)
{
    int i;
    lcd_set_address(h_lcdc, h_lcdc->graphic_home_addr);
    
    for (i=0; i<(h_lcdc->graphic_size); i++) {   	// must be  3840 or 5120
        lcd_write_data(h_lcdc, 0); 
        lcd_write_cmd(h_lcdc, T6963_DATA_WRITE_AND_INCREMENT);               // write data, inc ptr.
    }
}

void lcd_clear_CGRAM(LCDC_t* h_lcdc)
{
    lcd_set_address(h_lcdc, h_lcdc->external_CG_home_addr);

    for(int i = 0; i < 256 * 8; i++)    // clear whole CG_RAM area which is 256*8=2048 bytes.
    {
        lcd_write_data(h_lcdc, 0);
        lcd_write_cmd(h_lcdc, T6963_DATA_WRITE_AND_INCREMENT);
    }

}

void lcd_clear_text(LCDC_t* h_lcdc)
{
    lcd_set_address(h_lcdc, h_lcdc->text_home_addr);

    for(int i = 0; i < h_lcdc->text_size; i++) //  must be 480 or 640  
    {
        lcd_write_data(h_lcdc, 0);
        lcd_write_cmd(h_lcdc, T6963_DATA_WRITE_AND_INCREMENT);
    }
}

//-------------------------------------------------------------------------------

void lcd_xy(LCDC_t* h_lcdc, int x, int y)
{
    uint16_t addr;

    addr = h_lcdc->text_home_addr + (y * h_lcdc->bytes_per_row) + x;
    lcd_set_address(h_lcdc, addr);   // set LCD addr. pointer

}

//-------------------------------------------------------------------------------

void lcd_print_ram(LCDC_t* h_lcdc, int x,int y,char *string)  // send string of characters to LCD
{
    int i;
    int c;
    lcd_xy(h_lcdc, x, y);
    for (i=0; string[i]!=0; i++)
    {
        c = string[i] - 0x20;     // convert ASCII to LCD char address
        if ( c < 0 ) c = 0;
        lcd_write_data(h_lcdc, c);
        lcd_write_cmd(h_lcdc, T6963_DATA_WRITE_AND_INCREMENT);               // write character, increment memory ptr.
    }
}

//-------------------------------------------------------------------------------

void lcd_print(LCDC_t* h_lcdc, int x,int y,char *string)
{
    int i;
    int c;
    lcd_xy(h_lcdc, x, y);
    for (i=0; string[i]!=0; i++)
    {
        c = string[i] - 0x20;     // convert ASCII to LCD char address
        if (c<0) c=0;
        lcd_write_data(h_lcdc, c);
        lcd_write_cmd(h_lcdc, T6963_DATA_WRITE_AND_INCREMENT);               // write character, increment memory ptr.
    }
}
//--------------------------------------------------------------------------------

void lcd_pixel(LCDC_t* h_lcdc, int16_t column, int16_t row, char show)
{
    int addr;       // memory address of byte containing pixel to write

    if (column < 0 || row < 0)
        return;
    if( (column >= h_lcdc->max_horizontal_dot) || (row >= h_lcdc->max_vertical_dot) )
        return;
    
    addr =  h_lcdc->graphic_home_addr + (row * h_lcdc->bytes_per_row)  + (column / h_lcdc->font_width);
    lcd_set_address(h_lcdc, addr);
    
    if(show)
        lcd_write_cmd(h_lcdc, T6963_SET_BIT | ((h_lcdc->font_width - 1 - column % h_lcdc->font_width)) );  // set bit-within-byte command
    else
        lcd_write_cmd(h_lcdc, T6963_CLR_BIT | ((h_lcdc->font_width - 1 - column % h_lcdc->font_width )) );  // set bit-within-byte command
}

//---------------------------------------------------------------------------------


void lcd_show(LCDC_t* h_lcdc, uint8_t * img, uint16_t start_line,int how_many_line)
{
    int addr,i;
    
    addr =  h_lcdc->graphic_home_addr + start_line * h_lcdc->bytes_per_row;
    
    lcd_set_address(h_lcdc, addr);
    lcd_write_cmd(h_lcdc, T6963_SET_DATA_AUTO_WRITE);
    
    for(i=0; i<how_many_line * h_lcdc->bytes_per_row; i++){			// * 30 or 40
        lcd_write_data(h_lcdc, img[i]); 
    }
    lcd_write_cmd(h_lcdc, T6963_AUTO_RESET);
}

//---------------------------------------------------------------------------------

void lcd_init(LCDC_t* h_lcdc)  // initialize LCD memory and display modes
{
    lcd_font_select(h_lcdc, h_lcdc->font_width);  // set font width
    
    lcd_Reset(h_lcdc, GPIO_PIN_RESET);
    HAL_Delay(1); // delay 5ms
    lcd_Reset(h_lcdc, GPIO_PIN_SET);
    HAL_Delay(1); // delay 5ms

    lcd_set_page_addr(h_lcdc, 0);
		
    lcd_write_data(h_lcdc, h_lcdc->graphic_home_addr & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->graphic_home_addr >> 8 );
    lcd_write_cmd(h_lcdc, T6963_SET_GRAPHIC_HOME_ADDRESS);

    lcd_write_data(h_lcdc, h_lcdc->bytes_per_row & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->bytes_per_row >> 8);
    lcd_write_cmd(h_lcdc, T6963_SET_GRAPHIC_AREA);

    lcd_write_data(h_lcdc, h_lcdc->text_home_addr & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->text_home_addr >> 8 );
    lcd_write_cmd(h_lcdc, T6963_SET_TEXT_HOME_ADDRESS);

    lcd_write_data(h_lcdc, h_lcdc->bytes_per_row & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->bytes_per_row >> 8);
    lcd_write_cmd(h_lcdc, T6963_SET_TEXT_AREA);

    lcd_write_data(h_lcdc, h_lcdc->offset_register& 0xff);
    lcd_write_data(h_lcdc, 0);
    lcd_write_cmd(h_lcdc, T6963_SET_OFFSET_REGISTER);

    lcd_write_cmd(h_lcdc, T6963_MODE_SET|T6963_INTERNAL_CG|T6963_EXOR_MODE); // use internal font
    lcd_write_cmd(h_lcdc, T6963_CURSOR_PATTERN_SELECT|T6963_CURSOR_8_LINE);
    
    lcd_write_data(h_lcdc, 0x00);
    lcd_write_data(h_lcdc, 0x00);
    lcd_write_cmd(h_lcdc, T6963_SET_CURSOR_POINTER);

    lcd_write_cmd(h_lcdc,   T6963_DISPLAY_MODE|T6963_CURSOR_DISPLAY_ON|
                            T6963_TEXT_DISPLAY_ON|T6963_GRAPHIC_DISPLAY_ON);

    //lcd_write_data(h_lcdc, 0x00);
    //lcd_write_data(h_lcdc, 0x00);
    //lcd_write_cmd(h_lcdc, T6963_SET_CURSOR_POINTER);

    #if 0   // T6963 doesn't support screen reverse function
    lcd_write_data(h_lcdc, T6963_SCREEN_REVERSE_ON);
    lcd_write_data(h_lcdc, 0x00);
    lcd_write_cmd(h_lcdc, T6963_SCREEN_REVERSE);
    #endif
    
}

void lcd_set_page_addr(LCDC_t* h_lcdc, uint16_t page_addr)
{
    h_lcdc->text_home_addr = page_addr;
    h_lcdc->graphic_home_addr = page_addr + h_lcdc->text_size;
    
    lcd_write_data(h_lcdc, h_lcdc->text_home_addr & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->text_home_addr >> 8);
    lcd_write_cmd(h_lcdc, T6963_SET_TEXT_HOME_ADDRESS);

    lcd_write_data(h_lcdc, h_lcdc->graphic_home_addr & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->graphic_home_addr >> 8);
    lcd_write_cmd(h_lcdc, T6963_SET_GRAPHIC_HOME_ADDRESS);
}
//---------------------------------------------------------------------------

void lcd_text_fill(LCDC_t* h_lcdc, uint8_t ch)
{
    lcd_write_data(h_lcdc, h_lcdc->text_home_addr & 0xff);
    lcd_write_data(h_lcdc, h_lcdc->text_home_addr >> 8);
    lcd_write_cmd(h_lcdc, T6963_SET_ADDRESS_POINTER);

    for(uint16_t i = 0; i < h_lcdc->text_size; i++) //  must be 480 or 640  
    {
        lcd_write_data(h_lcdc, ch - 0x20);
        lcd_write_cmd(h_lcdc, T6963_DATA_WRITE_AND_INCREMENT);
    }
}
void lcd_set_cursor(LCDC_t* h_lcdc, uint8_t x, uint8_t y)
{
    lcd_write_data(h_lcdc, x);
    lcd_write_data(h_lcdc, y);
    lcd_write_cmd(h_lcdc, T6963_SET_CURSOR_POINTER);
}
/***********************************************************************
Draws a line from x1,y1 go x2,y2. Line can be drawn in any direction.
Set show1 to 1 to draw pixel, set to 0 to hide pixel.
***********************************************************************/
void lcd_line(LCDC_t* h_lcdc, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t show) 
{
    int16_t    dy ;
    int16_t    dx ;
    int16_t    stepx, stepy, fraction;
    
    dy = y2 - y1;
    dx = x2 - x1;
    if (dy < 0) {
        dy = -dy;
        stepy = -1;
    } else  {
        stepy = 1;
    }
    
    if (dx < 0) {
        dx = -dx;
        stepx = -1;
    } else {
        stepx = 1;
    }
    
    dy <<= 1; dx <<= 1;
    lcd_pixel(h_lcdc, x1, y1, show);
    
    if (dx > dy) {
        fraction = dy - (dx >> 1); 
        while (x1 != x2) {
            if (fraction >= 0) {
                y1 += stepy;
                fraction -= dx;
            }
            x1 += stepx;
            fraction += dy;  
            lcd_pixel(h_lcdc, x1, y1, show);
        }
    } else {
        fraction = dx - (dy >> 1);
        while (y1 != y2)
        {
            if (fraction >= 0) {
                x1 += stepx;
                fraction -= dy;
            }
            y1 += stepy;
            fraction += dx;
            lcd_pixel(h_lcdc, x1, y1, show);
        }
    }
}


/**
	@brief	Draws a circle 
			In Anlehnung an Bresenham (http://groups.google.de/group/maus.mathe/browse_thread/thread/8a44a51d3f43c23d/ oder 
			http://groups.google.de/group/maus.lang.tpascal/browse_thread/thread/38f9e6d81de037f4/) wird nur 1/4 Kreis berechnet und
			die anderen 3/4 gespiegelt => schnell, da ohne floats
	@param 	x, y: center
			radius
			Set show1 to 1 to draw pixel, set to 0 to hide pixel.
	@return	none
*/
void lcd_circle(LCDC_t* h_lcdc, int16_t x, int16_t y, int16_t radius, uint8_t show)
{
    int16_t xc = 0;
    int16_t yc ;
    int16_t p ;
    yc = radius;
    p = 3 - (radius << 1);
    while (xc <= yc)  
    {
        lcd_pixel(h_lcdc, x + xc, y + yc, show);
        lcd_pixel(h_lcdc, x + xc, y - yc, show);
        lcd_pixel(h_lcdc, x - xc, y + yc, show);
        lcd_pixel(h_lcdc, x - xc, y - yc, show);
        lcd_pixel(h_lcdc, x + yc, y + xc, show);
        lcd_pixel(h_lcdc, x + yc, y - xc, show);
        lcd_pixel(h_lcdc, x - yc, y + xc, show);
        lcd_pixel(h_lcdc, x - yc, y - xc, show);
        if (p < 0)
            p += (xc++ << 2) + 6;
        else
            p += ((xc++ - yc--)<<2) + 10;
    }
}

void lcd_circle_half(LCDC_t* h_lcdc, int16_t x, int16_t y, int16_t radius, uint8_t show)
{
    int16_t xc = 0;
    int16_t yc ;
    int16_t p ;
    yc = radius;
    p = 3 - (radius << 1);
    while (xc <= yc)  
    {
        //lcd_pixel(h_lcdc, x + xc, y + yc, show);
        lcd_pixel(h_lcdc, x + xc, y - yc, show);
        //lcd_pixel(h_lcdc, x - xc, y + yc, show);
        lcd_pixel(h_lcdc, x - xc, y - yc, show);
        //lcd_pixel(h_lcdc, x + yc, y + xc, show);
        lcd_pixel(h_lcdc, x + yc, y - xc, show);
        //lcd_pixel(h_lcdc, x - yc, y + xc, show);
        lcd_pixel(h_lcdc, x - yc, y - xc, show);
        if (p < 0)
            p += (xc++ << 2) + 6;
        else
            p += ((xc++ - yc--)<<2) + 10;
    }
}

void lcd_box(LCDC_t* h_lcdc, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t show)
{
    lcd_line(h_lcdc, x1, y1, x2, y1, show);  // up
    lcd_line(h_lcdc, x1, y2, x2, y2, show);  // down
    lcd_line(h_lcdc, x2, y1, x2, y2, show);  // right
    lcd_line(h_lcdc, x1, y1, x1, y2, show);  // left
}

 
void lcd_fill(LCDC_t* h_lcdc, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t persent,char first)
{
    char M, horizon_line, horizon_line2, i, str1[10];
    
    if(persent>100) return;
    
    if(!first){
        lcd_line(h_lcdc, x1, y2, x2, y2, 1);  // down
        lcd_line(h_lcdc, x2, y1, x2, y2, 1);  // right
        lcd_line(h_lcdc, x1, y1, x1, y2, 1);  // left
        first = 1;
    }
    M = 100/abs(y2-y1);

    horizon_line=persent/M;
    for(i=0;i<horizon_line;i++)
    	lcd_line(h_lcdc, x1+2, y2-2-i, x2-2, y2-2-i, 1);

    horizon_line2 = 100/M;
    for(i=horizon_line; i<horizon_line2; i++)
    	lcd_line(h_lcdc, x1+2, y2-2-i, x2-2, y2-2-i, 0);


    sprintf(str1,"%02d%% ",persent);
    lcd_print_ram(h_lcdc, (x2+x1)/16-1 , (y2+y1)/16, str1);
}

/**
	@brief	Draws a segment of a circle 
			
	@param 	x, y: center
			radius
			Startwinkel, Endwinkel. 0=unten, 90 = rechts
			Set show1 to 1 to draw pixel, set to 0 to hide pixel.
	@return	none
*/
void lcd_circle_segment(LCDC_t* h_lcdc, int x, int y, int radius, int winkel_start, int winkel_end, unsigned char show)
{
	int	x_kreis, y_kreis;
	float winkel;
	#define DEGREE 2*3.14159265/360	
	
	for (winkel = (float)winkel_start; winkel <= winkel_end; winkel += .9)
	{
		x_kreis = (int) (sin (winkel * DEGREE) * radius);
		y_kreis = (int) (cos (winkel * DEGREE) * radius);
		lcd_pixel(h_lcdc, x + x_kreis, y + y_kreis, show);
	}
}

/**
	@brief	Zeichnet eine gefüllte Box
	@param 	x1, y1 : linke obere Ecke
			x2, y2 : rechte untere Ecke
			Set show1 to 1 to draw pixel, set to 0 to hide pixel.
	@return	none
*/
void lcd_box_filled (LCDC_t* h_lcdc, int x1, int y1, int x2, int y2, unsigned char show)
{
	int i;
	
	for (i = y1; i <= y2; i++)
		lcd_line(h_lcdc, x1, i, x2, i, show);
}

