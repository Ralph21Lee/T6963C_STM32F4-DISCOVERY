/********************************************************************************************************
  * file    drv_t6963c_lcdc.h
  * author  Ralph Lee
  * version V1.0
  * date    19-July-2017
  * brief   The is a drver code for driving a LCM with T6963C.
*********************************************************************************************************/
#ifndef __drv_t6963c_lcdc_h___
#define __drv_t6963c_lcdc_h___

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

typedef enum{
    LCM_FS  = 0,        // Font select, '1': 6x8, '0': 8x8
    LCM_RST = 1,        // Reset
}LcdPin_Typedef;

typedef enum {
    width_6_dots = 6,
    width_8_dots = 8,
}font_width_t;

typedef struct {
    volatile uint8_t*   pData;      // pointer to data port of LCDC
    volatile uint8_t*   pCmd;       // pointer to command port of LCDC
    volatile uint8_t*   pStatus;    // pointer to status port of LCDC

        font_width_t    font_width; // font selection
            uint16_t    max_vertical_dot;
            uint16_t    max_horizontal_dot;
            uint16_t    bytes_per_row;
            uint16_t    graphic_size;   // graphic area size in a page
            uint16_t    text_size;      // text area size in a page
            uint16_t    page_size;      // a page including text and graphic area
            uint16_t    text_home_addr;
            uint16_t    graphic_home_addr;
            uint16_t    external_CG_home_addr;
            uint8_t     offset_register;

            //----- GPIO related feilds that connected to LCD
      GPIO_TypeDef**    GPIO_PORT;    // pointer to GPIO address list used for controlling LCD
           uint16_t*    GPIO_PIN;       // pointer to GPIO pin number list
} LCDC_t;

// ----- LCD display size ------
//#define FONT_WIDTH_6_DOTS       6
//#define FONT_WIDTH_8_DOTS       8
#define LCD_FONT_WIDTH			width_6_dots

#define LCD_MAX_VERTICAL_DOTS	64
#define LCD_MAX_HORIZONTAL_DOTS	240

//#define GLCD_BYTES_PER_ROW    (LCD_MAX_HORIZONTAL_DOTS / LCD_FONT_WIDTH) 
#define LCD_BYTES_PER_ROW      (LCD_MAX_HORIZONTAL_DOTS / LCD_FONT_WIDTH)         // 30 or 40
#define LCD_GRAPHIC_SIZE       (LCD_BYTES_PER_ROW * LCD_MAX_VERTICAL_DOTS)        // 30 * 64 = 1920 or 40 * 64 = 2560
#define LCD_TEXT_SIZE          (LCD_BYTES_PER_ROW * (LCD_MAX_VERTICAL_DOTS/8))    // 30 * 8  = 240  or 40 * 8  = 320

#define LCD_TEXT_HOME          0
#define LCD_GRAPHIC_HOME       (LCD_TEXT_HOME + LCD_TEXT_SIZE)	// avoid to overlay with text area
#define LCD_OFFSET_REGISTER    2   // range: 0~31. Select where the 2KB CG RAM is placed. There are 32 blocks in 64 KB space. 
                                    // Be aware of the actual memory size installed in LCM. 
                                    // In most of LCMs, the installed SRAM is 32KB. That is, only 0~15 are avaliable.
#define LCD_EXTERNAL_CG_HOME   (LCD_OFFSET_REGISTER << 11)    // = OFFSET * 2048(= 2^11), each CG area is 2KB.

// --------- Definitions for T6963 LCDC commands ------------
#define T6963_SET_CURSOR_POINTER			0x21
#define T6963_SET_OFFSET_REGISTER			0x22
#define T6963_SET_ADDRESS_POINTER			0x24

#define T6963_SET_TEXT_HOME_ADDRESS			0x40
#define T6963_SET_TEXT_AREA				    0x41
#define T6963_SET_GRAPHIC_HOME_ADDRESS		0x42
#define T6963_SET_GRAPHIC_AREA				0x43

#define T6963_MODE_SET					    0x80
    #define T6963_OR_MODE                   0x00
    #define T6963_EXOR_MODE                 0x01
    #define T6963_AND_MODE                  0x03
    #define T6963_TEXT_ATTRIBUTE            0x04
        #define TEXT_ATTRIBUTE_NORMAL       0x00
        #define TEXT_ATTRIBUTE_REVERSE      0x05
        #define TEXT_ATTRIBUTE_INHIBIT      0x03
        #define TEXT_ATTRIBUTE_BLINK        0x08
    #define T6963_INTERNAL_CG               0x00
    #define T6963_EXTERNAL_CG               0x08

#define T6963_DISPLAY_MODE					0x90
	#define T6963_CURSOR_BLINK_ON			0x01
	#define T6963_CURSOR_DISPLAY_ON			0x02
	#define T6963_TEXT_DISPLAY_ON			0x04
	#define T6963_GRAPHIC_DISPLAY_ON		0x08	

#define T6963_CURSOR_PATTERN_SELECT			0xA0
	#define T6963_CURSOR_1_LINE				0x00
	#define T6963_CURSOR_2_LINE				0x01
	#define T6963_CURSOR_3_LINE				0x02
	#define T6963_CURSOR_4_LINE				0x03
	#define T6963_CURSOR_5_LINE				0x04
	#define T6963_CURSOR_6_LINE				0x05
	#define T6963_CURSOR_7_LINE				0x06
	#define T6963_CURSOR_8_LINE				0x07

#define T6963_SET_DATA_AUTO_WRITE			0xB0
#define T6963_SET_DATA_AUTO_READ			0xB1
#define T6963_AUTO_RESET				    0xB2

#define T6963_SCREEN_REVERSE                0xD0
#define T6963_SCREEN_REVERSE_OFF            0x00
#define T6963_SCREEN_REVERSE_ON             0x01

#define T6963_DATA_WRITE_AND_INCREMENT		0xC0
#define T6963_DATA_READ_AND_INCREMENT		0xC1
#define T6963_DATA_WRITE_AND_DECREMENT		0xC2
#define T6963_DATA_READ_AND_DECREMENT		0xC3
#define T6963_DATA_WRITE_AND_NONVARIABLE	0xC4
#define T6963_DATA_READ_AND_NONVARIABLE		0xC5

#define T6963_SCREEN_PEEK					0xE0
#define T6963_SCREEN_COPY					0xE8

#define T6963_SET_BIT                       0xF8
#define T6963_CLR_BIT                       0xF0
/*
*********************************************************************************************************
*                                          FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void lcd_pin_setup(LCDC_t* h_lcdc, LcdPin_Typedef lcd_pin);
void lcd_setup(LCDC_t* h_lcdc);
void lcd_Reset(LCDC_t* h_lcdc, GPIO_PinState state);
void lcd_font_select(LCDC_t* h_lcdc, font_width_t font_sel);

uint8_t lcd_read_data(LCDC_t* h_lcdc);
void lcd_write_data(LCDC_t* h_lcdc, uint8_t data);
void lcd_write_cmd(LCDC_t* h_lcdc, uint8_t cmd);
uint8_t lcd_read_status(LCDC_t* h_lcdc);

void lcd_set_address(LCDC_t* h_lcdc, uint16_t addr);
void lcd_setup(LCDC_t* h_lcdc);
void lcd_init(LCDC_t* h_lcdc);
void lcd_all_reset(void);

void lcd_clear_graph(LCDC_t* h_lcdc);
void lcd_clear_CGRAM(LCDC_t* h_lcdc);
void lcd_clear_text(LCDC_t* h_lcdc);

inline void lcd_set_graphic_home_addr(LCDC_t* h_lcdc);
inline void lcd_set_graphic_area(LCDC_t* h_lcdc);
inline void lcd_set_text_home_addr(LCDC_t* h_lcdc);
inline void lcd_set_text_area(LCDC_t* h_lcdc);
inline void lcd_set_offset_register(LCDC_t* h_lcdc);

void lcd_xy(LCDC_t* h_lcdc, int x, int y);
void lcd_print_ram(LCDC_t* h_lcdc, int x,int y,char *string);
void lcd_print(LCDC_t* h_lcdc, int x,int y,char *string);
void lcd_pixel(LCDC_t* h_lcdc, int16_t column, int16_t row, char show);
void lcd_show(LCDC_t* h_lcdc, uint8_t * img, uint16_t start_line,int how_many_line);

void lcd_init(LCDC_t* h_lcdc);
void lcd_set_page_addr(LCDC_t* h_lcdc, uint16_t page_addr);
void lcd_text_fill(LCDC_t* h_lcdc, uint8_t ch);
void lcd_line(LCDC_t* h_lcdc, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t show);
void lcd_circle(LCDC_t* h_lcdc, int16_t x, int16_t y, int16_t radius, uint8_t show);
void lcd_circle_half(LCDC_t* h_lcdc, int16_t x, int16_t y, int16_t radius, uint8_t show);
void lcd_box(LCDC_t* h_lcdc, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t show);
void lcd_fill(LCDC_t* h_lcdc, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t persent,char first);
void lcd_circle_segment(LCDC_t* h_lcdc, int x, int y, int radius, int winkel_start, int winkel_end, unsigned char show);
void lcd_box_filled (LCDC_t* h_lcdc, int x1, int y1, int x2, int y2, unsigned char show);
void lcd_set_cursor(LCDC_t* h_lcdc, uint8_t x, uint8_t y);
/*
*********************************************************************************************************
*/
#endif  //ifndef __drv_t6963c_lcdc_h___
