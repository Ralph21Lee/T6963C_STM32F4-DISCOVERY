/*
 *      
 *  DESCRIPTION for Display_12x24.c
 *      
 *   ---Date---     Author      Description
 *   Aug 1, 2017    Ralph Lee   Initial version
 */


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
/*  The screen is 240x64 in size. 
    To display 12x24 font, the screen is devided into two horizontal lines that is 32 dots each. 
    Since the font is 24 dots in height, it occupied the lower 24 dots and leave the upper 8 dots blank.
    Each line can display 20 characters. This make the screen capable of displaying 20 x 2 characters.
*/
#define	FONT_SIZE_12X24		(48)
#define FONT_HEIGHT_12X24   (24)
#define FONT_WIDTH_12X24    (12)
#define MAX_CURSOR_X_12X24  (240/12)
#define MAX_CURSOR_Y_12X24  (64/48)

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
static  uint16_t    cursor12x24_x = 0;
static  uint16_t    cursor12x24_y = 0;
static  uint8_t*    FontBaseAddr = (uint8_t*)(SDRAM_BANK_ADDR);
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/
int put_char_12x24(uint8_t char);
int put_char_12x24_at(uint16_t x, int16_t y, uint8_t char);

/*
*********************************************************************************************************
*                                      EXTERNAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*/



/*
*********************************************************************************************************
* Function    :  int put_char_12x24(uint8_t char)
*
* Description : put a character in 12x24 font at current cursor position
*
* Argument(s) : uint8_t  char
*
* Return(s)   :
*           0 : success
*           1 : fail, could be out of display
*
* Note(s)     : 
*********************************************************************************************************
*/
int put_char_12x24(uint8_t char)
{
  	uint8_t*	pFont = FontBaseAddr + char * FONT_SIZE_12X24;  // Get the start address of the wanted font
    uint16_t	x = cursor12x24_x * 2;                          // Convert the cursor in 12x24 coordicate system to 6x8 primitive one.
    uint8_t     mask = 0x80;
    
    for (uint8_t iy = 0; iy < FONT_HEIGHT_12X24; iy++, pFont++, mask = 0x80)
    {
        for (uint8_t ix = 0; ix < FONT_WIDTH_12X24; ix++)
        {
            lcd_pixel(h_lcdc, (x + ix), (y + iy), (*pFont & mask));
            if (ix == 7){
                pFont++;        // point to next byte
                mask = 0x80;    // re-initialize mask
            }else{
                mask >> 1;
            }
        }
    }
}
/*
*********************************************************************************************************
* Function    :  int put_char_12x24_at(uint16_t x, int16_t y, uint8_t char)
*
* Description : put a character in 12x24 font at specified cursor position
*
* Argument(s) : uint_t  char
*
* Return(s)   :
*           0 : success
*           1 : fail, could be out of display
*
* Note(s)     : 
*********************************************************************************************************
*/
int put_char_12x24_at(uint16_t x, int16_t y, uint8_t char)
{
  	uint8_t*	pFont = FontBaseAddr + char * FONT_SIZE_12X24;  // Get the start address of the wanted font
    uint16_t	x = cursor12x24_x * 2;                          // Convert the cursor in 12x24 coordicate system to 6x8 primitive one.
    
}

/*
*********************************************************************************************************
*                                       xxxxxx()
*
* Description : put a character at specified position
*
* Argument(s) :  
*
* Return(s)   :  
*********************************************************************************************************
*/


