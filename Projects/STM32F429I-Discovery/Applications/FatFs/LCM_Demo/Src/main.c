/**
  ******************************************************************************
  * file    LCM-Demo/Src/main.c 
  * author  Ralph Lee
  * version V0.0.1
  * date    26-July-2017
  * brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"    
#include "m_init.h"
#include "drv_t6963c_lcdc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variable/function prototypes -----------------------------------------------*/ 
extern LCDC_t  LCM1;


/* Private function prototypes -----------------------------------------------*/ 
int put_char_12x24_at(LCDC_t* h_lcdc, uint16_t x, uint16_t y, uint8_t ch);
int put_char_24x24_at(LCDC_t* h_lcdc, uint16_t x, uint16_t y, uint16_t ch);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    
    m_init();

    lcd_write_cmd(&LCM1, T6963_DISPLAY_MODE|T6963_GRAPHIC_DISPLAY_ON);

    for (uint8_t i = 0; i < 255; i ++)
    {
        put_char_12x24_at(&LCM1, (i*12)%240, (i/20)*24, i+'0');
    }

    uint8_t x = 0, y = 0;
    //lcd_box_filled(&LCM1, x, y, 239, y+24, 0);
    for (uint16_t ch = 0; ch < 13864; ch++){
        put_char_24x24_at(&LCM1, x, y, ch);
        x+=24;
        if (x >= 240){
            x = 0;  y += 24;
            if (y >= 64) y = 0;
            HAL_Delay(300);
            //lcd_box_filled(&LCM1, x, y, 239, y+24, 0);
        }
    }
    while (1)
    {
        for (uint8_t y = 0; y < LCM1.max_vertical_dot/8; y++)
        {
            for (uint8_t x = 0; x < LCM1.bytes_per_row; x++)
            {
                lcd_set_cursor(&LCM1, x, y);
                HAL_Delay(100);
            }
        }
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
