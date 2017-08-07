/**
  ******************************************************************************
  * filename :  m_lcm_sdram.c
  * author:     Ralph Lee
  * date:       25-July-2017
  ******************************************************************************
  */

/**************** Includes ****************************************************/

#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "usbh_diskio.h"
#include "m_init.h"
#include "drv_t6963c_lcdc.h"

/**************** Defintion for LCM *********************************************/
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

#define BUFFER_SIZE         ((uint32_t)0x0100)
#define WRITE_READ_ADDR     ((uint32_t)0x0800)
#define REFRESH_COUNT       ((uint32_t)0x0569)   /* SDRAM refresh counter (90MHz SD clock) */

SRAM_HandleTypeDef hsram;
FMC_NORSRAM_TimingTypeDef SRAM_Timing;

/************ Definition for SDRAM ***************************/
    
SDRAM_HandleTypeDef hsdram;
FMC_SDRAM_TimingTypeDef SDRAM_Timing;
FMC_SDRAM_CommandTypeDef command;

#if 0
/* Read/Write Buffers */
uint32_t aTxBuffer[BUFFER_SIZE];
uint32_t aRxBuffer[BUFFER_SIZE];
#endif

/* Status variables */
__IO uint32_t uwWriteReadStatus = 0;

/* Counter index */
uint32_t uwIndex = 0;

/* Private function prototypes -----------------------------------------------*/
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);
//static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset);

/************ Definition for accessing USB Disk ***************************/
#if 1
FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
FIL MyFile;                   /* File object */
char USBDISKPath[4];          /* USB Host logical drive path */
USBH_HandleTypeDef hUSB_Host; /* USB Host handle */

typedef enum {
  APPLICATION_IDLE = 0,  
  APPLICATION_START,    
  APPLICATION_RUNNING,
  APPLICATION_END,
}MSC_ApplicationTypeDef;

MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;

typedef struct {
    const char* filename;
    uint32_t    filesize;
}FontFile_t;
#define FONT_FILE_NUMBER   4
FontFile_t fontfile_list[FONT_FILE_NUMBER]={
    {"12x24.bin",   12288},
    {"SPCFSUPP.24", 26280},
    {"SPCFONT.24",  29376},
    {"stdfontm.24", 942768},
};

char* FontAddr[FONT_FILE_NUMBER];

#endif
/* Private function prototypes -----------------------------------------------*/ 
static void SystemClock_Config(void);
static void Error_Handler(void);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
static void LoadFont(void);
int put_char_12x24_at(LCDC_t* h_lcdc, uint16_t x, uint16_t y, uint8_t ch);
int put_char_24x24_at(LCDC_t* h_lcdc, uint16_t x, uint16_t y, uint16_t ch);

/**************** Local declaration *********************************************/

/**** I/O for LCM control *******/
GPIO_TypeDef* LCM1_GPIO_PORT[LCM_SIG_NO] = {
                                LCM0_FS_GPIO_PORT,
                                LCM0_RST_GPIO_PORT,
                                };
uint16_t LCM1_GPIO_PIN[LCM_SIG_NO] = {
                                LCM0_FS_GPIO_PIN,
                                LCM0_RST_PIN,
                                };
// LCM paramter structure  
LCDC_t  LCM1 = {
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
        LCM1_GPIO_PORT,                     // pointer to GPIO address list used for controlling LCD
        LCM1_GPIO_PIN,                      // pointer to GPIO pin number list
};
char* LCM_startup[] = {
    "   LCM initialized.",
    "   LCM size: 240x64",
};
char* sdram_startup[] = {
    "   SDRAM initialized.",
    "   Size: 8M Bytes",
    "   Address: 0xd00000000~0xd800000",
};
char* welcom[]={
    " T6963C compatible LCM Demostration",
};
char* string[] = {
    "Menu List:",
    "1. Set font width to 6 or 8 dots.",
    "2. Select page from 0 ~ 10.",
    "3. Select CG RAM offset from 0~15. ",
    "4. Select internal or external font.",
    "5. Set cursor on/off.",
    "6. Set display mode.",
    "7. Select cursor pattern.",
};

int m_init(void)
{    
    /* STM32F4xx HAL library initialization:
        - Configure the Flash prefetch, instruction and Data caches
        - Configure the Systick to generate an interrupt each 1 msec
        - Set NVIC Group Priority to 4
        - Global MSP (MCU Support Package) initialization
     */
    HAL_Init();

    /* Configure LED3 and LED4 */
    BSP_LED_Init(LED3);  
    BSP_LED_Init(LED4);

    /* Configure the system clock to 168 MHz */
    SystemClock_Config();

    //************* Initialize LCM ***********************
    lcd_setup(&LCM1);
    
    /*##-1- Configure the SRAM device ##########################################*/
    /* SRAM device configuration */ 
  
    hsram.Instance  = FMC_NORSRAM_DEVICE;
    hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

    SRAM_Timing.AddressSetupTime       = 10;     // 2
    SRAM_Timing.AddressHoldTime        = 3;     // 1
    SRAM_Timing.DataSetupTime          = 10;     // 2
    SRAM_Timing.BusTurnAroundDuration  = 3;     // 1
    SRAM_Timing.CLKDivision            = 2;     // 2
    SRAM_Timing.DataLatency            = 2;     // 2
    SRAM_Timing.AccessMode             = FMC_ACCESS_MODE_A;

    hsram.Init.NSBank             = FMC_NORSRAM_BANK2;
    hsram.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
    hsram.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
    hsram.Init.MemoryDataWidth    = SRAM_MEMORY_WIDTH;
    hsram.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE;
    hsram.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram.Init.WrapMode           = FMC_WRAP_MODE_DISABLE;
    hsram.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
    hsram.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
    hsram.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
    hsram.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;
    hsram.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;
    hsram.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ASYNC;

    /* Initialize the SRAM controller */
    if(HAL_SRAM_Init(&hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler(); 
    }
    lcd_init(&LCM1);
    lcd_clear_text(&LCM1);
    lcd_clear_graph(&LCM1);
    lcd_clear_CGRAM(&LCM1);

#if 1
    lcd_clear_text(&LCM1);
    lcd_box(&LCM1, 0, 0, 239, 63, 1);
    lcd_print(&LCM1, 1, 1, LCM_startup[0]);
    lcd_print(&LCM1, 1, 1, LCM_startup[1]);
    HAL_Delay(100);
#else
    lcd_box(&LCM1, 0, 0, 239, 63, 1);
    for (int y = 0; y < 8; y++)
    {
        lcd_print(&LCM1, 0, y, string[y]);
        //lcd_box_filled(&LCM1, y*20+20, (y*8), y*20+40, (y*8)+5, 1);
    }
    lcd_box_filled(&LCM1, 0, 7, LCD_MAX_HORIZONTAL_DOTS-1, 8*2-1, 1);
#endif

    //************* Initialize SDRAM ***********************

    /*##-1- Configure the SDRAM device #########################################*/
    /* SDRAM device configuration */ 
    hsdram.Instance = FMC_SDRAM_DEVICE;

    /* Timing configuration for 90 MHz of SD clock frequency (180MHz/2) */
    /* TMRD: 2 Clock cycles */
    SDRAM_Timing.LoadToActiveDelay    = 2;
    /* TXSR: min=70ns (6x11.90ns) */
    SDRAM_Timing.ExitSelfRefreshDelay = 7;
    /* TRAS: min=42ns (4x11.90ns) max=120k (ns) */
    SDRAM_Timing.SelfRefreshTime      = 4;
    /* TRC:  min=63 (6x11.90ns) */        
    SDRAM_Timing.RowCycleDelay        = 7;
    /* TWR:  2 Clock cycles */
    SDRAM_Timing.WriteRecoveryTime    = 2;
    /* TRP:  15ns => 2x11.90ns */
    SDRAM_Timing.RPDelay              = 2;
    /* TRCD: 15ns => 2x11.90ns */
    SDRAM_Timing.RCDDelay             = 2;

    hsdram.Init.SDBank             = FMC_SDRAM_BANK2;
    hsdram.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;
    hsdram.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;
    hsdram.Init.MemoryDataWidth    = SDRAM_MEMORY_WIDTH;
    hsdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
    hsdram.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
    hsdram.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    hsdram.Init.SDClockPeriod      = SDCLOCK_PERIOD;
    hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_DISABLE;
    hsdram.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_1;

    /* Initialize the SDRAM controller */
    if(HAL_SDRAM_Init(&hsdram, &SDRAM_Timing) != HAL_OK)
    {
    /* Initialization Error */
    Error_Handler(); 
    }

    /* Program the SDRAM external device */
    SDRAM_Initialization_Sequence(&hsdram, &command);   

    lcd_clear_text(&LCM1);
    lcd_box(&LCM1, 0, 0, 239, 63, 1);
    lcd_print(&LCM1, 1, 1, sdram_startup[0]);
    lcd_print(&LCM1, 1, 2, sdram_startup[1]);
    lcd_print(&LCM1, 1, 3, sdram_startup[2]);


    /************* Initialize USB disk and load font ***********************
      It will wait for USB disk been detected, then download the font file listed 
      in the fontfile_list[]. After downloaded, stop the USB host working.
      **********************************************************************/
      
    /*##-1- Link the USB Host disk I/O driver ##################################*/
    if(FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
    {
        /*##-2- Init Host Library ################################################*/
        USBH_Init(&hUSB_Host, USBH_UserProcess, 0);

        /*##-3- Add Supported Class ##############################################*/
        USBH_RegisterClass(&hUSB_Host, USBH_MSC_CLASS);

        /*##-4- Start Host Process ###############################################*/
        USBH_Start(&hUSB_Host);

        /*##-5- Run Application (Blocking mode) ##################################*/
        while (1)
        {
            /* USB Host Background task */
            USBH_Process(&hUSB_Host);

            /* Mass Storage Application State Machine */
            switch(Appli_state)
            {
                case APPLICATION_START:
                    LoadFont();
                    Appli_state = APPLICATION_END;
                    break;
                case APPLICATION_IDLE:
                default:
                    break;      
            }
            if (Appli_state == APPLICATION_END) break;
        }
        USBH_Stop(&hUSB_Host);
    }

    
    lcd_clear_text(&LCM1);
    lcd_box(&LCM1, 0, 0, 239, 63, 1);
    lcd_print(&LCM1, 1, 2, welcom[0]);
    return 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig (&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
  }
}

/**
  * @brief  Perform the SDRAM exernal memory inialization sequence
  * @param  hsdram: SDRAM handle
  * @param  Command: Pointer to SDRAM command structure
  * @retval None
  */
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
  __IO uint32_t tmpmrd =0;
  /* Step 3:  Configure a clock configuration enable command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 4: Insert 100 ms delay */
  HAL_Delay(100);
    
  /* Step 5: Configure a PALL (precharge all) command */ 
  Command->CommandMode 			 = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget 	     = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);  
  
  /* Step 6 : Configure a Auto-Refresh command */ 
  Command->CommandMode 			 = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 4;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);
  
  /* Step 7: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_3           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
  
  Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK2;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);
  
  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT); 
}
/**
  * @brief  User Process
  * @param  phost: Host handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{  
  switch(id)
  { 
  case HOST_USER_SELECT_CONFIGURATION:
    break;
    
  case HOST_USER_DISCONNECTION:
    Appli_state = APPLICATION_IDLE;
    BSP_LED_Off(LED3); 
    BSP_LED_Off(LED4);      
    f_mount(NULL, (TCHAR const*)"", 0);      
    break;
    
  case HOST_USER_CLASS_ACTIVE:
    Appli_state = APPLICATION_START;
    break;
    
  default:
    break;
  }
}

static void LoadFont(void)
{
    FRESULT res;                                          /* FatFs function common result code */
    uint32_t byteswritten, bytesread;                     /* File write/read counts */
    //uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
    uint8_t* font_addr = (uint8_t*)(SDRAM_BANK_ADDR);
    const char* filename;
    uint32_t    filesize;
  
    /* Register the file system object to the FatFs module */
    if(f_mount(&USBDISKFatFs, (TCHAR const*)USBDISKPath, 0) != FR_OK)
    {
        /* FatFs Initialization Error */
        Error_Handler();
    }
    else
    {
        for (int i = 0; i < FONT_FILE_NUMBER; i++)
        {
            filename = fontfile_list[i].filename;
            filesize = fontfile_list[i].filesize;
            if(f_open(&MyFile, filename, FA_READ) != FR_OK)
            {
                Error_Handler();
            }else{
                res = f_read(&MyFile, font_addr, filesize, (void *)&bytesread);

                if((bytesread == 0) || (res != FR_OK)) {
                    Error_Handler();
                }
                else
                {
                    f_close(&MyFile);

                    /* Compare read data with the expected data */
                    if((bytesread != filesize))
                    {                
                        /* Read data is different from the expected data */
                        Error_Handler();
                    }
                    FontAddr[i] = font_addr;
                    font_addr += filesize;  // point to next available space.
                }
            }
        }
        #if 0
        /* Open the text file object with read access */
        if(f_open(&MyFile, "12x24.bin", FA_READ) != FR_OK)
        {
            /* 'STM32.TXT' file Open for read Error */
            Error_Handler();
        }
        else
        {
            /* Read data from the text file */
            res = f_read(&MyFile, rtext, (256*48), (void *)&bytesread);

            if((bytesread == 0) || (res != FR_OK))
            {
                /* 'STM32.TXT' file Read or EOF Error */
                Error_Handler();
            }
            else
            {
                /* Close the open text file */
                f_close(&MyFile);

                /* Compare read data with the expected data */
                if((bytesread != (256*48)))
                {                
                    /* Read data is different from the expected data */
                    Error_Handler();
                }
                else
                {
                    /* Success of the demo: no error occurrence */
                    BSP_LED_On(LED3);
                }
            }
        }
        #endif
    }
  
  /* Unlink the USB disk I/O driver */
  FATFS_UnLinkDriver(USBDISKPath);
}    
                  
/**
  * @brief  Fills buffer with user predefined data.
  * @param  pBuffer: pointer on the buffer to fill
  * @param  uwBufferLenght: size of the buffer to fill
  * @param  uwOffset: first value to fill on the buffer
  * @retval None
  */
static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLenght, uint32_t uwOffset)
{
  uint32_t tmpIndex = 0;

  /* Put in global buffer different values */
  for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
  {
    pBuffer[tmpIndex] = tmpIndex + uwOffset;
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
#define	FONT_SIZE_12X24		(48)
#define FONT_HEIGHT_12X24   (24)
#define FONT_WIDTH_12X24    (12)

#define	FONT_SIZE_24X24		(72)
#define FONT_HEIGHT_24X24   (24)
#define FONT_WIDTH_24X24    (24)

#define MAX_CURSOR_X_12X24  (240/12)
#define MAX_CURSOR_Y_12X24  (64/48)

static  uint8_t*    FontBaseAddr = (uint8_t*)(SDRAM_BANK_ADDR);

int put_char_12x24_at(LCDC_t* h_lcdc, uint16_t x, uint16_t y, uint8_t ch)
{
  	uint8_t*	pFont = FontBaseAddr + ch * FONT_SIZE_12X24;  // Get the start address of the wanted font
    //uint16_t	x = cursor12x24_x * 2;                          // Convert the cursor in 12x24 coordicate system to 6x8 primitive one.
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
                mask >>= 1;
            }
        }
    }
}
int put_char_24x24_at(LCDC_t* h_lcdc, uint16_t x, uint16_t y, uint16_t ch)
{
  	uint8_t*	pFont = FontAddr[1] + ch * FONT_SIZE_24X24;  // Get the start address of the wanted font
    //uint16_t	x = cursor12x24_x * 2;                          // Convert the cursor in 12x24 coordicate system to 6x8 primitive one.
    uint8_t     mask = 0x80;
    uint8_t     ix, iy;
    for (iy = 0; iy < FONT_HEIGHT_24X24; iy++, mask = 0x80)
    {
        for (ix = 0; ix < FONT_WIDTH_24X24; ix++)
        {
            lcd_pixel(h_lcdc, (x + ix), (y + iy), (*pFont & mask));
            if (ix % 8 == 7){
                pFont++;        // point to next byte
                mask = 0x80;    // re-initialize mask
            }else{
                mask >>= 1;
            }
        }
    }
}

/**
  * @}
  */ 

/**
  * @}
  */ 


