/**
  ******************************************************************
  * @file    bsp_lcd.c
  * @author  fire
  * @version V2.0
  * @date    2018-xx-xx
  * @brief   lcdÓ¦ÓÃº¯Êý½Ó¿Ú
  ******************************************************************
  * @attention
  *
  * ÊµÑéÆ½Ì¨:Ò°»ð  i.MXRT1052¿ª·¢°å 
  * ÂÛÌ³    :http://www.firebbs.cn
  * ÌÔ±¦    :https://fire-stm32.taobao.com
  *
  ******************************************************************
  */
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"  
#include "fsl_elcdif.h" 
#include "fsl_clock.h"
#include "fsl_pxp.h"

	
#include "pad_config.h"  
#include "./lcd/bsp_lcd.h" 



/*******************************************************************************
 * ±äÁ¿
 ******************************************************************************/

/* Ö¡ÖÐ¶Ï±êÖ¾ */
volatile bool s_frameDone = false;

/* Ö¡Êý¼ÆÊýÆ÷£¬Ê¹ÄÜÖ¡ÖÐ¶Ï²ÅÓÐÐ§ */
__IO uint32_t s_frame_count = 0;

/* ÏÔ´æ */
AT_NONCACHEABLE_SECTION_ALIGN( pixel_t s_psBufferLcd[2][LCD_PIXEL_HEIGHT][LCD_PIXEL_WIDTH], FRAME_BUFFER_ALIGN);

/*ÓÃÓÚ´æ´¢µ±Ç°Ñ¡ÔñµÄ×ÖÌå¸ñÊ½*/
static sFONT *LCD_Currentfonts = &Font24x48;
/* ÓÃÓÚ´æ´¢µ±Ç°×ÖÌåÑÕÉ«ºÍ×ÖÌå±³¾°ÑÕÉ«µÄ±äÁ¿*/
static pixel_t CurrentTextColor   = CL_WHITE;
static pixel_t CurrentBackColor   = CL_BLACK;

/* Ö¸Ïòµ±Ç°µÄÏÔ´æ£¬ÓÉÓÚÊÇµØÖ·£¬ËùÒÔÓÃ32Î»±äÁ¿ */
static uint32_t CurrentFrameBuffer = (uint32_t)s_psBufferLcd[0];

/*******************************************************************************
 * ºê
 ******************************************************************************/
/* ËùÓÐÒý½Å¾ùÊ¹ÓÃÍ¬ÑùµÄPADÅäÖÃ */
#define LCD_PAD_CONFIG_DATA            (SRE_1_FAST_SLEW_RATE| \
                                        DSE_6_R0_6| \
                                        SPEED_3_MAX_200MHz| \
                                        ODE_0_OPEN_DRAIN_DISABLED| \
                                        PKE_1_PULL_KEEPER_ENABLED| \
                                        PUE_0_KEEPER_SELECTED| \
                                        PUS_0_100K_OHM_PULL_DOWN| \
                                        HYS_0_HYSTERESIS_DISABLED)   
    /* ÅäÖÃËµÃ÷ : */
    /* ×ª»»ËÙÂÊ: ×ª»»ËÙÂÊ¿ì
        Çý¶¯Ç¿¶È: R0/6 
        ´ø¿íÅäÖÃ : max(200MHz)
        ¿ªÂ©ÅäÖÃ: ¹Ø±Õ 
        À­/±£³ÖÆ÷ÅäÖÃ: Ê¹ÄÜ
        À­/±£³ÖÆ÷Ñ¡Ôñ: ±£³ÖÆ÷
        ÉÏÀ­/ÏÂÀ­Ñ¡Ôñ: 100KÅ·Ä·ÏÂÀ­(Ñ¡ÔñÁË±£³ÖÆ÷´ËÅäÖÃÎÞÐ§)
        ÖÍ»ØÆ÷ÅäÖÃ: ½ûÖ¹ */
        
/*******************************************************************************
 * ÉùÃ÷
 ******************************************************************************/
static void LCD_IOMUXC_MUX_Config(void);
static void LCD_IOMUXC_PAD_Config(void);
static void LCD_ELCDIF_Config(void);


/**
* @brief  ³õÊ¼»¯LCDÏà¹ØIOMUXCµÄMUX¸´ÓÃÅäÖÃ
* @param  ÎÞ
* @retval ÎÞ
*/
static void LCD_IOMUXC_MUX_Config(void)
{
    /* ËùÓÐÒý½Å¾ù²»¿ªÆôSION¹¦ÄÜ */
    /* Ê±Ðò¿ØÖÆÐÅºÅÏß */
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_00_LCD_CLK, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_01_LCD_ENABLE, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_02_LCD_HSYNC, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_03_LCD_VSYNC, 0U);
  
    /* RGB565Êý¾ÝÐÅºÅÏß£¬
     DATA0~DATA4:B3~B7
     DATA5~DATA10:G2~G7
     DATA11~DATA15:R3~R7 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_04_LCD_DATA00, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_05_LCD_DATA01, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_06_LCD_DATA02, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_07_LCD_DATA03, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_08_LCD_DATA04, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_09_LCD_DATA05, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_10_LCD_DATA06, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_11_LCD_DATA07, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_12_LCD_DATA08, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_13_LCD_DATA09, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_14_LCD_DATA10, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_15_LCD_DATA11, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_00_LCD_DATA12, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_01_LCD_DATA13, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_02_LCD_DATA14, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_03_LCD_DATA15, 0U); 

		/* ÈôÊ¹ÓÃ24Î»Êý¾ÝÐÅºÅÏßÐèÒª³õÊ¼»¯ÆäÓàÊý¾ÝÐÅºÅÏß */
#if LCD_BUS_24_BIT
		IOMUXC_SetPinMux(IOMUXC_GPIO_B1_04_LCD_DATA16, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_05_LCD_DATA17, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_06_LCD_DATA18, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_07_LCD_DATA19, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_08_LCD_DATA20, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_09_LCD_DATA21, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_10_LCD_DATA22, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_B1_11_LCD_DATA23, 0U);                                    
#endif 
		
    /* LCD_BL±³¹â¿ØÖÆÐÅºÅÏß */
    IOMUXC_SetPinMux(LCD_BL_IOMUXC, 0U); 
}


/**
* @brief  ³õÊ¼»¯LCDÏà¹ØIOMUXCµÄPADÊôÐÔÅäÖÃ
* @param  ÎÞ
* @retval ÎÞ
*/
static void LCD_IOMUXC_PAD_Config(void)
{  
    /* ËùÓÐÒý½Å¾ùÊ¹ÓÃÍ¬ÑùµÄPADÅäÖÃ */
    /* Ê±Ðò¿ØÖÆÐÅºÅÏß */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_00_LCD_CLK,LCD_PAD_CONFIG_DATA);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_01_LCD_ENABLE, LCD_PAD_CONFIG_DATA);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_02_LCD_HSYNC, LCD_PAD_CONFIG_DATA);  
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_03_LCD_VSYNC, LCD_PAD_CONFIG_DATA); 

    /* RGB565Êý¾ÝÐÅºÅÏß£¬
     DATA0~DATA4:B3~B7
     DATA5~DATA10:G2~G7
     DATA11~DATA15:R3~R7 */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_04_LCD_DATA00, LCD_PAD_CONFIG_DATA); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_05_LCD_DATA01, LCD_PAD_CONFIG_DATA); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_06_LCD_DATA02, LCD_PAD_CONFIG_DATA);  
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_07_LCD_DATA03, LCD_PAD_CONFIG_DATA); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_08_LCD_DATA04, LCD_PAD_CONFIG_DATA); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_09_LCD_DATA05, LCD_PAD_CONFIG_DATA);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_10_LCD_DATA06, LCD_PAD_CONFIG_DATA);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_11_LCD_DATA07, LCD_PAD_CONFIG_DATA);  
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_12_LCD_DATA08, LCD_PAD_CONFIG_DATA);  
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_13_LCD_DATA09, LCD_PAD_CONFIG_DATA);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_14_LCD_DATA10, LCD_PAD_CONFIG_DATA); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_15_LCD_DATA11, LCD_PAD_CONFIG_DATA);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_00_LCD_DATA12, LCD_PAD_CONFIG_DATA);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_01_LCD_DATA13, LCD_PAD_CONFIG_DATA);
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_02_LCD_DATA14, LCD_PAD_CONFIG_DATA); 
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_03_LCD_DATA15, LCD_PAD_CONFIG_DATA); 
		
		/* ÈôÊ¹ÓÃ24Î»Êý¾ÝÐÅºÅÏßÐèÒª³õÊ¼»¯ÆäÓàÊý¾ÝÐÅºÅÏß */
#if LCD_BUS_24_BIT
		IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_04_LCD_DATA16, LCD_PAD_CONFIG_DATA);                                    
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_05_LCD_DATA17, LCD_PAD_CONFIG_DATA);                                    
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_06_LCD_DATA18, LCD_PAD_CONFIG_DATA);                                    
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_07_LCD_DATA19, LCD_PAD_CONFIG_DATA);                                    
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_08_LCD_DATA20, LCD_PAD_CONFIG_DATA);                                    
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_09_LCD_DATA21, LCD_PAD_CONFIG_DATA);                                    
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_10_LCD_DATA22, LCD_PAD_CONFIG_DATA);                                    
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_11_LCD_DATA23, LCD_PAD_CONFIG_DATA);                                    
#endif     
    /* LCD_BL±³¹â¿ØÖÆÐÅºÅÏß */
    IOMUXC_SetPinConfig(LCD_BL_IOMUXC, LCD_PAD_CONFIG_DATA);
}

/**
* @brief  ³õÊ¼»¯ELCDIFÍâÉè
* @param  ÎÞ
* @retval ÎÞ
*/
static void LCD_ELCDIF_Config(void)
{	
    const elcdif_rgb_mode_config_t config = {
        .panelWidth = LCD_PIXEL_WIDTH,
        .panelHeight = LCD_PIXEL_HEIGHT,
        .hsw = LCD_HSW,
        .hfp = LCD_HFP,
        .hbp = LCD_HBP,
        .vsw = LCD_VSW,
        .vfp = LCD_VFP,
        .vbp = LCD_VBP,
        .polarityFlags =  LCD_POLARITY_FLAGS,													
        .bufferAddr = (uint32_t)s_psBufferLcd[0],
        .pixelFormat = ELCDIF_PIXEL_FORMAT,
        .dataBus = LCD_DATA_BUS_WIDTH,
    };
 
  ELCDIF_RgbModeInit(LCDIF, &config);
  ELCDIF_RgbModeStart(LCDIF);
}

/**
* @brief  ³õÊ¼»¯ELCDIFÊ¹ÓÃµÄÊ±ÖÓ
* @param  ÎÞ
* @retval ÎÞ
*/
void LCD_InitClock(void)
{
    /*
     * Òª°ÑÖ¡ÂÊÉèÖÃ³É60Hz£¬ËùÒÔÏñËØÊ±ÖÓÆµÂÊÎª:
     * Ë®Æ½ÏñËØÊ±ÖÓ¸öÊý£º(LCD_IMG_WIDTH + LCD_HSW + LCD_HFP + LCD_HBP ) 
     * ´¹Ö±ÐÐÊý£º(LCD_IMG_HEIGHT + LCD_VSW + LCD_VFP + LCD_VBP)
     * 
     * ÏñËØÊ±ÖÓÆµÂÊ£º(800 + 1 + 22 + 46) * (480 + 1 + 22 + 23) * 60 = 27.4M.
     * ±¾Àý×ÓÉèÖÃ LCDIF ÏñËØÊ±ÖÓÆµÂÊÎª 27M.
     *	 LCDµÄÖ¡ÂÊÒÔÊµ²âµÄÎª×¼¡£
     */

    /*
     * ³õÊ¼»¯ Vedio PLL£¬¼´PLL5
     * Video PLL Êä³öÆµÂÊÎª 
     * OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 108MHz.
     */
    clock_video_pll_config_t config = {
        .loopDivider = 36, .postDivider = 8, .numerator = 0, .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    /* Ñ¡ÔñÎªvedio PLL£¬¼´PLL5 */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

    /* ÉèÖÃÔ¤·ÖÆµ */  
    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 1);

		/* ÉèÖÃ·ÖÆµ */  
    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
}

/**
* @brief  ³õÊ¼»¯±³¹âÒý½Å²¢µãÁÁ
* @param  ÎÞ
* @retval ÎÞ
*/
void LCD_BackLight_ON(void)
{    
    /* ±³¹â£¬¸ßµçÆ½µãÁÁ */
    gpio_pin_config_t config = {
      kGPIO_DigitalOutput, 
      1,
      kGPIO_NoIntmode
    };

    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);
}



/**
* @brief  ³õÊ¼»¯Òº¾§ÆÁ
* @param  enableInterrupt £ºÊÇ·ñÊ¹ÄÜÖÐ¶Ï
*		@arg LCD_INTERRUPT_DISABLE ²»Ê¹ÄÜ
*		@arg LCD_INTERRUPT_ENABLE  Ê¹ÄÜ
* @retval ÎÞ
*/
void LCD_Init(bool enableInterrupt)
{
#if LCD_RGB_888	
	/* 
  * ±¾´úÂëÅäÖÃLCD read_qos ¼° write_qos ¼Ä´æÆ÷£¬Ö§³ÖÅäÖÃÖµµÄ·¶Î§Îª0x0-0xF£¬
  * ´Ë´¦ÉèÖÃqosÎª0xF×î´óÖµ¡£
	*	Qos£º
  * The Quality of Service (QoS) tidemark value represents the maximum
	*	permitted number of active transactions before the QoS mechanism is
	*	activated¡£
	*  ÏêÏ¸ËµÃ÷¼û
	* ¡¶IMXRT1050RM¡·£¨²Î¿¼ÊÖ²á£©µÄÕÂ½Ú¡¶Network Interconnect Bus System (NIC-301)¡·
	* ¼°¡¶CoreLink  Network Interconnect (NIC-301)Technical Reference Manua r2p3¡·
	* @note 
  *  ¼òµ¥À´Ëµ¾ÍÊÇÌá¸ßLCDÊ¹ÓÃRT1052ÄÚ²¿×ÜÏßµÄ´ø¿íÊýÁ¿¡¢ÄÜÁ¦
	*  ¶ÔÓÚ800*480@XRGB8888@60HzµÄÏÔÊ¾±ØÐëÒªÕâÑùÅäÖÃ£¬
  *  ¶ÔÓÚ800*480@RGB565@60HzµÄÏÔÊ¾²»ÐèÒªÅäÖÃ£¬±£³ÖÄ¬ÈÏ¼´¿É£¨ÍÆ¼ö£©
	*/
  *((uint32_t *)0x41044100) = 0x0000000f;
	*((uint32_t *)0x41044104) = 0x0000000f;
#endif
	
	/* ³õÊ¼»¯eLCDIFÒý½Å¡¢Ê±ÖÓ ¡¢Ä£Ê½¡¢±³¹âÒÔ¼°ÖÐ¶Ï*/
  LCD_IOMUXC_MUX_Config();
  LCD_IOMUXC_PAD_Config();
  LCD_InitClock();
	LCD_ELCDIF_Config();
  LCD_BackLight_ON();
  
  if(enableInterrupt)
  {
    LCD_InterruptConfig();
  }
}

/***************************ÖÐ¶ÏÏà¹Ø******************************/
/**
* @brief  ÅäÖÃELCDIFÖÐ¶Ï
* @param  ÎÞ
* @retval ÎÞ
*/
void LCD_InterruptConfig(void)
{
  /* Ê¹ÄÜÖÐ¶Ï */
  EnableIRQ(LCDIF_IRQn);
   
  /* ÅäÖÃELCDIFÎªCurFrameDoneInterruptÖÐ¶Ï */
  ELCDIF_EnableInterrupts(LCDIF, kELCDIF_CurFrameDoneInterruptEnable);
}

/**
* @brief  ELCDIFÖÐ¶Ï·þÎñº¯Êý
* @param  ÎÞ
* @retval ÎÞ
*/
void LCDIF_IRQHandler(void)
{
    uint32_t intStatus;

    intStatus = ELCDIF_GetInterruptStatus(LCDIF);

    ELCDIF_ClearInterruptStatus(LCDIF, intStatus);

   if (intStatus & kELCDIF_CurFrameDone)
    {
				/* µ±Ç°Ö¡´¦ÀíÍê³É±êÖ¾ */
        s_frameDone = true;
				/* Ö¡¼ÆÊýÆ÷ */
				s_frame_count++;

    }

    /* ÒÔÏÂ²¿·ÖÊÇÎª ARM µÄ¿±Îó838869Ìí¼ÓµÄ, 
       ¸Ã´íÎóÓ°Ïì Cortex-M4, Cortex-M4FÄÚºË£¬       
       Á¢¼´´æ´¢¸²¸ÇÖØµþÒì³££¬µ¼ÖÂ·µ»Ø²Ù×÷¿ÉÄÜ»áÖ¸Ïò´íÎóµÄÖÐ¶Ï
        CM7²»ÊÜÓ°Ïì£¬´Ë´¦±£Áô¸Ã´úÂë
    */  
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/***************************ÏÔÊ¾Ó¦ÓÃÏà¹Ø******************************/

/***************************ÏÔÊ¾×Ö·ûÏà¹Ø******************************/

/**
  * @brief  ÉèÖÃ×ÖÌåµÄÑÕÉ«¼°×ÖÌåµÄ±³¾°ÑÕÉ«
  * @param  TextColor: ×ÖÌåÑÕÉ«
  * @param  BackColor: ×ÖÌåµÄ±³¾°ÑÕÉ«
  * @retval None
  */
void LCD_SetColors(pixel_t TextColor, pixel_t BackColor) 
{
  CurrentTextColor = TextColor; 
  CurrentBackColor = BackColor;
}

/**
  * @brief »ñÈ¡µ±Ç°ÉèÖÃµÄ×ÖÌåÑÕÉ«ºÍ×ÖÌåµÄ±³¾°ÑÕÉ«
  * @param  TextColor: Ö¸Ïò×ÖÌåÑÕÉ«µÄÖ¸Õë
  * @param  BackColor: Ö¸Ïò×ÖÌå±³¾°ÑÕÉ«µÄÖ¸Õë
  * @retval None
  */
void LCD_GetColors(pixel_t *TextColor, pixel_t *BackColor)
{
  *TextColor = CurrentTextColor;
  *BackColor = CurrentBackColor;
}

/**
  * @brief  ÉèÖÃ×ÖÌåÑÕÉ«
  * @param  Color: ×ÖÌåÑÕÉ«
  * @retval None
  */
void LCD_SetTextColor(pixel_t Color)
{
  CurrentTextColor = Color;
}

/**
  * @brief  ÉèÖÃ×ÖÌåµÄ±³¾°ÑÕÉ«
  * @param  Color: ×ÖÌåµÄ±³¾°ÑÕÉ«
  * @retval None
  */
void LCD_SetBackColor(pixel_t Color)
{
  CurrentBackColor = Color;
}

/**
  * @brief  ÉèÖÃ×ÖÌå¸ñÊ½(Ó¢ÎÄ)
  * @param  fonts: Ñ¡ÔñÒªÉèÖÃµÄ×ÖÌå¸ñÊ½
  * @retval None
  */
void LCD_SetFont(sFONT *fonts)
{
  LCD_Currentfonts = fonts;
}

/**
  * @brief  »ñÈ¡µ±Ç°×ÖÌå¸ñÊ½(Ó¢ÎÄ)
  * @param  None.
  * @retval µ±Ç°Ó¦ÓÃµÄ¸ñÊ½
  */
sFONT *LCD_GetFont(void)
{
  return LCD_Currentfonts;
}


/**
  * @brief  ÔÚÏÔÊ¾Æ÷ÉÏÏÔÊ¾Ò»¸öÓ¢ÎÄ×Ö·û
  * @param  Xpos £º×Ö·ûµÄÆðÊ¼X×ø±ê
  * @param  Ypos £º×Ö·ûµÄÆðÊ¼Y×ø±ê
  * @param  Ascii: ÒªÏÔÊ¾µÄ×Ö·ûµÄASCIIÂë
  * @retval None
  */
void LCD_DisplayChar(uint16_t Xpos, uint16_t Ypos, char Ascii)
{
  uint16_t fontLength;	
  uint16_t page, column;

	uint16_t relativePositon;
	uint8_t *pfont;
  
  uint32_t yBufferPos = 0;
  uint32_t xPixelPos = 0;
  
  /*yBufferPos±íÊ¾µ±Ç°ÐÐµÄÏÔ´æÆ«ÒÆÎ»ÖÃ*/
  yBufferPos = Ypos*LCD_PIXEL_WIDTH*LCD_BPP;
  
  /*xpixelPos±íÊ¾²¿·ÖÏñËØµãÎ»ÖÃ
    LCD_BPP*xPixelPos + yBufferPos ¾ÍÊÇµ±Ç°ÏñËØµãµÄÏÔ´æÎ»ÖÃ
  */
  xPixelPos += Xpos;
	
	//¶ÔasciiÂë±íÆ«ÒÆ£¨×ÖÄ£±í²»°üº¬ASCII±íµÄÇ°32¸ö·ÇÍ¼ÐÎ·ûºÅ£©
	relativePositon = Ascii - ' ';
	
	//Ã¿¸ö×ÖÄ£µÄ×Ö½ÚÊý
	fontLength = (LCD_Currentfonts->Width*LCD_Currentfonts->Height)/8;
		
	//×ÖÄ£Ê×µØÖ·
	/*asciiÂë±íÆ«ÒÆÖµ³ËÒÔÃ¿¸ö×ÖÄ£µÄ×Ö½ÚÊý£¬Çó³ö×ÖÄ£µÄÆ«ÒÆÎ»ÖÃ*/
	pfont = (uint8_t *)&LCD_Currentfonts->table[relativePositon * fontLength];
	
  //Ã¿¸ö×ÖÄ£ÓÐLCD_Currentfonts->HeightÐÐ£¬±éÀúÃ¿Ò»ÐÐ
  for ( page = 0; page < LCD_Currentfonts->Height; page++ )
	{    
    //Ã¿¸ö×ÖÄ£ÓÐLCD_Currentfonts->Width/8 ¸ö×Ö½Ú£¬±éÀúÃ¿¸ö×Ö½Ú
    for ( column = 0; column < LCD_Currentfonts->Width/8; column++ ) 
		{	
      uint8_t bitCount = 0;

      //Ã¿¸ö×Ö½ÚÓÐ8¸öÊý¾ÝÎ»£¬±éÀúÃ¿¸öÊý¾ÝÎ»
      for(bitCount=0; bitCount<8; bitCount++)
      {
        if(*pfont & (0x80>>bitCount))
        {
           //×ÖÌåÉ«
           *(__IO pixel_t*)(CurrentFrameBuffer + (LCD_BPP*xPixelPos) + yBufferPos) = CurrentTextColor;        
        }
        else
        {
          //±³¾°É«
          *(__IO pixel_t*)(CurrentFrameBuffer + (LCD_BPP*xPixelPos) + yBufferPos) = CurrentBackColor; 
        }
        /*Ö¸Ïòµ±Ç°ÐÐµÄÏÂÒ»¸öµã*/
        xPixelPos++;		
      }
      
      /* Ö¸Ïò×ÖÄ£Êý¾ÝµÄÒ»ÏÂ¸ö×Ö½Ú */
      pfont++;
    }      
    /*ÏÔÊ¾ÍêÒ»ÐÐ*/
    /*Ö¸Ïò×Ö·ûÏÔÊ¾¾ØÕóÏÂÒ»ÐÐµÄµÚÒ»¸öÏñËØµã*/
    xPixelPos += (LCD_PIXEL_WIDTH - LCD_Currentfonts->Width);		
  }
}

/**
 * @brief  ÔÚÏÔÊ¾Æ÷ÉÏÏÔÊ¾ÖÐÓ¢ÎÄ×Ö·û´®,³¬³öÒº¾§¿í¶ÈÊ±»á×Ô¶¯»»ÐÐ¡£
 * @param  Xpos £º×Ö·ûµÄÆðÊ¼X×ø±ê
 * @param  Ypos £º×Ö·ûµÄÆðÊ¼Y×ø±ê
 * @param  pStr £ºÒªÏÔÊ¾µÄ×Ö·û´®µÄÊ×µØÖ·
 * @retval ÎÞ
 */
void LCD_DispString(uint16_t Xpos, uint16_t Ypos, const uint8_t * pStr )
{
	while( * pStr != '\0' )
	{	
    /*×Ô¶¯»»ÐÐ*/
    if ( ( Xpos + LCD_Currentfonts->Width ) > LCD_PIXEL_WIDTH )
    {
      Xpos = 0;
      Ypos += LCD_Currentfonts->Height;
    }
    
    if ( ( Ypos + LCD_Currentfonts->Height ) > LCD_PIXEL_HEIGHT )
    {
      Xpos = 0;
      Ypos = 0;
    }			
        
    /* ÏÔÊ¾µ¥¸ö×Ö·û */
    LCD_DisplayChar(Xpos,Ypos,*pStr);
    
    Xpos += LCD_Currentfonts->Width;
  
    pStr ++;
  }		
} 

/**
  * @brief  ÏÔÊ¾×Ö·û´®(Ó¢ÎÄ)
  * @param  Line: ¸ù¾Ýµ±Ç°×ÖÌå¶ø±äµÄÐÐºÅ
  *     @arg Line(1),Line(2)µÈ
  * @param  *ptr: ÒªÏÔÊ¾µÄ×Ö·û´®
  * @retval None
  */
void LCD_DisplayStringLine(uint16_t Line, uint8_t *ptr)
{  
  uint16_t refcolumn = 0;
  /* Ñ­»·ÏÔÊ¾×Ö·û£¬Ö±ÖÁÓöµ½×Ö·û´®½áÊø·û
    »òÖ±µ½µ¥ÐÐÏÔÊ¾²»ÏÂ×Ö·û
  */
  while ((refcolumn < LCD_PIXEL_WIDTH) && ((*ptr != 0) & 
    ((refcolumn + LCD_Currentfonts->Width) <= LCD_PIXEL_WIDTH)))
  {
    /* ÏÔÊ¾µ¥¸ö×Ö·û */
    LCD_DisplayChar(refcolumn,Line , *ptr);
    /* Æ«ÒÆ×Ö·ûÏÔÊ¾Î»ÖÃ */
    refcolumn += LCD_Currentfonts->Width;
    /* Ö¸ÏòÏÂÒ»¸ö×Ö·û */
    ptr++;
  }
}

/**
  * @brief  Çå³ýÖ¸¶¨ÐÐµÄ×Ö·û
  * @param  Line: ÒªÇå³ýµÄÐÐ,×¢ÒâLINEºêÊÇ¸ù¾Ýµ±Ç°×ÖÌå¶ø±äµÄ
  *     @arg LINE(1),LINE(2)
  * @retval None
  */
void LCD_ClearLine(uint16_t Line)
{
  uint16_t refcolumn = 0;
  /* Ñ­»·ÏÔÊ¾ÖÁÆÁÄ»×îÓÒ²à */
  while ((refcolumn < LCD_PIXEL_WIDTH) && 
    (((refcolumn + LCD_Currentfonts->Width)& 0xFFFF) >= LCD_Currentfonts->Width))
  {
    /* ÏÔÊ¾¿Õ¸ñ£¨Ïàµ±ÓÚÇå³ýµÄÐ§¹û£© */
    LCD_DisplayChar(refcolumn, Line, ' ');
    /* Æ«ÒÆ×Ö·ûÏÔÊ¾Î»ÖÃ */
    refcolumn += LCD_Currentfonts->Width;
  }
}

/**
  * @brief  ÉèÖÃÏÔÊ¾×ø±ê
  * @param  Xpos: x×ø±ê
  * @param  Ypos: y×ø±ê
  * @retval ÏÔ´æµÄµØÖ·
  */
uint32_t LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{  
  return CurrentFrameBuffer + LCD_BPP*(Xpos + (LCD_PIXEL_WIDTH*Ypos));
}

/***************************ÏÔÊ¾Í¼ÐÎÏà¹Ø******************************/
/**
  * @brief Ñ¡Ôñµ±Ç°Òª²Ù×÷µÄÏÔ´æÇøÓò
  * @param  index: 0»ò1
  * @retval None
  */
void LCD_SetFrameBuffer(uint8_t index)
{
  CurrentFrameBuffer = (uint32_t)s_psBufferLcd[index];
}

/**
  * @brief ÉèÖÃºóÃæÒªÏÔÊ¾µÄÏÔ´æÇøÓò
  * @param  index: 0»ò1
  * @retval None
  */
void LCD_SetDisplayBuffer(uint8_t index)
{
  /* ÉèÖÃELCDIFµÄÏÂÒ»¸ö»º³åÇøµØÖ· */
  ELCDIF_SetNextBufferAddr(LCDIF, (uint32_t)s_psBufferLcd[index]);

}

/**
  * @brief Ê¹ÓÃµ±Ç°ÑÕÉ«ÔÚÖ¸¶¨µÄÎ»ÖÃ»æÖÆÒ»¸öÏñËØµã
  * @param  Xpos: x×ø±ê
  * @param  Ypos: y×ø±ê
  * @note ¿ÉÊ¹ÓÃLCD_SetBackColor¡¢LCD_SetTextColor¡¢LCD_SetColorsº¯ÊýÉèÖÃÑÕÉ«
  * @retval None
  */
void PutPixel(uint16_t Xpos, uint16_t Ypos)
{   
	if ( ( Xpos < LCD_PIXEL_WIDTH ) && ( Ypos < LCD_PIXEL_HEIGHT ) )
  {
		*(pixel_t *)(CurrentFrameBuffer + LCD_BPP*(Xpos + (LCD_PIXEL_WIDTH*Ypos))) = CurrentTextColor;
	}
}

/**
  * @brief  ÒÔµ±Ç°±³¾°ÑÕÉ«Çå³ýÕû¸öÆÁÄ»
  * @param  ÎÞ
  * @note ¿ÉÊ¹ÓÃLCD_SetBackColor¡¢LCD_SetTextColor¡¢LCD_SetColorsº¯ÊýÉèÖÃÑÕÉ«
  * @retval ÎÞ
  */
void LCD_Clear(uint32_t Color)
{
  /* Çå³ý»º³åÇøÄÚÈÝ */
  uint16_t page, column;  
  
  /* Ö¸Ïò¾ØÐÎµÚÒ»¸öÏñËØµãµÄÏÔ´æÎ»ÖÃ */
  pixel_t *pRectImage = (pixel_t*)CurrentFrameBuffer ;
  
  /* ±éÀúÃ¿Ò»ÐÐ */
  for ( page = 0; page < LCD_PIXEL_HEIGHT; page++ )
  {    
    /* ±éÀúÃ¿Ò»ÁÐ */
    for ( column = 0; column < LCD_PIXEL_WIDTH; column++ ) 
    {	
      *pRectImage = Color;
      
      /* Ö¸ÏòÏÂÒ»¸öÏñËØµãµÄÏÔ´æÎ»ÖÃ */
      pRectImage++;
    }      
  }
}

/**
  * @brief ÏÔÊ¾Ò»ÌõÖ±Ïß
  * @param Xpos: Ö±ÏßÆðµãµÄx×ø±ê
  * @param Ypos: Ö±ÏßÆðµãµÄy×ø±ê
  * @param Length: Ö±ÏßµÄ³¤¶È
  * @param Direction: Ö±ÏßµÄ·½Ïò£¬¿ÉÊäÈë
      @arg LINE_DIR_HORIZONTAL(Ë®Æ½·½Ïò) 
      @arg LINE_DIR_VERTICAL(´¹Ö±·½Ïò).
  * @note ¿ÉÊ¹ÓÃLCD_SetBackColor¡¢LCD_SetTextColor¡¢LCD_SetColorsº¯ÊýÉèÖÃÑÕÉ«
  * @retval None
  */
void LCD_DrawLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, bool Direction)
{
  uint16_t index; 
  
  uint16_t realLength;
  
  /* Ö¸ÏòÖ±ÏßµÚÒ»¸öÏñËØµãµÄÏÔ´æÎ»ÖÃ */
  pixel_t *pLineImage = (pixel_t*)(CurrentFrameBuffer + LCD_BPP*(Xpos + (LCD_PIXEL_WIDTH*Ypos)));

  if(Direction == LINE_DIR_HORIZONTAL)
  {
    realLength = LCD_PIXEL_WIDTH-Xpos-Length > 0 ? Length : LCD_PIXEL_WIDTH - Xpos;
    
    /* ±éÀúÃ¿Ò»ÁÐ */
    for ( index = 0; index < realLength; index++ ) 
    {
        *pLineImage = CurrentTextColor;
        
        /* Ö¸ÏòÏÂÒ»¸öÏñËØµãµÄÏÔ´æÎ»ÖÃ */
        pLineImage++;
    }
  }
  else
  {
    realLength = LCD_PIXEL_HEIGHT-Ypos-Length > 0 ? Length : LCD_PIXEL_HEIGHT - Ypos;
    
    /* ±éÀúÃ¿Ò»ÐÐ */
    for ( index = 0; index < realLength; index++ ) 
    {
        *pLineImage = CurrentTextColor;
        
        /* Ö¸ÏòÏÂÒ»¸öÏñËØµãµÄÏÔ´æÎ»ÖÃ */
        pLineImage += LCD_PIXEL_WIDTH;
    }
  }   
}

/**
 * @brief  ÔÚÒº¾§ÆÁÉÏÊ¹ÓÃ Bresenham Ëã·¨»­Ïß¶Î£¨»ùÓÚÁ½µã£© 
 * @param  Xpos1 £ºÏß¶ÎµÄÒ»¸ö¶ËµãX×ø±ê
 * @param  Ypos1 £ºÏß¶ÎµÄÒ»¸ö¶ËµãY×ø±ê
 * @param  Xpos2 £ºÏß¶ÎµÄÁíÒ»¸ö¶ËµãX×ø±ê
 * @param  Ypos2 £ºÏß¶ÎµÄÁíÒ»¸ö¶ËµãY×ø±ê
 * @note ¿ÉÊ¹ÓÃLCD_SetBackColor¡¢LCD_SetTextColor¡¢LCD_SetColorsº¯ÊýÉèÖÃÑÕÉ«
 * @retval ÎÞ
 */
void LCD_DrawUniLine ( uint16_t Xpos1, uint16_t Ypos1, uint16_t Xpos2, uint16_t Ypos2 )
{
	uint16_t us; 
	uint16_t usX_Current, usY_Current;
	
	int32_t lError_X = 0, lError_Y = 0, lDelta_X, lDelta_Y, lDistance; 
	int32_t lIncrease_X, lIncrease_Y; 	
	
	
	lDelta_X = Xpos2 - Xpos1; //¼ÆËã×ø±êÔöÁ¿ 
	lDelta_Y = Ypos2 - Ypos1; 
	
	usX_Current = Xpos1; 
	usY_Current = Ypos1; 
	
	
	if ( lDelta_X > 0 ) 
		lIncrease_X = 1; //ÉèÖÃµ¥²½·½Ïò 
	
	else if ( lDelta_X == 0 ) 
		lIncrease_X = 0;//´¹Ö±Ïß 
	
	else 
  { 
    lIncrease_X = -1;
    lDelta_X = - lDelta_X;
  } 

	
	if ( lDelta_Y > 0 )
		lIncrease_Y = 1; 
	
	else if ( lDelta_Y == 0 )
		lIncrease_Y = 0;//Ë®Æ½Ïß 
	
	else 
  {
    lIncrease_Y = -1;
    lDelta_Y = - lDelta_Y;
  } 

	
	if (  lDelta_X > lDelta_Y )
		lDistance = lDelta_X; //Ñ¡È¡»ù±¾ÔöÁ¿×ø±êÖá 
	
	else 
		lDistance = lDelta_Y; 

	
	for ( us = 0; us <= lDistance + 1; us ++ )//»­ÏßÊä³ö 
	{  
		PutPixel ( usX_Current, usY_Current );//»­µã 
		
		lError_X += lDelta_X ; 
		lError_Y += lDelta_Y ; 
		
		if ( lError_X > lDistance ) 
		{ 
			lError_X -= lDistance; 
			usX_Current += lIncrease_X; 
		}  
		
		if ( lError_Y > lDistance ) 
		{ 
			lError_Y -= lDistance; 
			usY_Current += lIncrease_Y; 
		} 
		
	}  	
	
}   

/**
  * @brief  »æÖÆ¿ÕÐÄ¾ØÐÎ
  * @param  Xpos £º¾ØÐÎ×óÉÏ½Ç¶ËµãX×ø±ê
  * @param  Ypos £º¾ØÐÎ×óÉÏ½Ç¶ËµãY×ø±ê
  * @param  Width £º¾ØÐÎ¿í
  * @param  Height £º¾ØÐÎ¸ß
  * @note ¿ÉÊ¹ÓÃLCD_SetBackColor¡¢LCD_SetTextColor¡¢LCD_SetColorsº¯ÊýÉèÖÃÑÕÉ«
  * @retval ÎÞ
  */
void LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{  
  uint16_t realHeight,realWidth;
  
  realHeight = LCD_PIXEL_HEIGHT-Ypos-Height > 0 ? Height : LCD_PIXEL_HEIGHT - Ypos;
  realWidth = LCD_PIXEL_WIDTH-Xpos-Width > 0 ? Width : LCD_PIXEL_WIDTH - Xpos;
  
  LCD_DrawLine(Xpos, Ypos, realWidth, LINE_DIR_HORIZONTAL);
  LCD_DrawLine(Xpos, Ypos, realHeight, LINE_DIR_VERTICAL);
  LCD_DrawLine(Xpos + realWidth - 1, Ypos, realHeight, LINE_DIR_VERTICAL);
  LCD_DrawLine(Xpos, Ypos + realHeight - 1, realWidth, LINE_DIR_HORIZONTAL);
}

/**
  * @brief  »æÖÆÊµÐÄ¾ØÐÎ
  * @param  Xpos £º¾ØÐÎ×óÉÏ½Ç¶ËµãX×ø±ê
  * @param  Ypos £º¾ØÐÎ×óÉÏ½Ç¶ËµãY×ø±ê
  * @param  Width £º¾ØÐÎ¿í
  * @param  Height £º¾ØÐÎ¸ß
  * @note ¿ÉÊ¹ÓÃLCD_SetBackColor¡¢LCD_SetTextColor¡¢LCD_SetColorsº¯ÊýÉèÖÃÑÕÉ«
  * @retval ÎÞ
  */
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  uint16_t page, column; 
  
  uint16_t realHeight,realWidth;
  
  realHeight = LCD_PIXEL_HEIGHT-Ypos-Height > 0 ? Height : LCD_PIXEL_HEIGHT - Ypos;
  realWidth = LCD_PIXEL_WIDTH-Xpos-Width > 0 ? Width : LCD_PIXEL_WIDTH - Xpos;
  
  /* Ö¸Ïò¾ØÐÎµÚÒ»¸öÏñËØµãµÄÏÔ´æÎ»ÖÃ */
  pixel_t *pRectImage = (pixel_t*)(CurrentFrameBuffer + LCD_BPP*(Xpos + (LCD_PIXEL_WIDTH*Ypos)));
  
  /* ±éÀúÃ¿Ò»ÐÐ */
  for ( page = 0; page < realHeight; page++ )
  {    
    /* ±éÀúÃ¿Ò»ÁÐ */
    for ( column = 0; column < realWidth; column++ ) 
    {	
      *pRectImage = CurrentTextColor;
      
      /* Ö¸ÏòÏÂÒ»¸öÏñËØµãµÄÏÔ´æÎ»ÖÃ */
      pRectImage++;
    }      
    /*ÏÔÊ¾ÍêÒ»ÐÐ*/
    /*Ö¸ÏòÏÂÒ»ÐÐµÄµÚÒ»¸öÏñËØµãµÄÏÔ´æÎ»ÖÃ*/
    pRectImage += (LCD_PIXEL_WIDTH - realWidth);		
  }
}

/**
 * @brief  »æÖÆÒ»¸ö¿ÕÐÄÔ²
 * @param  Xpos: Ô²ÐÄX×ø±ê
 * @param  Ypos: Ô²ÐÄY×ø±ê
 * @param  Radius: °ë¾¶
 * @retval None
 */
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
   int x = -Radius, y = 0, err = 2-2*Radius, e2;
   do {
       *(__IO pixel_t*) (CurrentFrameBuffer + (LCD_BPP*((Xpos-x) + LCD_PIXEL_WIDTH*(Ypos+y)))) = CurrentTextColor;
       *(__IO pixel_t*) (CurrentFrameBuffer + (LCD_BPP*((Xpos+x) + LCD_PIXEL_WIDTH*(Ypos+y)))) = CurrentTextColor;
       *(__IO pixel_t*) (CurrentFrameBuffer + (LCD_BPP*((Xpos+x) + LCD_PIXEL_WIDTH*(Ypos-y)))) = CurrentTextColor;
       *(__IO pixel_t*) (CurrentFrameBuffer + (LCD_BPP*((Xpos-x) + LCD_PIXEL_WIDTH*(Ypos-y)))) = CurrentTextColor;

       e2 = err;
       if (e2 <= y) {
           err += ++y*2+1;
           if (-x == y && e2 <= x) e2 = 0;
       }
       if (e2 > x) err += ++x*2+1;
   }
   while (x <= 0);
}

/**
 * @brief  »æÖÆÒ»¸öÊµÐÄÔ²
 * @param  Xpos: Ô²ÐÄX×ø±ê
 * @param  Ypos: Ô²ÐÄY×ø±ê
 * @param  Radius: °ë¾¶
 * @retval None
 */
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{  
  /* »æÖÆÊµÐÄÔ²ÐèÒªÔö¼ÓµÄ²Ù×÷ */
   int32_t  D;    /* Decision Variable */
   uint32_t  CurX;/* Current X Value */
   uint32_t  CurY;/* Current Y Value */

   D = 3 - (Radius << 1);

   CurX = 0;
   CurY = Radius;

   while (CurX <= CurY)
   {
     if(CurY > 0)
     {
       LCD_DrawLine(Xpos - CurX, Ypos - CurY, 2*CurY, LINE_DIR_VERTICAL);
       LCD_DrawLine(Xpos + CurX, Ypos - CurY, 2*CurY, LINE_DIR_VERTICAL);
     }

     if(CurX > 0)
     {
       LCD_DrawLine(Xpos - CurY, Ypos - CurX, 2*CurX, LINE_DIR_VERTICAL);
       LCD_DrawLine(Xpos + CurY, Ypos - CurX, 2*CurX, LINE_DIR_VERTICAL);
     }
     if (D < 0)
     {
       D += (CurX << 2) + 6;
     }
     else
     {
       D += ((CurX - CurY) << 2) + 10;
       CurY--;
     }
     CurX++; 
    }
 
   LCD_DrawCircle(Xpos, Ypos, Radius);

}
/*********************************************END OF FILE**********************/
