/*
 * ssd1306.h
 *
 *  Created on: May 8, 2024
 *
 * from Adafruit lib
 * OLED SSD1306
 */

#ifndef SSD1306_SSD1306_H_
#define SSD1306_SSD1306_H_

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

    #define software_lcd "lcd ssd1306 1.0"

    #define BLACK 0
    #define WHITE 1
    #define INVERSE 2

    #define SSD1306_I2C_ADDRESS   0x3C  // 011110+SA0+RW - 0x3C or 0x3D
    // Address for 128x32 is 0x3C
    // Address for 128x64 is 0x3D (default) or 0x3C (if SA0 is grounded)

    /*=========================================================================
    SSD1306 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
    Select the appropriate display below to create an appropriately
    sized framebuffer, etc.

    SSD1306_128_64  128x64 pixel display

    SSD1306_128_32  128x32 pixel display

    SSD1306_96_16

    -----------------------------------------------------------------------*/

#define SSD1306_128_64
//   #define SSD1306_128_32
//   #define SSD1306_96_16
/*=========================================================================*/

#if defined SSD1306_128_64
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 64
#endif
#if defined SSD1306_128_32
  #define SSD1306_LCDWIDTH                  128
  #define SSD1306_LCDHEIGHT                 32
#endif
#if defined SSD1306_96_16
  #define SSD1306_LCDWIDTH                  96
  #define SSD1306_LCDHEIGHT                 16
#endif

#define SSD1306_SETCONTRAST         0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON        0xA5
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_INVERTDISPLAY       0xA7
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF

#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETCOMPINS          0xDA

#define SSD1306_SETVCOMDETECT       0xDB

#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9

#define SSD1306_SETMULTIPLEX        0xA8

#define SSD1306_SETLOWCOLUMN        0x00
#define SSD1306_SETHIGHCOLUMN       0x10

#define SSD1306_SETSTARTLINE        0x40

#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COLUMNADDR          0x21
#define SSD1306_PAGEADDR            0x22

#define SSD1306_COMSCANINC          0xC0
#define SSD1306_COMSCANDEC          0xC8

#define SSD1306_SEGREMAP            0xA0

#define SSD1306_CHARGEPUMP          0x8D

#define SSD1306_EXTERNALVCC         0x1
#define SSD1306_SWITCHCAPVCC        0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL              0x2F
#define SSD1306_DEACTIVATE_SCROLL            0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA     0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL      0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL       0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  0x2A


/* MACRO -------------------------------------------------------------*/

#define _ABS(x)          ( (x>=0)? x : ((-1)*(x)) )
#define _MIN(x, y)       ( (x>y)? y : x )

#define _swap_int16_t(a, b) { short t = a; a = b; b = t; }
#define _swap(a, b)         { signed short t = a; a = b; b = t; }


/* Typedef -----------------------------------------------------------*/

/* NOTES: To provide these function */
typedef void (*functWritePin)(uint8_t);
typedef void (*functWrite)(uint8_t* data, uint16_t len);
typedef void (*functDelay)(uint32_t);

typedef struct{
	functWritePin CS;
	functWritePin DC;
	functWritePin RES;
	functWrite Write;
	functDelay delay;
}StructSpi;


/* Global function prototype -----------------------------------------*/
	bool SSD1306_SPI_Initialize(StructSpi* pSpi);


/* SSD1306 base -----------------------------------------------------*/
    void SSD1306_Invert(bool invert);
    void SSD1306_Clear(void);
    void SSD1306_Display(void);
    void SSD1306_DisplayOnOff(bool onoff);

    void startscrollright(unsigned char start, unsigned char stop);
    void startscrollleft(unsigned char start, unsigned char stop);

    void startscrolldiagright(unsigned char start, unsigned char stop);
    void startscrolldiagleft(unsigned char start, unsigned char stop);
    void stopscroll(void);

    void SSD1306_Dim(bool dim);

    void SSD1306_DrawPixel(int16_t x, int16_t y, uint8_t color);

    void SSD1306_DrawFastVLine(int32_t x, int32_t y, int32_t h, uint8_t color);
    void SSD1306_DrawFastHLine(int32_t x, int32_t y, int32_t w, uint8_t color);
    void SSD1306_DrawFastHLineInternal(int32_t x, int32_t y, int32_t w, uint8_t color);
    void SSD1306_DrawFastVLineInternal(int32_t x, int32_t __y, int32_t __h, uint8_t color);
    void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);
    void SSD1306_WriteLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);

    void SSD1306_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
    void SSD1306_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
    void SSD1306_DrawCircle(int16_t x, int16_t y, int16_t r, uint8_t color);
	void SSD1306_FillCircle(int16_t x, int16_t y, int16_t radius, uint8_t color);
	void SSD1306_FillCircleHelper(int16_t x0, int16_t y0, int16_t r, int8_t cornername, int16_t delta, uint8_t color);
	void SSD1306_DrawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint8_t color);
	void SSD1306_DrawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint8_t color);
	void SSD1306_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t color);
	void SSD1306_FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t color);

    void SSD1306_SetTextSize(uint8_t s);
    void SSD1306_SetTextColor(uint8_t color);
    void SSD1306_SetCursor(int16_t x, int16_t y);
    void SSD1306_WriteChar(uint8_t c);
    void SSD1306_DrawChar(int16_t x, int16_t y, uint8_t c, uint8_t color, uint16_t bg, uint8_t size) ;

/* Custom ----------------------------------------------------------*/

    void SSD1306_Println(const char* str);
    void ssd1306_select_page(unsigned char index_page);





#ifdef	__cplusplus
}
#endif

#endif /* SSD1306_SSD1306_H_ */
