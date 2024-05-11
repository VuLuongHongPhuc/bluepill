/*
 * ssd1306_test.c
 *
 *  Created on: May 9, 2024
 *      Author: admin
 */

#include "ssd1306_test.h"
#include "ssd1306.h"

typedef void (*functDelay)(uint32_t millis);
static functDelay Delay;

void ssd1306_initialize_test(void* fDelay)
{
	Delay = (functDelay)fDelay;
}

void ssd1306_testsequence(uint8_t c)
{
    switch(c)
    {
        default:
        case 0:
            SSD1306_Display();
            break;

        case 1:
            SSD1306_Clear();
            SSD1306_DrawPixel(10, 10, WHITE);
            SSD1306_Display();
            break;

        case 2:
            SSD1306_Clear();
            ssd1306_testdrawline();
            SSD1306_Display();
            break;

        case 3:
            SSD1306_Clear();
            // draw rectangles
            ssd1306_testdrawrect();
            SSD1306_Display();
            break;

        case 4:
            SSD1306_Clear();
            // draw multiple rectangles
            ssd1306_testfillrect();
            SSD1306_Display();
            break;

        case 5:
            SSD1306_Clear();
            // draw mulitple circles
            ssd1306_testdrawcircle();
            SSD1306_Display();
            break;

        case 6:
            SSD1306_Clear();
            // draw a white circle, 10 pixel radius
            SSD1306_FillCircle(SSD1306_LCDWIDTH/2, SSD1306_LCDHEIGHT/2, 10, WHITE);
            SSD1306_Display();
            break;

        case 7:
            SSD1306_Clear();
            ssd1306_testdrawroundrect();
            break;

        case 8:
            SSD1306_Clear();
            ssd1306_testdrawtriangle();
            break;

        case 9:
            SSD1306_Clear();
            ssd1306_testfilltriangle();
            break;

        case 10:
            SSD1306_Clear();
            // draw the first ~12 characters in the font
            ssd1306_testdrawchar();
            break;

        case 11:
            SSD1306_Clear();
            break;
    }
}

void ssd1306_testdrawchar(void)
{
    unsigned char i;

    SSD1306_SetTextSize(1);
    SSD1306_SetTextColor(WHITE);
    SSD1306_SetCursor(0,0);

    for ( i=0; i < 168; i++) {
        if (i == '\n') continue;
            SSD1306_WriteChar(i);
        if ((i > 0) && (i % 21 == 0))
        	SSD1306_Println("");
    }
    SSD1306_Display();
}

void ssd1306_testfilltriangle(void) {
  unsigned char color = WHITE;
  short i;
  for (i=_MIN(SSD1306_LCDWIDTH,SSD1306_LCDHEIGHT)/2; i>0; i-=5) {
    SSD1306_FillTriangle(SSD1306_LCDWIDTH/2, SSD1306_LCDHEIGHT/2-i,
                     SSD1306_LCDWIDTH/2-i, SSD1306_LCDHEIGHT/2+i,
                     SSD1306_LCDWIDTH/2+i, SSD1306_LCDHEIGHT/2+i, WHITE);
    if (color == WHITE) color = BLACK;
    else color = WHITE;
    SSD1306_Display();
  }
}

void ssd1306_testdrawtriangle(void) {
    short i;
  for ( i=0; i<_MIN(SSD1306_LCDWIDTH,SSD1306_LCDHEIGHT)/2; i+=5) {
	  SSD1306_DrawTriangle(SSD1306_LCDWIDTH/2, SSD1306_LCDHEIGHT/2-i,
                     SSD1306_LCDWIDTH/2-i, SSD1306_LCDHEIGHT/2+i,
                     SSD1306_LCDWIDTH/2+i, SSD1306_LCDHEIGHT/2+i, WHITE);
    SSD1306_Display();
  }
}

void ssd1306_testdrawroundrect(void) {
    short i;
  for ( i=0; i<SSD1306_LCDHEIGHT/2-2; i+=2) {
	  SSD1306_DrawRoundRect(i, i, SSD1306_LCDWIDTH-2*i, SSD1306_LCDHEIGHT-2*i, SSD1306_LCDHEIGHT/4, WHITE);
    SSD1306_Display();
  }
}

void ssd1306_testdrawcircle(void) {
    short i;
  for ( i=0; i<SSD1306_LCDHEIGHT; i+=2) {
	  SSD1306_FillCircle(SSD1306_LCDWIDTH/2, SSD1306_LCDHEIGHT/2, i, WHITE);
    SSD1306_Display();
  }
}

void ssd1306_testfillrect(void) {
  unsigned char color = 1;
  short i;
  for (i=0; i<SSD1306_LCDHEIGHT/2; i+=3) {
    // alternate colors
	  SSD1306_FillRect(i, i, SSD1306_LCDWIDTH-i*2, SSD1306_LCDHEIGHT-i*2, color%2);
    SSD1306_Display();
    color++;
  }
}

void ssd1306_testdrawrect(void) {
    short i;
  for ( i=0; i<SSD1306_LCDHEIGHT/2; i+=2) {
	  SSD1306_DrawRect(i, i, SSD1306_LCDWIDTH-2*i, SSD1306_LCDHEIGHT-2*i, WHITE);
    SSD1306_Display();
  }
}

void ssd1306_testdrawline(void) {
    short i;

  for ( i=0; i<SSD1306_LCDWIDTH; i+=4) {
    SSD1306_DrawLine(0, 0, i, SSD1306_LCDHEIGHT-1, WHITE);
    SSD1306_Display();
  }
  for ( i=0; i<SSD1306_LCDHEIGHT; i+=4) {
    SSD1306_DrawLine(0, 0, SSD1306_LCDWIDTH-1, i, WHITE);
    SSD1306_Display();
  }
  Delay(250);

  SSD1306_Clear();
  for ( i=0; i<SSD1306_LCDWIDTH; i+=4) {
    SSD1306_DrawLine(0, SSD1306_LCDHEIGHT-1, i, 0, WHITE);
    SSD1306_Display();
  }
  for ( i=SSD1306_LCDHEIGHT-1; i>=0; i-=4) {
    SSD1306_DrawLine(0, SSD1306_LCDHEIGHT-1, SSD1306_LCDWIDTH-1, i, WHITE);
    SSD1306_Display();
  }
  Delay(250);

  SSD1306_Clear();
  for ( i=SSD1306_LCDWIDTH-1; i>=0; i-=4) {
    SSD1306_DrawLine(SSD1306_LCDWIDTH-1, SSD1306_LCDHEIGHT-1, i, 0, WHITE);
    SSD1306_Display();
  }
  for ( i=SSD1306_LCDHEIGHT-1; i>=0; i-=4) {
    SSD1306_DrawLine(SSD1306_LCDWIDTH-1, SSD1306_LCDHEIGHT-1, 0, i, WHITE);
    SSD1306_Display();
  }
  Delay(250);

  SSD1306_Clear();
  for ( i=0; i<SSD1306_LCDHEIGHT; i+=4) {
    SSD1306_DrawLine(SSD1306_LCDWIDTH-1, 0, 0, i, WHITE);
    SSD1306_Display();
  }
  for (i=0; i<SSD1306_LCDWIDTH; i+=4) {
    SSD1306_DrawLine(SSD1306_LCDWIDTH-1, 0, i, SSD1306_LCDHEIGHT-1, WHITE);
    SSD1306_Display();
  }
  Delay(250);
}

/*EOF*/
