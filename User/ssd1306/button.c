/*
 * button.c
 *
 *  Created on: May 10, 2024
 *      Author: admin
 */


/* Includes -----------------------------------------------------------------*/
#include "ssd1306.h"
#include "button.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BUTTON_HEIGHT   11      /*!< height min = 11 */

/* Private variables ---------------------------------------------------------*/
static int16_t _btnWidth = 50;
static uint8_t _cornerRadius = 3;



void SetButtonCornerRadius(uint8_t radius)
{
	_cornerRadius = radius;
}


void SetButtonWidth(int16_t width)
{
	_btnWidth = width;
}



void DrawButtonLine(int16_t x, int16_t y, int16_t width, uint8_t color)
{
	if (color == WHITE)
	{
		SSD1306_DrawFastHLine(x, y, width, WHITE);
	}
	else
	{
		SSD1306_DrawPixel(x, y, WHITE);
		SSD1306_DrawPixel(x+width-1, y, WHITE);
	}
}


/*! \brief Draw button
 * \param x Coordinate X
 * \param y Coordinate Y
 * \param text Content to display
 * \param textSize Font size
 * \param selected To highlight the button
 * */
void DrawButton(int16_t x, int16_t y, const char* text, uint8_t textSize, bool selected)
{
	int16_t width = _btnWidth;

	int16_t height = BUTTON_HEIGHT * textSize;

	// Do bounds/limit checks
	if(y < 0 || (y+height) >= SSD1306_LCDHEIGHT) { return; }

	// make sure we don't try to draw below 0
	if(x < 0)
	{
		width += x;
		x = 0;
	}

	// make sure we don't go off the edge of the display
	if( (x + width) > SSD1306_LCDWIDTH)
	{
		width = (SSD1306_LCDWIDTH - x);
	}

	// if our width is now negative, punt
	if(width <= 0) { return; }


	uint8_t color;
	uint16_t txtColor;
	if (selected)
	{
		color = WHITE;
		txtColor = BLACK;
	}
	else
	{
		color = BLACK;
		txtColor = WHITE;
	}


	// offset for radius
	int offsetL = 3;
	int offsetR = 6;

	for(int i=0; i < height; i++)
	{
		if (i < _cornerRadius)
		{
			DrawButtonLine(x+offsetL, y+i, width-offsetR, (i==0) ? WHITE: color);
			offsetL --;
			offsetR -= 2;
		}
		else if ((i + 1) >= (height - _cornerRadius))
		{
			DrawButtonLine(x+offsetL, y+i, width-offsetR, (i==height-1) ? WHITE: color);
			offsetL ++;
			offsetR += 2;
		}
		else
		{
			DrawButtonLine(x, y+i, width, color);
		}
	}

	uint16_t offset = x+5;
	while(*text != '\0')
	{
		SSD1306_DrawChar(offset, y+(2*textSize), *(text++), txtColor, txtColor, textSize);
		offset += textSize*6;
	}

}



/*EOF*/
