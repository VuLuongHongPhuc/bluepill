/*
 * ssd1306.c
 *
 *  Created on: May 8, 2024
 *
 * Write in C for compatibility with Microchip (class not supported)
 *
 * OLED SSD1306
 * SPI config:
 *   +- MSBFISRT
 *   +- CPOL=0 (low)
 *   +- CPHA=0 (1edge)
 */




/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ssd1306.h"
#include "font.h"


/* Private typedef -----------------------------------------------------------*/
#define SPI_SSD1306


/* Private define ------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static int16_t _width;// Display w/h as modified by current rotation
static int16_t _height;

static StructSpi* _pSpi;

static int16_t _cursor_x;
static int16_t _cursor_y;
static uint8_t _textcolor;
static uint8_t _textbgcolor;
static uint8_t _textsize;
static uint8_t _rotation;


static uint8_t _vccstate;



static uint8_t oled_buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x80, 0x80, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF,
#if (SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH > 96*16)
0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
0x80, 0xFF, 0xFF, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x8C, 0x8E, 0x84, 0x00, 0x00, 0x80, 0xF8,
0xF8, 0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80,
0x00, 0xE0, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xC7, 0x01, 0x01,
0x01, 0x01, 0x83, 0xFF, 0xFF, 0x00, 0x00, 0x7C, 0xFE, 0xC7, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF,
0xFF, 0xFF, 0x00, 0x38, 0xFE, 0xC7, 0x83, 0x01, 0x01, 0x01, 0x83, 0xC7, 0xFF, 0xFF, 0x00, 0x00,
0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x7F, 0xFF,
0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF,
0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x8F,
0x8F, 0x9F, 0xBF, 0xFF, 0xFF, 0xC3, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC,
0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x01, 0x03, 0x03, 0x03,
0x03, 0x03, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01,
0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x00, 0x00,
0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x03,
0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#if (SSD1306_LCDHEIGHT == 64)
0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F, 0x0F,
0x87, 0xC7, 0xF7, 0xFF, 0xFF, 0x1F, 0x1F, 0x3D, 0xFC, 0xF8, 0xF8, 0xF8, 0xF8, 0x7C, 0x7D, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x00, 0x30, 0x30, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xC0, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xC0, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0x1F,
0x0F, 0x07, 0x1F, 0x7F, 0xFF, 0xFF, 0xF8, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF8, 0xE0,
0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00,
0x00, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x0E, 0xFC, 0xF8, 0x00, 0x00, 0xF0, 0xF8, 0x1C, 0x0E,
0x06, 0x06, 0x06, 0x0C, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0x00, 0xFC,
0xFE, 0xFC, 0x00, 0x18, 0x3C, 0x7E, 0x66, 0xE6, 0xCE, 0x84, 0x00, 0x00, 0x06, 0xFF, 0xFF, 0x06,
0x06, 0xFC, 0xFE, 0xFC, 0x0C, 0x06, 0x06, 0x06, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0xC0, 0xF8,
0xFC, 0x4E, 0x46, 0x46, 0x46, 0x4E, 0x7C, 0x78, 0x40, 0x18, 0x3C, 0x76, 0xE6, 0xCE, 0xCC, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x1F, 0x0F, 0x03,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00,
0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x03, 0x07, 0x0E, 0x0C,
0x18, 0x18, 0x0C, 0x06, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x01, 0x0F, 0x0E, 0x0C, 0x18, 0x0C, 0x0F,
0x07, 0x01, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00,
0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x07,
0x07, 0x0C, 0x0C, 0x18, 0x1C, 0x0C, 0x06, 0x06, 0x00, 0x04, 0x0E, 0x0C, 0x18, 0x0C, 0x0F, 0x07,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#endif
#endif
};



/* Private function prototypes -----------------------------------------------*/
void SSD1306_Initialize(uint8_t vccstate, bool reset);
void SSD1306_command(uint8_t cmd);



/* Private function ---------------------------------------------------------*/
void SSD1306_command(uint8_t cmd)
{
	// CS HIGH
	_pSpi->CS(1);

	// DC LOW
	_pSpi->DC(0);

	// CS LOW
	_pSpi->CS(0);

	// Write command
	_pSpi->Write(&cmd, 1);

	// CS HIGH
	_pSpi->CS(1);
}

#if 0
    // TODO: I2C
    unsigned char _i2caddr

    void SSD1306_command(unsigned char c) {
        unsigned char control = 0x00;   // Co = 0, D/C = 0
        Wire.beginTransmission(_i2caddr);
        Wire.write(control);
        Wire.write(c);
        Wire.endTransmission();
    }
#endif




/* Initialize Code -----------------------------------------------------------------*/

bool SSD1306_SPI_Initialize(StructSpi* pSpi)
{
	if (pSpi == NULL) {return false;}

	_pSpi = pSpi;


	_width = SSD1306_LCDWIDTH;
	_height = SSD1306_LCDHEIGHT;

	SSD1306_Initialize(SSD1306_SWITCHCAPVCC, 1);

	return true;
}


void SSD1306_Initialize(uint8_t vccstate, bool reset)
{
	// pause wait display to power up
	_pSpi->delay(500);

	// set default value
	_pSpi->CS(0);
	_pSpi->DC(0);
	_pSpi->RES(0);

	_rotation    = 0;
	_cursor_x    = 0;
	_cursor_y    = 0;
	_textsize    = 1;
	_textcolor   = WHITE;
	_textbgcolor = BLACK;

	_vccstate = vccstate;//SSD1306_SWITCHCAPVCC;

	/* Initialize LCD */
	if (reset)
	{
		// Set reset pin HIGH (used by both SPI and I2C)
		_pSpi->RES(1);

		// VDD (3.3V) goes high at start, lets just chill for a ms
		_pSpi->delay(1);

		// bring reset LOW
		_pSpi->RES(0);

		// wait 10ms
		_pSpi->delay(10);

		// bring out of reset
		_pSpi->RES(1);

		// turn on VCC (9V?)
	}

	// Init sequence
	SSD1306_command(SSD1306_DISPLAYOFF);                    // 0xAE
	SSD1306_command(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
	SSD1306_command(0x80);                                  // the suggested ratio 0x80

	SSD1306_command(SSD1306_SETMULTIPLEX);                  // 0xA8
	SSD1306_command(SSD1306_LCDHEIGHT - 1);

	SSD1306_command(SSD1306_SETDISPLAYOFFSET);              // 0xD3
	SSD1306_command(0x0);                                   // no offset
	SSD1306_command(SSD1306_SETSTARTLINE | 0x0);            // line #0
	SSD1306_command(SSD1306_CHARGEPUMP);                    // 0x8D
	if (vccstate == SSD1306_EXTERNALVCC)
	{ SSD1306_command(0x10); }
	else
	{ SSD1306_command(0x14); }
	SSD1306_command(SSD1306_MEMORYMODE);                    // 0x20
	SSD1306_command(0x00);                                  // 0x0 act like ks0108
	SSD1306_command(SSD1306_SEGREMAP | 0x1);
	SSD1306_command(SSD1306_COMSCANDEC);

	#if defined SSD1306_128_32
	SSD1306_command(SSD1306_SETCOMPINS);                    // 0xDA
	SSD1306_command(0x02);
	SSD1306_command(SSD1306_SETCONTRAST);                   // 0x81
	SSD1306_command(0x8F);

	#elif defined SSD1306_128_64
	SSD1306_command(SSD1306_SETCOMPINS);                    // 0xDA
	SSD1306_command(0x12);
	SSD1306_command(SSD1306_SETCONTRAST);                   // 0x81
	if (vccstate == SSD1306_EXTERNALVCC)
	{ SSD1306_command(0x9F); }
	else
	{ SSD1306_command(0xCF); }

	#elif defined SSD1306_96_16
	SSD1306_command(SSD1306_SETCOMPINS);                    // 0xDA
	SSD1306_command(0x2);   //ada x12
	SSD1306_command(SSD1306_SETCONTRAST);                   // 0x81
	if (vccstate == SSD1306_EXTERNALVCC)
	{ SSD1306_command(0x10); }
	else
	{ SSD1306_command(0xAF); }

	#endif

	SSD1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
	if (vccstate == SSD1306_EXTERNALVCC)
	{ SSD1306_command(0x22); }
	else
	{ SSD1306_command(0xF1); }
	SSD1306_command(SSD1306_SETVCOMDETECT);                 // 0xDB
	SSD1306_command(0x40);
	SSD1306_command(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
	SSD1306_command(SSD1306_NORMALDISPLAY);                 // 0xA6

	SSD1306_command(SSD1306_DEACTIVATE_SCROLL);

	SSD1306_command(SSD1306_DISPLAYON);//--turn on oled panel
}





/* Global --------------------------------------------------------------------*/
void SSD1306_Display(void)
{
	SSD1306_command(SSD1306_COLUMNADDR);
	SSD1306_command(0);   // Column start address (0 = reset)
	SSD1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

	SSD1306_command(SSD1306_PAGEADDR);
	SSD1306_command(0); // Page start address (0 = reset)
	#if SSD1306_LCDHEIGHT == 64
	  SSD1306_command(7); // Page end address
	#endif
	#if SSD1306_LCDHEIGHT == 32
	  SSD1306_command(3); // Page end address
	#endif
	#if SSD1306_LCDHEIGHT == 16
	  SSD1306_command(1); // Page end address
	#endif



	_pSpi->CS(1); // HIGH
	_pSpi->DC(1); // HIGH
	_pSpi->CS(0); // LOW

	_pSpi->Write(oled_buffer, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT)/8);

	_pSpi->CS(1); // HIGH
}

void SSD1306_DisplayOnOff(bool onoff)
{
    if (onoff)
    {
    	SSD1306_command(SSD1306_DISPLAYON);//--turn on oled panel
    }
    else
    {
    	SSD1306_command(SSD1306_DISPLAYOFF);//--turn off oled panel --> sleep mode
    }
}

void SSD1306_Dim(bool dim)
{
    uint8_t contrast;

    if (dim)
    {
        contrast = 0; // Dimmed display
    }
    else
    {
        if (_vccstate == SSD1306_EXTERNALVCC)
        {
            contrast = 0x9F;
        }
        else
        {
            contrast = 0xCF;
        }
    }
    // the range of contrast to too small to be really useful
    // it is useful to dim the display
    SSD1306_command(SSD1306_SETCONTRAST);
    SSD1306_command(contrast);
}

/* Clear buffer */
void SSD1306_Clear(void)
{
	memset(oled_buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8));
}

/* Invert screen */
void SSD1306_Invert(bool invert)
{
  if (invert)
  {
    SSD1306_command(SSD1306_INVERTDISPLAY);
  }
  else
  {
    SSD1306_command(SSD1306_NORMALDISPLAY);
  }
}


// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void startscrollright(uint8_t start, uint8_t stop)
{
  SSD1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
  SSD1306_command(0x00);
  SSD1306_command(start);
  SSD1306_command(0x00);
  SSD1306_command(stop);
  SSD1306_command(0x00);
  SSD1306_command(0xFF);
  SSD1306_command(SSD1306_ACTIVATE_SCROLL);
}


// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void startscrollleft(uint8_t start, uint8_t stop)
{
  SSD1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
  SSD1306_command(0x00);
  SSD1306_command(start);
  SSD1306_command(0x00);
  SSD1306_command(stop);
  SSD1306_command(0x00);
  SSD1306_command(0xFF);
  SSD1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void startscrolldiagright(uint8_t start, uint8_t stop)
{
  SSD1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
  SSD1306_command(0x00);
  SSD1306_command(SSD1306_LCDHEIGHT);
  SSD1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  SSD1306_command(0x00);
  SSD1306_command(start);
  SSD1306_command(0x00);
  SSD1306_command(stop);
  SSD1306_command(0x01);
  SSD1306_command(SSD1306_ACTIVATE_SCROLL);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void startscrolldiagleft(uint8_t start, uint8_t stop)
{
  SSD1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
  SSD1306_command(0x00);
  SSD1306_command(SSD1306_LCDHEIGHT);
  SSD1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  SSD1306_command(0x00);
  SSD1306_command(start);
  SSD1306_command(0x00);
  SSD1306_command(stop);
  SSD1306_command(0x01);
  SSD1306_command(SSD1306_ACTIVATE_SCROLL);
}


void SSD1306_DrawPixel(int16_t x, int16_t y, uint8_t color)
{
	if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
		return;

	// check _rotation, move pixel around if necessary
	switch (_rotation)
	{
		case 1:
			_swap(x, y);
			x = _width - x - 1;
			break;
		case 2:
			x = _width - x - 1;
			y = _height - y - 1;
			break;
		case 3:
			_swap(x, y);
			y = _height - y - 1;
			break;

		default: break;
	}

	// x is which column
	switch (color)
	{
		case WHITE:   oled_buffer[x+ (y/8)*SSD1306_LCDWIDTH] |=  (1 << (y&7)); break;
		case BLACK:   oled_buffer[x+ (y/8)*SSD1306_LCDWIDTH] &= ~(1 << (y&7)); break;
		case INVERSE: oled_buffer[x+ (y/8)*SSD1306_LCDWIDTH] ^=  (1 << (y&7)); break;
	}
}


void SSD1306_DrawFastVLine(int32_t x, int32_t y, int32_t h, uint8_t color)
{
	bool bSwap = false;
	switch(_rotation)
	{
		case 0:
		  break;
		case 1:
		  // 90 degree _rotation, swap x & y for _rotation, then invert x and adjust x for h (now to become w)
		  bSwap = true;
		  _swap(x, y);
		  x = SSD1306_LCDWIDTH - x - 1;
		  x -= (h-1);
		  break;
		case 2:
		  // 180 degree _rotation, invert x and y - then shift y around for height.
		  x = SSD1306_LCDWIDTH - x - 1;
		  y = SSD1306_LCDHEIGHT - y - 1;
		  y -= (h-1);
		  break;
		case 3:
		  // 270 degree _rotation, swap x & y for _rotation, then invert y
		  bSwap = true;
		  _swap(x, y);
		  y = SSD1306_LCDHEIGHT - y - 1;
		  break;
	}

	if(bSwap)
	{
		SSD1306_DrawFastHLineInternal(x, y, h, color);
	}
	else
	{
		SSD1306_DrawFastVLineInternal(x, y, h, color);
	}
}


void SSD1306_DrawFastHLine(int32_t x, int32_t y, int32_t w, uint8_t color)
{
	bool bSwap = false;
	switch(_rotation)
	{
		default:
		case 0:
			// 0 degree _rotation, do nothing
			break;
		case 1:
			// 90 degree _rotation, swap x & y for _rotation, then invert x
			bSwap = true;
			_swap(x, y);
			x = SSD1306_LCDWIDTH - x - 1;
			break;
		case 2:
			// 180 degree _rotation, invert x and y - then shift y around for height.
			x = SSD1306_LCDWIDTH - x - 1;
			y = SSD1306_LCDHEIGHT - y - 1;
			x -= (w-1);
			break;
		case 3:
			// 270 degree _rotation, swap x & y for _rotation, then invert y  and adjust y for w (not to become h)
			bSwap = true;
			_swap(x, y);
			y = SSD1306_LCDHEIGHT - y - 1;
			y -= (w-1);
			break;
	}

	if(bSwap)
	{
		SSD1306_DrawFastVLineInternal(x, y, w, color);
	}
	else
	{
		SSD1306_DrawFastHLineInternal(x, y, w, color);
	}
}


void SSD1306_DrawFastHLineInternal(int32_t x, int32_t y, int32_t w, uint8_t color)
{
	// Do bounds/limit checks
	if(y < 0 || y >= SSD1306_LCDHEIGHT) { return; }

	// make sure we don't try to draw below 0
	if(x < 0)
	{
		w += x;
		x = 0;
	}

	// make sure we don't go off the edge of the display
	if( (x + w) > SSD1306_LCDWIDTH)
	{
		w = (SSD1306_LCDWIDTH - x);
	}

	// if our width is now negative, punt
	if(w <= 0) { return; }

	// set up the pointer for  movement through the buffer
	register uint8_t *pBuf = oled_buffer;
	// adjust the buffer pointer for the current row
	pBuf += ((y/8) * SSD1306_LCDWIDTH);
	// and offset x columns in
	pBuf += x;

	register uint8_t mask = 1 << (y&7);

	switch (color)
	{
	default:
		case WHITE:         while(w--) { *pBuf++ |= mask; }; break;
		case BLACK: mask = ~mask;   while(w--) { *pBuf++ &= mask; }; break;
		case INVERSE:         while(w--) { *pBuf++ ^= mask; }; break;
	}
}


void SSD1306_DrawFastVLineInternal(int32_t x, int32_t __y, int32_t __h, uint8_t color)
{
    // do nothing if we're off the left or right side of the screen
  if(x < 0 || x >= SSD1306_LCDWIDTH) { return; }

  // make sure we don't try to draw below 0
  if(__y < 0) {
    // __y is negative, this will subtract enough from __h to account for __y being 0
    __h += __y;
    __y = 0;

  }

  // make sure we don't go past the height of the display
  if( (__y + __h) > SSD1306_LCDHEIGHT) {
    __h = (SSD1306_LCDHEIGHT - __y);
  }

  // if our height is now negative, punt
  if(__h <= 0) {
    return;
  }

  // this display doesn't need ints for coordinates, use local byte registers for faster juggling
  register uint8_t y = __y;
  register uint8_t h = __h;


  // set up the pointer for fast movement through the buffer
  register uint8_t *pBuf = oled_buffer;
  // adjust the buffer pointer for the current row
  pBuf += ((y/8) * SSD1306_LCDWIDTH);
  // and offset x columns in
  pBuf += x;

  // do the first partial byte, if necessary - this requires some masking
  register uint8_t mod = (y&7);
  if(mod) {
    // mask off the high n bits we want to set
    mod = 8-mod;

    // note - lookup table results in a nearly 10% performance improvement in fill* functions
    // register uint8_t mask = ~(0xFF >> (mod));
    static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
    register uint8_t mask = premask[mod];

    // adjust the mask if we're not going to reach the end of this byte
    if( h < mod) {
      mask &= (0XFF >> (mod-h));
    }

  switch (color)
    {
    case WHITE:   *pBuf |=  mask;  break;
    case BLACK:   *pBuf &= ~mask;  break;
    case INVERSE: *pBuf ^=  mask;  break;
    }

    // fast exit if we're done here!
    if(h<mod) { return; }

    h -= mod;

    pBuf += SSD1306_LCDWIDTH;
  }


  // write solid bytes while we can - effectively doing 8 rows at a time
  if(h >= 8) {
    if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
      do  {
      *pBuf=~(*pBuf);

        // adjust the buffer forward 8 rows worth of data
        pBuf += SSD1306_LCDWIDTH;

        // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
        h -= 8;
      } while(h >= 8);
      }
    else {
      // store a local value to work with
      register uint8_t val = (color == WHITE) ? 255 : 0;

      do  {
        // write our value in
      *pBuf = val;

        // adjust the buffer forward 8 rows worth of data
        pBuf += SSD1306_LCDWIDTH;

        // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
        h -= 8;
      } while(h >= 8);
      }
    }

  // now do the final partial byte, if necessary
  if(h) {
    mod = h & 7;
    // this time we want to mask the low bits of the byte, vs the high bits we did above
    // register uint8_t mask = (1 << mod) - 1;
    // note - lookup table results in a nearly 10% performance improvement in fill* functions
    static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
    register uint8_t mask = postmask[mod];
    switch (color)
    {
      case WHITE:   *pBuf |=  mask;  break;
      case BLACK:   *pBuf &= ~mask;  break;
      case INVERSE: *pBuf ^=  mask;  break;
    }
  }
}

void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color)
{
    // Update in subclasses if desired!
    if(x0 == x1)
    {
        if(y0 > y1) _swap_int16_t(y0, y1);
        SSD1306_DrawFastVLine(x0, y0, y1 - y0 + 1, color);
    }
    else if(y0 == y1)
    {
        if(x0 > x1) _swap_int16_t(x0, x1);
        SSD1306_DrawFastHLine(x0, y0, x1 - x0 + 1, color);
    }
    else
    {
    	SSD1306_WriteLine(x0, y0, x1, y1, color);
    }
}



// Bresenham's algorithm - thx wikpedia
void SSD1306_WriteLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color)
{

	int16_t steep = _ABS(y1 - y0) > _ABS(x1 - x0);
    if (steep)
    {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1)
    {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = _ABS(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    for (; x0<=x1; x0++)
    {
        if (steep)
        {
            SSD1306_DrawPixel(y0, x0, color);
        }
        else
        {
        	SSD1306_DrawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0)
        {
            y0 += ystep;
            err += dx;
        }
    }
}

void SSD1306_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
    // Update in subclasses if desired!
    int16_t i;
    for ( i=x; i<x+w; i++)
    {
    	SSD1306_DrawFastVLine(i, y, h, color);
    }
}


// Draw a rectangle
void SSD1306_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
	SSD1306_DrawFastHLine(x, y, w, color);
	SSD1306_DrawFastHLine(x, y+h-1, w, color);
    SSD1306_DrawFastVLine(x, y, h, color);
    SSD1306_DrawFastVLine(x+w-1, y, h, color);
}

// Draw a circle outline
void SSD1306_DrawCircle(int16_t x, int16_t y, int16_t r, uint8_t color)
{
	// x and y is the center coordinate

    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x_circle = 0;
    int16_t y_circle = r;


    SSD1306_DrawPixel(x  , y+r, color);
    SSD1306_DrawPixel(x  , y-r, color);
    SSD1306_DrawPixel(x+r, y  , color);
    SSD1306_DrawPixel(x-r, y  , color);

    while (x_circle<y_circle)
    {
        if (f >= 0)
        {
            y_circle--;
            ddF_y += 2;
            f += ddF_y;
        }
        x_circle++;
        ddF_x += 2;
        f += ddF_x;

        SSD1306_DrawPixel(x + x_circle, y + y_circle, color);
        SSD1306_DrawPixel(x - x_circle, y + y_circle, color);
        SSD1306_DrawPixel(x + x_circle, y - y_circle, color);
        SSD1306_DrawPixel(x - x_circle, y - y_circle, color);
        SSD1306_DrawPixel(x + y_circle, y + x_circle, color);
        SSD1306_DrawPixel(x - y_circle, y + x_circle, color);
        SSD1306_DrawPixel(x + y_circle, y - x_circle, color);
        SSD1306_DrawPixel(x - y_circle, y - x_circle, color);
    }

}


void SSD1306_FillCircle(int16_t x, int16_t y, int16_t radius, uint8_t color)
{
	SSD1306_DrawFastVLine(x, y-radius, 2*radius+1, color);
	SSD1306_FillCircleHelper(x, y, radius, 3, 0, color);
}

// Used to do circles and roundrects
void SSD1306_FillCircleHelper(int16_t x0, int16_t y0, int16_t r, int8_t cornername, int16_t delta, uint8_t color)
{

    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;

        if (cornername & 0x1)
        {
        	SSD1306_DrawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
        	SSD1306_DrawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
        }
        if (cornername & 0x2)
        {
        	SSD1306_DrawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
        	SSD1306_DrawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
        }
    }
}

// Draw a rounded rectangle
void SSD1306_DrawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint8_t color)
{
    // smarter version

	SSD1306_DrawFastHLine(x+r  , y    , w-2*r, color); // Top
	SSD1306_DrawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
	SSD1306_DrawFastVLine(x    , y+r  , h-2*r, color); // Left
	SSD1306_DrawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
    // draw four corners
	SSD1306_DrawCircleHelper(x+r    , y+r    , r, 1, color);
	SSD1306_DrawCircleHelper(x+w-r-1, y+r    , r, 2, color);
	SSD1306_DrawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
	SSD1306_DrawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}


void SSD1306_DrawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint8_t color)
{
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4)
        {
            SSD1306_DrawPixel(x0 + x, y0 + y, color);
            SSD1306_DrawPixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2)
        {
            SSD1306_DrawPixel(x0 + x, y0 - y, color);
            SSD1306_DrawPixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8)
        {
            SSD1306_DrawPixel(x0 - y, y0 + x, color);
            SSD1306_DrawPixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1)
        {
            SSD1306_DrawPixel(x0 - y, y0 - x, color);
            SSD1306_DrawPixel(x0 - x, y0 - y, color);
        }
    }
}


// Draw a triangle
void SSD1306_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t color)
{
	SSD1306_DrawLine(x0, y0, x1, y1, color);
	SSD1306_DrawLine(x1, y1, x2, y2, color);
	SSD1306_DrawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
void SSD1306_FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t color)
{

    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1)
    {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }
    if (y1 > y2)
    {
        _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);
    }
    if (y0 > y1)
    {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }


    if(y0 == y2)
    { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if(x1 < a)      a = x1;
        else if(x1 > b) b = x1;
        if(x2 < a)      a = x2;
        else if(x2 > b) b = x2;
        SSD1306_DrawFastHLine(a, y0, b-a+1, color);

        return;
    }

    int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
    int
    sa   = 0,
    sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if(y1 == y2) last = y1;   // Include y1 scanline
    else         last = y1-1; // Skip it

    for(y=y0; y<=last; y++)
    {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        SSD1306_DrawFastHLine(a, y, b-a+1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for(; y<=y2; y++)
    {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        SSD1306_DrawFastHLine(a, y, b-a+1, color);
    }

}



/* Text Code ----------------------------------------------------------------------*/

void SSD1306_SetTextSize(uint8_t s)
{
    _textsize = (s > 0) ? s : 1;
}

void SSD1306_SetTextColor(uint8_t color)
{
    _textcolor = color;

    // the background is always oppose
    if (color == WHITE)
    	_textbgcolor = BLACK;
    else
    	_textbgcolor = WHITE;
}

void SSD1306_SetCursor(int16_t x, int16_t y)
{
    _cursor_x = x;
    _cursor_y = y;
}

void SSD1306_WriteChar(uint8_t c)
{
    // 'Classic' built-in font

	// Newline?
    if(c == '\n')
    {
        _cursor_x  = 0;                     // Reset x to zero,
        _cursor_y += _textsize * 8;          // advance y one line
    }
    // Ignore carriage returns
    else if(c != '\r')
    {
        //if(wrap && ((_cursor_x + _textsize * 6) > _width)) { // Off right?
        if ((_cursor_x + _textsize * 6) > _width)
        { // Off right?
            _cursor_x  = 0;                 // Reset x to zero,
            _cursor_y += _textsize * 8;      // advance y one line
        }
        SSD1306_DrawChar(_cursor_x, _cursor_y, c, _textcolor, _textbgcolor, _textsize);
        _cursor_x += _textsize * 6;          // Advance x one char
    }
}

// Draw a character
void SSD1306_DrawChar(int16_t x, int16_t y, uint8_t c, uint8_t color, uint16_t bg, uint8_t size)
{

	uint8_t i,j;
	uint8_t line; // -> vertical line

	if ((x >= _width)            || // Clip right
		(y >= _height)           || // Clip bottom
		((x + 6 * size - 1) < 0) || // Clip left
		((y + 8 * size - 1) < 0))   // Clip top
	return;

	for (i=0; i<6; i++ ) // -> vertical
	{
		if (i == 5)
			line = 0x0;
		else
			line = *(font+(c*5)+i);

		for ( j = 0; j<8; j++) // j -> byte of line = 8
		{
			if (line & 0x1)
			{
				if (size == 1) // default size
					SSD1306_DrawPixel(x+i, y+j, color);
				else
				{  // big size
					SSD1306_FillRect(x+(i*size), y+(j*size), size, size, color);
				}
			}
			else
			{
				if ( (color == BLACK) && (bg != color))
				{
					if (size == 1) // default size
						SSD1306_DrawPixel(x+i, y+j, bg);
					else
					{  // big size
						SSD1306_FillRect(x+i*size, y+j*size, size, size, bg);
					}
				}
			}
			line >>= 1;
		}
	}
}



/* Custom ----------------------------------------------------------*/

void SSD1306_Println(const char* str)
{
    while(*str != '\0')
    {
    	SSD1306_DrawChar(_cursor_x, _cursor_y, *(str++), _textcolor, _textbgcolor, _textsize);
        _cursor_x += _textsize*6;
    }

    _cursor_y += _textsize*8;
    _cursor_x = 0;
}

void ssd1306_select_page(unsigned char index)
{
    // une page = 8 lignes

	SSD1306_command(SSD1306_COLUMNADDR);
    SSD1306_command(0);   // Column start address (0 = reset)
    SSD1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

    SSD1306_command(SSD1306_PAGEADDR);
    SSD1306_command(0); // Page start address (0 = reset)
    SSD1306_command(7); // Page end address
    /*
    #if SSD1306_LCDHEIGHT == 64
    if(index < 8 ){
    #endif
    #if SSD1306_LCDHEIGHT == 32
    if(index < 4 ){
    #endif
    #if SSD1306_LCDHEIGHT == 16
    if(index < 2 ){
    #endif

        page_index = index;
    }
    else {
        page_index = 0;
    }

    SSD1306_command(page_index); // Page start address (0 = reset)
    SSD1306_command(page_index+1); dress
     */
}







/*EOF*/

