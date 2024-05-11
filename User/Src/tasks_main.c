/*
 * mainTask.c
 *
 *  Created on: May 6, 2024
 *      Author: Phuc VU
 *
 *
 */



/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "cmsis_os.h"
//#include "semphr.h"

#include "../ssd1306/ssd1306.h"
#include "../ssd1306/ssd1306_test.h"
#include "../ssd1306/button.h"
#include "../ssd1306/convert.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//extern CAN_HandleTypeDef hcan;
extern SPI_HandleTypeDef hspi1;

static uint8_t _usb_txbuf[64] = {0};
static uint8_t _usb_rxbuf[64] = {0};

/* Private function prototypes -----------------------------------------------*/
static inline void SPI_Transmit(uint8_t* data, uint16_t len);
static inline void SPI_Delay(uint32_t milis);
static inline void GPIO_WritePinCS(GPIO_PinState PinState);
static inline void GPIO_WritePinDC(GPIO_PinState PinState);
static inline void GPIO_WritePinRESET(GPIO_PinState PinState);
static inline void EnableCoreSysClock(void);
static inline uint32_t GetSysCoreClockCount();


/* Global function prototypes -----------------------------------------------*/
void onUsbReceive(uint8_t* pBuf, uint32_t* pLen);


/* Private code --------------------------------------------------------------*/



/* Global code ---------------------------------------------------------------*/

void onUsbReceive(uint8_t* pBuf, uint32_t* pLen)
{

	for(int i=0; i< *pLen; i++)
	{
		_usb_rxbuf[i] = pBuf[i];
	}

	if ( (_usb_rxbuf[0] == 0x5A) && (_usb_rxbuf[1] == 0x63))
	{
		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
	}
}



void Task_main(void *argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();

	_usb_txbuf[0] = 0xAA;

	while(1)
	{
		osDelay(500);
		//HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

		CDC_Transmit_FS(_usb_txbuf, 4);
	}
}

/* Through SPI: USB and CAN incompatible (know problem) */
void Task_can(void *argument)
{
	while(1)
	{
		osDelay(1);
	}
}



#define DefNbCharIndex 2	// index
#define DefNbCharTimeElapse 5
void Task_display(void *argument)
{
	char cnumber[DefNbCharIndex+1];
	int index = 0;
	StructSpi SPI;

	char ctimenumber[DefNbCharTimeElapse+1];
	uint32_t timeElapse;
	uint32_t rawTimeOpe;


	// init. SSD1306
	SPI.delay = &SPI_Delay;
	SPI.Write = &SPI_Transmit;
	SPI.CS    = &GPIO_WritePinCS;
	SPI.DC    = &GPIO_WritePinDC;
	SPI.RES   = &GPIO_WritePinRESET;
	SSD1306_SPI_Initialize(&SPI);

	// init. SSD1306 for test
	//ssd1306_initialize_test(&SPI_Delay);

	// display logo prebuild
	SSD1306_Display();

	EnableCoreSysClock();
	//xTaskGetTickCount();

	while(1)
	{
		osDelay(2000);

		// test SSD1306
		//for( int i=0; i<11; i++) { ssd1306_testsequence(i); osDelay(1000); }

		// begin time elapse
		rawTimeOpe = GetSysCoreClockCount();


		SSD1306_Clear();

		// default button configuration width=50, height=11, radius=3
		DrawButton(0,0 , "01", 1, (index==0) ? true : false);
		DrawButton(0,12, "02", 1, (index==1) ? true : false);
		DrawButton(0,24, "03", 1, (index==2) ? true : false);
		DrawButton(0,36, "04", 1, (index==3) ? true : false);

		// round + text index
		SSD1306_FillCircle(89, 31, 25, WHITE);
		SSD1306_SetCursor(79, 23);
		SSD1306_SetTextSize(2);
		SSD1306_SetTextColor(BLACK);
		ConvertIntToChar( index, cnumber, DefNbCharIndex);
		SSD1306_Println((const char*) cnumber);

		// display time elapse
		SSD1306_SetCursor(0, 48);
		SSD1306_SetTextSize(2);
		SSD1306_SetTextColor(WHITE);
		ConvertIntToChar(timeElapse, ctimenumber, DefNbCharTimeElapse);
		SSD1306_Println((const char*) ctimenumber);

		SSD1306_Display();

		// inc. index for next button
		index ++;

		// check limit
		if (index == 4) { index = 0; }

		// end time elapse
		timeElapse = (GetSysCoreClockCount() - rawTimeOpe);
		timeElapse /= (SystemCoreClock/1000000);// time in µs

	}
}


/* SPI Hardware dependencies */

inline void SPI_Transmit(uint8_t* data, uint16_t len)
{
	//HAL_SPI_Transmit(hspi, pData, Size, Timeout); // Timeout millisecond
	HAL_SPI_Transmit(&hspi1, data, len, 20);
}

inline void SPI_Delay(uint32_t milis)
{
	osDelay(milis);
}

/* SPI CS -> PB8 */
inline void GPIO_WritePinCS(GPIO_PinState PinState)
{
	if (PinState != GPIO_PIN_RESET)
	{
		GPIOB->BSRR = GPIO_PIN_8;
	}
	else
	{
		GPIOB->BSRR = (uint32_t)GPIO_PIN_8 << 16u;
	}
}

/* SPI DC -> PB7 */
inline void GPIO_WritePinDC(GPIO_PinState PinState)
{
	if (PinState != GPIO_PIN_RESET)
	{
		GPIOB->BSRR = GPIO_PIN_7;
	}
	else
	{
		GPIOB->BSRR = (uint32_t)GPIO_PIN_7 << 16u;
	}
}

/* SPI RESET -> PB6 */
inline void GPIO_WritePinRESET(GPIO_PinState PinState)
{
	if (PinState != GPIO_PIN_RESET)
	{
		GPIOB->BSRR = GPIO_PIN_6;
	}
	else
	{
		GPIOB->BSRR = (uint32_t)GPIO_PIN_6 << 16u;
	}
}

/* END: SPI Hardware dependencies */



inline void EnableCoreSysClock()
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

inline uint32_t GetSysCoreClockCount()
{
	return DWT->CYCCNT;
}

/* OK
// from arduino-ststm32 :
static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

uint32_t getCurrentMicros(void)
{
  // Ensure COUNTFLAG is reset by reading SysTick control and status register
  LL_SYSTICK_IsActiveCounterFlag();

  uint32_t m = HAL_GetTick();

  const uint32_t tms = SysTick->LOAD + 1;

  __IO uint32_t u = tms - SysTick->VAL;

  if (LL_SYSTICK_IsActiveCounterFlag())
  {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }

  return (m * 1000 + (u * 1000) / tms);
}
*/


#if 0
void CAN_SetBaudrate(uint16_t baudrate)
{
	// With APB2 = 72MHz

	(void)HAL_CAN_Stop(&hcan);

	(void)HAL_CAN_DeInit(&hcan);


	switch(baudrate)
	{
		case 10:
		{
			hcan.Init.Prescaler = 225;
			hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;

		case 20:
		{
			hcan.Init.Prescaler = 100;
			hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;

		case 50:
		{
			hcan.Init.Prescaler = 45;
			hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;

		case 100:
		{
			hcan.Init.Prescaler = 20;
			hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;

		case 125:
		{
			hcan.Init.Prescaler = 18;
			hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;

		case 250:
		{
			hcan.Init.Prescaler = 9;
			hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;

		default:
		case 500:
		{
			hcan.Init.Prescaler = 4;
			hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;

		case 800:
		{
			hcan.Init.Prescaler = 3;
			hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;

		case 1000:
		{
			hcan.Init.Prescaler = 2;
			hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
			hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
		}break;
	}


	hcan.Instance = CAN1;
	//hcan.Init.Prescaler = 4;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	//hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
	//hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}

	(void)HAL_CAN_Start(&hcan);
}
#endif

/*EOF*/
