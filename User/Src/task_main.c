/*
 * mainTask.c
 *
 *  Created on: May 6, 2024
 *      Author: Phuc VU
 *
 *
 */



/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "cmsis_os.h"
//#include "semphr.h"




/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

static uint8_t _usb_txbuf[64] = {0};
static uint8_t _usb_rxbuf[64] = {0};

/* Private function prototypes -----------------------------------------------*/


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

/* Through SPI because USB and CAN incompatible (know problem) */
void Task_can(void *argument)
{
	/* SPI2
	 * PB15 MOSI
	 * PB14 MISO
	 * PB13 SCK
	 * PB12 NSS
	 *
	 * baud 9 Mbits
	 * */
	while(1)
	{
		osDelay(1);


	}
}

/* SPI Hardware dependencies */

inline void SPI_Transmit(uint8_t* data, uint16_t len)
{
	//HAL_SPI_Transmit(hspi, pData, Size, Timeout); // Timeout millisecond
	HAL_SPI_Transmit(&hspi2, data, len, 20);
}

inline void SPI_Receive(uint8_t* data, uint16_t len)
{
	HAL_SPI_Receive(&hspi2, data, len, 20);
}


/*EOF*/
