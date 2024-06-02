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
#include "semphr.h"

#include <stdbool.h>
#include <user_def.h>

#include "../mcp2515/can.h"
#include "../mcp2515/mcp2515.h"


/* Private typedef -----------------------------------------------------------*/

typedef struct __attribute__((packed))
{
	uint32_t id;
	uint8_t dlc;
	uint8_t data[8];
}CAN2USB_Msg_TypeDef;


/* Private define ------------------------------------------------------------*/
#define DEF_TIMEOUT_QUEUE_SEND       (5 * portTICK_PERIOD_MS)
#define DEF_TIMEOUT_QUEUE_RECEIVE    (1 * portTICK_PERIOD_MS)
#define DEF_TIMEOUT_SEMAPHORE        (5 * portTICK_PERIOD_MS)

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

static uint8_t _usb_rxbuf[64] = {0};

SemaphoreHandle_t xSemaphore = NULL;
StaticSemaphore_t xSemaphoreBuffer;

extern osMessageQId host2deviceHandle;

/* Private function prototypes -----------------------------------------------*/
static inline void SPI_Delay(uint32_t millis);
static inline void GPIO_WritePinCS(GPIO_PinState PinState);
static inline void SPI_Transmit(uint8_t data);
static inline uint8_t SPI_Receive(void);


/* Global function prototypes -----------------------------------------------*/
void onUsbReceive(const uint8_t* const pBuf, const uint32_t* const pLen);



/* Code ---------------------------------------------------------------------*/

void onUsbReceive(const uint8_t* const pBuf, const uint32_t* const pLen)
{
	static TickType_t last_time = 0;
	static int index = 0;
	static USB_Host2Device_TypeDef msg_from_usb;
	static uint16_t less_counter = 0;



	TickType_t time_elapse = xTaskGetTickCount() - last_time;
	time_elapse *= portTICK_PERIOD_MS;

	/* if > 20 ms -> reset index */
	if (time_elapse > 20)
	{
		index = 0;

		/* Save current time */
		last_time = xTaskGetTickCount();
	}




	// debug
	if (*pLen < sizeof(USB_Host2Device_TypeDef))
	{
		less_counter++;
	}


	for(int i=0; i< *pLen; i++)
	{
		_usb_rxbuf[index++] = pBuf[i];
	}

	if (index >= sizeof(USB_Host2Device_TypeDef))
	{
		memcpy( &msg_from_usb, &_usb_rxbuf[0], sizeof(USB_Host2Device_TypeDef));

		index = 0;

		/* put message in queue */
		xQueueSend(host2deviceHandle, &msg_from_usb, DEF_TIMEOUT_QUEUE_SEND);
	}

//	if ( (_usb_rxbuf[0] == 0x5A) && (_usb_rxbuf[1] == 0x63))
//	{
//		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
//	}
}



void Task_main(void *argument)
{
	static TickType_t curr_time = 0;

	xSemaphore = xSemaphoreCreateBinaryStatic( &xSemaphoreBuffer );
	configASSERT( xSemaphore );

	static USB_Host2Device_TypeDef usb_msg;

	static CAN2USB_Msg_TypeDef USB_buf = {0};

	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();


	struct can_frame canMsg;

	StructSpi mcpSPI;
	mcpSPI.Delay = &SPI_Delay;
	mcpSPI.Read  = &SPI_Receive;
	mcpSPI.Write = &SPI_Transmit;
	mcpSPI.CS    = &GPIO_WritePinCS;

	if (MCP2515_SPI_initialize(&mcpSPI) == false) { Error_Handler(); }

	MCP2515_CAN_initialize();
	MCP2515_SetBitrate(CAN_500KBPS);
	MCP2515_SetNormalMode();



	while(1)
	{
		//osDelay(500);
		//HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);


		if( xSemaphoreTake( xSemaphore, ( TickType_t ) DEF_TIMEOUT_SEMAPHORE ) == pdTRUE )
		{
			/*/INT triggered -> CAN message received */

			if (MCP2515_ReadMessage(&canMsg) == ERROR_OK)
			{
				// id can : 0x80 01 03 21 -> bit.31 = 1

				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

				USB_buf.id = canMsg.can_id;
				USB_buf.dlc = (canMsg.can_dlc > 8) ? 8 : canMsg.can_dlc;

				USB_buf.data[0] = canMsg.data[0];
				USB_buf.data[1] = canMsg.data[1];
				USB_buf.data[2] = canMsg.data[2];
				USB_buf.data[3] = canMsg.data[3];
				USB_buf.data[4] = canMsg.data[4];
				USB_buf.data[5] = canMsg.data[5];
				USB_buf.data[6] = canMsg.data[6];
				USB_buf.data[7] = canMsg.data[7];

				CDC_Transmit_FS((uint8_t*)&USB_buf, sizeof(USB_buf));
			}
		}

		// TODO: manage incoming queue message
		TickType_t time_elapse = xTaskGetTickCount() - curr_time;

		//if (xQueueReceive(host2deviceHandle, &usb_msg, ( TickType_t ) DEF_TIMEOUT_QUEUE_RECEIVE) == pdTRUE)
		if (time_elapse > 2000)
		{
			curr_time = xTaskGetTickCount();

			canMsg.can_id = 0x1f000004 | CAN_EFF_FLAG;
			canMsg.can_dlc = 3;
			canMsg.data[0] = 0x01;
			canMsg.data[1] = 0x02;
			canMsg.data[2] = 0x03;
//			canMsg.can_id = (usb_msg.data[3] << 24) |
//					        (usb_msg.data[2] << 16) |
//					        (usb_msg.data[1] << 8)  |
//					        (usb_msg.data[0]);
//
//			canMsg.can_dlc = usb_msg.data[5];
//
//			for(int i=0; i < 8; i++)
//			{
//				canMsg.data[i] = usb_msg.data[6+i];
//			}

			MCP2515_WriteMessage(&canMsg);
		}
	}

}



/* External interrupt: INT CAN module */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// check trigger
	if (GPIO_Pin != MCP2515_INT_Pin) {return; }

	//HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

	/* Is it time for vATask() to run? */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Unblock the task by releasing the semaphore. */
	xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );

	/* Yield if xHigherPriorityTaskWoken is true. */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}




/** @addtogroup SPI2_Hardware_dependencies
  * @{
  */

/*! \brief Delay function */
static inline void SPI_Delay(uint32_t millis)
{
	osDelay(millis);
}

/* SPI CS -> PB12 */
static inline void GPIO_WritePinCS(GPIO_PinState PinState)
{
	if (PinState != GPIO_PIN_RESET)
	{
		GPIOB->BSRR = GPIO_PIN_12;
	}
	else
	{
		GPIOB->BSRR = (uint32_t)GPIO_PIN_12 << 16u;
	}
}

static inline void SPI_Transmit(uint8_t data)
{
	HAL_SPI_Transmit(&hspi2, &data, 1, 20);
}

static inline uint8_t SPI_Receive(void)
{
	uint8_t rxbuf = 0;

	HAL_SPI_Receive(&hspi2, &rxbuf, 1, 20);

	return rxbuf;
}


/**
  * @}
  */

/*EOF*/
