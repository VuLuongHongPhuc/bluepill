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
}CAN_Msg_TypeDef;


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
	static Message_FromHost_TypeDef msg_from_usb;
	static uint16_t less_counter = 0;

	/* Is it time for vATask() to run? */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;


	TickType_t time_elapse = xTaskGetTickCount() - last_time;
	time_elapse *= portTICK_PERIOD_MS;

	/* if > 20 ms -> timeout continuity */
	if (time_elapse > 20)
	{
		index = 0;

		/* Save current time */
		last_time = xTaskGetTickCount();
	}


	// debug : check if we receive less than sizeof(Message_FromHost_TypeDef)
	if (*pLen < sizeof(Message_FromHost_TypeDef))
	{
		less_counter++;
	}


	for(int i=0; i< *pLen; i++)
	{
		_usb_rxbuf[index++] = pBuf[i];
	}

	if (index >= sizeof(Message_FromHost_TypeDef))
	{
		memcpy( &msg_from_usb, &_usb_rxbuf[0], sizeof(Message_FromHost_TypeDef));

		index -= sizeof(Message_FromHost_TypeDef);

		/* put message in queue */
		xQueueSendFromISR(host2deviceHandle, &msg_from_usb, &xHigherPriorityTaskWoken);
	}

	/* Yield if xHigherPriorityTaskWoken is true. */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}



void Task_main(void *argument)
{
	xSemaphore = xSemaphoreCreateBinaryStatic( &xSemaphoreBuffer );
	configASSERT( xSemaphore );

	static Message_FromHost_TypeDef usb_msg;
	static CAN_Msg_TypeDef msg_to_host = {0};

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
			/* /INT triggered -> CAN message received ready */

			if (MCP2515_ReadMessage(&canMsg) == ERROR_OK)
			{
				// id can |= (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)

				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

				msg_to_host.id = canMsg.can_id;

				msg_to_host.dlc = (canMsg.can_dlc > 8) ? 8 : canMsg.can_dlc;

				msg_to_host.data[0] = canMsg.data[0];
				msg_to_host.data[1] = canMsg.data[1];
				msg_to_host.data[2] = canMsg.data[2];
				msg_to_host.data[3] = canMsg.data[3];
				msg_to_host.data[4] = canMsg.data[4];
				msg_to_host.data[5] = canMsg.data[5];
				msg_to_host.data[6] = canMsg.data[6];
				msg_to_host.data[7] = canMsg.data[7];

				CDC_Transmit_FS((uint8_t*)&msg_to_host, sizeof(msg_to_host));
			}
		}


		if (xQueueReceive(host2deviceHandle, &usb_msg, ( TickType_t ) DEF_TIMEOUT_QUEUE_RECEIVE) == pdTRUE)
		{
			// TODO: get status for extended
			/* extended id -> add CAN_EFF_FLAG */
			canMsg.can_id = usb_msg.id | CAN_EFF_FLAG;

			canMsg.can_dlc = usb_msg.dlc;

			for(int i=0; i < 8; i++)
			{
				canMsg.data[i] = usb_msg.data[i];
			}

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
	uint8_t c = 0;

	HAL_SPI_Receive(&hspi2, &c, 1, 20);

	return c;
}


/**
  * @}
  */

/*EOF*/
