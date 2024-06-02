/*
 * user_task.h
 *
 *  Created on: Jun 2, 2024
 *      Author: admin
 */

#ifndef INC_USER_DEF_H_
#define INC_USER_DEF_H_

#include <stdint.h>

typedef struct __attribute__((packed))
{
	uint8_t to;
	uint8_t dlc;
	uint8_t reserved;
	uint8_t data[16];
}USB_Host2Device_TypeDef;


#endif /* INC_USER_DEF_H_ */
