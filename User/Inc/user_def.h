/*
 * user_task.h
 *
 *  Created on: Jun 2, 2024
 *      Author: admin
 */

#ifndef INC_USER_DEF_H_
#define INC_USER_DEF_H_

#include <stdint.h>

/* size = 16 bytes */
typedef struct __attribute__((packed))
{
	uint8_t toDevice;
	uint8_t dlc;
	uint8_t reserved[2];
	uint32_t id;
	uint8_t data[8];
}Message_FromHost_TypeDef;


#endif /* INC_USER_DEF_H_ */
