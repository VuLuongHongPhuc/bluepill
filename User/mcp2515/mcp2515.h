/*
 * mcp2515.h
 *
 *  Created on: May 29, 2024
 *      Author: admin
 */

#ifndef MCP2515_MCP2515_H_
#define MCP2515_MCP2515_H_











/* NOTES: To provide these function */
//typedef void (*functWritePin)(uint8_t);
typedef void (*functWrite)(uint8_t* data, uint16_t len);
typedef void (*functRead)(uint8_t* data, uint16_t len);
//typedef void (*functDelay)(uint32_t);

typedef struct{
	functWrite Write;
	functRead  Read;
}StructSpi;


void MCP2515_init(StructSpi* pSPI);



#endif /* MCP2515_MCP2515_H_ */
