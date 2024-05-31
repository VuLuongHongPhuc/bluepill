/*
 * mcp2515.c
 *
 *  Created on: May 29, 2024
 *      Author: admin
 */

#include <stddef.h>
#include <stdint.h>
#include "mcp2515.h"







static StructSpi* hSPI = NULL;

void MCP2515_init(StructSpi* pSPI)
{
	if (pSPI == NULL) { return; }

	hSPI = pSPI;

}


/*EOF*/
