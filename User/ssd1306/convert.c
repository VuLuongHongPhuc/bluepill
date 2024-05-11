/*
 * convert.c
 *
 *  Created on: May 11, 2024
 *
 */


#include <stdint.h>
#include "convert.h"


void ConvertIntToChar(int32_t value, char* str, uint8_t len)
{
	//TODO: signed

	int32_t temp;

	str[len+1] = 0;

	for(int i=0; i<len; i++)
	{
		if (value != 0)
		{
			temp = value%10;
			str[len-(i+1)] = temp + 0x30;
			value = (int32_t)(value/10);
		}
		else
		{
			str[len-(i+1)] = 0x30;
		}
	}
}

/*EOF*/
