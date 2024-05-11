/*
 * ssd1306_test.h
 *
 *  Created on: May 9, 2024
 *      Author: admin
 */

#ifndef SSD1306_SSD1306_TEST_H_
#define SSD1306_SSD1306_TEST_H_


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

	void ssd1306_initialize_test(void* pSpi);
	void ssd1306_testsequence(uint8_t c);

	void ssd1306_testdrawline(void);
    void ssd1306_testdrawrect(void);
    void ssd1306_testfillrect(void);
    void ssd1306_testdrawcircle(void);
    void ssd1306_testdrawroundrect(void);
    void ssd1306_testdrawtriangle(void);
    void ssd1306_testfilltriangle(void);
    void ssd1306_testdrawchar(void);




/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* SSD1306_SSD1306_TEST_H_ */
