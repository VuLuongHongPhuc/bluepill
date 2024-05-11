/*
 * button.h
 *
 *  Created on: May 10, 2024
 *      Author: admin
 */

#ifndef SSD1306_BUTTON_H_
#define SSD1306_BUTTON_H_

#ifdef	__cplusplus
extern "C" {
#endif

	//private void SetButtonCornerRadius(uint8_t radius);
	void SetButtonWidth(int16_t width);
	void DrawButton(int16_t x, int16_t y, const char* text, uint8_t textSize, bool selected);

#ifdef	__cplusplus
}
#endif

#endif /* SSD1306_BUTTON_H_ */
