# Bluepill
Model for future project:
- FreeRTOS 10.0.1
- USB CDC
- SSD1306 OLED through SPI1
- MCP2515 CAN through SPI2

IDE Tools version:
  - STM32CubeIDE v1.13.2

# LED_BUILTIN pin configuration
- PC13 ouput

# SPI1 pin configuration : OLED 96" SSD1306
- SCK  -> PA5
- MOSI -> PA7
- CS   -> PB8
- DC   -> PB7
- RES  -> PB6
- bits order : MSBFIST
- bits length: 8
- CPOL = 0 (idle low)
- CPHA = 0 (1 edge)

# SPI2 pin configuration : MCP2515 CAN module 8 Mhz
- SCK  -> PB13 -> 9M bauds
- MOSI -> PB15
- MISO -> PB14
- CS   -> PB12
- CPOL = 0
- CPHA = 0
- bit order  : MSBFIRST
- bits length: 8
- External interrupt -> PA8 (falling edge)

# NOTE: CAN Bus and USB can't work together (know problem on this MCU)