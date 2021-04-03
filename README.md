# RazorBoard 1.0

Brief description:

RazorBoard is a PCB with most hardware integrated for building a DIY Robotic Lawn Mower.

Hardware:

The PCB consist of an STM32F4 ARM processor running at 168 MHz with 1 MB flash and 192 KB RAM + 4 KB SRAM and with the option to snap on a Raspberry Pi 4B.
It also integrates 3 motor drivers (Two wheel drivers and one cutter driver)
Every driver is equipt with a current sensor.
Input voltage is from ~12 to 25.2, do NOT go above this.
For boundary, the hardware is ready for up to four sensors. 2 will be default, one left and one right.
Various interfaces are available:

STM32:
- 2 UART
- 1 HIGH SPEED UART CONNECTED TO RPI4
- 1 SPI
- 1 I2C
- 20+ Digital Pins
- 8 Analog Pins
- 1 DAC
- 1 CAN
- 1 ST-LINK
- 1 RTC Clock with calendar
- 1 RTC BAT connection
- 4 Boundary Sensors connections (2 is default)
- 4 5V connections
- 5 3.3V connections

RPI4:
- 4 UART
- 1 HIGH SPEED UART CONNECTED TO STM32
- 1 SPI
- 1 I2C
- 1 DHT11/DHT22 interface
- 1 FAN (30 X 30 mm)
- 1 FAN power connector (5V or 3.3V)
- 4 Additional GPIO pins (all outbreak connections are also available as GPIO)
- All RPI connections are available (USB, HDMI etc....)

Getting Started:

Software needed to upload .bin file: "Flash Loader Demonstator" from ST Micro

1. First change the jumper from RUN to UPGRADE
2. Power the board
3. Start "Flash Loader Demonstrator"
4. Click NEXT 3 times.
5. Select "Download to Device"
6. Locate the .bin file.
7. Click Next and your file will now be uploaded.
8. Remove Power from board.
9. Change the jumper back to RUN.
10. Power the board.


