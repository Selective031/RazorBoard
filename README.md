# RazorBoard 1.0 rev. E

Welcome to Razorboard! This is the first version publicaly available.

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
- 1 RNG (True Random Number generator) in hardware
- 1 DSP (Digital Signal Processing) in hardware

RPI4:
- 4 UART
- 1 HIGH SPEED UART CONNECTED TO STM32
- 1 SPI
- 1 I2C
- 1 DHT11/DHT22 interface
- 1 FAN (30 X 30 mm)
- 2 FAN power connector (5V or 3.3V)
- 4 Additional GPIO pins (all outbreak connections are also available as GPIO)
- All RPI connections are available (USB, HDMI etc....)

GETTING STARTED:

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

SAFETY:

Safety is my top priority.
The mower will only mow within your boundary wire.
When powering up the mower, it will collect INSIDE messages, when enough INSIDE messages are reached it will start. If, at anytime during initial startup sequens an OUTSIDE message is received, it will reset the initial sequens.
Razorboard will keep track of time between INSIDE messages, if the messages are not received, the mower will stop, this threshold is user customized.
It will also count the time for when it is OUTSIDE, a limit here is also applied, if it is OUTSIDE for too long, it will stop.
With the help of the 6050 IMU, it will also sense when it is tilted, default is 35 degrees, when reaching 35 or more, it will do a HARDBREAK on the cutterdisk, it will stop within 2 seconds.
Razorboard can also sense when it is hitting an object, when doing so it will stop and go backward and turn in another direction.
If the current is too high on the cutting disk, it will HARDBREAK and STOP, until reset by the user.

SLOPE MANAGEMENT:

With the help of the 6050 IMU, the Razorboard will try to compensate for slopes, so it can maintain a straight line.

BEHAVIOUR:

When Razorboard senses that both boundary sensors are outside, it will go backward and then randomly select left or right and also randomly for how much.
If only one sensor is outside, it will go backward and then turn in the opposite direction, for randomly amount of time.

GOING HOME:

When Razorboard is low on battery, a perimeter tracking sequens is initiated. The cutting disk will turn off, and it will go and find the boundary wire.
Once the boundary wire is located it will follow it to the left (default), until the charging station is found.
Once the charging station is found, it will charge the battery. If the battery is fully charged and the time is within the working hours, it will undock and start mowing again. This will continue to happen until it is outside the working hours, once in outside working hours, it will rest until inside again.

MOTORS:

Razorboard will ramp the motors up and down, to preserve the cogs in the motors, also it look much nicer. Two exceptions exist, when you tilt/overturn the mower it will hardbreak the motors. The second is when it is hitting an object.

TROUBLESHOOTING:

Connect a USB cable to the STM32 connector (Upper left corner)
Set the COM port to 115200
Press ENTER, this will disable the RazorBoard
Type <help> for a menu
