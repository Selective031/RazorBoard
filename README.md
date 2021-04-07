# RazorBoard 1.0 rev. E

Welcome to Razorboard! This is the first version publicaly available.

# Brief description:

RazorBoard is a PCB with most hardware integrated for building a DIY Robotic Lawn Mower.

# Hardware:

The PCB consist of an STM32F4 ARM processor running at 168 MHz with 1 MB flash and 192 KB RAM + 4 KB SRAM and with the option to snap on a Raspberry Pi 4B.
It also integrates 3 motor drivers (Two wheel drivers and one cutter driver)
Every driver is equipt with a current sensor.
Input voltage is from ~12 to 25.2, do NOT go above this.
For boundary, the hardware is ready for up to 4 sensors. 2 will be default, one left and one right.

A signal generator board is also available to be used together with Razorboard.

# THERE IS NO PROTECTION FOR REVERSED POLARITY - DOUBLE CHECK, TRIPPLE CHECK BEFORE POWERING ON!!!

Various interfaces available:

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
- 1 IWDG (Indepandent Watchdog)

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

# GETTING STARTED:

Explaination of connectors:

- M1 = Left Wheel Motor
- M2 = Right Wheel Motor
- C1 = Cutter Motor

- Power Input = Connection to your battery, ~12 to 25.2 Volt
- Charge Connector = Connect this to your charge pins at the front of your mower
- Battery Charge = Connect this your battery charge cable, if your battery does not have a dedicated charge cable, you can connect it to the battery.
- BWF1 = Connect to your Left front sensor  (default)
- BWF2 = Connect to you Right front sensor  (default)
- BWF3 = Currently not in use in software
- BWF4 = Currently not in use in software
- STM32 = Used for upgrading firmware in STM32, is also used for debugging and interfacing with the STM32
- I2C_2 SDA = Connect this to your MPU-6050
- I2C_2 SCL = Connect this to your MPU-6050

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

# SAFETY:

Safety is my top priority.
The mower will only mow within your boundary wire.
When powering up the mower, it will collect INSIDE messages, when enough INSIDE messages are reached it will start. If, at anytime during initial startup sequens an OUTSIDE message is received, it will reset the initial sequens.
Razorboard will keep track of time between INSIDE messages, if the messages are not received, the mower will stop, this threshold is user customized.
It will also count the time for when it is OUTSIDE, a limit here is also applied, if it is OUTSIDE for too long, it will stop.
With the help of the 6050 IMU, it will also sense when it is tilted, default is 35 degrees, when reaching 35 or more, it will do a HARDBREAK on the cutterdisk, it will stop within 2 seconds.
Razorboard can also sense when it is hitting an object, by sensing spikes in motor current, when doing so it will stop and go backward and turn in another direction.
If the current is too high on the cutting disk, it will HARDBREAK and STOP, until reset by the user.

Inside the STM32 there is something called IWDG, independant watchdog. With the help of this watchdog, if any halt/crash occurs on the system due to a bug or user programming error, the whole STM32 will reset after 2 seconds, and after reset, the whole initial startup sequens is starting over again. So there are no worries that the mower will run forever (unless you have programmed it to do so, of course).

# SLOPE MANAGEMENT:

With the help of the 6050 IMU, the Razorboard will try to compensate for slopes, so it can maintain a straight line.

# BEHAVIOUR:

When Razorboard senses that both boundary sensors are outside, it will go backward and then randomly select left or right and also randomly for how much.
If only one sensor is outside, it will go backward and then turn in the opposite direction, for randomly amount of time.
When Razorboard is sensing a crash against an object, the motors will go into hardbreak for 0.5 seconds, then go backward, and randomly select left,right.

# GOING HOME:

When Razorboard is low on battery, a perimeter tracking sequens is initiated. The cutting disk will turn off, and it will go and find the boundary wire.
Also, when crossing the boundary and the time is outside working hours, it will initiate perimeter tracking sequens, for example: you have configured working hours to be between 10:00 and 20:00, and when Razorboard crosses the boundary the time is 20:00 or later, it will go home.
Once the boundary wire is located it will follow it to the left (default), until the charging station is found.
Once the charging station is found, it will charge the battery. If the battery is fully charged and the time is within the working hours it will undock and start mowing again. This will continue to happen until it is outside the working hours, once outside working hours, it will rest until inside again.

# MOTORS:

Razorboard will ramp the motors up and down, to preserve the cogs in the motors, also it looks much nicer. Two exceptions exist, when you tilt/overturn the mower it will hardbreak the motors. The second is when it is crashing into an object.

For each startup of cutting disk, Razorbord will randomly select clockwise or anti-clockwise direction. Utilizing the pivot knifes on both sides.

# RTC CLOCK:

On the PCB you can find one connection for a RTC battery, and also a jumper. This is VERY important. Never ever connect a battery when the jumper is attached.
When the jumper is attached the RTC clock gets power from the main 3.3V power rail.
Please remove the jumper BEFORE attaching the RTC battery. RTC battery can be MAX 3.6V, and minimum 1.8V. Nominal volt should be around 3.3V.
When a battery is connected, you need to set the time and date. To do this, connect a USB cable to the STM32 interface (Upper left corner) and use the commands:
"set time hour minute second" - for example "set time 14 10 0"
To set date "set date year month day weekday" - for example "set date 21 4 3 6" (2021 04 03 6) Monday is 1, Sunday is 7.
In a future firmware version, a calendar function will be available, so you can set a specific day to mow and what your working hours will be.

# SRAM:

- In a future firmware version -
The STM32F4 does not contain any EEPROM, but instead it has 4KB of SRAM, which is retained during power off if you have an RTC battery conncted.
The RTC battery will last for many years, so as long as you have the battery conncted all settings will be retained even when power is off.
Also, there will be an export function, so you can backup your settings if needed.

# NOISE:

Noise is something we need deal with, and also have to live with. In particular with DC motors driven by PWM. PWM as you might know is a technique to rapidly turn a signal on and off very quickly. Usually you want to have a PWM freq above the 20KHz, because that is what we humans cannot hear, going below and you might hear some whisling noise.
However, PWM will also put some stress on the hardware, creating high frequency noise which can interfere with other cricuits.
The boundary wire sensors are VERY sensitive to noise. The signal they pickup is amplified 556 times. That also means the noise is amplified 556 times. To mitigate some of it I have implemented a FIR filter (finite impulse response), this filter can filter out noise in a very specific way, however there are some limitations of course. It is very CPU intensive. Luckily the STM32F4 MCU have dedicated hardware (DSP - Digital Signal Processor), so I have offloaded it to the hardware instead. This means the performace is so much greater than if it was done in software. Still there is another limitation, and following the Nyquist sampling theorem, it dictates that you need to sample at least twice the frequency you want to filter out. So, if our motors are running at 25KHz, we need to sample it at least at 50KHz. Lucky for us that is no problem for a STM32 MCU. Still, a filter is a filter, and there is no perfect filter. But this can help us alot. The weaker the signal gets, the harder it is to filter and read correctly.
Because the Razorboard is a DIY board, people will use different motors and every setup will be unique. So if you get a bad sensor reading you might want to play with the PWM frequency and sampling rate. But some general tips, use as short motor leads as possible, twist the leads as much as you can. You can also use a ferrite ring and go around it a couple of times. Some people say that you can solder a ceramic capacitor between the motor leads, however be careful with this, as that will interfere with the PWM signal, if you want to test, use something in the range of 10nF.
Keep the boundary sensors away as far as possible from the motors. If possible use shielded cables to the sensors.
If you still get bad readings, you might need to raise the volt/current on the boundary wire.

To test your readings, take the mower and a laptop, go to the center of your lawn, (or the worst part of the lawn), place the mower on something so the wheels doesn´t touch the ground, connect to Razorboard with a USB cable. Power on Razorboard, and open a serial console and press ENTER (or type DISABLE), this will disable the system.
Now, type "DEBUG ON", and hopfully you will see the boundary signals. And now you can test to run the motors and see how that will affect the readings.

# TROUBLESHOOTING:

Connect a USB cable to the STM32 connector (Upper left corner)
Set the COM port to 115200 baud
Press ENTER, this will disable the RazorBoard (for safety)
Type "help" for a menu
  
Before releasing the Razorboard in the wild, you can check the motors if they spin in the right direction.
In the debug menu you can test the motors on the "bench". If they are spinning in the wrong direction, swap the motor leads on the connector.
In a future firmware this can be done in software.
With the command "DEBUG ON" you can verify that the boundary signals are received correctly. Also how many you receive per second. This can be a great tool to check the signal by placing the mower in the center of you lawn and check if you still receive the messages.

