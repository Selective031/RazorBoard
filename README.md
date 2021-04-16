# RazorBoard 1.0 rev. E

Welcome to Razorboard! This is the first GA version of RazorBoard.

# Brief description:

RazorBoard is a PCB with most hardware integrated for building a DIY Robotic Lawn Mower.
With this revision, you need to get a MPU-6050 and a RTC battery. In a future revision, this will be included on the PCB.

# Hardware:

- The PCB consist of an STM32F415VGT6 ARM processor which is a 100 pin MCU, running at 168 MHz with 1 MB flash and 192 KB RAM + 4 KB SRAM and with the option to snap on a Raspberry Pi 4B.
- It also integrates 3 motor drivers (drv8871) for brushed DC motors (Two wheel drivers and one cutter driver)
- Every driver is equipt with a current sensor which are rated for up to 20A.
- Each driver can support 2.1A continues and peak at 3.6A
- Three different voltage levels, the input voltage (12-24), 5V rail and a 3.3V rail.
- One 16 MHz crystal oscillator (main oscillator)
- One 32.768 KHz crystal oscillator (for the RTC clock)
- Two fuses, default at the moment is 8A for mowing, and 4A for charging.
- Two relays for power and charging in various modes.
- Input voltage is from ~12 to 25.2, do NOT go above this.
- For boundary, the hardware is ready for up to 4 sensors. 2 will be default, one left and one right.
- Board size: 180 x 100mm

The datasheet for STM32F415VGT6 is located in the repository.

A signal generator board is also available to be used together with Razorboard. Although it is not required if you already have a generator, but you will need to sync the signal.
Board size: 95 x 95mm

# Required add-ons

- MPU-6050
- Power Resistor for each loop
- RTC battery

# To successfully build a robot, you need:

- 1 Razorboard
- 2 Boundary Sensors
- 1 Boundary wire signal generator
- 2 motors for the wheels
- 2 wheels
- 1 motor for the cutter disk
- 1 cutter disk
- 3 or 4 pivo knifes
- 1 battery
- 1 battery charger
- 1 chassi
- 1 or 2 front wheels
- 2 charging pins
- Cables/connectors
- Patience 
- 100 beers or 50 Whiskeys (or was it 100 Whiskeys and 50 beers.. don´t remember...)
- Soldering Iron
- a wife/husband/girlfriend/boyfriend who understands the importance of making your own robot
- Curiosity
- Some technical/mechanical skills
- a "DIY Robot Lawn Mower" group on facebook :)

# THERE IS NO PROTECTION FOR REVERSED POLARITY - DOUBLE CHECK, TRIPPLE CHECK BEFORE POWERING ON!!!

Various interfaces available:

STM32:
- 2 UART
- 1 High speed UART internally connected to the RPi
- 1 SPI
- 1 I2C
- 20+ Digital Pins
- 8 Analog Pins
- 1 DAC
- 1 CAN
- 1 ST-LINK
- 1 RTC Clock with calendar
- 1 RTC BAT connection
- 4 Boundary Sensor connections (2 is used by default, BWF1 & BWF2)
- 4 5V connections
- 5 3.3V connections
- 1 RNG (True Random Number generator) in hardware
- 1 DSP (Digital Signal Processing) in hardware
- 1 IWDG (Indepandent Watchdog)

RPI4:
- 4 UART
- 1 High speed UART internally connected to STM32
- 1 SPI
- 1 I2C
- 1 DHT11/DHT22 interface
- 1 FAN (30 X 30 mm)
- 2 FAN power connectors (5V or 3.3V)
- 4 Additional GPIO pins (all outbreak connections are also available as GPIO)
- All RPI connections are available (USB, HDMI etc....)

# GETTING STARTED:

Explaination of connectors:

- M1 = Left Wheel Motor
- M2 = Right Wheel Motor
- C1 = Cutter Motor

- Power Input = Connection to your battery, ~12 to 25.2 Volt
- Charge Pins = Connect this to your charge pins at the front of your mower
- Battery Charge = Connect this your battery charge cable, if your battery does not have a dedicated charge cable, you can connect it to the battery.
- BWF1 = Connect to your Left front sensor  (default)
- BWF2 = Connect to you Right front sensor  (default)
- BWF3 = Currently not in use in software
- BWF4 = Currently not in use in software
- USB STM32 = Used for upgrading firmware in STM32, also used for debugging and interfacing with the STM32
- I2C_2 SDA = Connect this to your MPU-6050 (located to the left of the motors connections)
- I2C_2 SCL = Connect this to your MPU-6050 (located to the left of the motors connections)

Software needed to upload .bin file: "Flash Loader Demonstrator" from ST Micro
https://www.st.com/en/development-tools/flasher-stm32.html#overview

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

After this inital upload, you can use the ST-Link interface for upgrading without changing jumper.
Also, in the debug menu you can find the "upgrade" command, this will force the board into bootloader mode, and no need to change jumper.

Software needed to compile firmware: CubeIDE
https://www.st.com/en/development-tools/stm32cubeide.html

# Safety:

Safety is my top priority.
The mower will only mow within your boundary wire.
When powering up the mower, it will collect INSIDE messages, when enough INSIDE messages are reached it will start. If, at anytime during initial startup sequens an OUTSIDE message is received, it will reset the initial sequens.
Razorboard will keep track of time between INSIDE messages, if the messages are not received, the mower will stop, this threshold is user customized.
It will also count the time for when it is OUTSIDE, a limit here is also applied, if it is OUTSIDE for too long, it will stop.
With the help of the 6050 IMU, it will also sense when it is tilted, default is 35 degrees, when reaching 35 or more, it will do a HARDBREAK on the cutterdisk, it will stop within 2 seconds.
If the boundary wire signal is lost, the Razorboard will stop within 6 seconds (default at the moment, this can be changed to anything you like).
Razorboard can also sense when it is hitting an object, by sensing spikes in motor current, when doing so it will stop and go backward and turn in another direction.
If the current is too high on the cutting disk, it will HARDBREAK and STOP, until reset by the user.

Inside the STM32 there is something called IWDG, independant watchdog. With the help of this watchdog, if any halt/crash occurs on the system due to a bug or user programming error, the whole STM32 will reset after 2 seconds, and after reset, the whole initial startup sequens is starting over again. So there are no worries that the mower will run forever (unless you have programmed it to do so, of course).

# Slope Management:

With the help of the 6050 IMU, the Razorboard will try to compensate for slopes, so it can maintain a straight line.
At the moment the power to the motors are fixed values based on the number of degrees the the mower is leaning.
TODO - A proper PID controller utilizing the 6050 MPU.

# Behaviour:

When Razorboard senses that both boundary sensors are outside, it will go backward and then randomly select left or right and also randomly for how much.
If only one sensor is outside, it will go backward and then turn in the opposite direction, for randomly amount of time.
When Razorboard is sensing a crash against an object, the motors will go into hardbreak for 0.5 seconds, then go backward, and randomly select left,right.

# Going Home:

When Razorboard is low on battery, a perimeter tracking sequens is initiated. The cutting disk will turn off, and it will go and find the boundary wire.
Also, when crossing the boundary and the time is outside working hours, it will initiate perimeter tracking sequens, for example: you have configured working hours to be between 10:00 and 20:00, and when Razorboard crosses the boundary when the time is 20:00 or later, it will go home.
Once the boundary wire is located it will follow it to the left (default), until the charging station is found.
Once the charging station is found, it will charge the battery. If the battery is fully charged and the time is within the working hours it will undock and start mowing again. This will continue to happen until it is outside the working hours, once outside working hours, it will rest until inside again.
TODO - Obstacle avoidance along the boundary wire.

# Motors:

Razorboard will ramp the motors up and down, to preserve the cogs in the motors, also it looks much nicer. Two exceptions exist, when you tilt/overturn the mower it will hardbreak the motors. The second is when it is crashing into an object.

For each startup of cutting disk, Razorbord will randomly select clockwise or anti-clockwise direction. Utilizing the pivot knifes on both sides.

# Battery:

Razorboard can handle any type of battery as it does not include any charge circuits, it´s using two relays to on/off the power to the batteries. Therefore, if you use a lithium battery you need to have a BMS (Battery Management System), many lithium batteries already have this inside their batterypacks. Razorbaord can monitor the voltage on the battery and simply disconnect the charging when a limit has been reached.
If you battery has a dedicated charge cable, use this to the "Charge Battery" connector. If it does not have one, you simply connect the "Charge Battery" connector back to the battery. The relays are designed so that you cannot draw power from the main battery and output power to the "Charge Battery" connector at the same time, it has to draw power from the "Charge Pins". The logic is as follows:

- When no relay is active, power comes from the "Main Battery" connector, both "Charge Pins" and "Battery Charge" connectors are disabled.
- When the right relay is active, power comes from the "Charge Pins" connector, disabling "Main Battery" connector.
- When power comes from "Charge Pins", you can now activate the left relay. Left relay will output power to the "Charge Battery" connector.
- When battery is fully charged it will disable left relay, and only draw power from "Charge Pins", so the battery is completely disconnected.
- When it´s time to undock, the right relay will disable and draw power from the "Main Battery" connector again.

Do NOT connect a 7 cell battery! When fully charged it will output 29.4 Volt (7 * 4.2 = 29.4 Volt)
If you really need to provide a 7 cell battery, you need to replace a couple of resistors, they are in the package of SMD 0603. Without replacing them you might risk to blow up a couple of pins on the STM32 MCU.

The resistor that needs to be replaced: 
-R3 30K
-R4 7.5K
-R6 30K 
-R5 7.5K

With the above resistors, an input voltage of 25.2 would give us 5.0.4 volt, which we can handle.
But if using 30 Volt you will get about 6V which will burn the pins. You need to replace two of them (R3 or R4 and R5 or R6), so you don´t go above 5 Volt.
Use this link for calculating the divider. Also when you replaced them, you need to update the software to calculate the correct voltage.
https://ohmslawcalculator.com/voltage-divider-calculator
Do this at your own risk.

# RTC Clock:

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

# Noise:

Noise is something we need deal with, and also have to live with. In particular with DC motors driven by PWM. PWM as you might know is a technique to rapidly turn a signal on and off very quickly. Usually you want to have a PWM freq above the 20KHz, because that is what we humans cannot hear, going below and you might hear some whisling noise.
However, PWM will also put some stress on the hardware, creating high frequency noise which can interfere with other cricuits.
The boundary wire sensors are VERY sensitive to noise. The signal they pickup is amplified 556 times. That also means the noise is amplified 556 times. To mitigate some of it I have implemented a FIR filter (finite impulse response), this filter can filter out noise in a very specific way, however there are some limitations of course. It is very CPU intensive. Luckily the STM32F4 MCU have dedicated hardware (DSP - Digital Signal Processor), so I have offloaded it to the hardware instead. This means the performace is so much greater than if it was done in software. Still there is another limitation, and following the Nyquist sampling theorem, it dictates that you need to sample at least twice the frequency you want to filter out. So, if our motors are running at 25KHz, we need to sample it at least at 50KHz. Lucky for us that is no problem for a STM32 MCU. Still, a filter is a filter, and there is no perfect filter. But this can help us alot. The weaker the signal gets, the harder it is to filter and read correctly.
Because the Razorboard is a DIY board, people will use different motors and every setup will be unique. So if you get a bad sensor reading you might want to play with the PWM frequency and sampling rate. But some general tips, use as short motor leads as possible, twist the leads as much as you can. You can also use a ferrite ring and go around it a couple of times. Some people say that you can solder a ceramic capacitor between the motor leads, however be careful with this, as that will interfere with the PWM signal, if you want to test, use something in the range of 10nF.
Keep the boundary sensors away as far as possible from the motors. If possible use shielded cables to the sensors.
If you still get bad readings, you might need to increase the volt/current on the boundary wire.

To test your readings, take the mower and a laptop, go to the center of your lawn, (or the worst part of the lawn), place the mower on something so the wheels doesn´t touch the ground, connect to Razorboard with a USB cable. Power on Razorboard, and open a serial console and press ENTER (or type DISABLE), this will disable the system.
Now, type "DEBUG ON", and hopfully you will see the boundary signals. And now you can test to run the motors and see how that will affect the readings.

# Boundary Wire

The Boundary Wire Signal generator, outputs a signal in 10KHz, with 10 bits, so one complete message is about 1ms. The sampling rate on Razorboard is currently at 67.3 KHz.
The ADC on STM32 is running in a continues scan mode, meaning if you sample 512 samples into a buffer, odd numbers in the buffer (1,3,5 etc..) are BWF1, all even numbers (0,2,4 etc...) are BWF2. After we have sampled we need to break out the streams into BWF1 and BWF2, after that we run the streams in a FIR filter, eliminating any noise from the motors.
Now we need to compare this signal with a reference signal, we do this by a technique called Cross-Correlation. After the Cross-Correlation we get a number within the range of 1.0 and -1.0.

- 1.0 Means we have 100% match for an INSIDE message.
- 0.0 Means no match at all for either message.
- -1.0 Means we have 100% match for an OUTSIDE message.

By default at the moment, Razorboard classifies 0.80 and above as INSIDE, the same goes for OUTSIDE (-0.80). This can be changed in software.
In the "debug" menu, you can record a new signature and see how well it it detected. At the moment it only saves the signature to memory and will be lost after power-down.
You can also type "show sig" to plot the signature onto a plotter (like the Arduino Plotter).
Type "export sig" and you will get the signature as an array if you like to save the signture in software and compile it.

# MPU-6050

You only need to solder 4 pins on the 6050: SDA, SCL, GND and VCC.
When connecting the MPU-6050 to the SDA and SCL, use 3.3V for power. You will find one 3.3V pin just above the I2C connection.
The pins SDA, SCL are located just to left of the motor connections (on the lower right side of the STM32, at the bord edge), they are shared with UART3 (although UART3 can be moved to different pins in software).
Nothin else needs to be done, the code includes a calibration sequens.

# Boundary Wire Signal Generator PCB

- Voltage input: ~8 - 25.2 volt.
- Connection for charging your mower, voltage is a passthrough from input.
- 2 Boundary Wire connections, the plan is to support 2 boundary wires from the same PCB, OR 1 boundary wire + guide wire to go directly home.
- 1 step-down voltage regulator to control the voltage/current for the boundary wire.
- 1 USB connector for upgrading firmware
- 1 Red LED to show status
- 1 Fuse of 4A

You need to add a power resistor for each loop you want to use. Without a power resistor you can damage the driver.
Your total Ohm should be in the range of 5-12 ohm.

So lets take an example, you measure your loop with a multimeter and you discover you have about 2.5 ohms.
Then find a power resistor, minimum of 50W, lets say you find a resistor of 8 ohm. 2.5 + 8 = 10.5 ohm.

This power resistor will get hot, the more current the hotter, so mount it on a good heatsink.
By default, Loop1 is the primary connector for the boundary wire. Loop2 will come later.

At power up, the PCB takes a series of measurements to determine what the current level on the charge connector is (to be able to detect when the mower is docked). Do not have the mower connected to the connector, that will interfere with the measurements. If you are certain about the level, you could hardcode it and compile a new .bin file.
When the red LED is static, the PCB is powered. When blinking (2 hz) the boundary wire is active.

# Importing the project

Download the Github repository.
- Inside CubeIDE
- File -> Import -> File System

Select the folder where you downloaded the repository.


Make sure you set the project as "Release": 
- Project -> Build Configurations -> Set Active -> Release.

This will speed up the firmware quite a lot.

After import you need to add a few things:

- Right click on the project
- Select "Properties"
- Click on "C/C++ Build"
- Click on "Settings"
- Enable "Use float with printf"
- Enable "Use float with scanf"
- Click on "MCU GCC Linker"
- Click on "Libraries"
- Add a new under "Libraries (-l)"
- Type: arm_cortexM4lf_math
- Add a new search path under "Libraries search path (-L)"
- find this path: "C:\<your CubeIDE path>\STM32Cube\Repository\STM32Cube_FW_F4_V1.26.0\Drivers\CMSIS\Lib\GCC"

- Apply and close

You should now be able to "Build Project" from:
- Project -> Build Project

# Hardware hacks

- If you experience random connections issues when uploading firmware, you can solder a bit of wire between two pins, a picture called "USB" shows which pins.
- A low pass filter in hardware for the boundary sensors, solder a 22pF ceramic capacitor between the legs of each resistor, R22, R11, R25 and R16. This is not yet tested in the field, but looks promising on the oscilloskop, with 22pF the cutoff frequency is at 13KHz. Everything under 13KHz should pass, while blocking anything above. A picture called "LOWPASS" shows how to solder.
- As you might noticed, when using the MPU-6050, you use SDA and SCL which are shared with UART3. However, in software you can move UART3 to PD8 and PD9. So you dont loose UART3 just because an MPU-6050 is connected. You got to love STM32 ;)

# Troubleshooting:

Connect a USB cable to the STM32 connector (Upper left corner)
Set the COM port to 115200 baud
Press ENTER, this will disable the RazorBoard (for safety)
Type "help" for a menu
  
Before releasing the Razorboard in the wild, you can check the motors if they spin in the right direction.
In the debug menu you can test the motors on the "bench". If they are spinning in the wrong direction, swap the motor leads on the connector.
In a future firmware this can be done in software.
With the command "DEBUG ON" you can verify that the boundary signals are received correctly. Also how many you receive per second. This can be a great tool to check the signal by placing the mower in the center of you lawn and check if you still receive the messages.

# Bugs

If you find any bugs please report them so we can fix them!

# Questions

You can find me here: calle ( at ) lanstep.com

I whish you a succesfull robot build! 😎


