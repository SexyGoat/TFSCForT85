TFSCForT85 Firmware for Temperature-based Fan Speed Control on an ATtiny85 MCU
------------------------------------------------------------------------------


(c) Copyright 2022, Daniel Neville (creamygoat@gmail.com)


This file is part of TFSCForT85.

TFSCForT85 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

TFSCForT85 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with TFSCForT85.


------------------------------------------------------------------------------
Program Description
------------------------------------------------------------------------------


The AVR assembly program (with C preprocessor) is for an Atmel
(now Microchip) ATtiny85 microcontroller. Once programmed, the
microcontroller takes an analogue input at POT (usually from a
potentiometer) and outputs 25kHz PWM signals for controlling
the speed of a 4-wire fan (or two).

The analogue input should be between 0V and VDD (+5V). If a lower
reference voltage is requried, set UPB_POTREF to 1 and connect the
potentiometer reference voltage to pin AREF (pin 5). The default
setting is for VDD to make it convenient to experiment with the
project while the programmer is still plugged in, powering the MCU.

A bonus feature is the ability to read the temperature reported
by a single DS18B20 digital thermometer. If communication with
the DS18B20 fails to result with a temperature reading and a valid
CRC after eight consecutive attempts, the analogue input will be
used instead.

In the case of the DS18B20 being detected and found to be working,
the 4.4 format bias adjustment (EEPROM byte 0) and fan speed curve
data (also in EEPROM) will be used.

Strong S-curve filtering is applied via the use of extravagant
accumulating type averaging buffers, consuming most of the SRAM
on the chip.

A moderate on-off hysteresis is used to take advantage of Mode B
fans, which have a minimum running speed (usually 30%) and turn
off fully when given a zero-duty PWM signal. In this implementation
Fan B has its on-off threshold hysteresis region set about the
15% position. Mode A fans will never switch off or run below their
minimum speed. As the PWM outputs are independent, it is important
to avoid wiring the PWM outputs together.


------------------------------------------------------------------------------
Default Pin Assignment
------------------------------------------------------------------------------

             ______                 A DS1811 Reset Controller is recommended.
    ~RESET 1|o AT  |8 VDD (+5V)     THERMIO requires a 4.7k pull-up whether
   THERMIO 2| tiny |7 POT           the DS18B20 thermometer is used or not.
  FANBCTRL 3|  85  |6 FANBCTRL      A linear 10kΩ potentiometer is connected
       GND 4|______|5 (POTREF)      between GND and VDD with the wiper
                                    connection going to POT.

The source file may be easily modified to reassign the pins, to enable
complementary outputs (with optional dead time generation), to alter the
filtering and to alter the relationship between the A and B PWM outputs.

By default, VDD is the reference voltage for the potentiometer input and
the POTREF function is unused. Therefore pin 5 may be assigned a different
function. If POTREF is to be used, it must be on Pin 5 because only Pin 5
has the alternate function AREF. In tfsc.S, adjust UPB_POTREF to 0 or 1 as
required.


------------------------------------------------------------------------------
The DS18B20 Digital Thermometer (Optional)
------------------------------------------------------------------------------


A single DS18B20 digital thermometer may be connected to a 5V supply,
ground and the THERMIO pin (Pin 2) of the MCU.

When the thermometer is detected, the MCU will switch from using the
potentiometer input to using the thermometer and the Fan Speed Curve
stored in EEPROM.

EEPROM memory map:

 Offset
  000 TempAdj  Adjustment to add (2’s complement, 1/16 deg C steps)
  001 LowT     Lowest temperature (2’s complement, degs C) of curve domain
  002 HighT    Highest temperature (2’s complement, degs C) of curve domain
  003 SpeedPct Speed percentage (0..100) to be implied by level 255 in curve
  004 Curve    Fan speed curve in eight bytes ranging from 0 to 255
  00C (Unused region)

The fan speed curve is not required to be monotonic. It may even be reversed.
If LowT and HighT are the same, the output will be bi-level, with the final
curve table entry being used when the temperature matches or exceeds the
temperature value LowT and HighT share.

When testing a new DS18B20, it is helpful to set LoWT and HighT to a
narrow range so that the output of MCU can be monitored with an LED or
an oscilloscope when the DS18B20 is held between thumb and forefinger.


------------------------------------------------------------------------------
