;-----------------------------------------------------------------------------
; tfsc.S - Temperature-based Fan Speed Control (with Potentiometer fallback)
;-----------------------------------------------------------------------------
;
; TFSCForT85 Fan Speed PWM Driver
; Version: 1.0.0.0
; (c) Copyright 2022, Daniel Neville (creamygoat@gmail.com)
;
; This program is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program. If not, see <https://www.gnu.org/licenses/>.
;
;-----------------------------------------------------------------------------
;
; DESCRIPTION
;
; This AVR assembly program (with C preprocessor) is for an Atmel
; (now Microchip) ATtiny85 microcontroller. Once programmed, the
; microcontroller takes an analogue input at POT (usually from a
; potentiometer) and outputs 25kHz PWM signals for controlling
; the speed of a 4-wire fan (or two).
;
; The analogue input should be between 0V and AREF (pin 5), which
; is usually the same voltage as VDD but may be lower.
;
; A bonus feature is the ability to read the temperature reported
; by a single DS18B20 digital thermometer. If communication with
; the DS18B20 fails to result with a temperature reading and a valid
; CRC after eight consecutive attempts, the analogue input will be
; used instead.
;
; In the case of the DS18B20 being detected and found to be working,
; the 4.4 format bias adjustment (EEPROM byte 0) and fan speed curve
; data (also in EEPROM) will be used.
;
; Strong S-curve filtering is applied via the use of extravagant
; accumulating type averaging buffers, consuming most of the SRAM
; on the chip.
;
; A moderate on-off hysteresis is used to take advantage of Mode B
; fans, which have a minimum running speed (usually 30%) and turn
; off fully when given a zero-duty PWM signal. In this implementation
; Fan B has its on-off threshold hysteresis region set about the
; 15% position. Mode A fans will never switch off or run below their
; minimum speed. As the PWM outputs are independent, it is important
; to avoid wiring the PWM outputs together.
;
;
; PIN ASSIGNMENT
;               ____
;      ~RESET 1| AT |8 VDD           A DS1811 Reset Controller is recommended.
;     THERMIO 2|tiny|7 POT
;    FANBCTRL 3| 85 |6 FANACTRL
;         GND 4|____|5 POTREF        THERMIO requires a 4.7k pull-up.
;
;-----------------------------------------------------------------------------
; Notes
;-----------------------------------------------------------------------------
;
; Example configuration in EEPROM:
;   FDEE28640022233348C0E5FF
;   FD: -0.1875 degrees C adjustment to apply to output of DS18B20 thermometer
;   EE: -18 degrees C for LowT
;   28: 40 degrees C for HighT
;   64: Scale of 100% for output of fan speed curve function
;   0022233348C0E5FF: Curve shape (8 uint8s) for domain [LowT, HighT]
;
; Fuses:
;   Low fuse: 0xe2 (5-4:SUT[1:0]=3, 3-0:CKSEL=8MHz system clock)
;   High fuse: 0xd7 (6:SPIEN, 3:EESAVE)
;   Extended fuse: 0xff
;
; Fuse programming
;   If the standard 8MHz system clock is used:
;     avrdude -p t85 -c usbasp -U lfuse:w:0xe2:m
;   If instead, the 16MHz system clock (via 64MHz PLL) is used:
;     avrdude -p t85 -c usbasp -U lfuse:w:0xf1:m
;   (See the USE_16MHZ_CK symbol)
;   avrdude -p t85 -c usbasp -U hfuse:w:0xd7:m
;   avrdude -p t85 -c usbasp -U efuse:w:0xff:m
;
; Compiling, inspecting and programming:
;   avr-gcc -mmcu=attiny85 -o tfsc.o -c tfsc.S
;   avr-gcc -mmcu=attiny85 -o tfsc.elf tfsc.o
;   avr-objcopy -O ihex tfsc.elf tfsc.hex
;   avrdude -p t85 -c usbasp -U flash:w:tfsc.hex:i
;   avrdude -p t85 -c usbasp -U eeprom:w:tfsc.hex:i
;
; Default calling convention:
;   r0, r12-r17 may be modified
;   r1 = 0, r2 = 1, r3 = 255
;   Data arguments and data return values use registers starting at r16.
;
;-----------------------------------------------------------------------------

;-----------------------------------------------------------------------------
; Index
;-----------------------------------------------------------------------------


; Imports
; Exports
; Constants
; Structures
; EEPROM addresses
;
; Interrupt handlers:
;   __vector_default
;   TIM1_OVF_vect
;
; Initialisation:
;   InitialiseRegisterConstants
;   InitialiseWatchDogTimer
;   InitialiseHardware
;   ClearGlobals
;
; EEPROM functions:
;   ReadEEPROMByte
;   WriteEEPROMByte
;   ReadEEPROMBlock
;   WriteEEPROMBlock
;
; Utility functions:
;   ClearSRAMBlock
;
; Mathematical functions:
;   Mult_u8
;   Mult_u16
;   Div16_u24
;   Div8_u24
;   Div4_u24
;   Div32_u21
;   Div64_u22
;   Div128_u23
;   Div128_u23_RHtE
;   Div64_u22_RHtE
;   Div32_u21_RHtE
;   Div16_u24_RHtE
;   Div8_u24_RHtE
;   Div4_u24_RHtE
;   Div_u24
;
; Application functions:
;   UpdateBoxFilterBuffer
;   ScaleADC10LARToPWMDuty
;   OCRValueFromDuty
;   Dither
;   IncrementCounters
;   UpdateHardwarePWMEnableStates
;   UpdatePWMEnableFlags
;   LowLevelThermIO
;   AdvanceThermometerState
;   LoadSpeedFnParamsFromEEPROM
;   LoadSpeedFnCurveFromEEPROM
;   FinishSpeedFnSetup
;   AdvanceFanSpeedFn
;
; main
;
; Data in program memory
;
; Globals in SRAM


;-----------------------------------------------------------------------------
; Imports
;-----------------------------------------------------------------------------


#include <avr/common.h>
#include <avr/io.h>

#include "upb8macros.h"
#include "ldimacros.h"
#include "structmacros.h"


;-----------------------------------------------------------------------------
; Exports
;-----------------------------------------------------------------------------


.global main
.global __vector_default
.global TIM1_OVF_vect


;-----------------------------------------------------------------------------
; Constants
;-----------------------------------------------------------------------------


; System clock speed mode (to be adjusted along with fuses)
USE_16MHZ_CK = 0  ; LFUSE = 0xE2
;USE_16MHZ_CK = 1  ; LFUSE = 0xF1

; Inputs and outputs
UPB_FANACTRL = UPB('B', 1)  ; Fixed at 6:OC1A
UPB_FANBCTRL = UPB('B', 4)  ; Fixed at 3:OC1B
UPB_THERMIO = UPB('B', 3)   ; 2:PB3
UPB_POT = UPB('B', 2)       ; 7:ADC1
UPB_POTREF = UPB('B', 0)    ; Fixed at 5:AREF

USE_POTREF = 1

; On/off hysteresis, measured in (1/160) duty values 0..160
; PWM output enabled when duty ratio >= HIGH / 160
; PWM output disabled when duty ratio < LOW / 160
; LOW = HIGH implies zero hysteresis.
; Note: A duty ratio of 1/160 implies an OCR valeu of zero, which
; in turn, implies a special zero-duty case for newer revisions of
; the ATtiny85 chip.
FANA_OFF_THRES = 2
FANA_ON_THRES = 6
FANB_OFF_THRES = 20
FANB_ON_THRES = 24

; Box filter buffers, using vast amounts of SRAM
BOXFBUF_BYTES_PER_SAMPLE = 2  ; Fixed at 2 by implementation
ADC_BF_INDEX_BITS = 5  ; 5 is recommended to ensure overlap.
; If the outputs are tied to one input and stronger damping is
; required, the two sets of output buffers can be made smaller
; while the input buffers are made larger.
INPUT_BF1_INDEX_BITS = 6
INPUT_BF2_INDEX_BITS = 5
OUTA_BF1_INDEX_BITS = 4
OUTA_BF2_INDEX_BITS = 4
OUTB_BF1_INDEX_BITS = 4
OUTB_BF2_INDEX_BITS = 4
ADC_BF_BUFFER_LENGTH = 1 << ADC_BF_INDEX_BITS
INPUT_BF1_BUFFER_LENGTH = 1 << INPUT_BF1_INDEX_BITS
INPUT_BF2_BUFFER_LENGTH = 1 << INPUT_BF2_INDEX_BITS
OUTA_BF1_BUFFER_LENGTH = 1 << OUTA_BF1_INDEX_BITS
OUTA_BF2_BUFFER_LENGTH = 1 << OUTA_BF2_INDEX_BITS
OUTB_BF1_BUFFER_LENGTH = 1 << OUTB_BF1_INDEX_BITS
OUTB_BF2_BUFFER_LENGTH = 1 << OUTB_BF2_INDEX_BITS
ADC_BF_BUFFER_SIZE = ADC_BF_BUFFER_LENGTH * BOXFBUF_BYTES_PER_SAMPLE
INPUT_BF1_BUFFER_SIZE = INPUT_BF1_BUFFER_LENGTH * BOXFBUF_BYTES_PER_SAMPLE
INPUT_BF2_BUFFER_SIZE = INPUT_BF2_BUFFER_LENGTH * BOXFBUF_BYTES_PER_SAMPLE
OUTA_BF1_BUFFER_SIZE = OUTA_BF1_BUFFER_LENGTH * BOXFBUF_BYTES_PER_SAMPLE
OUTA_BF2_BUFFER_SIZE = OUTA_BF2_BUFFER_LENGTH * BOXFBUF_BYTES_PER_SAMPLE
OUTB_BF1_BUFFER_SIZE = OUTB_BF1_BUFFER_LENGTH * BOXFBUF_BYTES_PER_SAMPLE
OUTB_BF2_BUFFER_SIZE = OUTB_BF2_BUFFER_LENGTH * BOXFBUF_BYTES_PER_SAMPLE

; General flags
GFLAG_BIT_PWMA = 0
GFLAG_BIT_PWMB = 1
GFLAG_BIT_PWMA_SHUTDOWN_PENDING = 2
GFLAG_BIT_PWMB_SHUTDOWN_PENDING = 3
GFLAG_BIT_HYSTLATCHA = 4
GFLAG_BIT_HYSTLATCHB = 5

; Digital thermometer IO
DTIO_BUFFER_SIZE = 11  ; Allows for SkipROM, [MemCmd], 8 bytes and CRC.

; Digital thermometer lwo-level IO flags
DTIOFLAG_BUSY = 7
DTIOFLAG_DATA = 6
DTIOFLAG_REST2 = 5
DTIOFLAG_WRITE0 = 4
DTIOFLAG_REST1 = 3
DTIOFLAG_RESET_REQUEST = 2
DTIOFLAG_RESETTING = 1
DTIOFLAG_PRESENT = 0

; Digital thermometer (high level) state
DTSTATE_IDLE = 0
DTSTATE_START = 1
DTSTATE_CONVERTING = 2
DTSTATE_READING = 3
DTSTATE_CONFIGURING = 4
DTSTATE_COMPLETED = 5
DTSTATE_FAILED = 6

; Fan Speed Function
NUM_FSCURVE_POINTS = 8  ; May be from 1 to 15

; Fan Speed Function state
FSFSTATE_IDLE = 0
FSFSTATE_START = 1
FSFSTATE_TSCALE1 = 2
FSFSTATE_TSCALE2 = 3
FSFSTATE_TSCALE3 = 4
FSFSTATE_LERP = 5
FSFSTATE_SSCALE1 = 6
FSFSTATE_SSCALE2 = 7
FSFSTATE_COMPLETE = 8


;-----------------------------------------------------------------------------
; Structures
;-----------------------------------------------------------------------------


.stroffsets  ; Interrupt Service Routine data
;.field  ISRData_Counter, 2
;.field  ISRData_Serviced, 1
.strend ISRDataSize

.stroffsets  ; Dithering of OCR registers Note: Duty = (OCR + 1) / T.
.field  DitherRec_Target, 2
.field  DitherRec_Error, 2
.field  DitherRec_Dithered, 1
.strend DitherRecSize

.stroffsets  ; Globals accessed with Y register
.field  G_Counter25kHz, 1
.field  G_Counter2500Hz, 1
.field  G_Counter100Hz, 1
.field  G_Flags, 1
.field  G_ADCResult, 2
.field  G_ThermometerState, 1
.field  G_ThermErrCount, 1
.field  G_TemperatureAdj, 1  ; Adjustment to add in 4.4 fixed point format
.field  G_Temperature, 2  ; Sixteenths of a degree in Two's Complement
.field  G_TempFnOutput, 2
.field  G_ScaledADCResult, 2
.field  G_PrefilteredInput, 2
.field  G_FilteredInput, 2
.field  G_FilteredOutputA, 2
.field  G_FilteredOutputB, 2
.field  G_OutADitherRec, DitherRecSize
.field  G_OutBDitherRec, DitherRecSize
.strend GSize

.stroffsets
.field  BFBuf_Sum, BOXFBUF_BYTES_PER_SAMPLE + 1
.field  BFBuf_Index, 1
.field  BFBuf_MaxIndex, 1
.strend BFBufHdrSize

.stroffsets
.field  DTIO_Flags, 1
.field  DTIO_NumBitsToWrite, 1
.field  DTIO_NumBitsToRead, 1
.field  DTIO_BitMask, 1
.field  DTIO_ByteIx, 1
.field  DTIO_CRC, 1
.field  DTIO_Buffer, DTIO_BUFFER_SIZE
.strend DTIOSize

.stroffsets
.field  SpeedFn_State, 1
.field  SpeedFn_LowT, 1
.field  SpeedFn_HighT, 1
.field  SpeedFn_HighSpeedPct, 1
.field  SpeedFn_Temperature, 2
.field  SpeedFn_T2SCIScale, 3
.field  SpeedFn_S2PWMScale, 2
.field  SpeedFn_SCIndex, 2
.field  SpeedFn_PartialProduct, 3
.field  SpeedFn_LerpedValue, 2
.field  SpeedFn_PWMDuty, 2
.field  SpeedFn_Curve, NUM_FSCURVE_POINTS
.strend SpeedFnSize


;-----------------------------------------------------------------------------
; EEPROM addresses
;-----------------------------------------------------------------------------


.stroffsets
.field  EEPROM_TempAdj, 1
.field  EEPROM_LowT, 1
.field  EEPROM_HighT, 1
.field  EEPROM_SpeedPct, 1
.field  EEPROM_SpeedCurve, NUM_FSCURVE_POINTS
.strend EEPROM_End


.if EEPROM_End > 512
  .error "There is not enough space in the 512-byte EEPROM."
.endif
.if EEPROM_End > 256
  .warning "Data for EEPROM exceeds 256 bytes."
.endif


;-----------------------------------------------------------------------------
.section .text
;-----------------------------------------------------------------------------

;-----------------------------------------------------------------------------
; Macros
;-----------------------------------------------------------------------------


.macro DIVU_TO_16_BIT_Q_RHTE Divisor
  .if (\Divisor) == 128
    rcall   Div128_u23_RHtE  ; 29 cycles
  .else
    .if (\Divisor) == 64
      rcall   Div64_u22_RHtE  ; 39 cycles
    .else
      .if (\Divisor) == 32
        rcall   Div32_u21_RHtE  ; 40 cycles
      .else
        .if (\Divisor) == 16
          rcall   Div16_u24_RHtE  ; 42 cycles
        .else
          .if (\Divisor) == 8
            rcall   Div8_u24_RHtE  ; 39 cycles
          .else
            .if (\Divisor) == 4
              rcall   Div4_u24_RHtE  ; 29 cycles
            .else
              .error "Value of NumBits for DIVU macro not handled!"
            .endif
          .endif
        .endif
      .endif
    .endif
  .endif
.endm


;-----------------------------------------------------------------------------
; Interrupt handlers
;-----------------------------------------------------------------------------


__vector_default:

        reti


;-----------------------------------------------------------------------------


TIM1_OVF_vect:
; Timer 1 overflow interrupt service routine

        sbi     _SFR_IO_ADDR(GPIOR0), 0
        reti

        ; 9 cycles including interrupt call and return

;        push    r16
;        ldi     r16, 255
;        sts     ISRData + ISRData_Serviced, r16
;        pop     r16
;        reti
;
;        ; 15 cycles including interrupt call and return


;        ; Save the flags and a minimal set of registers.
;        push    r16
;        in      r16, _SFR_IO_ADDR(SREG)
;        push    r16
;        push    r17
;        ; Note that the Zero Register r1 might not be zero here.
;
;        ; r17 will be the Zero Register for as long as we need it.
;        ldi     r17, 0
;
;        ; Indicate to the main program (which may have been wakened
;        ; by any cause) that this interrupt occurred.
;        ldi     r16, 255
;        sts     ISRData + ISRData_Serviced, r16
;
;        ; Update the 16-bit interrupt counter the main program can
;        ; use for basic timing.
;        sec
;        lds     r16, ISRData + ISRData_Counter + 0
;        adc     r16, r17
;        sts     ISRData + ISRData_Counter + 0, r16
;        lds     r16, ISRData + ISRData_Counter + 1
;        adc     r16, r17
;        sts     ISRData + ISRData_Counter + 1, r16
;
;9:      pop     r17
;        pop     r16
;        out     _SFR_IO_ADDR(SREG), r16
;        pop     r16
;        ; 29 cycles
;        reti
;
;        ; 37 cycles including interrupt call and return


;-----------------------------------------------------------------------------
; Initialisation
;-----------------------------------------------------------------------------


InitialiseRegisterConstants:
; Out: r1 = 0
;      r2 = 1
;      r3 = 255
;      Y = Globals

        ; r1 = 0
        clr     r1
        ; r2 = 1, handy for carry-changing inc/dec via add/sub.
        clr     r2
        inc     r2
        ; r3 = 255 for sign extension.
        clr     r3
        dec     r3
        ldiw    Y, Globals
        ; 7 cycles
        ret

        ; 14 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


InitialiseWatchDogTimer:

        ; Reset the watchdog timer so that the watchdog doesn't force
        ; a reset during the configuration of the WDT itself.
        wdr
        ; Clear the watchdog reset flag so that it doesn't
        ; disable the watchdog timer.
        in      r16, _SFR_IO_ADDR(MCUSR)
        andi    r16, ~(1 << WDRF)
        out     _SFR_IO_ADDR(MCUSR), r16
        ; Start the watchdog timer and have it force a reset of the MCU
        ; if the timer isn't reset at least once every 0.5 seconds.
        in      r16, _SFR_IO_ADDR(WDTCR)
        ori     r16, (1 << WDCE) |  (1 << WDE)
        mov     r17, r16
        andi    r17, ~((1 << WDCE) | (1 << WDP3) | (0b111 << WDP0))
        ori     r17, (1 << WDE) | (0 << WDP3) | (0b101 << WDP0)
        out     _SFR_IO_ADDR(WDTCR), r16
        out     _SFR_IO_ADDR(WDTCR), r17
        ; 11 cycles
        ret

        ; 18 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


InitialiseHardware:

        ; Enable power to the ADC module.
        ; The Universal Serial Interface module is not needed.
        ; Bit 1s in the Power Reduction Register mean "disable".
        ldi     r16, (0 << PRADC) | (1 << PRUSI)
        out     _SFR_IO_ADDR(PRR), r16

        ; Adjust the oscillator calibration
;        ldi     r16, 0x93
;        out     _SFR_IO_ADDR(OSCCAL), r16

        ; Pause for 1000 clock cycles
        ldi     r16, 200
1:      adiw    Z, 0
        dec     r16
        brne    1b

.if USE_16MHZ_CK
        ; Remember to clear (program) clock source and Start-Up Time
        ; fuses appropriately. Typical values for the Low Fuse Byte
        ; are 0xE2 for 8Mhz and 0xF1 for 16MHz.

        ; See if the phase-locked loop has already been started.
        in      r16, _SFR_IO_ADDR(PLLCSR)
        sbrc    r16, PLOCK
        rjmp    2f
        ; Start the phase-locked loop.
        ldi     r16, (1 << PLLE)
        out     _SFR_IO_ADDR(PLLCSR), r16
        out     _SFR_IO_ADDR(GTCCR), r1
        ; Wait for 100us for the PLL to stop swinging wildly.
        ldiw    r17:r16, 100 * 16/4
1:      sub     r16, r2
        sbc     r17, r1
        brne    1b
        ; Wait for the PLL to be properly locked.
1:      in      r16, _SFR_IO_ADDR(PLLCSR)
        sbrs    r16, PLOCK
        rjmp    1b
2:
.endif

        ; Configure Timer/Counter 1 to run at 4MHz,
        ; producing f = 25kHz when C is set to 159
        ; where f = Timer_frequency / (C + 1).
        ;
        ; TCCR1.CTC1 = 1
        ; TCCR1.CS1[3:0] = 0b0011 => CK/2 (4MHz)
        ;                  0b0010 => CK/4 (4MHz) if 16MHz via PLL is used
        ; TCCR1.PWM1A = 1 => PWM enabled for OC1A
        ; GTCCR.COM1A[1:0] = 0b10 => Non-inverting PWM for OC1A
        ; TCCR1.COM1B[1:0] = GTCCR.COM1A[1:0] to fix OC1B bug
        ; GTCCR.PWM1B = 1 => PWM enabled for OC1B
        ; GTCCR.FOC1B = 1
        ; GTCCR.FOC1A = 1
        ; GTCCR.PSR1 = 1
        ; TCNT1 = 0
        ; OCR1A = 0
        ; OCR1B = 0
        ; OCR1C = 159
        ;
        ; Note: The datasheet says that in Fast PWM Modem an OCR value of
        ; zero causes the output pin to generate 1-count pulses so that the
        ; (non-inverting) duty cycle = (OCR + 1)/(TOP + 1). However, some
        ; versions of the ATtiny85 treat an OCR value of zero as a special
        ; case and output a zero-duty signal.

        ldi     r16, (\
                  (1 << TSM) |\
                  (1 << PWM1B) |\
                  (0b10 << COM1B0) |\
                  (1 << FOC1B) |\
                  (1 << FOC1A) |\
                  (1 << PSR1) |\
                  (0 << PSR0) |\
                0)
        out     _SFR_IO_ADDR(GTCCR), r16

        ldi     r16, 159
        out     _SFR_IO_ADDR(OCR1C), r16
        ldi     r16, 0
        ldi     r17, 0
        out     _SFR_IO_ADDR(OCR1A), r16
        out     _SFR_IO_ADDR(OCR1B), r17

.if USE_16MHZ_CK
        TIMER1_SOURCE_IX = 0b0011  ; CK/4
.else
        TIMER1_SOURCE_IX = 0b0010  ; CK/2
.endif

        ldi     r16, (\
                  (1 << CTC1) |\
                  (1 << PWM1A) |\
                  (0b10 << COM1A0) |\
                  ((TIMER1_SOURCE_IX) << CS10) |\
                0)
        out     _SFR_IO_ADDR(TCCR1), r16
        out     _SFR_IO_ADDR(TCNT1), r1

        ; Start the counting.
        in      r16, _SFR_IO_ADDR(GTCCR)
        andi    r16, ~((1 << TSM) | (1 << PSR1) | (1 << PSR0))
        out     _SFR_IO_ADDR(GTCCR), r16

        ; Allow Timer 1 to generate an interrupt on overflow.
        ldi     r16, (1 << TOIE1)
        out     _SFR_IO_ADDR(TIMSK), r16

        ; Configure the ADC to measure the potentiometer position.
        ;
        ; ADMUX.REFS2:REFS[1:0] = 0b01 => Pin 5:AREF is the reference
        ;                      or 0b00 => Pin 8:VDD is the reference
        ; ADMUX.ADLAR = 1 => Data register left-aligned
        ; ADMUX.MUX = 0b0001 => 7:ADC1 (overriding PB2)
        ;          or 0b0011 => 2:ADC3 (overriding PB3)
        ;          or 0b0010 => 3:ADC2 (overriding PB4)
        ;          or 0b0000 => 1:ADC0 (overriding PB5/~RESET)
        ;
        ; ADCSRB.BIN = 0 => Unipolar input mode (unsigned)
        ; ADCSRB.IPR = 0 => Non-reversed input
        ; ADCSRB.ATS = 0b000 => Free running mode (when ADATE = 1)
        ;
        ; ADCSRA.ADEN = 1 => ADC enabled
        ; ADCSRA.ADSC = 0 => Don't start ADC conversion yet.
        ; ADCSRA.ADATE = 0 => No automatic triggering
        ; ADCSRA.ADPS = 0b111 => CLK/128 = 62.5kHz (50..200kHz is normal.)

        ldi     r16, (\
                  (0 << REFS2) |\
                  (USE_POTREF << REFS0) |\
                  (1 << ADLAR) |\
                  (((0x023100 >> PBIT(UPB_POT)) & 0b0011) << MUX0) |\
                0)
        out     _SFR_IO_ADDR(ADMUX), r16
        ldi     r16, (\
                  (0 << BIN) |\
                  (0 << IPR) |\
                  (0b000 << ADTS0) |\
                0)
        out     _SFR_IO_ADDR(ADCSRB), r16
        ldi     r16, (\
                  (1 << ADEN) |\
                  (0 << ADSC) |\
                  (0 << ADATE) |\
                  (0b111 << ADPS0) |\
                0)
        out     _SFR_IO_ADDR(ADCSRA), r16

        ; Set Data Direction and pull-ups
        UPM_FULL_IO = 0b011111 << 8
        DDR_UPM = (\
          (1 << UPB_FANACTRL) |\
          (1 << UPB_FANBCTRL) |\
        0)
        PORT_UPM = UPM_FULL_IO & (\
          (1 << UPB_THERMIO) |\
          (0 << UPB_FANACTRL) |\
          (0 << UPB_FANBCTRL) |\
        0)
        ldi     r16, PBMASK_UPM(PORT_UPM)
        ldi     r17, PBMASK_UPM(DDR_UPM)
        out     _SFR_IO_ADDR(PORTB), r16
        out     _SFR_IO_ADDR(DDRB), r17
        ; The digital thermometer's IO is open collector/drain.
        cbi     PORT_REGBIT_IO(UPB_THERMIO)

        ; Disable (with bit 1s) any unused digital input buffers to
        ; avoid floating inputs and to save power.
        ; Note. Despite the register bits for DIDR0 being named for
        ; the alternative functions to disable, the register bits
        ; have the same positions for each IO pin as they do as for
        ; the PORTB, PINB and DDRB registers.
        ldi     r16, PBMASK_UPM(UPM_FULL_IO) & (\
                  (1 << UPB_POT) |\
                  (USE_POTREF << UPB_POTREF) |\
                0)
        out     _SFR_IO_ADDR(DIDR0), r16

        ret

        ; Stack Depth = 2


;-----------------------------------------------------------------------------


ClearGlobals:
        clr     r1
        movw    r14, X
        ldiw    X, Globals
        ldiw    r17:r16, (GlobalsEnd - Globals)
        rcall   ClearSRAMBlock
        movw    X, r14
        ret

        ; Stack Depth = 4


;-----------------------------------------------------------------------------
; EEPROM functions
;-----------------------------------------------------------------------------


ReadEEPROMByte:
; In:  Z = Address in EEPROM
; Out: r0 = Byte fetched
;      Z postincremented
;      r16, r17 unmodified

1:      sbic    _SFR_IO_ADDR(EECR), EEPE
        rjmp    1b
        out     _SFR_IO_ADDR(EEARH), ZH
        out     _SFR_IO_ADDR(EEARL), ZL
        sbi     _SFR_IO_ADDR(EECR), EERE
        in      r0, _SFR_IO_ADDR(EEDR)
        adiw    Z, 1
        ret

        ; 17 cycles including rcall and 4 cycle CPU halt, if EEPROM was ready
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


WriteEEPROMByte:
; In: Z = Address in EEPROM
;     r0 = Byte to write
; Out: Z postincremented
;      r16, r17 unmodified

        push    r16
        ldi     r16, (0 << EEPM1) | (0 << EEPM0) ; Erase and write
1:      sbic    _SFR_IO_ADDR(EECR), EEPE
        rjmp    1b
        out     _SFR_IO_ADDR(EECR), r16
        out     _SFR_IO_ADDR(EEARH), ZH
        out     _SFR_IO_ADDR(EEARL), ZL
        out     _SFR_IO_ADDR(EEDR), r0
        sbi     _SFR_IO_ADDR(EECR), EEMPE
        sbi     _SFR_IO_ADDR(EECR), EEPE
        pop     r16
        adiw    Z, 1
        ret

        ; 22 cycles including rcall and 2 cycle CPU halt, if EEPROM was ready
        ; Stack Depth = 3


;-----------------------------------------------------------------------------


ReadEEPROMBlock:
; In: Z = Source in EEPROM
;     X = Destination in SRAM
;     r16 = Number of bytes to read
; Out: Z and X incremented by the given block size
;      r16 modifed

        tst     r16
        breq    2f
1:      rcall   ReadEEPROMByte
        st      X+, r0
        dec     r16
        brne    1b
2:      ret

        ; n > 0: 1 + 24n cycles including rcall, if EEPROM ready
        ; n = 0: 10 cycles including rcall
        ; Stack Depth = 4


;-----------------------------------------------------------------------------


WriteEEPROMBlock:
; In: X = Source in SRAM
;     Z = Destination in EEPROM
;     r16 = Number of bytes to write
; Out: X and Z incremented by the given block size
;      r16 modifed

        tst     r16
        breq    2f
1:      ld      r0, X+
        rcall   WriteEEPROMByte
        dec     r16
        brne    1b
2:      ret

        ; Stack Depth = 5


;-----------------------------------------------------------------------------
; Utility functions
;-----------------------------------------------------------------------------


ClearSRAMBlock:
; Fill zero or more bytes of SRAM with zeros.
;
; In: X = Destination in SRAM
;     r17:r16 = Number of bytes to fill with zeros
; Out: X incremented by the given block size
;      r17:r16 cleared

        lsr     r16
        brcc    1f
        st      X+, r1
1:      lsr     r16
        brcc    1f
        .rept   2
        st      X+, r1
        .endr
1:      breq    2f
1:      .rept   4
        st      X+, r1
        .endr
        dec     r16
        brne    1b
2:      tst     r17
        breq    1f
        dec     r17
        ldi     r16, 64
        rjmp    1b
1:      ret

        ; Stack Depth = 2


;-----------------------------------------------------------------------------
; Mathematical functions
;-----------------------------------------------------------------------------


Mult_u8:
; In: r16 = Multiplier
;     r17 = Multiplicand
; Out: r17:r16 = Product

        mov     r0, r17
        clr     r17
        lsr     r16
.rept 8
        brcc    1f
        add     r17, r0
1:      ror     r17
        ror     r16
.endr
        ; 35 cycles
        ret

        ; 42 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


Mult_u16:
; In: r17:r16 = Multiplier
;     r19:r18 = Multiplicand
; Out: r19:r18:r17:r16 = Product

        movw    r0, r18
        ldiw    r19:r18, 0
        lsr     r17
        ror     r16
.rept 16
        brcc    1f
        add     r18, r0
        adc     r19, r1
1:      ror     r19
        ror     r18
        ror     r17
        ror     r16
.endr
        clr     r1  ; Reset r1 after using it as a scratch register.
        ; 118 cycles at most
        ret

        ; 125 cycles at most, including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


Div16_u24:
; In: r18:r17:r16 = Dividend
; Out: r18:r17:r16 = Quotient for Dividend / 16

        lsr     r18
        ror     r17
        ror     r16
        ; Fall through to Div8_u24.
Div8_u24:
        lsr     r18
        ror     r17
        ror     r16
        ; Fall through to Div4_u24.
Div4_u24:
        lsr     r18
        ror     r17
        ror     r16
        lsr     r18
        ror     r17
        ror     r16
        ; 12 cycles
        ret

        ; 19 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


Div32_u21:
; In: r18:r17:r16 = Dividend
; Out: r18:r17:r16 = Quotient for Dividend / 32
;      r18 cleared

        lsl     r16
        rol     r17
        rol     r18
        ; Fall through to Div64_u22.
Div64_u22:
        lsl     r16
        rol     r17
        rol     r18
        ; Fall through to Div128_u23.
Div128_u23:
        lsl     r16
        rol     r17
        rol     r18
        mov     r16, r17
        mov     r17, r18
        clr     r18
        ; 12 cycles
        ret

        ; 19 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


Div128_u23_RHtE:
; In: r18:r17:r16 = Dividend
; Out: r18:r17:r16 = Dividend / 64, rounded half to even

        mov     r0, r19
        mov     r19, r16
        lsl     r16
        rol     r17
        rol     r18
        mov     r16, r17
        mov     r17, r18
        clr     r18
        bst     r19, 6
        andi    r19, 0x3F
        rjmp    DivPo2_Common

        ; 29 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


Div64_u22_RHtE:
; In: r18:r17:r16 = Dividend
; Out: r18:r17:r16 = Dividend / 64, rounded half to even

        mov     r0, r19
        mov     r19, r16
        rcall   Div64_u22  ; 16 cycles, SD = 2
        bst     r19, 5
        andi    r19, 0x1F
        rjmp    DivPo2_Common

        ; 39 cycles including rcall
        ; Stack Depth = 4


;-----------------------------------------------------------------------------


Div32_u21_RHtE:
; In: r18:r17:r16 = Dividend
; Out: r18:r17:r16 = Dividend / 32, rounded half to even
;      r18 cleared

        mov     r0, r19
        mov     r19, r16
        rcall   Div32_u21  ; 19 cycles, SD = 2
        bst     r19, 4
        andi    r19, 0x0F
DivPo2_Common:
        bld     r19, 7
        lsl     r19
        mov     r19, r16
        brne    1f
        lsr     r19
1:      adc     r16, r1
        adc     r17, r1
        ; Undo up-rounding in case of overflow.
        sbc     r16, r1
        sbc     r17, r1
        mov     r19, r0
        ; 33 cycles
        ret

        ; 40 cycles including rcall
        ; Stack Depth = 4


;-----------------------------------------------------------------------------


Div16_u24_RHtE:
; In: r18:r17:r16 = Dividend
; Out: r18:r17:r16 = Dividend / 16, rounded half to even

        mov     r0, r19
        mov     r19, r16
        rcall   Div16_u24  ; 19 cycles, SD = 2
        bst     r19, 3
        andi    r19, 0x07
        rjmp    DivPo2_Common

        ; 42 cycles including rcall
        ; Stack Depth = 4


;-----------------------------------------------------------------------------


Div8_u24_RHtE:
; In: r18:r17:r16 = Dividend
; Out: r18:r17:r16 = Dividend / 8, rounded half to even

        mov     r0, r19
        mov     r19, r16
        rcall   Div16_u24  ; 16 cycles, SD = 2
        bst     r19, 2
        andi    r19, 0x03
        rjmp    DivPo2_Common

        ; 39 cycles including rcall
        ; Stack Depth = 4


;-----------------------------------------------------------------------------


Div4_u24_RHtE:
; In: r18:r17:r16 = Dividend
; Out: r18:r17:r16 = Dividend / 4, rounded half to even

        mov     r0, r19
        mov     r19, r16
        lsr     r18
        ror     r17
        ror     r16
        lsr     r18
        ror     r17
        ror     r16
        bst     r19, 1
        andi    r19, 0x01
        rjmp    DivPo2_Common

        ; 29 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


Div_u24:
; In: r18:r17:r16 = Dividend
;     r21:r20:r19 = Divisor
; Out: r18:r17:r16 = Quotient
;      r15:r14:r13 = Remainder

        mov     r0, r22
        clr     r13
        clr     r14
        clr     r15
        ldi     r22, 25
1:      lsl     r16
2:      rol     r17
        rol     r18
        dec     r22
        breq    3f
        rol     r13
        rol     r14
        rol     r15
        cp      r13, r19
        cpc     r14, r20
        cpc     r15, r21
        brcs    1b
        sub     r13, r19
        sbc     r14, r20
        sbc     r15, r20
        sec
        rol     r16
        rjmp    2b
3:      mov     r22, r0
        ; 444 cycles at most
        ret

        ; 451 cycles at most, including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------
; Application functions
;-----------------------------------------------------------------------------


UpdateBoxFilterBuffer:
; In: Z = BFBuf object
;     r17:r16 = Unfiltered value to enter
; Out: r18:r17:r16 = Accumulated sum of buffered values

        movw    r14, r16
        ldd     r16, Z + BFBuf_Index
        ldd     r17, Z + BFBuf_MaxIndex
        inc     r16
        cp      r17, r16
        brcc    1f
        clr     r16
1:      std     Z + BFBuf_Index, r16
        push    ZH
        push    ZL
        adiw    Z, BFBufHdrSize
        lsl     r16
        adc     ZH, r1
        add     ZL, r16
        adc     ZH, r1
        ld      r16, Z+
        ld      r17, Z+
        st      -Z, r15
        st      -Z, r14
        pop     ZL
        pop     ZH
        clr     r0
        sub     r14, r16
        sbc     r15, r17
        sbc     r0, r1
        ldd     r16, Z + BFBuf_Sum + 0
        ldd     r17, Z + BFBuf_Sum + 1
        ldd     r18, Z + BFBuf_Sum + 2
        add     r16, r14
        adc     r17, r15
        adc     r18, r0
        std     Z + BFBuf_Sum + 0, r16
        std     Z + BFBuf_Sum + 1, r17
        std     Z + BFBuf_Sum + 2, r18

        ; 52 cycles
        ret

        ; 59 cycles including rcall
        ; Stack Depth = 4


;-----------------------------------------------------------------------------


ScaleADC10LARToPWMDuty:
; Scale a 10-bit ADC value (left-aligned in 16 bits) to the PWM Duty
; range, which goes up to 160.0 to represent 100% duty. The output is
; 8.8 fixed binary point format. A return value of 0x91.40 means that
; the duty cycle should exactly spend 25% of the time at 145/160 and
; 75% of the time at 146/160 and that the OCR values are 144 for 25%
; of the time and 145 for 75% of the time.
;
; Using duty times means exact scaling is easy. Only in the dithering
; stage are OCR values used.
;
; In: r17:16 = 0x0000 to 0xFFC0
; Out: r17:16 = 0x0000 to 0xA000

        mov     r12, r18
        ; Example:
        ; 0xFFC0, the highest 10-bit left-aligned ADC value
        movw  r18, r16
        lsr   r19
        ror   r18
        lsr   r19
        ror   r18
        add   r16, r18
        adc   r17, r19
        ror   r17
        ror   r16
        ; (0xFFC0 + 0x3FF0) >> 1 = 0x9FD8 ; xxxx xxxx xxxx x000
        movw  r18, r16
        lsr   r19
        ror   r18
        lsr   r19
        ror   r18
        add   r16, r19
        adc   r17, r1
        ; r17:r16:18
        ; = 0x9FD800 + (0x9FD8 >> 2)
        ; = 0x9FFFF6 ; xxxx xxxx xxxx xxxx xxxx xxx0
        lsr   r19
        lsr   r19
        add   r18, r19
        adc   r16, r1
        adc   r17, r1
        ; r17:r16:18
        ; = 0x9FFFF6 + (((0x9FD8 >> 2) >> 8) >> 2)
        ; = 0x9FFFFF
        ; Half-up rounding wil do here. For all left-aligned
        ; 10240bit ADC values half-up rounding produces the
        ; same results as round half-to-even.
        lsl   r18
        adc   r16, r1
        adc   r17, r1
        ; 25 cycles
        ret

        ; 32 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


OCRValueFromDuty:
; Subtracts 1.0 from a duty time (out of 160.0) to get an OCR value, guarding
; against the zero-duty special case of newer ATtiny85 revisions and against
; dithering between OCR values 0.0 and 1.0.
;
; In: r17:r16 = Duty time in steps of 1/160 in 8.8 fixed binary point format.
; Out: r17:r16 = OCR value and dither

        subi    r17, 1
        brcc    1f
        ldi     r16, 0
1:      adc     r17, r1
        brne    2f
        lsl     r16  ; Avoid dithering between OCR values 0 and 1!
        rol     r17
        clr     r16
9:      ; 8 cycles
        ret
2:      rjmp    9b

        ; 15 cycles includign rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


Dither:
; In: Z = DitherRec

        ldd     r16, Z + DitherRec_Target + 0
        ldd     r17, Z + DitherRec_Target + 1

        ; Add the error (compensation)
        ldd     r14, Z + DitherRec_Error + 0
        ldd     r15, Z + DitherRec_Error + 1
        add     r16, r14
        adc     r17, r15
        ; Clamp the sum of target and error to prevent wind-up.
        tst     r15
        brpl    3f
        brcs    1f
        clr     r16
        clr     r17
        rjmp    4f
1:      nop
2:      rjmp    4f
3:      brcc    2b
        clr     r17
        dec     r17
        clr     r16
4:      ; 17 cycles

        ; Suppress dithering between 0 and 1 to avoid upsetting a
        ; Mode B 4-wire fan, which might see a very low PWM value
        ; as an oscillating shutdown condition.
        tst     r17
        brne    1f
        clr     r16
1:      ; 20 cycles

        ; Perform the rounding.
        movw    r14, r16
        lsl     r14
        adc     r15, r1
        sbc     r15, r1
        std     Z + DitherRec_Dithered, r15
        ; 26 cycles

        ; The new error value is ideal + error - rounded(ideal + error).
        sub     r17, r15
        std     Z + DitherRec_Error + 0, r16
        std     Z + DitherRec_Error + 1, r17
        ; 31 cycles
        ret

        ; 38 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


IncrementCounters:
; In: Y = Globals
        ldd     r16, Y + G_Counter25kHz
        inc     r16
        ldd     r17, Y + G_Counter2500Hz
        cpi     r16, 10
        brcs    1f
        ldi     r16, 0
1:      std     Y + G_Counter25kHz, r16
        inc     r17
        sbc     r17, r1
        ldd     r16, Y + G_Counter100Hz
        cpi     r17, 25
        brcs    1f
        ldi     r17, 0
1:      std     Y + G_Counter2500Hz, r17
        inc     r16
        sbc     r16, r1
        std     Y + G_Counter100Hz, r16
        ; 23 cycles
        ret

        ; 30 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


UpdateHardwarePWMEnableStates:
; ATtiny85 chips vary in how they interpret a zero-valued OCR register.
; The older chips obey the rule that in Fast PWM Mode, the duty time is
; (OCRnx+1)/Period. In Fast Mode, Period is either 256 or OCRnC + 1.
; That means 1-count spikes are produced for an OCR value of zero.
; Newer revisions interpret a zero OCR value as a special case of a
; steady zero output (implying that 1-count spikes cannot generated
; except for an OCR value of 254 in inverting output mode).
;
; This function allows the PWM enable bits in G_Flags to control the
; hardware enable states of each of OCR1A and OCR1B so that the
; application's behaviour is the same for older and newer chips.
;
; In: Y = Globals
;     Globals.Flags[GFLAG_BIT_PWMA]
;     Globals.Flags[GFLAG_BIT_PWMB]
; Out: Globals.Flags[GFLAG_BIT_PWMA_SHUTDOWN_PENDING] modified
;      Globals.Flags[GFLAG_BIT_PWMV_SHUTDOWN_PENDING] modified
;      TCCR1.PWM1A modified
;      GTCCR.PWM1B modified

        push    r18
        in      r16, _SFR_IO_ADDR(TCCR1)
        in      r17, _SFR_IO_ADDR(GTCCR)
        ldd     r18, Y + G_Flags
        ; 6 cycles in block

        ; Check whether PWM output is to be enabled for OC1A
        sbrs    r18, GFLAG_BIT_PWMA
        rjmp    2f
        ; Enable PWM output for OC1A.
        ori     r16, (0b10 << COM1A0)
        andi    r18, ~(1 << GFLAG_BIT_PWMA_SHUTDOWN_PENDING)
        adiw    Z, 0
        adiw    Z, 0
        rjmp    3f
1:      ; Disable the PWM output right now, after one or more zero OCR cycles.
        andi    r16, ~(0b11 << COM1A0)
        andi    r18, ~(1 << GFLAG_BIT_PWMA_SHUTDOWN_PENDING)
        rjmp    3f
2:      ; Disable PWM for OC1A, sooner or later.
        sbrc    r18, GFLAG_BIT_PWMA_SHUTDOWN_PENDING
        rjmp    1b
        sbrc    r16, PWM1A
        ori     r18, 1 << GFLAG_BIT_PWMA_SHUTDOWN_PENDING
        sbrc    r16, PWM1A
        out     _SFR_IO_ADDR(OCR1A), r1
        nop
3:      ; 10 cycles in block

        ; Check whether PWM output is to be enabled for OC1B
        sbrs    r18, GFLAG_BIT_PWMB
        rjmp    2f
        ; Enable PWM output for OC1B.
        ori     r17, 0b10 << COM1B0
        andi    r18, ~(1 << GFLAG_BIT_PWMB_SHUTDOWN_PENDING)
        adiw    Z, 0
        adiw    Z, 0
        rjmp    3f
1:      ; Disable the PWM output right now, after one or more zero OCR cycles.
        andi    r17, ~(0b11 << COM1B0)
        andi    r18, ~(1 << GFLAG_BIT_PWMB_SHUTDOWN_PENDING)
        rjmp    3f
2:      ; Disable PWM for OC1B, sooner or later.
        sbrc    r18, GFLAG_BIT_PWMB_SHUTDOWN_PENDING
        rjmp    1b
        sbrc    r17, PWM1B
        ori     r18, 1 << GFLAG_BIT_PWMB_SHUTDOWN_PENDING
        sbrc    r17, PWM1B
        out     _SFR_IO_ADDR(OCR1B), r1
        nop
3:      ; 10 cycles in block

        out     _SFR_IO_ADDR(TCCR1), r16
        out     _SFR_IO_ADDR(GTCCR), r17
        std     Y + G_Flags, r18
        ; 4 cycles in block

        pop     r18
        ; 32 cycles
        ret

        ; 39 cycles including rcall
        ; Stack Depth = 3


;-----------------------------------------------------------------------------


UpdatePWMEnableFlags:
; Decide whether to enable to flag the enabling or disabling of PWM outputs
; based on zero-valued OCR values. (OCR values are one less than the duty
; time they would imply: Duty = (OCR+1)/T, where T is 256 or OCR1C+1.)
;
; The fractional part of the OCR values is ignored here, since dithering
; between OCR values 0 and 1 from an ATtiny85 revision which treats a zero
; OCR as a special zero-duty case would yield PWM signals of unacceptably
; low frequencies.
;
; In: Y = Globals
;     Globals.Flags
; Out: Globals.Flags modified

        ldd     r17, Y + G_Flags
        andi    r17, ~((1 << GFLAG_BIT_PWMA) | (1 << GFLAG_BIT_PWMB))

        ldd     r16, Y + G_OutADitherRec + DitherRec_Target + 1
        cpi     r16, 1
        brcs    1f
        ori     r17, 1 << GFLAG_BIT_PWMA
1:
        ldd     r16, Y + G_OutBDitherRec + DitherRec_Target + 1
        cpi     r16, 1
        brcs    1f
        ori     r17, 1 << GFLAG_BIT_PWMB
1:
        std     Y + G_Flags, r17
        ; 15 cycles
        ret

        ; 22 cycles including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


LowLevelThermIO:
; In: r16 = 2500Hz slot number 4..9
;     Z = Digital Thermometer IO (DTIO) record

        movw    r14, r18
        in      r19, PIN_REG_IO(UPB_THERMIO)
        ldiw    Z, ThermIO
        ldd     r17, Z + DTIO_Flags
        mov     r18, r17
        andi    r18, (\
                  (1 << DTIOFLAG_RESET_REQUEST) |\
                  (1 << DTIOFLAG_RESETTING) |\
                  (1 << DTIOFLAG_WRITE0) |\
                  (1 << DTIOFLAG_REST2) |\
                  (1 << DTIOFLAG_REST1) |\
                0)
        brne    LLThermIO_Flag  ; 10 cycles
        ldd     r18, Z + DTIO_NumBitsToWrite
        cpse    r18, r1
        rjmp    LLThermIO_Write  ; 14 cycles
        ldd     r18, Z + DTIO_NumBitsToRead
        cpse    r18, r1
        rjmp    LLThermIO_Read  ; 18 cycles
        andi    r17, ~(1 << DTIOFLAG_BUSY)
        std     Z + DTIO_Flags, r17
        rjmp    LLThermIO_Done  ; 22 cycles

LLThermIO_Flag:
        ; 10 cycles
        mov     r18, r17
        andi    r18, (\
                  (1 << DTIOFLAG_RESET_REQUEST) |\
                  (1 << DTIOFLAG_RESETTING) |\
                0)
        brne    LLThermIO_Reset  ; 14 cycles
        sbrc    r17, DTIOFLAG_REST2
        rjmp    2f  ; 16 cycles
        sbrc    r17, DTIOFLAG_REST1
        rjmp    3f  ; 18 cycles
        sbrs    r17, DTIOFLAG_WRITE0
        rjmp    4f  ; 20 cycles
1:      ; A zero is being written.
        ; Writing a zero requires a long low signal.
        ; 19 cycles
        andi    r17, ~((1 << DTIOFLAG_WRITE0) | (1 << DTIOFLAG_REST2))
        ori     r17, (1 << DTIOFLAG_REST1) | (1 << DTIOFLAG_BUSY)
        std     Z + DTIO_Flags, r17
        rjmp    LLThermIO_Done  ; 25 cycles
2:      ; Rest for two ticks.
        ; 16 cycles
        andi    r17, ~((1 << DTIOFLAG_WRITE0) | (1 << DTIOFLAG_REST2))
        ori     r17, (1 << DTIOFLAG_REST1) | (1 << DTIOFLAG_BUSY)
        std     Z + DTIO_Flags, r17
        rjmp    LLThermIO_Done  ; 22 cycles
3:      ; Rest for one tick.
        ; 18 cycles
        cbi     DDR_REGBIT_IO(UPB_THERMIO)
        andi    r17, ~(\
                  (1 << DTIOFLAG_WRITE0) |\
                  (1 << DTIOFLAG_REST2)  |\
                  (1 << DTIOFLAG_REST1)  |\
                0)
        ; Fall through, since the required rest will have
        ; been taken when the next iteration is executed.
        ; 20 cycles
4:      ; See if the BUSY flag can be cleared.
        ; 20 cycles
        mov     r18, r17
        andi    r18, (\
                  (1 << DTIOFLAG_RESET_REQUEST) |\
                  (1 << DTIOFLAG_RESETTING) |\
                  (1 << DTIOFLAG_WRITE0) |\
                  (1 << DTIOFLAG_REST2) |\
                  (1 << DTIOFLAG_REST1) |\
                0)
        brne    1f
        ldd     r18, Z + DTIO_NumBitsToWrite
        ldd     r0, Z + DTIO_NumBitsToRead
        or      r18, r0
        brne    1f
        andi    r17, ~(1 << DTIOFLAG_BUSY)
1:      std     Z + DTIO_Flags, r17
        rjmp    LLThermIO_Done  ; 34 cycles at most

LLThermIO_Reset:
        ; See if a reset is requested. Retriggering is permitted, though
        ; a 10-tick initial rest period will ensure the reset pulse is not
        ; lengthened by retriggering.
        ; 14 cycles
        sbrs    r17, DTIOFLAG_RESET_REQUEST
        rjmp    2f  ; 17 cycles
        ; A reset request! Begin resetting when the time is convenient.
        cpi     r16, 4
        breq    1f
        rjmp    LLThermIO_Done  ; 20 cycles
1:      ; Begin the reset procedure, starting with an enforced nap.
        ; 20 cycles
        ldi     r17, (1 << DTIOFLAG_BUSY) | (1 << DTIOFLAG_RESETTING)
        cbi     DDR_REGBIT_IO(UPB_THERMIO)
        ldi     r18, 29
        std     Z + DTIO_ByteIx, r18  ; Repurpose as reset sequence state.
        std     Z + DTIO_BitMask, r2
        std     Z + DTIO_CRC, r1
        std     Z + DTIO_Flags, r17
        rjmp    LLThermIO_Done  ; 33 cycles
2:      ; Resetting is in progress.
        ;
        ; T4 29:1   0us  23:1 400us  17:0 280us  11:1 680us   5:1 400us
        ; T5 28:1  40us  22:1 440us  16:0 320us  10:R  40us   4:1 440us
        ; T6 27:1  80us  21:1 480us  15:0 360us   9:R  80us   3:1 480us
        ; T7 26:1 120us  20:0   0us  14:0 400us   8:R 120us   2:1 520us
        ; T8 25:1 160us  19:0  40us  13:0 440us   7:R 160us   1:1 560us
        ; T9 24:1 200us  18:0  80us  12:0 480us   6:1 200us   0:1 600us
        ;
        ; 17 cycles
        ldd     r18, Z + DTIO_ByteIx  ; Repurposed as reset counter
        sub     r18, r2
        adc     r18, r1
        std     Z + DTIO_ByteIx, r18
        cpi     r18, 20
        brne    1f
        sbi     DDR_REGBIT_IO(UPB_THERMIO)  ; Begin low pulse.
1:      cpi     r18, 12
        brcc    2f  ; 29 cycles
        cbi     DDR_REGBIT_IO(UPB_THERMIO)  ; End low pulse.
        cpi     r18, 11
        brcc    2f  ; 32 cycles
        cpi     r18, 7
        brcs    2f  ; 34 cycles
        ; Read the presence bit. The timing from the DS18B20 at this
        ; stage is highly variable, so multiple reads are required,
        sbrs    r19, PBIT(UPB_THERMIO)      ; Read presence ("0") bit.
        ori     r17, 1 << DTIOFLAG_PRESENT
        sbis    PIN_REGBIT_IO(UPB_THERMIO)
        ori     r17, 1 << DTIOFLAG_PRESENT
2:      ; 38 cycles at most
        tst     r18
        brne    1f
        ; Reset complete!
        andi    r17, ~(1 << DTIOFLAG_RESETTING)
        ; Clear the BUSY flag if there was no presence detected.
        sbrs    r17, DTIOFLAG_PRESENT
        andi    r17, ~(1 << DTIOFLAG_BUSY)
        ; The BUSY flag is not cleared if presence is detected and
        ; there is data to read or write.
        ldd     r18, Z + DTIO_NumBitsToWrite
        ldd     r0, Z + DTIO_NumBitsToRead
        or      r18, r0
        brne    1f
        andi    r17, ~(1 << DTIOFLAG_BUSY)
1:      ; 50 cycles at most
        std     Z + DTIO_Flags, r17
        rjmp    LLThermIO_Done  ; 54 cycles at most

LLThermIO_Write:
        ; 14 cycles
        cpi     r16, 8
        brcc    9f  ; Not enough time for a write cycle!
        ; Begin the long or short low pulse.
        sbi     DDR_REGBIT_IO(UPB_THERMIO)
.if USE_16MHZ_CK
        ldi     r16, 24/3
1:      dec     r16
        brne    1b
.endif
        ; Fetch the next bit to read.
        movw    X, Z
        adiw    X, DTIO_Buffer
        ldd     r18, Z + DTIO_ByteIx
        cpi     r18, DTIO_BUFFER_SIZE
        brcs    1f
        ldi     r18, 0
1:      add     XL, r18
        adc     XH, r1
        ori     r17, (1 << DTIOFLAG_WRITE0) | (1 << DTIOFLAG_BUSY)
        andi    r17, ~(1 << DTIOFLAG_REST2)
        ldi     r16, (1 << DTIOFLAG_REST2) | (1 << DTIOFLAG_WRITE0)
        ld      r0, X
        ldd     r19, Z + DTIO_BitMask
        and     r0, r19
        breq    1f
        eor     r17, r16
1:      std     Z + DTIO_Flags, r17
        sbrs    r17, DTIOFLAG_WRITE0
        cbi     DDR_REGBIT_IO(UPB_THERMIO)
        ; The low pulse ended low pulse after 24 cycles (48 if 16MHz PLL used)
        ; Advance to the next bit.
        ; 41/65 cycles
        mov     r0, r19
        lsl     r0
        rol     r19
        brcc    1f
        inc     r18
1:      cpi     r18, DTIO_BUFFER_SIZE
        brcs    1f
        ldi     r18, 0
1:      std     Z + DTIO_BitMask, r19
        std     Z + DTIO_ByteIx, r18
        ldd     r18, Z + DTIO_NumBitsToWrite
        sub     r18, r2
        adc     r18, r1
        std     Z + DTIO_NumBitsToWrite, r18
        rjmp    LLThermIO_Done  ; 61/85 cycles
9:      rjmp    LLThermIO_BusyDone  ; 19 cycles

LLThermIO_Read:
        ; 18 cycles
        cpi     r16, 8
        brcc    LLThermIO_BusyDone  ; Not enough time for a read cycle!
        ; Begin a 24 or 48 cycle low pulse.
        sbi     DDR_REGBIT_IO(UPB_THERMIO)
.if USE_16MHZ_CK
        ldi     r16, 4 + 24/3
.else
        ldi     r16, 4
.endif
1:      dec     r16
        brne    1b
        ; 33 cycles
        ldd     r18, Z + DTIO_NumBitsToRead
        sub     r18, r2
        adc     r18, r1
        std     Z + DTIO_NumBitsToRead, r18
        movw    X, Z
        adiw    X, DTIO_Buffer
        ldd     r18, Z + DTIO_ByteIx
        cbi     DDR_REGBIT_IO(UPB_THERMIO)
        ; The low pulse ended low pulse after 24 cycles (48 if 16MHz PLL used)
        ; 45/69 cycles
        ; Rest for 72 cycles (144 if 16MHz PLL used)
        ; Do some work during this rest.
        cpi     r18, DTIO_BUFFER_SIZE
        brcs    1f
        ldi     r18, 0
1:      add     XL, r18
        adc     XH, r1
        ld      r16, X
        ldd     r19, Z + DTIO_BitMask
        ; The bit to be read will be stored where X is presently pointing,
        ; according to the 1-bit mask in r19.
        ; Advance the bit mask and the byte index for the next bit to be
        ; read while keeping access to the current bit to be read.
        ; 9 cycles since the start of the rest
        ; 54/78 cycles
        ori     r17, (1 << DTIOFLAG_REST2) | (1 << DTIOFLAG_BUSY)
        mov     r0, r19
        lsl     r0
        mov     r0, r19
        rol     r0
        std     Z + DTIO_BitMask, r0
        adc     r18, r1
        cpi     r18, DTIO_BUFFER_SIZE
        brcs    1f
        ldi     r18, 0
1:      std     Z + DTIO_ByteIx, r18
        ; 22 cycles since the start of the rest
        ; 67/91 cycles
.if USE_16MHZ_CK
        ldi     r18, 15 + 72/3
.else
        ldi     r18, 15
.endif
1:      dec     r18
        brne    1b
        ; 112/208 cycles
        com     r19
        and     r16, r19
        com     r19
        andi    r17, ~(1 << DTIOFLAG_DATA)
        sbic    PIN_REGBIT_IO(UPB_THERMIO)
        ; 72 cycles after start of rest (144 if 16Mhz PLL used)
        ; 117/213 cycles
        ori     r17, 1 << DTIOFLAG_DATA
        sbrs    r17, DTIOFLAG_DATA
        clr     r19
        or      r16, r19
        st      X, r16
        ; Update the CRC (without requiring eight zeroes to be appended)
        ldd     r16, Z + DTIO_CRC
        lsr     r16
        sbc     r18, r18
        sbrc    r17, DTIOFLAG_DATA
        com     r18
        andi    r18, 0x8C
        eor     r16, r18
        std     Z + DTIO_CRC, r16
        ; 16 cycles after reading of bit from DS18B20
        ; 133/229 cycles
        ; Fall through to LLThermIO_BusyDone.
LLThermIO_BusyDone:
        ; 133/229 cycles at most
        ori     r17, 1 << DTIOFLAG_BUSY
        std     Z + DTIO_Flags, r17
        ; Fall through to LLThermIO_Done.
LLThermIO_Done:
        movw    r18, r14
        ; 137/233 cycles at most
        ret

        ; 144 cycles at most, including rcall (240 if 16MHz PLL used)
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


AdvanceThermometerState:
; In: Y = Globals
;     Z = Digital Thermometer IO (DTIO) record

        ldd     r16, Y + G_ThermometerState
        ldd     r17, Z + DTIO_Flags
        cpi     r16, 3
        breq    ATS_Reading       ; State 3 (reading)
        brcc    1f                ; State 4+
        cpi     r16, 1
        breq    ATS_Start         ; State 1 (start)
        brcs    9f                ; State 0 (idle)
        rjmp    ATS_Converting    ; State 2 (converting)
1:      cpi     r16, 5
        brcc    2f
        rjmp    ATS_Configuring   ; State 4 (configuring)
2:      ; Terminal state: Completed, failed or invalid state
        cpi     r16, DTSTATE_COMPLETED
        breq    9f
        ldi     r16, DTSTATE_FAILED
        std     Y + G_ThermometerState, r16
9:      ret

ATS_Start:
        ; 10 or 21 cycles
        sbrc    r17, DTIOFLAG_BUSY
        rjmp    ATS_Done
        ldi     r17, (1 << DTIOFLAG_RESET_REQUEST) | (1 << DTIOFLAG_BUSY)
        std     Z + DTIO_Flags, r17
        ldi     r16, 0xCC  ; Skip ROM
        std     Z + DTIO_Buffer + 0, r16
        ldi     r16, 0x44  ; Begin temperature ADC conversion
        std     Z + DTIO_Buffer + 1, r16
        ldi     r16, 16
        std     Z + DTIO_NumBitsToWrite, r16
        std     Z + DTIO_NumBitsToRead, r2
        ldi     r16, DTSTATE_CONVERTING
        std     Y + G_ThermometerState, r16
        rjmp    ATS_Done  ; 31 or 42 cycles

ATS_Converting:
        ; 12 cycles
        sbrc    r17, DTIOFLAG_BUSY
        rjmp    ATS_Done
        sbrs    r17, DTIOFLAG_PRESENT
        rjmp    ATS_NoPresence  ; 17 cycles
        sbrs    r17, DTIOFLAG_DATA
        rjmp    ATS_Wait  ; 19 cycles
        ; Conversion is complete. Now to read the DS18B20 Scratchpad.
        ldi     r17, (1 << DTIOFLAG_RESET_REQUEST) | (1 << DTIOFLAG_BUSY)
        std     Z + DTIO_Flags, r17
        ldi     r16, 0xCC  ; Skip ROM
        std     Z + DTIO_Buffer + 0, r16
        ldi     r16, 0xBE  ; Read Scratchpad
        std     Z + DTIO_Buffer + 1, r16
        ldi     r16, 16
        std     Z + DTIO_NumBitsToWrite, r16
        ldi     r16, 72
        std     Z + DTIO_NumBitsToRead, r16
        ; Example of output from Read Scratchpad (0xBE) 50057F807FFF0C1093
        ; -> Temp = 85.0 degC, TH=127, TL=-128, Res=12-bit, xxxxxx, CRC = 0x93
        ; The 2's complement temperature in 16ths of a degree Celsius is
        ; stored in bytes 0 (LSB) and 1 (MSB). Byte 2 is the high temperature
        ; and byte 3 is the low temperature limit. The resolution index bits
        ; R1 and R0 are in bits 6 and 5 of byte 4 (which defaults to 0x7F).
        ; R1:R0 indexes 9-bit, 10-bit, 11-bit and 12-bit resolutions.
        ldi     r16, DTSTATE_READING
        std     Y + G_ThermometerState, r16
        rjmp    ATS_Done  ; 38 cycles

ATS_Reading:
        ; 7 cycles
        sbrc    r17, DTIOFLAG_BUSY
        rjmp    ATS_Done
        sbrs    r17, DTIOFLAG_PRESENT
        rjmp    ATS_NoPresence  ; 12 cycles
        ldd     r16, Z + DTIO_CRC
        tst     r16
        brne    ATS_BadCRC  ; 16 cycles
        ldd     r0, Y + G_TemperatureAdj
        ldd     r16, Z + DTIO_Buffer + 2 + 0
        add     r16, r0  ; Signed 4.4 format
        std     Y + G_Temperature + 0, r16
        ldd     r16, Z + DTIO_Buffer + 2 + 1
        adc     r16, r1
        sbrc    r0, 7  ; Sign extention
        dec     r16
        std     Y + G_Temperature + 1, r16
        ldd     r0, Z + DTIO_Buffer + 2 + 4
        mov     r16, r0
        andi    r16, ~(0b11 << 5)
        ori     r16, 0b11 << 5  ; Desired resolution index 0b11 -> 12 bits
        cp      r0, r16
        brne    ATS_Configure  ; 37 cycles
        ldi     r16, DTSTATE_COMPLETED
        std     Y + G_ThermometerState, r16
        std     Y + G_ThermErrCount, r1
        rjmp    ATS_Done  ; 43 cycles

ATS_Wait:
        ; 19 cycles
        ori     r17, 1 << DTIOFLAG_BUSY
        std     Z + DTIO_Flags, r17
        std     Z + DTIO_NumBitsToRead, r2
        rjmp    ATS_Done  ; 26 cycles

ATS_NoPresence:
ATS_BadCRC:
        ; 17 cycles at most
        ldi     r16, DTSTATE_FAILED
        std     Y + G_ThermometerState, r16
        ldd     r16, Y + G_ThermErrCount
        inc     r16
        sbrs    r16, 3
        ldi     r16, 0x87  ; Bit 7 set means "too many errors to count."
        std     Y + G_ThermErrCount, r16
        ; 27 cycles
        ; Fall through to ATS_Done.

ATS_Done:
        ; 64 cycles at most
        ret

ATS_Configure:
; In: r16 = New configuration
        ; 37 cycles
        std     Z + DTIO_Buffer + 2 + 2, r16  ; Configuration
        ldi     r16, 127  ; Default
        std     Z + DTIO_Buffer + 2 + 0, r16  ; High temperature limit
        ldi     r16, -128  ; Default
        std     Z + DTIO_Buffer + 2 + 1, r16  ; Low temperature limit
        ldi     r17, (1 << DTIOFLAG_RESET_REQUEST) | (1 << DTIOFLAG_BUSY)
        std     Z + DTIO_Flags, r17
        ldi     r16, 0xCC  ; Skip ROM
        std     Z + DTIO_Buffer + 0, r16
        ldi     r16, 0x4E  ; Write Scratchpad
        std     Z + DTIO_Buffer + 1, r16
        ldi     r16, 16 + 24
        std     Z + DTIO_NumBitsToWrite, r16
        std     Z + DTIO_NumBitsToRead, r1
        ldi     r16, DTSTATE_CONFIGURING
        std     Y + G_ThermometerState, r16
        rjmp    ATS_Done  ; 64 cycles

ATS_Configuring:
        ; 12 cycles
        sbrc    r17, DTIOFLAG_BUSY
        rjmp    ATS_Done
        sbrs    r17, DTIOFLAG_PRESENT
        rjmp    ATS_NoPresence
        ; There is no need to wear out the DS18B20's EEPROM saving config.
        ; Just restart the temperature ADC conversion from here.
        ; The desired configuration will persist through resets while
        ; there is still power supplied to the DS18820.
        ldi     r16, DTSTATE_START
        std     Y + G_ThermometerState, r16
        rjmp    ATS_Start  ; 21 cycles

        ; 71 cycles at most, including rcall
        ; Stack Depth = 2


;-----------------------------------------------------------------------------


LoadSpeedFnParamsFromEEPROM:
; in: Z = SpeedFn structure

        movw    r14, Z
        movw    r12, X
        movw    X, Z
        adiw    X, SpeedFn_LowT
        ldiw    Z, EEPROM_LowT
        ldi     r16, 3
        rcall   ReadEEPROMBlock  ; 1 + 24n cycles if EEPROM ready
        movw    Z, r14
        movw    X, r12
        ; 83 cycles if EEPROM ready
        ret

        ; 90 cycles at most, if EEPROM ready
        ; Stack Depth = 6


;-----------------------------------------------------------------------------


LoadSpeedFnCurveFromEEPROM:
; in: Z = SpeedFn structure

        movw    r14, Z
        movw    r12, X
        movw    X, Z
        adiw    X, SpeedFn_Curve
        ldiw    Z, EEPROM_SpeedCurve
        ldi     r16, NUM_FSCURVE_POINTS
        rcall   ReadEEPROMBlock  ; 1 + 24n cycles if EEPROM ready
        movw    Z, r14
        movw    X, r12
        ; 203 cycles if EEPROM ready
        ret

        ; 210 cycles at most, if EEPROM ready
        ; Stack Depth = 6


;-----------------------------------------------------------------------------


FinishSpeedFnSetup:
; in: Z = SpeedFn structure

        push    r18
        push    r19
        push    r20
        push    r21
        ldi     r16, FSFSTATE_IDLE
        std     Z + SpeedFn_State, r16
        ; Validate the speed percentage.
        ldd     r16, Z + SpeedFn_HighSpeedPct
        cpi     r16, 100
        brcs    1f
        ldi     r16, 100
1:      std     Z + SpeedFn_HighSpeedPct, r16
        ; 18 cycles
        ; Validate the temperature range.
        ldd     r16, Z + SpeedFn_LowT
        ldd     r17, Z + SpeedFn_HighT
        cp      r17, r16
        brge    1f
        mov     r16, r17  ; Force LowT to equal HighT.
1:      std     Z + SpeedFn_LowT, r16
        std     Z + SpeedFn_HighT, r17
        brne    1f
        ; HighT is equal to LowT. This is permitted, but
        ; the curve index scale factor needs to be zero.
        std     Z + SpeedFn_T2SCIScale + 0, r1
        std     Z + SpeedFn_T2SCIScale + 1, r1
        std     Z + SpeedFn_T2SCIScale + 2, r1
        rjmp    2f
1:      ; 31 cycles at most
        ; Calculate the scale factor from 12.4 format temperature to the
        ; 8.8 format piecewise linear function table index. The divisor,
        ; r21:r20:r19, is the temperature range in whole degrees Celsius.
        mov     r19, r17
        sub     r19, r16
        clr     r20
        clr     r21
        ; The dividend is set such that the quotient divided by 1<<24
        ; is the temperature to speed curve index scale factor. That
        ; factor, when multiplied by the temperature offset in 12.4
        ; format, yields a curve index in 8.8 format.
        ldi     r18, ((NUM_FSCURVE_POINTS) - 1) << 4
        ldi     r17, 0
        ldi     r16, 0
        ; 38 cycles at most
        rcall   Div_u24  ; 451 cycles at most, SD = 2
        ; Half-up rounding will suffice.
        ; See if the remainder meets or exceeds half the dividend.
        ldi     r19, (((NUM_FSCURVE_POINTS) - 1) << (20 - 1)) & 0xFF
        sub     r13, r19
        ldi     r19, ((((NUM_FSCURVE_POINTS) - 1) << (20 - 1)) >> 8) & 0xFF
        sbc     r14, r19
        ldi     r19, ((((NUM_FSCURVE_POINTS) - 1) << (20 - 1)) >> 16) & 0xFF
        sbc     r15, r19
        sbc     r15, r15
        com     r15
        lsl     r15
        adc     r16, r1
        adc     r17, r1
        adc     r18, r1
        ; A temperature range of 1..255 imples a scale factor of
        ; 0x7.00000..0x0.07070 in 4.20 format.
        std     Z + SpeedFn_T2SCIScale + 0, r16
        std     Z + SpeedFn_T2SCIScale + 1, r17
        std     Z + SpeedFn_T2SCIScale + 2, r18
2:      ; 502 cycles at most
        ; Calculate the PWM scale factor given the percentage scale
        ldiw    r17:r16, ((((160<<(25-2))/(255*100>>2)+1)>>1)) & 0xFFFF
        ldd     r18, Z + SpeedFn_HighSpeedPct
        ldi     r19, 0
        rcall   Mult_u16  ; 125 cycles at most, SD = 2
        movw    r12, r16
        ldd     r16, Z + SpeedFn_HighSpeedPct
        ldi     r17, ((((160<<(25-2))/(255*100>>2)+1)>>1)) >> 16
        rcall   Mult_u8  ; 42 cycles at most, SD = 2
        add     r18, r16
        adc     r19, r17
        lsl     r12
        adc     r13, r1
        adc     r18, r1
        ; No need for "adc r19, r1"
        std     Z + SpeedFn_S2PWMScale + 0, r13
        std     Z + SpeedFn_S2PWMScale + 1, r18
        ; 692 cycles at most
        pop     r21
        pop     r20
        pop     r19
        pop     r18
        ; 700 cycles at most
        ret

        ; 707 cycles at most, including rcall
        ; Stack Depth = 8


;-----------------------------------------------------------------------------


AdvanceFanSpeedFn:
; In: Z = SpeedFn

        ldd     r16, Z + SpeedFn_State
        sbrc    r16, 3
        rjmp    3f
        sbrc    r16, 2
        rjmp    2f
        ; 6 cycles
        sbrc    r16, 1
        rjmp    1f
        sbrs    r16, 0
        rjmp    9f            ; 11 cycles
        rjmp    AFSF_Start    ; 12 cycles
1:      sbrs    r16, 0
        rjmp    AFSF_TScale1  ; 12 cycles
        rjmp    AFSF_TScale2  ; 13 cycles
2:      ; 7 cycles
        sbrc    r16, 1
        rjmp    1f
        sbrs    r16, 0
        rjmp    AFSF_TScale3  ; 12 cycles
        rjmp    AFSF_Lerp     ; 13 cycles
1:      sbrs    r16, 0
        rjmp    AFSF_SScale1  ; 13 cycles
        rjmp    AFSF_SScale2  ; 14 cycles
3:      ; Terminal state: Either Complete or an invalid state
        ; 5 cycles
        cpi     r16, FSFSTATE_COMPLETE
        breq    9f
        ldi     r16, FSFSTATE_IDLE
        std     Z + SpeedFn_State, r16
9:      ret


AFSF_Start:
; 12 cycles
        movw    r14, r18
        ldd     r16, Z + SpeedFn_Temperature + 0
        ldd     r17, Z + SpeedFn_Temperature + 1
        ldd     r18, Z + SpeedFn_HighT
        swap    r18
        mov     r19, r18
        andi    r18, 0xF0
        andi    r19, 0x0F
        sbrc    r19, 3
        ori     r19, 0xF0
        cp      r16, r18
        cpc     r17, r19
        brlt    1f
        movw    r16, r18  ; Guaranteed to never fall short after scaling
1:      ldd     r18, Z + SpeedFn_LowT
        swap    r18
        mov     r19, r18
        andi    r18, 0xF0
        andi    r19, 0x0F
        sbrc    r19, 3
        ori     r19, 0xF0
        sub     r16, r18
        sbc     r17, r19
        brge    1f
        clr     r16
        clr     r17
1:      std     Z + SpeedFn_SCIndex + 0, r16  ; Repurposed as rel T
        std     Z + SpeedFn_SCIndex + 1, r17  ; Repurposed as rel T
        ldi     r16, FSFSTATE_TSCALE1
        std     Z + SpeedFn_State, r16
        movw    r18, r14
        ; 50 cycles at most
        ret

AFSF_TScale1:
; 12 cycles
        ; Begin the multiplication of the 12.4 format temperature relative
        ; to the minimum temperature by the 4.20 format Temperature to
        ; Speed Curve Index Scale calculated during setup.
        ldd     r16, Z + SpeedFn_SCIndex + 0  ; Repurposed as rel T[7:0]
        ldd     r17, Z + SpeedFn_T2SCIScale + 0
        rcall   Mult_u8  ; 42 cycles, SD = 2
        movw    r14, r16
        ldd     r16, Z + SpeedFn_SCIndex + 1  ; Repurposed as rel T[15:8]
        ldd     r17, Z + SpeedFn_T2SCIScale + 0
        rcall   Mult_u8  ; 42 cycles, SD = 2
        add     r16, r15
        adc     r17, r1
        std     Z + SpeedFn_PartialProduct + 1, r16
        std     Z + SpeedFn_PartialProduct + 2, r17
        ; Bits 32 to 39 of the product is always zero.
        ldi     r16, FSFSTATE_TSCALE2
        std     Z + SpeedFn_State, r16
        ; 114 cycles
        ret

AFSF_TScale2:
; 13 cycles
        ; Continue the calculation of the speed curve index.
        ldd     r16, Z + SpeedFn_SCIndex + 0  ; Repurposed as rel T[7:0]
        ldd     r17, Z + SpeedFn_T2SCIScale + 1
        rcall   Mult_u8  ; 42 cycles, SD = 2
        movw    r14, r16
        ldd     r16, Z + SpeedFn_SCIndex + 1  ; Repurposed as rel T[15:8]
        ldd     r17, Z + SpeedFn_T2SCIScale + 1
        rcall   Mult_u8  ; 42 cycles, SD = 2
        add     r16, r15
        adc     r17, r1
        ldd     r15, Z + SpeedFn_PartialProduct + 1
        add     r14, r15
        ldd     r15, Z + SpeedFn_PartialProduct + 2
        adc     r16, r15
        adc     r17, r1
        std     Z + SpeedFn_PartialProduct + 0, r14
        std     Z + SpeedFn_PartialProduct + 1, r16
        std     Z + SpeedFn_PartialProduct + 2, r17
        ldi     r16, FSFSTATE_TSCALE3
        std     Z + SpeedFn_State, r16
        ; 124 cycles at most
        ret

AFSF_TScale3:
; 12 cycles
        ; Complete the calculation of the speed curve index.
        ldd     r16, Z + SpeedFn_SCIndex + 0  ; Repurposed as rel T[7:0]
        ldd     r17, Z + SpeedFn_T2SCIScale + 2
        rcall   Mult_u8  ; 42 cycles, SD = 2
        movw    r14, r16
        ldd     r16, Z + SpeedFn_SCIndex + 1  ; Repurposed as rel T[15:8]
        ldd     r17, Z + SpeedFn_T2SCIScale + 2
        rcall   Mult_u8  ; 42 cycles, SD = 2
        add     r16, r15
        adc     r17, r1
        ldd     r15, Z + SpeedFn_PartialProduct + 1
        add     r14, r15
        ldd     r15, Z + SpeedFn_PartialProduct + 2
        adc     r16, r15
        ldd     r15, Z + SpeedFn_PartialProduct + 0
        ; Apply rounding. Round half up will do.
        lsl     r15
        adc     r14, r1
        adc     r16, r1
        ; There is no need to check for overshoot. The highest temperature
        ; always results in the index for the last curve table entry and
        ; a lerp parameter of zero.
        std     Z + SpeedFn_SCIndex + 0, r14  ; Curve (LPF) table index
        std     Z + SpeedFn_SCIndex + 1, r16  ; Lerp parameter
        ldi     r16, FSFSTATE_LERP
        std     Z + SpeedFn_State, r16
        ; 125 cycles
        ret

AFSF_Lerp:
; 13 cycles
        ldd     r16, Z + SpeedFn_SCIndex + 0
        ldd     r17, Z + SpeedFn_SCIndex + 1
        cpi     r17, NUM_FSCURVE_POINTS - 1
        brcs    1f
        ldi     r17, NUM_FSCURVE_POINTS - 1
        ldi     r16, 0
1:      add     ZL, r17
        adc     ZH, r1
        ldd     r14, Z + SpeedFn_Curve + 0
        mov     r15, r14
        cpse    r16, r1
        ldd     r15, Z + SpeedFn_Curve + 1
        sub     ZL, r17
        sbc     ZH, r1
        ; 41 cycles at most
        ; Lerp r14 to r15 by parameter r16
        mov     r17, r15
        sub     r17, r14
        brcs    1f
        rcall   Mult_u8  ; 42 cycles, SD = 2
        add     r17, r14
        rjmp    2f
1:      neg     r17
        rcall   Mult_u8  ; 42 cycles, SD = 2
        com     r16
        com     r17
        add     r16, r2
        adc     r17, r14
2:      std     Z + SpeedFn_LerpedValue + 0, r16
        std     Z + SpeedFn_LerpedValue + 1, r17
        ldi     r16, FSFSTATE_SSCALE1
        std     Z + SpeedFn_State, r16
        ; 89 cycles at most
        ret

AFSF_SScale1:
; 13 cycles
        ldd     r16, Z + SpeedFn_LerpedValue + 0
        ldd     r17, Z + SpeedFn_S2PWMScale + 0
        rcall   Mult_u8  ; 42 cycles, SD = 2
        movw    r14, r16
        ldd     r16, Z + SpeedFn_LerpedValue + 1
        ldd     r17, Z + SpeedFn_S2PWMScale + 0
        rcall   Mult_u8  ; 42 cycles, SD = 2
        add     r16, r15
        adc     r17, r1
        ; r17:r16:r14 = LerpedValue * S2PWMScale[7:0]
        std     Z + SpeedFn_PartialProduct + 0, r16
        std     Z + SpeedFn_PartialProduct + 1, r17
        ldi     r16, FSFSTATE_SSCALE2
        std     Z + SpeedFn_State, r16
        ; 115 cycles
        ret

AFSF_SScale2:
; 14 cycles
        ldd     r16, Z + SpeedFn_LerpedValue + 0
        ldd     r17, Z + SpeedFn_S2PWMScale + 1
        rcall   Mult_u8  ; 42 cycles, SD = 2
        movw    r14, r16
        ldd     r16, Z + SpeedFn_LerpedValue + 1
        ldd     r17, Z + SpeedFn_S2PWMScale + 1
        rcall   Mult_u8  ; 42 cycles, SD = 2
        add     r16, r15
        adc     r17, r1
        ; r17:r16:r14 = LerpedValue * S2PWMScale[15:8]
        ldd     r15, Z + SpeedFn_PartialProduct + 0
        add     r14, r15
        ldd     r15, Z + SpeedFn_PartialProduct + 1
        adc     r16, r15
        adc     r17, r1
        ; r17:r16:r14 = (LerpedValue * S2PWMScale) >> 8
        ; Apply rounding. Round half up will do.
        lsl     r14
        adc     r16, r1
        adc     r17, r1
        std     Z + SpeedFn_PWMDuty + 0, r16
        std     Z + SpeedFn_PWMDuty + 1, r17
        ldi     r16, FSFSTATE_COMPLETE
        std     Z + SpeedFn_State, r16
        ; 126 cycles at most
        ret

        ; 133 cycles at most, including rcall
        ; Stack Depth = 4


;-----------------------------------------------------------------------------
; main
;-----------------------------------------------------------------------------


main:

        cli
        wdr

        ; First, read and clear the Watchdog Reset Flag
        in      r16, _SFR_IO_ADDR(MCUSR)
        ; Store the reset flags in a place that won't be erased by the
        ; routine which clears the memory used by variables.
        sts     InitialMCUSR, r16
        clr     r1
        out     _SFR_IO_ADDR(MCUSR), r1
        rcall   InitialiseRegisterConstants
        rcall   InitialiseWatchDogTimer
        rcall   ClearGlobals
        rcall   InitialiseHardware

        ldiw    Z, EEPROM_TempAdj
        rcall   ReadEEPROMByte
        std     Y + G_TemperatureAdj, r0
        ldiw    Z, SpeedFn
        rcall   LoadSpeedFnParamsFromEEPROM
        rcall   LoadSpeedFnCurveFromEEPROM
        rcall   FinishSpeedFnSetup

        ldi     r16, ADC_BF_BUFFER_LENGTH - 1
        sts     ADCBoxFBuf + BFBuf_MaxIndex, r16
        ldi     r16, INPUT_BF1_BUFFER_LENGTH - 1
        sts     InBoxFBuf1 + BFBuf_MaxIndex, r16
        ldi     r16, INPUT_BF2_BUFFER_LENGTH - 1
        sts     InBoxFBuf2 + BFBuf_MaxIndex, r16
        ldi     r16, OUTA_BF1_BUFFER_LENGTH - 1
        sts     OutABoxFBuf1 + BFBuf_MaxIndex, r16
        ldi     r16, OUTA_BF2_BUFFER_LENGTH - 1
        sts     OutABoxFBuf2 + BFBuf_MaxIndex, r16
        ldi     r16, OUTB_BF1_BUFFER_LENGTH - 1
        sts     OutBBoxFBuf1 + BFBuf_MaxIndex, r16
        ldi     r16, OUTB_BF2_BUFFER_LENGTH - 1
        sts     OutBBoxFBuf2 + BFBuf_MaxIndex, r16

        std     Y + G_ThermErrCount, r3

        ldi     r16, (\
                  (1 << GFLAG_BIT_PWMA) |\
                  (1 << GFLAG_BIT_PWMB) |\
                0)
        std     Y + G_Flags, r16

.if 0
    ; Test the Fan Speed Function without
    ; mucking about with the EEPROM.
    ldiw    Z, SpeedFn
    ldi     r16, 0xD8
    std     Y + G_TemperatureAdj, r16
    ldi     r16, 0x12
    ldi     r17, 0x1C
    std     Z + SpeedFn_LowT, r16
    std     Z + SpeedFn_HighT, r17
    ldi     r18, 0x64
    std     Z + SpeedFn_HighSpeedPct, r18
    rcall   FinishSpeedFnSetup
    ldiw    r17:r16, 0x0153
    std     Z + SpeedFn_Temperature + 0, r16
    std     Z + SpeedFn_Temperature + 1, r17
  .if 1
      ldi     r16, 0x02
      ldi     r17, 0x04
      ldi     r18, 0x08
      ldi     r19, 0x10
      ldi     r20, 0x20
      ldi     r21, 0x40
      ldi     r22, 0x80
      ldi     r23, 0xFF
  .else
      ldi     r16, 0xFF
      ldi     r17, 0x80
      ldi     r18, 0x40
      ldi     r19, 0x10
      ldi     r20, 0x40
      ldi     r21, 0xFF
      ldi     r22, 0x08
      ldi     r23, 0xFF
  .endif
    ldiw    X, SpeedFn + SpeedFn_Curve
    st      X+, r16
    st      X+, r17
    st      X+, r18
    st      X+, r19
    st      X+, r20
    st      X+, r21
    st      X+, r22
    st      X+, r23
.endif

        ; Enable interrupts
        sei

main_Loop:
; 0 cycles:
;
; There must be fewer than 320 cycles in the loop if the counters are
; to be accurate and the PWM dithering is to be free of errors.

1:      sbis    _SFR_IO_ADDR(GPIOR0), 0
        rjmp    1b
        cbi     _SFR_IO_ADDR(GPIOR0), 0
        ; 14 cycles for interrupt servicing and detection

;1:      lds     r16, ISRData + ISRData_Serviced
;        tst     r16
;        breq    1f
;        sts     ISRData + ISRData_Serviced, r1

        ; 25kHz tasks

        rcall   IncrementCounters  ; 30 cycles

        ldiw    Z, Globals + G_OutADitherRec
        rcall   Dither  ; 38 cycles
        ldd     r16, Y + G_OutADitherRec + DitherRec_Dithered
        ldd     r18, Y + G_Flags
        sbrc    r18, GFLAG_BIT_PWMA
        out     _SFR_IO_ADDR(OCR1A), r16
        ; 46 cycles in block

        ldiw    Z, Globals + G_OutBDitherRec
        rcall   Dither  ; 38 cycles
        ldd     r16, Y + G_OutBDitherRec + DitherRec_Dithered
        ldd     r18, Y + G_Flags
        sbrc    r18, GFLAG_BIT_PWMB
        out     _SFR_IO_ADDR(OCR1B), r16
        ; 46 cycles in block

main_T:
; 136 cycles
        ; 10 x 2.5kHz task slots
        ldd     r16, Y + G_Counter25kHz
        cpi     r16, 3
        brcc    1f
        cpi     r16, 1
        brcs    main_T0_UpdatePWMEnableStates  ; 143 cycles
        breq    main_T1_ReadPot  ; 144 cycles
        rjmp    main_T2_PrefilterInput  ; 145 cycles
1:      cpi     r16, 4
        brcs    main_T3  ; 144 cycles
        rjmp    main_T4Plus_ThermIO  ; 145 cycles

main_T0_UpdatePWMEnableStates:
; 143 cycles
        rcall   UpdatePWMEnableFlags  ; 22 cycles
        rcall   UpdateHardwarePWMEnableStates  ; 39 cycles
        ; 61 cycles for task
        ; 204 cycles
        rjmp    main_LoopEnd

main_T1_ReadPot:
; 144 cycles
        ; Fetch the ADC result (for the attached potentiometer)
        sbis    _SFR_IO_ADDR(ADCSRA), ADSC
        rjmp    1f
        ; The ADC is busy for some reason.
        ; Make do with the previous result.
        ldd     r16, Y + G_ADCResult + 0
        ldd     r17, Y + G_ADCResult + 1
        rjmp    2f
1:      ; The ADC has returned a result.
        in      r16, _SFR_IO_ADDR(ADCL)
        in      r17, _SFR_IO_ADDR(ADCH)
        std     Y + G_ADCResult + 0, r16
        std     Y + G_ADCResult + 1, r17
        ; Begin another ADC round.
        sbi     _SFR_IO_ADDR(ADCSRA), ADSC
2:      ; 10 cycles at most for task
        rcall   ScaleADC10LARToPWMDuty  ; 32 cycles
        std     Y + G_ScaledADCResult + 0, r16
        std     Y + G_ScaledADCResult + 1, r17
        ; 54 cycles at most for task
        ; 190 cycles
        rjmp    main_LoopEnd

main_T2_PrefilterInput:
; 145 cycles
        ldd     r16, Y + G_ScaledADCResult + 0
        ldd     r17, Y + G_ScaledADCResult + 1
        ldiw    Z, ADCBoxFBuf
        rcall   UpdateBoxFilterBuffer  ; 59 cycles
        DIVU_TO_16_BIT_Q_RHTE  ADC_BF_BUFFER_LENGTH ; 42 cycles at most
        std     Y + G_PrefilteredInput + 0, r16
        std     Y + G_PrefilteredInput + 1, r17
        ; 111 cycles at most for task
        ; 256 cycles at most
        rjmp    main_LoopEnd

main_T3:
; 144 cycles
        ; 25 x 100Hz task slots
        ldd     r16, Y + G_Counter2500Hz
        sbrc    r16, 3
        rjmp    3f
        ; 148 cycles
        sbrc    r16, 2
        rjmp    2f
        ; 150 cycles
        sbrc    r16, 1
        rjmp    1f
        sbrs    r16, 0
        rjmp    main_T3_00_FilterInputPass1  ; 155 cycles
        rjmp    main_T3_01_FilterInputPass2  ; 156 cycles
1:      sbrs    r16, 0
        rjmp    main_T3_02_FilterOutputAPass1  ; 156 cycles
        rjmp    main_T3_03_FilterOutputAPass2  ; 157 cycles
2:      ; 151 cycles
        sbrc    r16, 1
        rjmp    1f
        sbrs    r16, 0
        rjmp    main_T3_04_FilterOutputBPass1  ; 156 cycles
        rjmp    main_T3_05_FilterOutputBPass2  ; 157 cycles
1:      sbrs    r16, 0
        rjmp    main_T3_06_Temperature  ; 157 cycles
        rjmp    main_T3_07_FanSpeedFn1  ; 158 cycles
3:      ; 149 cycles
        sbrc    r16, 2
        rjmp    2f
        ; 151 cycles
        sbrc    r16, 1
        rjmp    1f
        sbrs    r16, 0
        rjmp    main_T3_08_FanSpeedFn2  ; 156 cycles
        rjmp    main_T3_09_FanSpeedFn3  ; 157 cycles
1:      sbrs    r16, 0
        rjmp    main_T3_10_Dogwash  ; 157 cycles
        rjmp    main_T3_11_Output  ; 158 cycles
2:      ; 152 cycles
        rjmp    main_LoopEnd  ; 154 cycles

main_T3_00_FilterInputPass1:
; 155 cycles
        ldd     r16, Y + G_ThermErrCount
        sbrc    r16, 7
        rjmp    1f
        ldd     r16, Y + G_TempFnOutput + 0
        ldd     r17, Y + G_TempFnOutput + 1
        rjmp    2f
1:      ldd     r16, Y + G_PrefilteredInput + 0
        ldd     r17, Y + G_PrefilteredInput + 1
        nop
2:      ; 10 cycles in block
        ldiw    Z, InBoxFBuf1
        rcall   UpdateBoxFilterBuffer  ; 59 cycles
        DIVU_TO_16_BIT_Q_RHTE  INPUT_BF1_BUFFER_LENGTH ; 42 cycles at most
        std     Y + G_FilteredInput + 0, r16
        std     Y + G_FilteredInput + 1, r17
        ; 117 cycles at most for task
        ; 272 cycles
        rjmp    main_LoopEnd

main_T3_01_FilterInputPass2:
; 156 cycles
        ldd     r16, Y + G_FilteredInput + 0
        ldd     r17, Y + G_FilteredInput + 1
        ldiw    Z, InBoxFBuf2
        rcall   UpdateBoxFilterBuffer  ; 59 cycles
        DIVU_TO_16_BIT_Q_RHTE  INPUT_BF2_BUFFER_LENGTH ; 42 cycles at most
        std     Y + G_FilteredInput + 0, r16
        std     Y + G_FilteredInput + 1, r17
        ; 111 cycles at most for task
        ; 267 cycles at most
        rjmp    main_LoopEnd

main_T3_02_FilterOutputAPass1:
; 156 cycles
        ldd     r16, Y + G_FilteredInput + 0
        ldd     r17, Y + G_FilteredInput + 1
        ; Hysteresis for Fan A is performed here.
        ldd     r18, Y + G_Flags
        cpi     r17, FANA_ON_THRES
        brcs    1f
        ori     r18, 1 << GFLAG_BIT_HYSTLATCHA
        nop
        rjmp    2f
1:      cpi     r17, FANA_OFF_THRES
        brcc    2f
        andi    r18, ~(1 << GFLAG_BIT_HYSTLATCHA)
2:      sbrs    r18, GFLAG_BIT_HYSTLATCHA
        clr     r16
        sbrs    r18, GFLAG_BIT_HYSTLATCHA
        clr     r17
        std     Y + G_Flags, r18
        ; 18 cycles in block
        ldiw    Z, OutABoxFBuf1
        rcall   UpdateBoxFilterBuffer  ; 59 cycles
        DIVU_TO_16_BIT_Q_RHTE  OUTA_BF1_BUFFER_LENGTH ; 42 cycles at most
        std     Y + G_FilteredOutputA + 0, r16
        std     Y + G_FilteredOutputA + 1, r17
        ; 125 cycles for task
        ; 281 cycles
        rjmp    main_LoopEnd

main_T3_03_FilterOutputAPass2:
; 157 cycles
        ldd     r16, Y + G_FilteredOutputA + 0
        ldd     r17, Y + G_FilteredOutputA + 1
        ldiw    Z, OutABoxFBuf2
        rcall   UpdateBoxFilterBuffer  ; 59 cycles
        DIVU_TO_16_BIT_Q_RHTE  OUTA_BF2_BUFFER_LENGTH ; 42 cycles at most
        std     Y + G_FilteredOutputA + 0, r16
        std     Y + G_FilteredOutputA + 1, r17
        ; 111 cycles for task
        ; 268 cycles
        rjmp    main_LoopEnd

main_T3_04_FilterOutputBPass1:
; 156 cycles
        ldd     r16, Y + G_FilteredInput + 0
        ldd     r17, Y + G_FilteredInput + 1
        ; Hysteresis for Fan B is performed here.
        ; If only one set of averaging buffers were used for both
        ; Fan A and Fan B, the hysteresis logic for Fan B would
        ; be placed at a later output stage.
        ldd     r18, Y + G_Flags
        cpi     r17, FANB_ON_THRES
        brcs    1f
        ori     r18, 1 << GFLAG_BIT_HYSTLATCHB
        nop
        rjmp    2f
1:      cpi     r17, FANB_OFF_THRES
        brcc    2f
        andi    r18, ~(1 << GFLAG_BIT_HYSTLATCHB)
2:      sbrs    r18, GFLAG_BIT_HYSTLATCHB
        clr     r16
        sbrs    r18, GFLAG_BIT_HYSTLATCHB
        clr     r17
        std     Y + G_Flags, r18
        ; 18 cycles in block
        ldiw    Z, OutBBoxFBuf1
        rcall   UpdateBoxFilterBuffer  ; 59 cycles
        DIVU_TO_16_BIT_Q_RHTE  OUTB_BF1_BUFFER_LENGTH ; 42 cycles at most
        std     Y + G_FilteredOutputB + 0, r16
        std     Y + G_FilteredOutputB + 1, r17
        ; 123 cycles for task
        ; 279 cycles
        rjmp    main_LoopEnd

main_T3_05_FilterOutputBPass2:
; 157 cycles
        ldd     r16, Y + G_FilteredOutputB + 0
        ldd     r17, Y + G_FilteredOutputB + 1
        ldiw    Z, OutBBoxFBuf2
        rcall   UpdateBoxFilterBuffer  ; 59 cycles
        DIVU_TO_16_BIT_Q_RHTE  OUTB_BF2_BUFFER_LENGTH ; 42 cycles at most
        std     Y + G_FilteredOutputB + 0, r16
        std     Y + G_FilteredOutputB + 1, r17
        ; 111 cycles for task
        ; 268 cycles
        rjmp    main_LoopEnd

main_T3_06_Temperature:
; 157 cycles
        ldiw    Z, ThermIO
        ldd     r16, Y + G_ThermometerState
        cpi     r16, DTSTATE_IDLE
        brne    1f
        ldi     r16, DTSTATE_START
        std     Y + G_ThermometerState, r16
1:      rcall   AdvanceThermometerState  ; 71 cycles at most, SD = 2
        ldd     r16, Y + G_ThermometerState
        cpi     r16, DTSTATE_COMPLETED
        breq    1f
        cpi     r16, DTSTATE_FAILED
        brne    2f
1:      ldi     r16, DTSTATE_START
        std     Y + G_ThermometerState, r16
2:      ; 89 cycles at most for task
        ; 246 cycles
        rjmp    main_LoopEnd

main_T3_07_FanSpeedFn1:
; 158 cycles
        ldiw    Z, SpeedFn
        ldd     r16, Z + SpeedFn_State
        cpi     r16, FSFSTATE_IDLE
        brne    1f
        ldd     r16, Y + G_Temperature + 0
        ldd     r17, Y + G_Temperature + 1
.if 0
    ; Test fan speed curve with potentiometer input.
    ; 0x0000..0xA000 -> 0x0120..0x01C0
    ldd     r16, Y + G_PrefilteredInput + 0
    ldd     r17, Y + G_PrefilteredInput + 1
    mov     r16, r17
    clr     r17
    ldiw    r19:r18, 0x12 << 4
    add     r16, r18
    adc     r17, r19
.endif
        std     Z + SpeedFn_Temperature + 0, r16
        std     Z + SpeedFn_Temperature + 1, r17
        ldi     r16, FSFSTATE_START
        std     Z + SpeedFn_State, r16
1:      ; 17 cycles at most for task
        ; 175 cycles at most
        rjmp    main_LoopEnd

main_T3_08_FanSpeedFn2:
; 158 cycles
        ldiw    Z, SpeedFn
        rcall   AdvanceFanSpeedFn  ; 133 cycles at most, SD = 4
        ; 289 cycles at most
        rjmp    main_LoopEnd

main_T3_09_FanSpeedFn3:
; 157 cycles
        ldiw    Z, SpeedFn
        ldd     r16, Z + SpeedFn_State
        cpi     r16, FSFSTATE_COMPLETE
        brne    1f
        ldd     r16, Z + SpeedFn_PWMDuty + 0
        ldd     r17, Z + SpeedFn_PWMDuty + 1
        std     Y + G_TempFnOutput + 0, r16
        std     Y + G_TempFnOutput + 1, r17
        ldi     r16, FSFSTATE_IDLE
        std     Z + SpeedFn_State, r16
1:      ; 17 cycles at most for task
        ; 174 cycles
        rjmp    main_LoopEnd

main_T3_10_Dogwash:
; 157 cycles
        wdr
.if 0
    ; Timing test
    ; Flash at 2.56s intervals (about 0.4Hz)
    ; CK = 16MHz: Bright, with short dips in brightness
    ; CK = 8Mhz: Dark, with short flashes, not especially bright
    in      r18, _SFR_IO_ADDR(TCNT1)
    adiw    Z, 0
    adiw    Z, 0
    adiw    Z, 0
    adiw    Z, 0
    adiw    Z, 0
    adiw    Z, 0
    adiw    Z, 0
    nop
    in      r19, _SFR_IO_ADDR(TCNT1)
    sub     r19, r18
    andi    r19, 0x7F  ; Protect against wrap-around
    cpi     r19, 6
    sbc     r19, r19
    andi    r19, 0x9F  ; Bright if system clock at 16MHz, off otherwise
    clr     r18
    movw    r16, r18
    ldd     r18, Y + G_Counter100Hz
    cpi     r18, 50
    brcc    1f
    lsr     r17
    ror     r16
    lsr     r17
    ror     r16
    subi    r17, -20
1:  std     Y + G_OutBDitherRec + DitherRec_Target + 0, r16
    std     Y + G_OutBDitherRec + DitherRec_Target + 1, r17
.endif
        rjmp    main_LoopEnd

main_T3_11_Output:
; 158 cycles
        ldd     r16, Y + G_FilteredOutputA + 0
        ldd     r17, Y + G_FilteredOutputA + 1
        rcall   OCRValueFromDuty  ; 15 cycles, SD = 2
        std     Y + G_OutADitherRec + DitherRec_Target + 0, r16
        std     Y + G_OutADitherRec + DitherRec_Target + 1, r17
        ldd     r16, Y + G_FilteredOutputB + 0
        ldd     r17, Y + G_FilteredOutputB + 1
        rcall   OCRValueFromDuty  ; 15 cycles, SD = 2
        std     Y + G_OutBDitherRec + DitherRec_Target + 0, r16
        std     Y + G_OutBDitherRec + DitherRec_Target + 1, r17
        ; 46 cycles for task
        ; 204 cycles
        wdr
        rjmp    main_LoopEnd

main_T4Plus_ThermIO:
        ; 145 cycles
        ldiw    Z, ThermIO
        rcall   LowLevelThermIO  ; 144 cycles at most
        ; 149 cycles for task
        ; 294 cycles
        rjmp    main_LoopEnd

main_LoopEnd:
        rjmp    main_Loop


;-----------------------------------------------------------------------------
; Data in program memory
;-----------------------------------------------------------------------------


        ; Not a sausage!

        .balign 2


;-----------------------------------------------------------------------------
.section .bss
;-----------------------------------------------------------------------------

;-----------------------------------------------------------------------------
; Globals in SRAM
;-----------------------------------------------------------------------------


SRAMData:

Globals:        .fill   GSize
ISRData:        .fill   ISRDataSize
ThermIO:        .fill   DTIOSize
SpeedFn:        .fill   SpeedFnSize
ADCBoxFBuf:     .fill   BFBufHdrSize + ADC_BF_BUFFER_SIZE
InBoxFBuf1:     .fill   BFBufHdrSize + INPUT_BF1_BUFFER_SIZE
InBoxFBuf2:     .fill   BFBufHdrSize + INPUT_BF2_BUFFER_SIZE
OutABoxFBuf1:   .fill   BFBufHdrSize + OUTA_BF1_BUFFER_SIZE
OutABoxFBuf2:   .fill   BFBufHdrSize + OUTA_BF2_BUFFER_SIZE
OutBBoxFBuf1:   .fill   BFBufHdrSize + OUTB_BF1_BUFFER_SIZE
OutBBoxFBuf2:   .fill   BFBufHdrSize + OUTB_BF2_BUFFER_SIZE
GlobalsEnd:

; The following variables are not to be cleared by ClearGlobals.
InitialMCUSR:   .fill   1

SRAMDataEnd:

SRAM_BYTES_ALLOCATED = SRAMDataEnd - SRAMData
.if SRAM_BYTES_ALLOCATED > 498
   .warning "SRAM is starting to get rather full. Mind the Stack!"
.endif

        .balign 2


;-----------------------------------------------------------------------------
.end
;-----------------------------------------------------------------------------
