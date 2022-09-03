#ifndef _UPB8MACROS8_INCLUDED
#define _UFB8MACROS8_INCLUDED

;-----------------------------------------------------------------------------
; Unified Port Bit macros for 8-bit AVR MCUs
; (For use with avr-gcc, an assembler with a C preprocessor)
;-----------------------------------------------------------------------------
;
; (c) Copyright 2022, Daniel Neville (creamygoat@gmail.com)
;
; This file is part of TFSCForT85.
;
; TFSCForT85 is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; TFSCForT85 is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with TFSCForT85. If not, see <https://www.gnu.org/licenses/>.
;
;-----------------------------------------------------------------------------
;
; A Unified Port Bit (index) or UPB is a numerical code for a particular bit
; in a particular IO port. UPBs start at 0 for bit 0 of port A and end at 63
; for bit 7 of port H, if such a port existed. UPBs are not directly related
; to the pin numbers on the chip.
;
; Macros are provided to extract the PORT, PIN and DDR addresses and the bit
; numbers for any single UPB. Though these macros are convenient for dealing
; with single IO pins at a time, the Unified Port Mask macros are far more
; efficient for stuffing multiple bits at once into each Special Function
; Register for the IO ports.
;
; A Unified Port Mask can be assembled in the usual way by left-shifting bit
; 1s, bit 0s or multi-bit values by UPBs as if all the ports were accessed as
; a single 64-bit wide IO port. When it comes to dealing with the real 8-bit
; port registers, macros are provided to extract from a UPM the 8-bit field
; relevant to the SFR indicated by the macro name. Thus all twenty-four GPIO
; pins from PB0 to PD7 on an ATmega168 can have their data direction and
; pull-up or latch levels set with just six writes to SFRs.
;
; For example:
;   UPB_ACTIVITY_LED = UPB('C', 3)
; will set UPB_ACTIVITY_LED to 8*2 + 3 = 19
;
; Note that _single_ quotes are required for the C preprocessor to correctly
; recognise the port letter as a character and not a string, say.
;
; The SFR address (as an IO address or a memory address) is given by
;
;   PORT_REG_IO(u), PIN_REG_IO(u), DDR_REG_IO(u),
;   PORT_REG_MEM(u), PIN_REG_MEM(u) and DDR_REG_MEM(u)
;
; and the SFR register bit is given by
;
;   PBIT(u)
;
; (which merely extracts the lower three bits of the given UPB).
;
; For the SBI, CBI, SBIC and SBIS instructions, which take an IO address
; and a bit number, macros are provided to supply both arguments at once
; from one UPB number:
;
;   PORT_REGBIT_IO(upb), PIN_REGBIT_IO(upb), DDR_REGBIT_IO(upb)
;
; Remember that for AVRs, the PORTx symbols defined in the datasheets and
; the standard include files refer specifically to the output latch registers
; (which double as pull-up enable registers), not just the general concept of
; IO ports. (PINx registers are where the inputs are read.)
;
; A simple example which deals with single IO bits at a time:
;
;   UPB_MAGIC_BUTTON = UPB('B', 5)
;   UPB_SAUSAGE_READY = UPB('B', 0)
;   UPB_DOOM_LED = UPB('C', 7)
;           sbic    PIN_REGBIT_IO(UPB_MAGIC_BUTTON)
;           cbi     PORT_REGBIT_IO(UPB_DOOM_LED)
;           ; = cli    PPORT_REG_IO(UPB_DOOM_LED), PBIT(UPB_DOOM_LED)
;           in      r16, PORT_REG_IO(UPB_SAUSAGE_READY)
;           bst     PBIT(UPB_SAUSAGE_READY)
;
; A more complex example which deals with whole registers at a time:
;
;   UPB_BUTTON_MAGIC = UPB('B', 5) ; External pull-up provided
;   UPB_BUTTON_PIZZA = UPB('B', 4) ; Needs MCU weak pull-up enabled
;   UPB_BUTTON_BISCUITS = UPB('C', 0) ; Also needs MCU weak pull-ups.
;   UPB_I2C_SCL = UPB('B', 2) ; Latch high, toggle DDR to send
;   UPB_I2C_SDA = UPB('D', 6) ; Latch high, toggle DDR to send
;   UPB_DOOMSDAY = UPB('C', 1)
;   UPB_NOT_DISCONBOBULATE = UPB('D', 7)
;
;           Avoid setting reserved bits.
;           UPM_FULL_IO = 0xFFFFFF00
;           ; Set data direction.
;           DDR_UPM = ( \
;             (0 << UPB_BUTTON_MAGIC) | \
;             (0 << UPB_BUTTON_PIZZA) | \
;             (0 << UPB_BUTTON_BISCUITS) | \
;             (0 << UPB_I2C_SCL) | \
;             (0 << UPB_I2C_SCL) | \
;             (1 << UPB_DOOMSDAY) | \
;             (1 << UPB_NOT_DISCONBOBULATE) | \
;           0)
;           ; Set latches (for outputs) and pull-ups (for inputs).
;           PORT_UPM = UPM_FULL_IO & ( \
;             (0 << UPB_BUTTON_MAGIC) | \
;             (1 << UPB_BUTTON_PIZZA) | \
;             (1 << UPB_BUTTON_BISCUITS) | \
;             (1 << UPB_I2C_SCL) | \
;             (1 << UPB_I2C_SCL) | \
;             (0 << UPB_DOOMSDAY) | \
;             (1 << UPB_NOT_DISCONBULATE) | \
;           0)
;
;           ldi     r16, PBMASK_UPM(PORT_UPM)
;           ldi     r17, PBMASK_UPM(DDR_UPM)
;           out     _SFR_IO_ADDR(PORTB), r16
;           out     _SFR_IO_ADDR(DDRB), r17
;           ldi     r16, PCMASK_UPM(PORT_UPM)
;           ldi     r17, PCMASK_UPM(DDR_UPM)
;           out     _SFR_IO_ADDR(PORTC), r16
;           out     _SFR_IO_ADDR(DDRC), r17
;           ldi     r16, PDMASK_UPM(PORT_UPM)
;           ldi     r17, PDMASK_UPM(DDR_UPM)
;           out     _SFR_IO_ADDR(PORTD), r16
;           out     _SFR_IO_ADDR(DDRD), r17
;
; The SFR names must already be defined before this file is included so that
; the maps of valid PORTx, PINx and DDRx SFR addresses can be calculated.
;
;-----------------------------------------------------------------------------


; INTERNAL STUFF

; Define addresses for ports which may or may not exist.
; These are used to construct port register address maps
; in symbols defined further below. (The maps do not
; consume MCU registers or memory.)

_DUMMY_PORT_ADDR = 0xFF
_MAYBE_PORTA = _DUMMY_PORT_ADDR
_MAYBE_PORTB = _DUMMY_PORT_ADDR
_MAYBE_PORTC = _DUMMY_PORT_ADDR
_MAYBE_PORTD = _DUMMY_PORT_ADDR
_MAYBE_PORTE = _DUMMY_PORT_ADDR
_MAYBE_PORTF = _DUMMY_PORT_ADDR
_MAYBE_PORTG = _DUMMY_PORT_ADDR
_MAYBE_PORTH = _DUMMY_PORT_ADDR
_MAYBE_PINA = _DUMMY_PORT_ADDR
_MAYBE_PINB = _DUMMY_PORT_ADDR
_MAYBE_PINC = _DUMMY_PORT_ADDR
_MAYBE_PIND = _DUMMY_PORT_ADDR
_MAYBE_PINE = _DUMMY_PORT_ADDR
_MAYBE_PINF = _DUMMY_PORT_ADDR
_MAYBE_PING = _DUMMY_PORT_ADDR
_MAYBE_PINH = _DUMMY_PORT_ADDR
_MAYBE_DDRA = _DUMMY_PORT_ADDR
_MAYBE_DDRB = _DUMMY_PORT_ADDR
_MAYBE_DDRC = _DUMMY_PORT_ADDR
_MAYBE_DDRD = _DUMMY_PORT_ADDR
_MAYBE_DDRE = _DUMMY_PORT_ADDR
_MAYBE_DDRF = _DUMMY_PORT_ADDR
_MAYBE_DDRG = _DUMMY_PORT_ADDR
_MAYBE_DDRH = _DUMMY_PORT_ADDR

#ifdef PORTA
_MAYBE_PORTA = PORTA
_MAYBE_PINA = PINA
_MAYBE_DDRA = DDRA
#endif
#ifdef PORTB
_MAYBE_PORTB = PORTB
_MAYBE_PINB = PINB
_MAYBE_DDRB = DDRB
#endif
#ifdef PORTC
_MAYBE_PORTC = PORTC
_MAYBE_PINC = PINC
_MAYBE_DDRC = DDRC
#endif
#ifdef PORTD
_MAYBE_PORTD = PORTD
_MAYBE_PIND = PIND
_MAYBE_DDRD = DDRD
#endif

; Port address maps as 64-bit words (for internal use)
#define _PX_REG_MAP(a, b, c, d, e, f, g, h) (\
  ((h) << 56) | ((g) << 48) | ((f) << 40) | ((e) << 32) | \
  ((d) << 24) | ((c) << 16) | ((b) << 8) | (a))
#define _PORT_REG_MAP _PX_REG_MAP(\
  _MAYBE_PORTA, _MAYBE_PORTB, _MAYBE_PORTC, _MAYBE_PORTD, \
  _MAYBE_PORTE, _MAYBE_PORTF, _MAYBE_PORTG, _MAYBE_PORTH)
#define _PIN_REG_MAP _PX_REG_MAP(\
  _MAYBE_PINA, _MAYBE_PINB, _MAYBE_PINC, _MAYBE_PIND, \
  _MAYBE_PINE, _MAYBE_PINF, _MAYBE_PING, _MAYBE_PINH)
#define _DDR_REG_MAP _PX_REG_MAP(\
  _MAYBE_DDRA, _MAYBE_DDRB, _MAYBE_DDRC, _MAYBE_DDRD, \
  _MAYBE_DDRE, _MAYBE_DDRF, _MAYBE_DDRG, _MAYBE_DDRH)

; Generic port address mapping function (for internal use)
#define _PX_REG_AMBI(map, upb) (((map) >> (8 * ((upb) >> 3))) & 255)

; Safe register mask for any single Unified Port Bit (for internal use)
; (If the UPB is not for the given port (specified by here by the UPB
; bit position of bit 0 of that port), zero will be returned.)
#define _PXMASK(upb0, upb) (((255 << (upb0)) & (1 << (upb))) >> (upb0))

; Safe register mask for any Unified Port Mask (for internal use)
#define _PXMASK_UPM(upb0, upm) (((upm) >> (upb0)) & 255)

; Unprotected UPB definition macros
#define _UPB0_RAW(portletter) (((portletter) - 'A') * 8)
#define _UPB_RAW(portletter, bit) (_UPB0_RAW(portletter) + (bit))

; Port letter validation (returning 0 for False and 1 for True)
#define _PORT_EXISTS(portletter) ( \
  (((portletter) >= 'A') & ((portletter) <= 'H')) &\
  (((_PORT_REG_MAP) >> _UPB0_RAW(portletter)) & 255 != (_DUMMY_PORT_ADDR)) &\
  (((_PIN_REG_MAP) >> _UPB0_RAW(portletter)) & 255 != (_DUMMY_PORT_ADDR)) &\
  (((_DDR_REG_MAP) >> _UPB0_RAW(portletter)) & 255 != (_DUMMY_PORT_ADDR)) &\
  1)

; For invalid port letters, the guarded UPB macro throws a division by zero
; to draw attention.
#define _UPB_GUARDED(portletter, bit) ( \
  ((1 / _PORT_EXISTS(portletter)) * _UPB_RAW((portletter), (bit))))


;-----------------------------------------------------------------------------


; UNIFIED PORT BITS MACROS

; Unified Port Bit index assignment
#define UPB(portletter, bit) _UPB_GUARDED((portletter), (bit))

; Ambiguous IO/memory addresses
#define PORT_REG_AMBI(upb) (_PX_REG_AMBI((_PORT_REG_MAP), (upb)))
#define DDR_REG_AMBI(upb) (_PX_REG_AMBI((_DDR_REG_MAP), (upb)))
#define PIN_REG_AMBI(upb) (_PX_REG_AMBI((_PIN_REG_MAP), (upb)))

; IO addresses
#define PORT_REG_IO(upb) (_SFR_IO_ADDR(PORT_REG_AMBI(upb)))
#define DDR_REG_IO(upb) (_SFR_IO_ADDR(DDR_REG_AMBI(upb)))
#define PIN_REG_IO(upb) (_SFR_IO_ADDR(PIN_REG_AMBI(upb)))

; Memory addresses
#define PORT_REG_MEM(upb) (_SFR_MEM_ADDR(PORT_REG_AMBI(upb)))
#define DDR_REG_MEM(upb) (_SFR_MEM_ADDR(DDR_REG_AMBI(upb)))
#define PIN_REG_MEM(upb) (_SFR_MEM_ADDR(PIN_REG_AMBI(upb)))

; Register bit index (the lower three bits) of a Unified Port Bit index
#define PBIT(upb) ((upb) & 7)

; IO address and bit pairs for sbi/cbi and sbis/sbic
#define PORT_REGBIT_IO(upb) PORT_REG_IO(upb), ((upb) & 7)
#define DDR_REGBIT_IO(upb) DDR_REG_IO(upb), ((upb) & 7)
#define PIN_REGBIT_IO(upb) PIN_REG_IO(upb), ((upb) & 7)

; Safe port register masks for any single Unified Port Bit index
; (If the UPB bit is not for the given port, zero will be returned.)
#define PAMASK(upb) (_PXMASK(UPB('A', 0), (upb)))
#define PBMASK(upb) (_PXMASK(UPB('B', 0), (upb)))
#define PCMASK(upb) (_PXMASK(UPB('C', 0), (upb)))
#define PDMASK(upb) (_PXMASK(UPB('D', 0), (upb)))
#define PEMASK(upb) (_PXMASK(UPB('E', 0), (upb)))
#define PFMASK(upb) (_PXMASK(UPB('F', 0), (upb)))
#define PGMASK(upb) (_PXMASK(UPB('G', 0), (upb)))
#define PHMASK(upb) (_PXMASK(UPB('H', 0), (upb)))

; Safe register masks for any Unified Port Mask
; (The resulting 8-bit mask only contains bits from the UPM
; belonging to the specific port.)
#define PAMASK_UPM(upm) _PXMASK_UPM(UPB('A', 0), (upm))
#define PBMASK_UPM(upm) _PXMASK_UPM(UPB('B', 0), (upm))
#define PCMASK_UPM(upm) _PXMASK_UPM(UPB('C', 0), (upm))
#define PDMASK_UPM(upm) _PXMASK_UPM(UPB('D', 0), (upm))
#define PEMASK_UPM(upm) _PXMASK_UPM(UPB('E', 0), (upm))
#define PFMASK_UPM(upm) _PXMASK_UPM(UPB('F', 0), (upm))
#define PGMASK_UPM(upm) _PXMASK_UPM(UPB('G', 0), (upm))
#define PHMASK_UPM(upm) _PXMASK_UPM(UPB('H', 0), (upm))

; Return 1 if a port of the given letter exists, otherwise 0.
#define PORT_EXISTS(portletter) (_PORT_EXISTS(portletter))


;-----------------------------------------------------------------------------

#endif /* _UPB8MACROS8_INCLUDED */
