# Makefile for TFSCForT85, a 4-wire fan PWM driver


# (c) Copyright 2022, Daniel Neville (creamygoat@gmail.com)
#
# This file is part of TFSCForT85.
#
# TFSCForT85 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# TFSCForT85 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with TFSCForT85. If not, see <https://www.gnu.org/licenses/>.


.PHONY = help all clean reset burn \
  burn_eeprom burn_lfuse burn_hfuse burn_efuse burn_fuses burn_all

# The main output files, aside from their extensions, have a common filename.
MAIN_STEM := tfsc

# Files to "burn" into MCU memories
HEX_FILE := ${MAIN_STEM}.hex
EEP_HEX_FILE := ${MAIN_STEM}.eep

# Note that for assembly source files to use the C preprocessor,
# they must have a ".S" extension (with the S in upper case).
CC := avr-gcc
LD := avr-ld

# Rather inconveniently, the same MCU part has different names for
# each of the assembler (C compiler), linker and the programmer.
CC_MCU := attiny85
LD_MCU_EMULATION := avr25
PROG_MCU := t85
PROG_PROGRAMMER := usbasp

# Getting the fuses wrong can partially brick an AVR chip so that
# a programmer capable of using the High Voltage (12V) programming
# interface would be required to restore the fuses. (Also, setting
# the clock speed out of range of the programmer can have a similar
# effect.)
#
# The lower Fuse Byte should be 0xf1 if the system clock is to run
# at 16MHz, otherwise 0xe2 (for 8MHz operation).

#LFUSE_VALUE := 0xf1
LFUSE_VALUE := 0xe2
HFUSE_VALUE := 0xd7
EFUSE_VALUE := 0xff


help:
	@echo "Temperature-based Fan Speed Controller for an 8-bit AVR MCU"
	@echo "Target MCU:"
	@echo "  Assembler: ${CC_MCU}"
	@echo "  Linker: ${LD_MCU_EMULATION}"
	@echo "  Programmer: ${PROG_MCU}"
	@echo "make help:        Display this message."
	@echo "make clean:       Remove generated files such as object files."
	@echo "make all:         Build the ${HEX_FILE} file, ready for burning."
	@echo "make reset:       Reset the MCU (by having avrdude interrogate it)."
	@echo "make burn:        Program the MCU's Flash memory (via avrdude)."
	@echo "make burn_eeprom: Program the MCU's EEPROM (via avrdude)."
	@echo "make burn_lfuse:  Program the low fuse byte (with ${LFUSE_VALUE})."
	@echo "make burn_hfuse:  Program the high fuse byte (with ${HFUSE_VALUE})."
	@echo "make burn_efuse:  Program the extended fuse byte (with ${EFUSE_VALUE})."
	@echo "make burn_fuses:  Program all fuses."
	@echo "make burn_all:    Program Flash, EEPROM and fuses."

clean:
	rm -f ${MAIN_STEM}.o
	rm -f ${MAIN_STEM}.elf
	rm -f ${HEX_FILE}

all: ${HEX_FILE} tfsc.hex

${MAIN_STEM}.o: ${MAIN_STEM}.S $(wildcard *.h)
	${CC} -mmcu=${CC_MCU} -o ${MAIN_STEM}.o -c ${MAIN_STEM}.S

${MAIN_STEM}.elf: ${MAIN_STEM}.o
#	${LD} -m ${LD_MCU_EMULATION} -o ${MAIN_STEM}.elf ${MAIN_STEM}.o
# Instead of using the linker, have the compiler generate the
# ELF file complete with stack pointer initialisation code and
# a properly filled interrupt vector table.
	${CC} -mmcu=${CC_MCU} -o ${MAIN_STEM}.elf ${MAIN_STEM}.o

${HEX_FILE}: ${MAIN_STEM}.elf
	avr-objcopy -O ihex ${MAIN_STEM}.elf ${HEX_FILE}
	@echo "✅ Hex file" '"'"${HEX_FILE}"'"' "is ready to burn"\
    "(with" '"make burn"'")."

reset:
	avrdude -p ${PROG_MCU} -c ${PROG_PROGRAMMER}

burn_eeprom:
	avrdude -p ${PROG_MCU} -c ${PROG_PROGRAMMER} -U eeprom:w:${EEP_HEX_FILE}:i

burn: ${HEX_FILE}
	avrdude -p ${PROG_MCU} -c ${PROG_PROGRAMMER} -U flash:w:${HEX_FILE}:i

burn_lfuse:
	avrdude -p t85 -c ${PROG_PROGRAMMER} -U lfuse:w:${LFUSE_VALUE}:m

burn_hfuse:
	avrdude -p t85 -c ${PROG_PROGRAMMER} -U hfuse:w:${HFUSE_VALUE}:m

burn_efuse:
	avrdude -p t85 -c ${PROG_PROGRAMMER} -U efuse:w:${EFUSE_VALUE}:m

burn_fuses: burn_lfuse burn_hfuse burn_efuse

burn_all: burn burn_eeprom burn_fuses
