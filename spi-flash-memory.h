/*! \file
 *  spi-flash-memory.h
 *  xmega-usb
 *
 *  Copyright (c) 2010 Martin Clemons
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 *
 *  Basic read/write/erase functionality for Atmel AT26 series flash memory over SPI interface.
 *
 */

#include <inttypes.h>



/***            Definitions             ***/

/* LEDs */
#define     LED_GREEN_1_ON              PORTC.OUTSET = PIN4_bm
#define     LED_GREEN_1_OFF             PORTC.OUTCLR = PIN4_bm



/*! Flash Memory Logical Configuration\n
 *  BYTES PER SECTOR should equal the size of smallest memory block which can be erased at a time,
 *  and be an even multiple of the flash programming Page Size.\n
 *  NUMBER_OF_SECTORS = flash capacity / BYTES_PER_SECTOR.
 */
#define NUMBER_OF_SECTORS       256
#define BYTES_PER_SECTOR        4096

/*! FAT file system info\n
 *  START ADDRESS is the memory address of logfile beginning\n
 *  END ADDRESS is the memory address of logfile end\n
 *
 *  These addresses are used by flashLogData() as lower and upper bounds.\n
 *  Not used by USB interface, instead used for writing data to flash by microcontroller which is
 *  later retrieved over USB. To simplify microcontroller access to filesystem, a single file exists
 *  which uses all free sectors. The start address of this file is LOG_START_ADDRESS and the last
 *  valid address of this file is LOG_END_ADDRESS, which should also correspond to the end of the
 *  flash memory.\n
 *  Filessytem is initially imaged onto device by USB master.
 */
#define LOG_START_ADDRESS       0x7000
#define LOG_END_ADDRESS         0x0FFFFFUL


/***            Public Variables        ***/

/*! Constants returned by flash read/write/erase functions.
 */
enum flashCommandStatus
{
    FLASH_CMD_OK    = 0,    ///< command completed OK
    FLASH_BUS_ERROR = 1,    ///< error reading/writing data bus
    FLASH_HW_ERROR  = 2,    ///< possible flash hardware error
};



/***        Public Functions            ***/
void initFlash(void);

uint8_t flashSectorRead(uint32_t address, uint16_t count, uint8_t (*write)(uint8_t *, uint8_t));
uint8_t flashSectorErase(uint32_t address, uint16_t count);
uint8_t flashSectorWrite(uint32_t address, uint16_t count, uint8_t (*read)(uint8_t *));
uint8_t flashLogData(uint8_t *data, uint8_t length);