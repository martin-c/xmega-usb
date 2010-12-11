/*! \file
 *  spi-flash-memory.c
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
 */

#include "spi-flash-memory.h"
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>



/***            Definitions             ***/

/*! Size of buffer used for reading/writing to data bus.\n
 *  Should be same as USB FIFO size when this library is used by USB library.
 */
#define BUS_RW_BUFFER_SIZE      64

/* SPI Interface */
#define FLASH_USART         USARTC0
/*! Assert (set low) flash CS pin.\n
 *  Needs to be modified to match hardware.
 */
#define FLASH_ASSERT_CS     PORTC.OUTCLR = PIN0_bm
/*! Release (set high) flash CS pin.\n
 *  Needs to be modified to match hardware.
 */
#define FLASH_RELEASE_CS    PORTC.OUTSET = PIN0_bm

/* ATMEL AT26DF Bit Definitions */
#define AT26_STATUS_SPRL_bm     1<<7
#define AT26_STATUS_SPM_bm      1<<6
#define AT26_STATUS_EPE_bm      1<<5
#define AT26_STATUS_WPP_bm      1<<4
#define AT26_STATUS_SWP01_bm    1<<3
#define AT26_STATUS_SWP00_bm    1<<2
#define AT26_STATUS_WEL_bm      1<<1
#define AT26_STATUS_BUSY_bm     1<<0



/***            Private Globals                 ***/
/*! AT26DF Command constants */
typedef enum at26dfCommand_enum
{
    AT26_READ_ARRAY             = 0X0B,
    AT26_READ_ARRAY_LF          = 0X03,
    AT26_BLOCK_ERASE_4K         = 0X20,
    AT26_BLOCK_ERASE_32K        = 0X52,
    AT26_BLOCK_ERASE_64K        = 0XD8,
    AT26_CHIP_ERASE             = 0X60,
    AT26_PROGRAM                = 0X02,
    AT26_PROGRAM_SEQUENTIAL     = 0XAD,
    AT26_WRITE_ENABLE           = 0X06,
    AT26_WRITE_DISABLE          = 0X04,
    AT26_PROTECT_SECTOR         = 0X36,
    AT26_UNPROTECT_SECTOR       = 0X39,
    AT26_READ_SECTOR_PROTECTION = 0X3C,
    AT26_READ_STATUS            = 0X05,
    AT26_WRITE_STATUS           = 0X01,
    AT26_READ_MFG_DEV_ID        = 0X9F,
    AT26_DEEP_POWER_DOWN        = 0XB9,
    AT26_DEEP_POWER_DOWN_RESUME = 0XAB,
} at26dfCommand_t;

/*! Current write address for data logging */
static uint32_t currAddress;



/***        Private function portotypes         ***/
static inline void waitForTxComplete(void);
static inline void waitForDataEmpty(void);
static uint8_t flashReadStatus(void);
static void flashWriteStatus(uint8_t data);
static void flashWriteEnable(void);
static void flashWriteDisable(void);
static void flashReadArray(uint32_t *address, uint8_t *buffer, uint8_t length);
static void flashWritePage(uint32_t *address, uint8_t *buffer, uint8_t length);
static void flashBlockErase_4k(uint32_t *address);



/***               Private Functions            ***/

/*  waitForTxComplete:
 *  Loop until TX complete flag is set by USART, then clear flag and return
 */
static inline void waitForTxComplete(void)
{
    while (bit_is_clear(FLASH_USART.STATUS, USART_TXCIF_bp))
    {
        ;                                       // wait for TX complete flag to be set
    }
    FLASH_USART.STATUS = USART_TXCIF_bm;        // clear TX-complete flag
}



/*  waitForDataEmpty:
 *  Delay loop waiting for data ready flag to be set, indicating more data can be written to USART
 *  NOTE: DREIF bit is cleared by writing data to USART
 */
static inline void waitForDataEmpty(void)
{
    while (bit_is_clear(FLASH_USART.STATUS, USART_DREIF_bp))
    {
        ;                                       // wait for data ready flag to be set
    }
    // flag cleared by writing data
}



/*  Reads status register and returns status byte
 *
 *  Returns:    AT26 status byte
 */
static uint8_t flashReadStatus(void)
{
    FLASH_ASSERT_CS;
    FLASH_USART.DATA = AT26_READ_STATUS;                // send command byte
    FLASH_USART.DATA = 0;                               // send dummy byte
    waitForTxComplete();
    FLASH_RELEASE_CS;
    FLASH_USART.DATA;                                   // discard dummy byte
    return FLASH_USART.DATA;    
}



/*  Writes data to status register.
 *  Assumes Write Enable Latch (WEL) is set.
 *
 *  data:       Data to write to status register.
 */
static void flashWriteStatus(uint8_t data)
{
    FLASH_ASSERT_CS;
    FLASH_USART.DATA = AT26_WRITE_STATUS;               // send command byte
    FLASH_USART.DATA = data;                            // send status byte
    waitForTxComplete();
    FLASH_RELEASE_CS;
    FLASH_USART.DATA;                                   // discard dummy bytes
    FLASH_USART.DATA;
}



/*  Sends command to set the Write Enable Latch (WEL) bit.
 */
static void flashWriteEnable(void)
{
    FLASH_ASSERT_CS;
    FLASH_USART.DATA = AT26_WRITE_ENABLE;               // send command byte
    waitForTxComplete();
    FLASH_RELEASE_CS;
    FLASH_USART.DATA;                                   // discard dummy byte   
}



/*  Sends command to clear the Write Enable Latch (WEL) bit.
 */
static void flashWriteDisable(void)
{
    FLASH_ASSERT_CS;
    FLASH_USART.DATA = AT26_WRITE_DISABLE;              // send command byte
    waitForTxComplete();
    FLASH_RELEASE_CS;
    FLASH_USART.DATA;                                   // discard dummy byte   
}



/*  Performs a sequential read from flash memory into buffer starting at address supplied.
 *  Although address points to a 32-bit variable only the lower 24 bits are used. 
 *
 *  NOTE: Utilizes low-speed read opcode (AT26_READ_ARRAY_LF).
 *
 *  address:        Pointer to flash address from which to start reading.
 *  buffer:         Pointer to buffer into which to read.
 *  length:         Number of bytes to read.
 */
static void flashReadArray(uint32_t *address, uint8_t *buffer, uint8_t length)
{
    uint8_t *addressByte = (uint8_t *)address + 2;      // third byte of address
    uint8_t i;
    
    /* send command and 3 address bytes */
    FLASH_ASSERT_CS;
    
    FLASH_USART.DATA = AT26_READ_ARRAY_LF;
    FLASH_USART.DATA = *addressByte--;
    FLASH_USART.DATA = *addressByte--;
    waitForDataEmpty();
    FLASH_USART.DATA;
    FLASH_USART.DATA = *addressByte;
    waitForTxComplete();
    FLASH_USART.DATA;                                   // clear dummy bytes
    FLASH_USART.DATA;
    FLASH_USART.DATA;
    
    /* receive data */
    for (i=0; i<length; i++)
    {
        FLASH_USART.DATA = 0;                           // send dummy byte
        waitForTxComplete();
        buffer[i] = FLASH_USART.DATA;                   // store data byte
    }
    
    FLASH_RELEASE_CS;    
}



/*  Performs a paged write to a previously erased memory area starting at address specified in
 *  address. Although address points to a 32-bit variable only the lower 24 bits are used.
 *  Page size is 256 bytes and writes starting at other than a page boundary will wrap around to the
 *  beginning of the page once the end of that particular page has been reached.
 *  Write Enable Latch (WEL) must be set to perform this command.
 *
 *  NOTE: Command does not wait for write to complete.
 *
 *  address:        Pointer to flash address to start writing.
 *  buffer:         Pointer to buffer from which to read.
 *  length:         Number of bytes to write. 
 */
static void flashWritePage(uint32_t *address, uint8_t *buffer, uint8_t length)
{
    uint8_t *addressByte = (uint8_t *)address + 2;      // third byte of address
    uint8_t i;
    
    /* send command and 3 address bytes */
    FLASH_ASSERT_CS;
    
    FLASH_USART.DATA = AT26_PROGRAM;
    FLASH_USART.DATA = *addressByte--;
    FLASH_USART.DATA = *addressByte--;
    waitForDataEmpty();
    FLASH_USART.DATA;
    FLASH_USART.DATA = *addressByte;
    waitForTxComplete();
    FLASH_USART.DATA;                                   // clear dummy bytes
    FLASH_USART.DATA;
    FLASH_USART.DATA;
    
    /* transmit data */
    for (i=0; i<length; i++)
    {
        FLASH_USART.DATA = buffer[i];                   // store data byte
        waitForTxComplete();
        FLASH_USART.DATA;                               // discard dummy byte
    }
    FLASH_RELEASE_CS;    
}



/*  Erase 4Kb block specified in address.
 *  Write Enable Latch (WEL) must be set in order to perform this command.
 *
 *  NOTE: Command does not wait for erase to complete
 *
 *  address:        pointer to flash address which should be erased.
 */
static void flashBlockErase_4k(uint32_t *address)
{
    uint8_t *addressByte = (uint8_t *)address + 2;      // third byte of address
    
    /* send command and 3 address bytes */
    FLASH_ASSERT_CS;
    
    FLASH_USART.DATA = AT26_BLOCK_ERASE_4K;
    FLASH_USART.DATA = *addressByte--;
    FLASH_USART.DATA = *addressByte--;
    waitForDataEmpty();
    FLASH_USART.DATA;
    FLASH_USART.DATA = *addressByte;
    waitForTxComplete();
    FLASH_USART.DATA;                                   // clear dummy bytes
    FLASH_USART.DATA;
    FLASH_USART.DATA;
    
    FLASH_RELEASE_CS;
}



/***                Public Functions            ***/

/*! Configure flash memory interface and unprotect flash memory.
 *  Set flash memory logging address to start of logfile.
 */
void initFlash(void)
{
    /* USART Master SPI Configuration (USART shared with accelerometer) */
    FLASH_USART.BAUDCTRLA = 0;
    FLASH_USART.BAUDCTRLB = 0;
    FLASH_USART.CTRLC = USART_CMODE_MSPI_gc;
    FLASH_USART.CTRLB = USART_RXEN_bm | USART_TXEN_bm;  // Enable Rx, Tx
    
    /* unprotect memory */
    flashWriteEnable();
    flashWriteStatus(0x00);
    
    /* setup logging address */
    currAddress = LOG_START_ADDRESS;
}



/*! Read consecutive sectors starting at address.\n
 *  Function assumes sectors have been previously erased.\n
 *
 *  \param address Address from which to start reading.
 *  \param count Number of sectors to read.
 *  \param write Pointer to function to write data bus, which takes pointer to buffer, length,
 *  and returns 0 on success.
 *  \return Returns appropriate flashCommandStatus enum value.
 */
uint8_t flashSectorRead(uint32_t address, uint16_t count, uint8_t (*write)(uint8_t *, uint8_t))
{
    uint8_t buffer[BUS_RW_BUFFER_SIZE];
    uint16_t i;
    
    LED_GREEN_1_ON;
    address *= BYTES_PER_SECTOR;    // convert address from LBA to byte
    /* calculate total number of BUS writes necessary based on total transfer size */
    for (i=0; i < count*(BYTES_PER_SECTOR/sizeof buffer); i++)
    {
        /* for each buffer */
        flashReadArray(&address, buffer, sizeof buffer);
        if (write(buffer, sizeof buffer) != 0)
        {
            LED_GREEN_1_OFF;
            return FLASH_BUS_ERROR;
        }
        address += sizeof buffer;
    }
    LED_GREEN_1_OFF;
    return FLASH_CMD_OK;
}



/*! Erase consecutive sectors.\n
 *  NOTE: Update call to flahshBlockErase_ depending on block size used.\n
 *  NOTE: Update delay function to produce real delays of 10ms.\n
 *
 *  \param address Address from which to start erase.
 *  \param count Number of sectors to erase.
 *  \return Returns appropriate flashCommandStatus enum value.
 */
uint8_t flashSectorErase(uint32_t address, uint16_t count)
{
    LED_GREEN_1_ON;
    address *= BYTES_PER_SECTOR;            // convert address from LBA to byte
    /* erase blocks */
    while (count--)
    {
        flashWriteEnable();
        /* change this function to match sector size */
        flashBlockErase_4k(&address);
        
        while (flashReadStatus() & AT26_STATUS_BUSY_bm)     // while flash is busy
        {
            _delay_ms(160.0);               // delay 10ms
        }
        if (flashReadStatus() & AT26_STATUS_EPE_bm)         // erase/program error
        {
            LED_GREEN_1_OFF;
            return FLASH_HW_ERROR;
        }
        address += BYTES_PER_SECTOR;
    }
    
    LED_GREEN_1_OFF;
    return FLASH_CMD_OK;
}



/*! Write consecutive sectors.\n
 *
 *  \param address Address from which to start writing.
 *  \param count Number of sectors to write.
 *  \param read Pointer to function to read data bus, which takes pointer to buffer and 
 *  returns 0 on success.
 *  \return Returns appropriate flashCommandStatus enum value.
 */
uint8_t flashSectorWrite(uint32_t address, uint16_t count, uint8_t (*read)(uint8_t *))
{
    uint8_t buffer[BUS_RW_BUFFER_SIZE];
    uint16_t i;
    
    LED_GREEN_1_ON;
    address *= BYTES_PER_SECTOR;            // convert address from LBA to byte
    
    /* calculate total number of BUS reads necessary based on total transfer size */
    for (i=0; i < count*(BYTES_PER_SECTOR/sizeof buffer); i++)
    {
        /* for each buffer */
        flashWriteEnable();
        if (read(buffer) != sizeof buffer)
        {
            LED_GREEN_1_OFF;
            return FLASH_BUS_ERROR;
        }
        flashWritePage(&address, buffer, sizeof buffer);
        while (flashReadStatus() & AT26_STATUS_BUSY_bm)     // while flash is busy
        {
            _delay_ms(1.6);                 // delay 100us
        }
        if (flashReadStatus() & AT26_STATUS_EPE_bm)         // erase/program error
        {
            LED_GREEN_1_OFF;
            return FLASH_HW_ERROR;
        }        
        address += sizeof buffer;
    }
    
    LED_GREEN_1_OFF;
    return FLASH_CMD_OK;
}



/*! Write data to flash memory.\n
 *  Flash page size (256 bytes) must be evenly divisible by length
 *  Function not used by USB library, but used to write data to flash by microcontroller.
 *
 *  \param data Pointer to buffer from which to write to flash.
 *  \param length Number of bytes to write to flash.
 *  \return Returns appropriate flashCommandStatus enum value.
 */
uint8_t flashLogData(uint8_t *data, uint8_t length)
{
    /* wrap address at end of file */
    if (currAddress + length > LOG_END_ADDRESS + 1)
    {
        currAddress = LOG_START_ADDRESS;
    }
    
    /* erase 4k block if on block boundry */
    if (currAddress % 4096 == 0)
    {
        flashWriteEnable();
        flashBlockErase_4k(&currAddress);
        
        while (flashReadStatus() & AT26_STATUS_BUSY_bm)     // while flash is busy
        {
            _delay_ms(10.0);                                // delay 10ms
        }
        if (flashReadStatus() & AT26_STATUS_EPE_bm)         // erase/program error
        {
            return FLASH_HW_ERROR;
        }        
    }
    
    /* write data */
    flashWriteEnable();
    flashWritePage(&currAddress, data, length);
    currAddress += length;
    
    while (flashReadStatus() & AT26_STATUS_BUSY_bm)     // while flash is busy
    {
        ;
    }
    if (flashReadStatus() & AT26_STATUS_EPE_bm)         // erase/program error
    {
        return FLASH_HW_ERROR;
    }
    return FLASH_CMD_OK;
}