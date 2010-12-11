/*! \file
 *  scsi-commands.c
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

#include "scsi-commands.h"

#include <stdio.h>
#include <string.h>
#include <util/delay.h>

/* flash memory read/write/erase commands used by library */
#include "spi-flash-memory.h"



/***            Definitions             ***/



/***        Private Global Variables            ***/
/*! Fixed format sense data sent in response to a REQUEST_SENSE request.
 *  See SPC section 4.5.3 for more details
 *  Stored in global sense struct.
 */
static struct scsiSenseData
{
    uint8_t responseCode:7;         // response code 0x70 indicates fixed format data 
    uint8_t valid:1;                // 0/1 indicates invalid/valid content in information field
    uint8_t obsolete;               // as in in IE6
    uint8_t senseKey:4;             // contains sense key code
    uint8_t reserved:1;
    uint8_t ili:1;
    uint8_t eom:1;
    uint8_t filemark:1;
    uint8_t information[4];         // contents are command/device specific
    uint8_t additionalSenseLength;  // contains length beyond this byte (10)
    uint8_t commandSpecificInformation[4];  // contents are command specific
    uint8_t asc;                    // additional sense code
    uint8_t ascq;                   // additional sense code qualifier
    uint8_t fruc;                   // field replaceable unit code
    uint8_t senseKeySpecific[3];
} sense;

/*  SCSI Transaction struct used by SCSI library to communicate with USB library */
static scsiTransaction_t *xact;

/*! Minimum commands required by SPC-3 and SBC-3.
 *  Not full listing - see SPC-3 6.1 and SBC 5.1 for full listing.
 */
enum scsiCommandCodes
{
    SCSI_COMMAND_TEST_UNIT_READY    = 0x00,
    SCSI_COMMAND_REQUEST_SENSE      = 0x03,
    SCSI_COMMAND_FORMAT_UNIT        = 0x04,
    SCSI_COMMAND_READ_6             = 0x08,
    SCSI_COMMAND_WRITE_6            = 0x0A,
    SCSI_COMMAND_INQUIRY            = 0x12,
    SCSI_COMMAND_SEND_DIAGNOSTIC    = 0x1D,
    SCSI_COMMAND_READ_CAPACITY_10   = 0x25,
    SCSI_COMMAND_READ_10            = 0x28,
    SCSI_COMMAND_WRITE_10           = 0x2A,
    SCSI_COMMAND_REPORT_LUNS        = 0xA0,
};

/*! Sense codes returned in sense data.\n
 *  Note: This is not a full listing, see section 4.5.6 of the SPC specification for a full listing.
 */
enum scsiSenseKeyCodes
{
    SCSI_SK_NOT_READY               = 0x2,
    SCSI_SK_MEDIUM_ERROR            = 0x3,
    SCSI_SK_ILLEGAL_REQUEST         = 0x5,
    SCSI_SK_UNIT_ATTENTION          = 0x6,
    SCSI_SK_DATA_PROTECT            = 0x7,
};

/*! Additional sense code and additional sense code qualifier values returned in sense data.
 *  Note: This is not a full listing, see section 4.5.6 of the SPC specification for a full listing.
 */
enum scsiAscCodes
{
    SCSI_ASC_LBA_OUT_OF_RANGE               = 0x21,
    SCSI_ASC_MEDIUM_NOT_PRESENT             = 0x3A,
    SCSI_ASC_PERIPHERAL_DEVICE_WRITE_FAULT  = 0x03,
    SCSI_ASC_UNRECOVERED_READ_ERROR         = 0x11,
    SCSI_ASC_WRITE_PROTECTED                = 0x27,
    SCSI_ASC_INVALID_COMMAND_OPCODE         = 0x20,
};

enum scsiAscqCodes
{
    SCSI_ASCQ_LBA_OUT_OF_RANGE              = 0x00,
    SCSI_ASCQ_MEDIUM_NOT_PRESENT            = 0x00,
    SCSI_ASCQ_PERIPHERAL_DEVICE_WRITE_FAULT = 0x00,
    SCSI_ASCQ_UNRECOVERED_READ_ERROR        = 0x00,
    SCSI_ASCQ_WRITE_PROTECTED               = 0x00,
    SCSI_ASCQ_INVALID_COMMAND_OPCODE        = 0x00,
};

/*! Write enable state of SCSI library
 */
enum
{
    WE_FALSE,
    WE_TRUE,
} writeEnabled;

/*! Constant data sent as response to SCSI inquiry command */
static prog_uint8_t scsiInquiryResponse[] =
{
    0x00,       // direct access block device, connected
    0x80,       // device is removable
    0x05,       // SPC-3 Compliant
    0x02,       // response data format
    0x1F,       // additional length (n-4)
    0x00,       // no bits set
    0x00,       // no bits set
    0x00,       // no bits set
    'A','t','m','e','l',' ',' ',' ',    // 8 byte T10 assigned Vendor ID
    'M','a','s','s',' ','S','t','o','r','a','g','e',' ',' ',' ',' ',    // 16 byte product ID
    '0','0','0','1',                    // 4 byte product version
};



/***         Public Global Variables            ***/



/***        Private function portotypes         ***/
static inline void decode32(uint8_t *b, uint32_t *l);
static inline void decode24(uint8_t *b, uint32_t *l);
static inline void decode16(uint8_t *b, uint16_t *l);

static void inquiry(void);
static void requestSense(void);
static void readCapacity_10(void);
static void read_6(void);
static void read_10(void);
static void write_6(void);
static void write_10(void);
static void formatUnit(void);
static void reportLuns(void);
static void unsupportedCommandReadOnly(void);
static void unsupportedCommand(void);



/***                    ISRs                    ***/



/***               Private Functions            ***/
/*  Decode 4 big endian bytes in buffer b to little endian l.
 *
 *  b:      Pointer to big endian value.
 *  l:      Pointer to little endian value.
 */
static inline void decode32(uint8_t *b, uint32_t *l)
{
    uint8_t *lp = (uint8_t *)l;     // little endian pointer

    lp[0] = b[3];   // LSB
    lp[1] = b[2];   // LSB + 1
    lp[2] = b[1];   // LSB + 2
    lp[3] = b[0];   // MSB
}



/*  Decode 3 big endian bytes in buffer b to little endian l.
 *
 *  b:      Pointer to big endian value.
 *  l:      Pointer to little endian value.
 */
static inline void decode24(uint8_t *b, uint32_t *l)
{
    uint8_t *lp = (uint8_t *)l;     // little endian pointer
    
    lp[0] = b[2];   // LSB
    lp[1] = b[1];   // LSB + 1
    lp[2] = b[0];   // MSB
    lp[3] = 0;
}    



/*  Decode 2 big endian bytes in buffer b to little endian l.
 *
 *  b:      Pointer to big endian value.
 *  l:      Pointer to little endian value.
 */
static inline void decode16(uint8_t *b, uint16_t *l)
{
    uint8_t *lp = (uint8_t *)l;     // little endian pointer
    
    lp[0] = b[1];   // LSB
    lp[1] = b[0];   // MSB
}



/*  SCSI Inquiry Command: See section 6.4 of SPC
 */
static void inquiry(void)
{
    xact->write_P(scsiInquiryResponse, sizeof scsiInquiryResponse);
    xact->cmdStatus = 0;                    // command ok
}



/*  Send fixed-format SCSI sense data
 */
static void requestSense(void)
{
    /* set dafault fields in sense data */
    sense.responseCode = 0x70;              // fixed-format response to current command (not cached)
    sense.additionalSenseLength = 0x0A;     // fixed format additional sense length: 10 bytes
    
    /* write sense data to bus */
    xact->write((uint8_t *)&sense, sizeof sense);

    memset((uint8_t *)&sense, 0, sizeof sense);         // reset sense data for next transaction
    xact->cmdStatus = 0;                    // command ok
}



/*  Send LBA information\n
 *  NUMBER_OF_SECTORS and BYTES_PER_SECTOR are flash memory characteristics and therefore defined
 *  in flash memory library.
 */
static void readCapacity_10(void)
{
    uint8_t parameterData[] =
    {
        0x00,                           // Highest Logical Block Address, MSB
        0x00,
        (NUMBER_OF_SECTORS - 1) / 256,
        (NUMBER_OF_SECTORS - 1) % 256,  // LSB
        0x00,                           // Logical Block Length (bytes) MSB
        0x00,
        BYTES_PER_SECTOR / 256,
        BYTES_PER_SECTOR % 256,         // LSB
    };
    
    xact->write(parameterData, sizeof parameterData);
    xact->cmdStatus = 0;                // command ok
}



/*  Read from flash, 21 bit address
 */
static void read_6(void)
{
    uint32_t add;
    uint16_t len;
    
    xact->cdb[1] &= 0x1F;               // mask 3 reserved bits
    decode24(&xact->cdb[1], &add);
    
    if (xact->cdb[4] == 0)
    {
        len = 256;
    }
    else
    {
        len = xact->cdb[4];
    }
    
    /* check if address is in range */
    if (add + len > NUMBER_OF_SECTORS)
    {
        sense.senseKey = SCSI_SK_ILLEGAL_REQUEST;
        sense.asc      = SCSI_ASC_LBA_OUT_OF_RANGE;
        sense.ascq     = SCSI_ASCQ_LBA_OUT_OF_RANGE;
        xact->cmdStatus = 1;            // command fail
        return;
    }
    
    switch (flashSectorRead(add, len, xact->write))
    {
        case FLASH_CMD_OK:
            xact->cmdStatus = 0;
            break;
            
        case FLASH_BUS_ERROR:
            xact->cmdStatus = 1;
            break;
            
            /* unknown or hardware error */
        default:
            sense.senseKey  = SCSI_SK_MEDIUM_ERROR;
            sense.asc       = SCSI_ASC_UNRECOVERED_READ_ERROR;
            sense.ascq      = SCSI_ASCQ_UNRECOVERED_READ_ERROR;
            xact->cmdStatus = 1;
            break;
    }
}



/*  Read from flash, 32 bit address
 */
static void read_10(void)
{
    uint32_t add;
    uint16_t len;
    
    decode32(&xact->cdb[2], &add);
    decode16(&xact->cdb[7], &len);
    
    /* check if address is in range */
    if (add + len > NUMBER_OF_SECTORS)
    {
        sense.senseKey = SCSI_SK_ILLEGAL_REQUEST;
        sense.asc      = SCSI_ASC_LBA_OUT_OF_RANGE;
        sense.ascq     = SCSI_ASCQ_LBA_OUT_OF_RANGE;
        xact->cmdStatus = 1;            // command fail
        return;
    }
    
    switch (flashSectorRead(add, len, xact->write))
    {
        case FLASH_CMD_OK:
            xact->cmdStatus = 0;
            break;
            
        case FLASH_BUS_ERROR:
            xact->cmdStatus = 1;
            break;
            
        /* unknown or hardware error */
        default:
            sense.senseKey  = SCSI_SK_MEDIUM_ERROR;
            sense.asc       = SCSI_ASC_UNRECOVERED_READ_ERROR;
            sense.ascq      = SCSI_ASCQ_UNRECOVERED_READ_ERROR;
            xact->cmdStatus = 1;
            break;
    }
}



/*  Write to flash, 21 bit address
 */
static void write_6(void)
{
    uint32_t add;
    uint16_t len;
    
    xact->cdb[1] &= 0x1F;               // mask 3 reserved bits
    decode24(&xact->cdb[1], &add);
    
    if (xact->cdb[4] == 0)
    {
        len = 256;
    }
    else
    {
        len = xact->cdb[4];
    }    
    
    /* check if address is in range */
    if (add + len > NUMBER_OF_SECTORS)
    {
        sense.senseKey = SCSI_SK_ILLEGAL_REQUEST;
        sense.asc      = SCSI_ASC_LBA_OUT_OF_RANGE;
        sense.ascq     = SCSI_ASCQ_LBA_OUT_OF_RANGE;
        xact->cmdStatus = 1;    // command fail
        return;
    }
    
    switch (flashSectorErase(add, len))
    {
        case FLASH_CMD_OK:
            break;
            
            /* unknown or hardware error, terminate function */
        default:
            sense.senseKey  = SCSI_SK_MEDIUM_ERROR;
            sense.asc       = SCSI_ASC_PERIPHERAL_DEVICE_WRITE_FAULT;
            sense.ascq      = SCSI_ASCQ_PERIPHERAL_DEVICE_WRITE_FAULT;
            xact->cmdStatus = 1;
            return;
    }
    
    switch (flashSectorWrite(add, len, xact->read))
    {
        case FLASH_CMD_OK:
            xact->cmdStatus = 0;
            break;
            
        case FLASH_BUS_ERROR:
            xact->cmdStatus = 1;
            break;
            
            /* unknown or hardware error */
        default:
            sense.senseKey  = SCSI_SK_MEDIUM_ERROR;
            sense.asc       = SCSI_ASC_PERIPHERAL_DEVICE_WRITE_FAULT;
            sense.ascq      = SCSI_ASCQ_PERIPHERAL_DEVICE_WRITE_FAULT;
            xact->cmdStatus = 1;
            break;
    }
}



/*  Write to flash, 32 bit address
 */
static void write_10(void)
{
    uint32_t add;
    uint16_t len;
    
    decode32(&xact->cdb[2], &add);
    decode16(&xact->cdb[7], &len);
    
    /* check if address is in range */
    if (add + len > NUMBER_OF_SECTORS)
    {
        sense.senseKey = SCSI_SK_ILLEGAL_REQUEST;
        sense.asc      = SCSI_ASC_LBA_OUT_OF_RANGE;
        sense.ascq     = SCSI_ASCQ_LBA_OUT_OF_RANGE;
        xact->cmdStatus = 1;    // command fail
        return;
    }
    
    switch (flashSectorErase(add, len))
    {
        case FLASH_CMD_OK:
            break;
        
        /* unknown or hardware error, terminate function */
        default:
            sense.senseKey  = SCSI_SK_MEDIUM_ERROR;
            sense.asc       = SCSI_ASC_PERIPHERAL_DEVICE_WRITE_FAULT;
            sense.ascq      = SCSI_ASCQ_PERIPHERAL_DEVICE_WRITE_FAULT;
            xact->cmdStatus = 1;
            return;
    }
    
    switch (flashSectorWrite(add, len, xact->read))
    {
        case FLASH_CMD_OK:
            xact->cmdStatus = 0;
            break;
            
        case FLASH_BUS_ERROR:
            xact->cmdStatus = 1;
            break;
            
        /* unknown or hardware error */
        default:
            sense.senseKey  = SCSI_SK_MEDIUM_ERROR;
            sense.asc       = SCSI_ASC_PERIPHERAL_DEVICE_WRITE_FAULT;
            sense.ascq      = SCSI_ASCQ_PERIPHERAL_DEVICE_WRITE_FAULT;
            xact->cmdStatus = 1;
            break;
    }
}



/*  SCSI format command - does nothing
 */
static void formatUnit(void)
{
    xact->cmdStatus = 0;
}



/*  Report LUNs
 */
static void reportLuns(void)
{
    uint8_t luns[] =
    {
        0x00, 0x00, 0x00, 0x00,             // LUNs list length (n-7)
        0x00, 0x00, 0x00, 0x00,             // reserved
    };
    
    xact->write(luns, sizeof luns);
    xact->cmdStatus = 0;                    // command ok
}



/*  Fail command and set "data protect" in sense data
 */
static void unsupportedCommandReadOnly(void)
{
    /* file system is read only */
    sense.senseKey = SCSI_SK_DATA_PROTECT;
    sense.asc = SCSI_ASC_WRITE_PROTECTED;
    sense.ascq = SCSI_ASCQ_WRITE_PROTECTED;
    
    xact->cmdStatus = 1;        // command fail
}



/*  Sets SCSI sense data to indicate requested command is not supported
 */
static void unsupportedCommand(void)
{
    sense.senseKey = SCSI_SK_ILLEGAL_REQUEST;
    sense.asc = SCSI_ASC_INVALID_COMMAND_OPCODE;
    sense.ascq = SCSI_ASCQ_INVALID_COMMAND_OPCODE;
    xact->cmdStatus = 1;        // command fail
}



/***                Public Functions            ***/

/*! Parse SCSI command and execute as necessary.\n
 *  Parse SCSI CDB, execute command (sending and receiving data on data bus and to flash as necessary),
 *  and return when complete.
 *
 *  \param x Pointer to SCSI transaction data used in command.
 */
void processScsiCommand(scsiTransaction_t *x)
{
    xact = x;   // update global SCSI transaction data xact
    
    switch (xact->cdb[0])
    {
        case SCSI_COMMAND_TEST_UNIT_READY:
            xact->cmdStatus = 0;
            break;
            
        case SCSI_COMMAND_REQUEST_SENSE:
            requestSense();
            break;

        case SCSI_COMMAND_FORMAT_UNIT:
            if (writeEnabled)
            {
                formatUnit();
            }
            else
            {
                unsupportedCommandReadOnly();
            }
            break;
            
        case SCSI_COMMAND_READ_6:
            read_6();
            break;
            
        case SCSI_COMMAND_WRITE_6:
            if (writeEnabled)
            {
                write_6();
            }
            else
            {
                unsupportedCommandReadOnly();
            }
            break;
            
        case SCSI_COMMAND_INQUIRY:
            inquiry();
            break;
            
        case SCSI_COMMAND_SEND_DIAGNOSTIC:
            xact->cmdStatus = 0;
            break;
        
        case SCSI_COMMAND_READ_CAPACITY_10:
            readCapacity_10();
            break;
            
        case SCSI_COMMAND_READ_10:
            read_10();
            break;
                
        case SCSI_COMMAND_WRITE_10:
            if (writeEnabled)
            {
                write_10();
            }
            else
            {
                unsupportedCommandReadOnly();
            }            
            break;
            
        case SCSI_COMMAND_REPORT_LUNS:
            reportLuns();
            break;
            
        default:
            unsupportedCommand();
            break;
    }
}


/*! Reset SCSI device */
void scsiReset(void)
{
    memset((uint8_t *)&sense, 0, sizeof sense);             // reset sense data for next transaction
}



/*! Enable writing to SCSI device\n
 *  By default SCSI device is read only, but calling this function sets writes state.\n
 *  When writes are disabled SCSI commands that require writing or erasing fail and set
 *  SCSI_SK_DATA_PROTECT in sense data.
 *
 *  \param we Write enable value. Nonzero to enable write, 0 to disable write.
 */
void scsiWriteEnable(uint8_t we)
{
    if (we)
    {
        writeEnabled = WE_TRUE;
    }
    else
    {
        writeEnabled = WE_FALSE;
    }
}