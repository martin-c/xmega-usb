/*! \file
 *  usb-mass-storage.c
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

#include "usb-mass-storage.h"

#include <avr/io.h>
#include <stdio.h>
#include <string.h>

#include "max-3420.h"
#include "scsi-commands.h"


/// number of USB string descriptors
#define STRING_DESCRIPTOR_COUNT         4
/// USB endpoint buffer size. Should equal MAX3420E endpoint buffer size.
#define EP_BUFFER_SIZE                  64



/***            Globals (Private)               ***/
/* USB setup packet data */
typedef struct
{
    uint8_t     requestType;
    uint8_t     request;
    uint8_t     value_lo;
    uint8_t     value_hi;
    uint16_t    index;
    uint16_t    length;
} usbSetupPacket_t;

/* USB Bulk Transfer Command Block Wrapper data */
struct usbBulkCbw
{
    uint32_t    signature;
    uint32_t    tag;
    uint32_t    dataTransferLength;
    uint8_t     flags;
    uint8_t     lun;
    uint8_t     length;
    uint8_t     cb[16];
};

/* USB Bulk Transfer Command Status Wrapper data */
struct usbBulkCsw
{
    uint32_t    signature;
    uint32_t    tag;
    uint32_t    dataResidue;
    uint8_t     status;
};

/* setup request constants */
enum setupRequestType
{
    REQUEST_TYPE_STANDARD   = 0 << 5,
    REQUEST_TYPE_CLASS      = 1 << 5,
    REQUEST_TYPE_VENDOR     = 2 << 5,
    REQUEST_TYPE_RESERVED   = 3 << 5,
};

/* setup request recipient constants */
enum setupRequestRecipient
{
    REQUEST_RECP_DEVICE     = 0,
    REQUEST_RECP_INTERFACE  = 1,
    REQUEST_RECP_ENDPOINT   = 2,
    REQUEST_RECP_OTHER      = 3,
};

/* standard request value constants */
enum setupStandardRequests
{
    STANDARD_REQUEST_GET_STATUS         = 0,
    STANDARD_REQUEST_CLEAR_FEATURE      = 1,
    STANDARD_REQUEST_SET_FEATURE        = 3,
    STANDARD_REQUEST_SET_ADDRESS        = 5,
    STANDARD_REQUEST_GET_DESCRIPTOR     = 6,
    STANDARD_REQUEST_SET_DESCRIPTOR     = 7,
    STANDARD_REQUEST_GET_CONFIGURATION  = 8,
    STANDARD_REQUEST_SET_CONFIGURATION  = 9,
    STANDARD_REQUEST_GET_INTERFACE      = 10,
    STANDARD_REQUEST_SET_INTERFACE      = 17,
    STANDARD_REQUEST_SYNCH_FRAME        = 18,
};

/* interface request value constants */
enum setupInterfaceRequests
{
    INTERFACE_REQUEST_GET_STATUS        = 0,
    INTERFACE_REQUEST_CLEAR_FEATURE     = 1,
    INTERFACE_REQUEST_SET_FEATURE       = 3,
    INTERFACE_REQUEST_GET_INTERFACE     = 10,
    INTERFACE_REQUEST_SET_INTERFACE     = 17,
};

/* endpoint request value constants */
enum setupEndpointRequests
{
    ENDPOINT_REQUEST_GET_STATUS         = 0,
    ENDPOINT_REQUEST_CLEAR_FEATURE      = 1,
    ENDPOINT_REQUEST_SET_FEATURE        = 3,
    ENDPOINT_REQUEST_SYNCH_FRAME        = 18,
};

/* bulk command status constants */
enum blkCommandStatus
{
    COMMAND_STATUS_PASS     = 0,
    COMMAND_STATUS_FAIL     = 1,
    PHASE_ERROR             = 2,
};


/* deviceStatus:
 *  16-bit bitfield containing 14 reserved status flags, remote wakeup status flag, and self powered
 *  status flag. Queried by USB host in standard device requests.
 */
static struct
{
    uint8_t  selfPoweredF    :1;
    uint8_t  remoteWakeupF   :1;
    uint16_t reserved        :14;
} deviceStatus;

/* deviceConfigState:
 *  Used to track the configuration state of the USB device
 */
static enum
{
    STATE_NOT_CONFIGURED    = 0,
    STATE_CONFIGURED        = 1,
} deviceConfigState;

/* USB bulk transaction data
 *  Used to keep track of USB Bulk Data transaction parameters
 */
struct usbBulkTransaction
{
    struct usbBulkCbw           cmd;            // Command Block Wrapper
    struct usbBulkCsw           stat;           // Command Status Wrapper
    uint32_t                    bytesIn;        // bytes to be transferred over bulk in endpoint
    uint32_t                    bytesOut;       // bytes to be transferred over bulk out endpoint
    uint8_t                     xferError;      // bulk transport transfer error
} blkx = {
    .stat.signature = 0x53425355UL,
};

/* deviceDescriptor:
 *  This variable stores all data sent to the USB host as the Device Descriptor.
 *  Since this data is read-only it is stored in flash.
 *
 *  NOTE: multi-byte values are little-endian
 */
static prog_uint8_t deviceDescriptor[] =
{
    18,                 /// descriptor size bytes
    1,                  /// descriptor type - device
    0x00, 0x02,         /// USB Specification (BCD)
    0,0,0,              /// class, subclass, protocol
    64,                 /// maximum packet size for EP0
    0x6A,0x0B,          /// vendorID (MAXIM)
    0x46,0x53,          /// productID (5346)
    0x01,0x00,          /// device release number (BCD)
    1,                  /// product string index
    2,                  /// mfg string index
    3,                  /// serial number string index
    1,                  /// number of possible configurations
};

/* configurationDescriptor:
 *  Stores Configuration Descriptor data in flash memory
 *  Device has single configuration:
 *  bus powered, 500ma max current draw
 */
static prog_uint8_t configurationDescriptor[] =
{
    9,                  /// descriptor size in bytes
    2,                  /// descriptor type - configuration
    0x20, 0x00,         /// total length of this and subordinate descriptors
    1,                  /// number of interfaces
    1,                  /// identifier for this configuration
    0,                  /// configuration string index
    0x80,               /// attributes (bus powered, no remote wakeup)
    250,                /// maximum power consumption (500ma)
/* interfaceDescriptor
 *  Stores Device Descriptor data in flash memory
 *  Device has single interface:
 *  Mass Storage with SCSI transparent command set
 */
    9,                  /// descriptor size in bytes
    4,                  /// descriptor type - interface
    0,                  /// interface number
    0,                  /// alternate setting number
    2,                  /// number of endpoints
    8,                  /// class code - mass storage
    6,                  /// subclass code - SCSI transparent command set
    80,                 /// protocol code - bulk only transport
    0,                  /// interface string index
/* endpointDescriptor-1-out
 *  Stores Endpoint Descriptor data in flash memory for endpoint 1 out using bulk transfers
 */
    7,                  /// descriptor size in bytes
    5,                  /// descriptor type - endpoint
    0x01,               /// endpoint direction and number - out, 1
    2,                  /// transfer type - bulk
    64,0,               /// maximum packet size
    0,                  /// maximum NAK rate
/* endpointDescriptor-1-in
 *  Stores Endpoint Descriptor data in flash memory for endpoint 1 in using bulk transfers
 */
    7,                  /// descriptor size in bytes
    5,                  /// descriptor type - endpoint
    0x82,               /// endpoint direction and number - in, 2
    2,                  /// transfer type - bulk
    64,0,               /// maximum packet size
    0,                  /// not used
};

/* stringDescriptors[]:
 *  Array of string descriptors corresponding to indexes specified in Device Descriptor.
 *  Index 0 contains languageID
 */
static prog_uint8_t stringDescriptors[][26]=
{
    /*! index 0 - language */
    {
        4,              // length
        3,              // descriptor type - string
        0x09,0x04,      // languageID - United States
    },
    /*! index 1 - product ID */
    {
        18,             // length
        3,              // descriptor type - string
        'M',0,'A',0,'X',0,'3',0,'4',0,'2',0,'0',0,'E',0,
    },
    /*! index 2 - manufacturer ID */
    {
        12,             // length
        3,              // descriptor type - string
        'M',0,'a',0,'x',0,'i',0,'m',0,
    }, 
    /*! index 3 - serial number */
    {   26,             // length
        3,              // descriptor type - string
        '1',0,'2',0,'3',0,'4',0,'5',0,'6',0,'7',0,'8',0,'9',0,'A',0,'B',0,'C',0,
    }
};



/***            Globals (Public)                ***/



/***        Private function portotypes         ***/
static uint8_t sendDescriptor(uint8_t type, uint8_t index, uint16_t reqLen);
static void processStandardRequest(usbSetupPacket_t *setup);
static void processClassRequest(usbSetupPacket_t *setup);
static void padBytesIn(uint32_t bytes);
static void padBytesOut(uint32_t bytes);
static uint8_t blkCommandTransport(void);
static void blkStatusTransport(enum blkCommandStatus stat);
static void handleBulkTransportError(uint8_t scsiCmdStat);



/***                    ISRs                    ***/



/***               Private Functions            ***/
/*  Send string descriptor from program space specified by type and index. Will send up to but not
 *  more than reqLen.
 *  Note: Maximum descriptor length is 255.
 *
 *  type:       Type of descriptor (1=device, 2=config, 3=string).
 *  index:      Index of descriptor for particular type (used for string descriptors only).
 *  reqLen:     Requested length.
 *  Returns:    0 on success, 1 on failure.
 */
static uint8_t sendDescriptor(uint8_t type, uint8_t index, uint16_t reqLen)
{
    // maximum length 255
    uint8_t rl = (reqLen < 255) ? (uint8_t)reqLen : 255;

    switch (type)
    {
        /* device descriptor
         * NOTE: assumes device descriptor is 64 bytes or less */
        case 1:
        {
            const uint8_t al = sizeof deviceDescriptor;                 // length available
            return writeEp0_P(deviceDescriptor, (rl < al) ? rl : al);
        }
            
        /* configuration descriptor
         * send configuration descriptor and subordinates.
         * NOTE: assumes configuration descriptor and subordinates are 64 bytes or less!
         */
        case 2:
        {
            const uint8_t al = sizeof configurationDescriptor;          // length available
            return writeEp0_P(configurationDescriptor, (rl < al) ? rl : al);
        }
            
        /* string descriptor */
        case 3:
        {
            // length available
            if (index > STRING_DESCRIPTOR_COUNT - 1)
            {
                break;
            }
            uint8_t al = pgm_read_byte_near(&stringDescriptors[index][0]);
            return writeEp0_P(&stringDescriptors[index][0], (rl < al) ? rl : al);
        }
            
        /* unsupported descriptor */
        default:
            break;
    }
    return 1;   // error sending descriptor
}



/*  This function will parse setup packet and process USB standard
 *  requests accordingly. Function logic divides up requests by recipient, then by request.
 *
 *  setup:      Pointer to setup packet struct.
 */
static void processStandardRequest(usbSetupPacket_t *setup)
{
    /* determine request recipient */
    switch (setup->requestType & 0x0F) 
    {
        case REQUEST_RECP_DEVICE:
            /* determine which device request */
            switch (setup->request) 
            {
                case STANDARD_REQUEST_GET_STATUS:
                    /* send deviceStatus */
                    if (writeEp0((uint8_t *)(&deviceStatus), 2) == 0)
                    {
                        ackStatus();
                    }
                    else
                    {
                        stallEp(0);
                    }
                    break;
                    
                case STANDARD_REQUEST_CLEAR_FEATURE:
                    /* Feature selectors available for device are DEVICE_REMOTE_WAKEUP and TEST_MODE.
                     * Since remote wakeup is not supported by this device and clear feature cannot
                     * be used to clear TEST_MODE the request stalls EP0.
                     */
                    stallEp(0);
                    break;
                    
                case STANDARD_REQUEST_SET_FEATURE:
                    /* Feature selectors available for device are DEVICE_REMOTE_WAKEUP and TEST_MODE.
                     * Only test mode is supported.
                     */
                    ackStatus();
                    break;
                    
                case STANDARD_REQUEST_SET_ADDRESS:
                    /* Sets USB device address to address supplied in setup packet value field.
                     * Should not complete until after completion of status stage
                     */
                    setDeviceAddress(setup->value_lo);
                    break;
                    
                case STANDARD_REQUEST_GET_DESCRIPTOR:
                    /* Get device descriptor
                     * Descriptor type is in high byte of setup packet value field, and index is in
                     * low byte of value field.
                     */
                    if (sendDescriptor(setup->value_hi, setup->value_lo, (uint8_t)setup->length) == 0)
                    {
                        ackStatus();
                    }
                    else
                    {
                        stallEp(0);
                    }
                    break;
                    
                case STANDARD_REQUEST_SET_DESCRIPTOR:
                    /* Descriptors are read-only so this command is invalid */
                    stallEp(0);
                    break;
                    
                case STANDARD_REQUEST_GET_CONFIGURATION:
                    /* Sends device configuration state */
                    if (writeEp0(&deviceConfigState, 1) == 0)
                    {
                        ackStatus();
                    }
                    else
                    {
                        stallEp(0);
                    }
                    break;
                    
                case STANDARD_REQUEST_SET_CONFIGURATION:
                    /* Sets device configuration state to configured */
                    deviceConfigState = STATE_CONFIGURED;
                    ackStatus();
                    break;
                    
                default:
                    /* standard device request not recognized */
                    stallEp(0);
            }
            break;
            
        case REQUEST_RECP_INTERFACE:
            /* determine which interface request, since device has only one interface interface
             * selection in windex is ignored
             */
            switch (setup->request) 
            {
                /* get status of interface:
                 * both status bytes are reserved and thus returns 2 zero bytes
                 */
                case STANDARD_REQUEST_GET_STATUS:
                {
                    uint8_t status[] = {0, 0,};
                    if (writeEp0(status, 2) == 0)
                    {
                        ackStatus();
                    }
                    else
                    {
                        stallEp(0);
                    }
                } break;
                
                /* set/clear feature:
                 * no features are defined for interface and thus both clear feature and set feature
                 * stall EP0.
                 */
                case STANDARD_REQUEST_CLEAR_FEATURE:
                    stallEp(0);
                    break;
                    
                case STANDARD_REQUEST_SET_FEATURE:
                    stallEp(0);
                    break;
                    
                /* get/set interface:
                 * this device only has a single interface: interface 0
                 */
                case STANDARD_REQUEST_GET_INTERFACE:
                {
                    uint8_t interface = 0;
                    if(writeEp0(&interface, 1) == 0)
                    {
                        ackStatus();
                    }
                    else
                    {
                        stallEp(0);
                    }
                } break;

                case STANDARD_REQUEST_SET_INTERFACE:
                    stallEp(0);
                    break;
                    
                default:
                    stallEp(0);
            }
            break;
            
        case REQUEST_RECP_ENDPOINT:
            /* determine which endpoint request */
            switch (setup->request) 
            {
                /* get endpoint status:
                 * get endpoint status by calling isEpStalled. Mask everything except endpoint number
                 * in function call.
                 * If EP_HALT is set return 0x0001, otherwise return 0x0000.
                 */
                case STANDARD_REQUEST_GET_STATUS:
                {
                    uint16_t status = 0x0000;
                    
                    if (isEpStalled((uint8_t)setup->index & 0x0F))
                    {
                        status = 0x0001;
                    }
                    if (writeEp0((uint8_t *)&status, 2) == 0)
                    {
                        ackStatus();
                    }
                    else
                    {
                        stallEp(0);
                    }
                } break;
                
                /* set/clear endpoint feature:
                 * A single endpoint feature is defined - the halt state.
                 * Therefore set/clear feature set/clear the halt state of an endpoint
                 */
                case STANDARD_REQUEST_CLEAR_FEATURE:
                    clearStallEp((uint8_t)setup->index & 0x0F);
                    ackStatus();
                    break;
                    
                case STANDARD_REQUEST_SET_FEATURE:
                    stallEp((uint8_t)setup->index & 0x0F);
                    ackStatus();
                    break;
                
                /* synch frame */
                case STANDARD_REQUEST_SYNCH_FRAME:
                    ackStatus();
                    break;
                    
                default:
                    stallEp(0);
            }
            break;
            
        default:
            stallEp(0);
            break;
    }
}



/*  This function will parse setup packet and process USB Mass Storage Class requests.
 *
 *  setup:      Pointer to setup packet struct.
 */
static void processClassRequest(usbSetupPacket_t *setup)
{
    /* determine request type */
    switch (setup->request)
    {
        /* Get Max LUN request:
         * Only 1 LUN defined, return 0
         */
        case 0xFE:
        {
            uint8_t maxLun = 0;
            if (writeEp0(&maxLun, 1) == 0)
            {
                ackStatus();
            }
            else
            {
                stallEp(0);
            }
        } break;
            
        /* Bulk Only Mass Storage Reset request:
         * Ready device for next CBW from host. Bulk data toggle remains unchanged, and STALL
         * conditions remain unchanged.
         */
        case 0xFF:
            scsiReset();
            ackStatus();
            break;
            
        default:
            stallEp(0);
            break;
    }
}



/*  Send pad '0' bytes to host.
 *  Used for internal USB transfer synchronization.
 *
 *  bytes:      Number of pad bytes to send.
 */
static void padBytesIn(uint32_t bytes)
{
    uint8_t foo[EP_BUFFER_SIZE];
    
    memset(foo, 0, sizeof foo);
    while (bytes)
    {
        if (bytes > EP_BUFFER_SIZE)
        {
            if (writeEp2(foo, EP_BUFFER_SIZE) != 0)
            {
                break;      // endpoint buffer not available and another USB event active
            }
            bytes -= EP_BUFFER_SIZE;
        }
        else
        {
            writeEp2(foo, (uint8_t)bytes);
            break;
        }
    }
}



/*  Receive pad '0' bytes from host.
 *  Used for internal USB transfer synchronization.
 *
 *  bytes: Number of pad bytes to receive.
 */
static void padBytesOut(uint32_t bytes)
{
    uint8_t foo[EP_BUFFER_SIZE];
    
    while (bytes)
    {
        if (bytes > EP_BUFFER_SIZE)
        {
            if (readEp1(foo) != EP_BUFFER_SIZE)
            {
                break;      // endpoint buffer not available and another USB event active
            }
            bytes -= EP_BUFFER_SIZE;
        }
        else
        {
            readEp1(foo);
            break;
        }
    }    
}



/*  USB Bulk Transaction command transport, get and check CBW.
 *
 *  Returns:    0 on success, 1 on error.
 */
static uint8_t blkCommandTransport(void)
{
    uint8_t buffer[EP_BUFFER_SIZE];
    
    /* get new CBW */
    if (readEp1(buffer) != 31)
    {                         
        return 1;                       // incorrect length for CBW in endpoint buffer
    }
    memcpy(&blkx.cmd, buffer, sizeof blkx.cmd);
    
    /* check CBW signature, lun, and CB length */
    if (blkx.cmd.signature != 0x43425355UL  ||
        blkx.cmd.lun != 0                   ||
        blkx.cmd.length < 1                 ||
        blkx.cmd.length > 16)
    {
        return 1;
    }
    return 0;
}



/*  USB Bulk Transaction status transport, sends CSW to host.
 *
 *  stat:       USB Bulk Command Status to report to USB host.
 */
static void blkStatusTransport(enum blkCommandStatus stat)
{
    blkx.stat.tag = blkx.cmd.tag;
    blkx.stat.status = stat;
    
    writeEp2((uint8_t *)&blkx.stat, sizeof blkx.stat);  // write CSW to IN endpoint
}



/*  Respond to disagreement between host and device in bulk data transport
 *  See Universal Serial Bus Mass Storage Class Bulk-Only Transport 6.7 "The Thirteen Cases"
 *  for more information on the specific error cases.
 *
 *  scsiCmdStat:SCSI command status from SCSI command performed during bulk transaction.
 */
static void handleBulkTransportError(uint8_t scsiCmdStat)
{
    if ((blkx.cmd.flags & 0x80) && blkx.cmd.dataTransferLength > 0)
    /* data transport device -> host */
    {
        if (blkx.xferError)
        /* case 7 or 8 */
        {
            if (blkx.bytesIn)
            {
                padBytesIn(blkx.bytesIn);
            }
            blkStatusTransport(PHASE_ERROR);
        }
        else if (blkx.bytesIn)
        /* case 4 or 5 */
        {
            padBytesIn(blkx.bytesIn);
            blkx.stat.dataResidue = blkx.bytesIn;
            if (scsiCmdStat == 0)
            {
                blkStatusTransport(COMMAND_STATUS_PASS);
            }
            else
            {
                blkStatusTransport(COMMAND_STATUS_FAIL);
            }
        }
        else
        {
            blkStatusTransport(PHASE_ERROR);
        }        
    }
    else if (blkx.cmd.dataTransferLength > 0)
    /* data transport host -> device */
    {
        if (blkx.xferError)
        /* case 10 or 13 */
        {
            if (blkx.bytesOut)
            {
                padBytesOut(blkx.bytesOut);
            }
            blkStatusTransport(PHASE_ERROR);
        }
        else if (blkx.bytesOut)
        /* case 9, 11 or 12 */
        {
            padBytesOut(blkx.bytesOut);
            blkx.stat.dataResidue = blkx.bytesOut;
            if (scsiCmdStat == 0)
            {
                blkStatusTransport(COMMAND_STATUS_PASS);
            }
            else
            {
                blkStatusTransport(COMMAND_STATUS_FAIL);
            }
        }
        else
        {
            blkStatusTransport(PHASE_ERROR);
        }
    }
    else
    /* no data transport */
    {
        /* case 2 or case 3 */
        blkStatusTransport(PHASE_ERROR);
    }
}



/***                Public Functions            ***/

/*! Writes to bulk in endpoint as part of a bulk data transaction.\n
 *  Number of bytes sent is internally
 *  guarded so as not to exceed maximum specified in bulk transaction setup packet. If an attempt is
 *  made to send more bytes than requested by host, bulkx.xferError is set to 1 and write is not
 *  performed. Function tracks total bytes sent over numerous function calls.
 *
 *  \param buffer Pointer to buffer to write from.
 *  \param length Number of bytes to write.
 *  \return Returns 0 on success, 1 on error.
 */
uint8_t writeBulkIn(uint8_t *buffer, uint8_t length)
{
    /* check if host is expecting less than what is being attempted to be sent */
    if (length > blkx.bytesIn)
    {
        blkx.xferError = 1;
        return 1;
    }
    /* check if write fails */
    if (writeEp2(buffer, length) != 0)
    {
        return 1;
    }
    /* update byte in counter */
    blkx.bytesIn -= length;
    return 0;
}



/*! Same functionality as writeBulkIn() but utilizing buffer in program memory instead of SRAM.
 *
 *  \param buffer Pointer to program memory buffer to write from.
 *  \param length Number of bytes to write.
 *  \return Returns 0 on success, 1 on error.
 */
uint8_t writeBulkIn_P(prog_uint8_t *buffer, uint8_t length)
{
    /* check if host is expecting less than what is being attempted to be sent */
    if (length > blkx.bytesIn)
    {
        blkx.xferError = 1;
        return 1;
    }
    /* check if write fails */
    if (writeEp2_P(buffer, length) != 0)
    {
        return 1;
    }
    /* update byte in counter */
    blkx.bytesIn -= length;
    return 0;
}



/*! Reads from bulk out endpoint as part of a bulk data transaction.\n
 *  Number of bytes read is internally guarded and blkx.xferError is set to 1 if more bytes have been
 *  received than were specified to be available in bulk transfer setup packet.\n
 *  Function tracks number of bytes read over multiple function calls for the purpose of error tracking,
 *  but returns only the number of bytes read in each particular function call.
 *
 *  \param buffer Pointer to buffer to read into.
 *  \return Returns number of bytes successfully read.
 */
uint8_t readBulkOut(uint8_t *buffer)
{
    uint8_t length;
    length = readEp1(buffer);
    /* check if more bytes have been received than host specified */
    if (length > blkx.bytesOut)
    {
        blkx.bytesOut = 0;
        blkx.xferError = 1;
    }
    blkx.bytesOut -= length;
    return length;
}



/*! Response to BUS_RESET USB event.\n
 *  Any initialization of the mass-storage class should take place here
 */
void processBusReset(void)
{
    scsiReset();
}



/*! Response to SETUP_PACKET_AVAILABLE USB event.\n
 *  Get USB setup packet, parse, and process accordingly.\n
 *  Function returns when processing/communication is complete.
 */
void processSetupPacket(void)
{
    usbSetupPacket_t setup;
    
    if (getSetupPacket((uint8_t *)&setup) != 0)
    {
        stallEp(0);
        return;
    }
    
    /* mask everything but bits 5 and 6 which define the request type */
    switch (setup.requestType & 0x60)
    {
        case REQUEST_TYPE_STANDARD:
            processStandardRequest(&setup);
            break;
            
        case REQUEST_TYPE_CLASS:
            processClassRequest(&setup);
            break;
            
        case REQUEST_TYPE_VENDOR:
            stallEp(0);
            break;
            
        default:
            stallEp(0);
            break;
    }
}



/*! Response to EP1_OUT_DATA USB event.\n
 *  Gets bulk transaction CBW, parses and performs command, sends bulk transaction CSW, and handles
 *  communication errors.\n
 *  Returns when communication is complete.
 */
void processBulkOutTransaction(void)
{
    /*  SCSI Transaction struct used by USB library to communicate with SCSI library */
    scsiTransaction_t scsix =
    {
        .cdb = blkx.cmd.cb,
        .write = &writeBulkIn,
        .write_P = &writeBulkIn_P,
        .read = &readBulkOut,
    };

    /* initialize bulk transaction data */
    blkx.bytesIn = 0;
    blkx.bytesOut = 0;
    blkx.xferError = 0; 
    
    
    /* command transport */
    if (blkCommandTransport() != 0)
    {
        stallEp(1);
        return;
    }
    
    /* initialize data transport limits/checks */
    if ((blkx.cmd.flags & 0x80) && blkx.cmd.dataTransferLength > 0)
    {
        /* data transport device -> host */
        blkx.bytesIn = blkx.cmd.dataTransferLength;
    }
    else if (blkx.cmd.dataTransferLength > 0)
    {
        /* data transport host -> device */
        blkx.bytesOut = blkx.cmd.dataTransferLength;
    }
    
    /* process SCSI command */
    processScsiCommand(&scsix);
    
    
    /* check data transport */
    if (blkx.xferError || blkx.bytesIn || blkx.bytesOut)
    {
        /* bulk transfer data management error has occurred */
        handleBulkTransportError(scsix.cmdStatus);
    }
    else
    {
        blkx.stat.dataResidue = 0;
        if (scsix.cmdStatus == 0)
        {
            blkStatusTransport(COMMAND_STATUS_PASS);
        }
        else
        {
            blkStatusTransport(COMMAND_STATUS_FAIL);
        }
    }
}