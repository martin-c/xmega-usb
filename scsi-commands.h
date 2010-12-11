/*! \file
 *  scsi-commands.h
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
 *  Simple library which implements SPC-3 compliant SCSI device on top of small flash memory.\n
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>



/***            Definitions             ***/



/***         Public Variables           ***/
/*! SCSI Transaction struct used by USB library to communicate with SCSI library.\n
 *  SCSI Library must be able to:
 *  - Receive Commands
 *  - Write data read from SCSI device to some data bus
 *  - Read data from some data bus to write to SCSI device
 *  - Return command status codes
 *
 *  This struct provides a way to communicate data necessary to accomplish these tasks to the SCSI
 *  library that is as flexible as possible.
 */
typedef struct
{
    uint8_t cmdStatus;                          ///< SCSI command status: 0 ok, 1 fail
    uint8_t *cdb;                               ///< pointer to beginning of Command Descriptor Block
    uint8_t (*write)(uint8_t *, uint8_t);       ///< function pointer to write to data bus
    uint8_t (*write_P)(prog_uint8_t *, uint8_t);///< function pointer to write data bus from program space
    uint8_t (*read)(uint8_t *);                 ///< function pointer to read from data bus
} scsiTransaction_t;



/***         Public Functions           ***/
void scsiReset(void);
void scsiWriteEnable(uint8_t we);
void processScsiCommand(scsiTransaction_t *x);