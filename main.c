/*! \file
 *  main.c
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

/*! \mainpage xmega-usb: USB library for AVR XMEGA utilizing Maxim MAX3420E\n
 *  \section introduction Introduction
 *  The goal of this library is to bring USB connectivity to the AVR XMEGA series of micrcontrollers
 *  by utilizing the Maxim MAX3420E USB Peripheral Controller with SPI Interface. The library could
 *  also be easily adapted to work with a different AVR processor and perhaps different USB hardware
 *  since modularity was a major goal during coding.\n
 *  At this point the library supports the USB Mass Storage Class, implementing a SCSI compliant 
 *  LBA device utilizing an external flash memory.\n
 *  Future plans include adding other USB device capabilities such as a Communications Device Class
 *  and Human Interface Device Class support.\n
 *  The library has been tested on a prototype device which acts as a portable data logger and uses
 *  the USB interface to make logged data available to a PC.\n
 *
 *  \section software Software
 *
 *  \subsection module-overview Module Overview
 *  The library is divided into several modules:
 *  - MAX3420E Interface and Communications Code (max-3420.h, max-3420.c)
 *  - USB Mass Storage Class (usb-mass-storage.h, usb-mass-storage.c)
 *  - SCSI Command Library (scsi-commands.h, scsi-commands.c)
 *  - Flash Memory Library (spi-flash-memory.h, spi-flash-memory.c)
 *  
 *  \subsection max3420 MAX3420E Interface and Communications Code
 *  The max-3420 source code includes functions for basic communication with the '3420, including 
 *  initialization, USB connection, USB termination, and USB reset. It also includes functions for
 *  reading and writing the USB endpoints which the '3420 provides.\n
 *  Most importantly, the library provides the getUsbEvent() function which forms the basis of the
 *  "event loop" for USB connectivity. Basically getUsbEvent() waits for certain events such as USB
 *  reset, setup packet, or data available on an endpoint. The function then returns the event type,
 *  and the higher level USB library then handles the event using the utility and communications
 *  functions also provided by the library.
 *
 *  \subsection usb-mass-storage USB Mass Storage Class
 *  The USB Mass Storage Class library provides the functions for basic functionality of a USB mass
 *  storage device. This includes standard device and configuration descriptors; functions to handle
 *  standard USB device, interface, and endpoint requests; functions to handle bulk data transfers, and
 *  miscellaneous other details.\n
 *  The central functions in this library are processSetupPacket(), writeBulkIn(), and readBulkOut().
 *  processSetupPacket() is used to respond to a USB setup packet and thereby processes standard
 *  requests over the control pipe.\n
 *  writeBulkIn() and readBulkOut() are provided to send and receive data over the bulk transfer
 *  pipes which are part of any Mass Storage Class device. These functions not only act as a way to
 *  send data, but also make sure amount of data sent or received complies with amount specified in
 *  bulk CDB.
 *
 *  \subsection scsi-commands SCSI Command Library
 *  The SCSI Command Library provides a way to decode and execute SCSI commands sent by the USB host
 *  in the bulk transfer CB. These commands are what provide the low level interface to the data
 *  storage media for a Mass Storage Device.\n
 *  The most important function in this library is processScsiCommand() which is called by the USB
 *  Mass Storage library once a CBW has been received over the bulk transfer pipe.
 *
 *  \subsection spi-flash-memory Flash Memory Library
 *  The flash memory library provides basic data read/write/erase functionality. For this particular
 *  library, the flash memory is connected via SPI. The protocol is built for the Atmel AT26 series
 *  memories, although it could be adapted to work with other devices.\n
 *  This library is utilized by the SCSI library for actually reading/writing to flash memory.
 *
 *  \subsection v-bus USB Vbus Detection
 *  The entire library relies on a function to detect when +5V is present on the USB bus called
 *  vBusHi().\n
 *  This function must be provided by external code, since there are numerous ways to implement
 *  this. vBusHi() is expected to return 1 if Vbus is present, and 0 otherwise.\n
 *  Obviously in a bus powered application this sort of detection is unimportant, so in this case
 *  a simple macro will suffice:
 *  \code #define vBusHi()  1\endcode \n
 *  However, in a self powered application things get more complicated. See USB_SUSPEND and 
 *  "MAX3420E Programming Guide" HOSCSTEN programming notes for more details.
 *
 *  \subsection delay-loops Delay Loops
 *  In a few instances there are small delay loops in the code which reduce unnecessary SPI bus
 *  traffic when waiting for events such as MAX3420 oscillator stabilization and AT26 erase progress.
 *  At the moment, these delay loops are implemented using the avr-libc _delay_ms() and _delay_us()
 *  functions.
 *
 *  A problem arises when using these functions with numerous cpu clock frequencies which
 *  change at runtime. Since the functions rely on a compile time constant for clock frequency,
 *  changing this clock frequency on the fly changes the delay which these functions cause.
 *  In the reference design, F_CPU is defined as 2000000 (2 MHz), but the activation of the USB link
 *  causes CPU frequency to be changed to 32MHz. Thus, the delay loops are off by a factor of 16
 *  if this frequency change is not compensated for. This is why the following can be found in
 *  the code:\n
 *  \code _delay_ms(160.0);               // delay 10ms \endcode
 *  The actual delay is 10ms; The delay loop is calibrated for 2MHz CPU frequency, but the CPU
 *  frequency in this block of code is actually 32 MHz. Therefore the delay is increased 16x to
 *  produce the proper delay.
 *
 *  <b>These delay loop calls should be updated to produce the delay specified in the comment.</b>\n
 *  A frequency independent delay loops needs to be implemented in the long term.
 *
 *  \subsection leds-switches LEDs and Switches
 *  The turning on and off of user interface LEDs occurs in the flash memory library to show memory
 *  activity. The switches are used to detect if read-only or read-write flash memory access over USB
 *  should be enabled. The following macros are defined in a custom file called "io.h":\n
 *  \code
 *  #define     LED_GREEN_1_ON              PORTC.OUTSET = PIN4_bm
 *  #define     LED_GREEN_1_OFF             PORTC.OUTCLR = PIN4_bm
 *  #define     BUTTON_1_PRESSED            bit_is_clear(PORTA.IN, PIN4_bp)
 *  #define     BUTTON_2_PRESSED            bit_is_clear(PORTD.IN, PIN4_bp)
 *  \endcode
 *  It is up to the user to define these macros as necessary.
 * 
 *  \section hardware Hardware
 *  The basic hardware requirements are as follows:
 *  - XMEGA Device with 2 free USARTs in SPI mode
 *  - MAX3420E on SPI interface
 *  - AT26 flash memory on SPI interface
 *  See reference design schematic and device datasheets for details.\n
 *  In the reference design Vbus detection is done with an onboard analog comparator. This is not
 *  however the only way of doing this. It is also possible to utilize the GPX pin of the MAX3420E
 *  which would need to be connected to the XMEGA and properly configured. See USB_SUSPEND and 
 *  "MAX3420E Programming Guide" HOSCSTEN programming notes for more information.
 *
 *  \section references References
 *  The following references were useful in developing this library:
 *  - USB
 *      - <a href="http://www.amazon.com/USB-Mass-Storage-Designing-Programming/dp/1931448043/ref=sr_1_1?ie=UTF8&s=books&qid=1292037633&sr=8-1">
 *          USB Mass Storage by Jan Axelson</a>
 *      - Universal Serial Bus Specification Rev. 2.0
 *      - Universal Serial Bus Mass Storage Class Bulk-Only Transport Rev. 1.0
 *      - <a href="http://www.beyondlogic.org/usbnutshell/usb-in-a-nutshell.pdf">
 *          USB in a Nutshell. Making Sense of the USB Standard.</a>
 *  - XMEGA
 *      - <a href="http://www.atmel.com/dyn/resources/prod_documents/doc8077.pdf">XMEGA A Manual</a>
 *      - <a href="http://www.atmel.com/dyn/resources/prod_documents/doc8069.pdf">ATxmegaA4 Datasheet</a>
 *      - <a href="http://www.atmel.com/dyn/resources/prod_documents/doc8049.pdf">
 *          AVR1307: Using the XMEGA USART</a>
 *  - MAX3420E
 *      - <a href="http://datasheets.maxim-ic.com/en/ds/MAX3420E.pdf">MAX3420E datasheet</a>
 *      - <a href="http://www.maxim-ic.com/app-notes/index.mvp/id/3598">MAX3420E Programming Guide</a>
 *      - <a href="http://www.maxim-ic.com/app-notes/index.mvp/id/3661">
 *          The MAX3420E Interrupt System</a>
 *      - <a href="http://www.maxim-ic.com/app-notes/index.mvp/id/3663">
 *          Bringing Up a MAX3420E System</a>
 *      - <a href="http://pdfserv.maxim-ic.com/en/an/AN3690.pdf">
 *          USB Enumeration Code (and More) for the MAX3420E</a>
 *  - AT26 Flash
 *      - <a href="http://www.atmel.com/dyn/resources/prod_documents/doc3600.pdf">
 *          AT26DF081A Datasheet</a>
 *  - USB Protocol Debugging
 *      - <a href="http://www.totalphase.com/products/beagle_usb12/">
 *          Beagle USB12 Protocol Analyzer</a>
 */




#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "max-3420.h"
#include "usb-mass-storage.h"
#include "scsi-commands.h"
#include "spi-flash-memory.h"



/* UI switches, used to enable flash write */
#define     BUTTON_1_PRESSED            bit_is_clear(PORTA.IN, PIN4_bp)
#define     BUTTON_2_PRESSED            bit_is_clear(PORTD.IN, PIN4_bp)



void usbConnect(void);



/*! Main USB event loop
 */
void usbConnect(void)
{
    _delay_ms(400.0);
    /* init USB Connect state */
    initUsb();
    if (BUTTON_1_PRESSED && BUTTON_2_PRESSED)
    {
        scsiWriteEnable(1);
    }
    else
    {
        scsiWriteEnable(0);
    }
    
    /* get and process USB event while Vbus is present */
    while (vBusHi())
    {
        usbEvent_t event = getUsbEvent();
        
        switch (event)
        {
            case USB_VBUS_LOST:
                break;
                
            case BUS_RESET:
                processBusReset();
                break;
                
            case SETUP_PACKET_AVAILABLE:
                processSetupPacket();
                break;
                
            case EP1_OUT_DATA:
                processBulkOutTransaction();
                break;
                
            case USB_SUSPEND:
                usbSuspend();
                break;
                
            default:
                break;
        }
    }
    
    /* return from USB connect state */
    terminateUsb();
}



int main (void)
{
    
    _delay_ms(1000);
    
    /* config LEDs */
    PORTC.DIRSET = PIN4_bm;
    
    PMIC.CTRL |= PMIC_MEDLVLEN_bm;  // enable medium level interrupts
    sei();                          // enable global interrupts
    
    initFlash();
    initMax3420();
    
    while (1)
    {
        /* connect to USB host */
        if (vBusHi())
        {
            usbConnect();
        }
        
        sleep_mode();
    }
}