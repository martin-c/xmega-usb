/*! \file
 *  max-3420-usb.h
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
 *  Header file for basic SPI communication with MAXIM 3420E USB Peripheral Controller.
 *
 *  MAX3420 is connected to XMEGA USART peripheral, which is configured to run in master SPI mode.
 *  Using the USART peripheral as opposed to the SPI peripheral allows use of additional hardware
 *  data buffers thus simplifying interface code.
 *
 *  In addition to SPI interface connections, INT1 pin from MAX3420 is connected to a GPIO pin on
 *  the XMEGA to facilitate asynchronous detection of MAX3420 interrupts on the AVR.
 *  Finally, a method needs to be in place to detect the presence of USB bus voltage in a self
 *  powered USB device. This is implemented externally and may require the connection of MAX3420
 *  GPX pin to the AVR.
 *  See "MAX3420E Programming Guide" HOSCSTEN programming notes for more details.
 *  In this case vBus detection is provided externally by vBusHi().
 */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>



/***            Definitions             ***/

/*! Enable AVR interrupt to detect low level on MAX3420 INT1 pin.
 *  NOTE: Since AVR interrupt handler disables further pin interrupts the interrupt needs to be 
 *  enabled before each use.
 */
#define MAX_INT1_ENABLE         PORTE.INT0MASK |= PIN0_bm

/*! Check if MAX3420 INT1 pin is low
 */
#define MAX_INT1_ACTIVE         bit_is_clear(PORTE.IN, PIN0_bp)

/*! Check if MAX3420 INT1 pin is not low
 */
#define MAX_INT1_INACTIVE       bit_is_set(PORTE.IN, PIN0_bp)

/* suitable only for bus powered device */
#define vBusHi()        1



/***            Public Variables        ***/

/*! USB Event codes returned by getUsbEvent()
 *  - USB_VBUS_LOST\n
 *      USB Bus voltage not present, as determined by external function vBusHi().
 *  - BUS_RESET\n
 *      USB Reset has been detected. Returned when bus reset is complete and private 
 *      maxProcessBusReset() function has completed.
 *  - SETUP_PACKET_AVAILABLE\n
 *      A USB setup packed has been received and is valid. Automatically clears stall flag on EP0.
 *  - EP0_OUT_DATA\n
 *      Endpoint 0 Out has data available (unused - SETUP_PACKET_AVAILABLE used instead).
 *  - EP1_OUT_DATA\n
 *      Endpoint 1 Out has data available.
 *  - USB_SUSPEND\n
 *      USB suspend condition detected (no bus traffic for 3ms). This event will be returned every
 *      3 ms until usbSuspend() is called which clears interrupt and powers down MAX3420, thus
 *      terminating further suspend events.
 *      See MAXIM AN3661 "SUSP Interrupt Request Bit".\n
 *      Since unplugging of a USB device will also cause an absence of USB traffic, but a small
 *      amount of time is required to discharge USB vBus capacitors, it is likely that this event
 *      will also be returned when a self powered device is unplugged before the USB_VBUS_LOST event
 *      is returned.\n
 *      This is the main reason external function vBusHi() is required. Since the proper response to
 *      a suspend event may be powering down the MAX3420, and the MAX3420 VBUS interrupts require the
 *      '3420 oscillator to be running to update, an asynchronous method is required to detect the
 *      loss of Vbus.
 *  - USB_EVENT_ERROR\n
 *      A MAX3420 interrupt has been generated, but none of the interrupt flags are set to cause the
 *      return of any of the above events.
 */
typedef enum usbEvent
{
    USB_VBUS_LOST,
    BUS_RESET,
    SETUP_PACKET_AVAILABLE,
    EP0_OUT_DATA,
    EP1_OUT_DATA,
    USB_SUSPEND,
    USB_EVENT_ERROR,
} usbEvent_t;



/***        Public Functions            ***/
void initMax3420(void);
void initUsb(void);
void terminateUsb(void);
void maxPowerDown(void);
usbEvent_t getUsbEvent(void);
uint8_t getSetupPacket(uint8_t *buffer);
void setDeviceAddress(uint8_t address);
void usbSuspend(void);
void stallEp(uint8_t ep);
void clearStallEp(uint8_t ep);
uint8_t isEpStalled(uint8_t ep);
uint8_t writeEp0(uint8_t *buffer, uint8_t length);
uint8_t writeEp0_P(prog_uint8_t *buffer, uint8_t length);
void ackStatus(void);
uint8_t readEp1(uint8_t *buffer);
uint8_t writeEp2(uint8_t *buffer, uint8_t length);
uint8_t writeEp2_P(prog_uint8_t *buffer, uint8_t length);