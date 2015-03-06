This is firmware to drive my LPC812/RFM69HW radio board. The indended application is for
use with robot boat competitions, but can be used for any telemetry application.

The LPC812 with this firmware acts as a kinda radio operating system, providing a simplified
interface between the RFM69 module and the host application. 

Built with LPCxpresso v7.3.0 (free IDE from NXP). 

Draft documentation on the UART protocol is
here:
https://docs.google.com/document/d/1FBZINVb_g0gnWlEkQYNBEHTwiyHNY0fn5J37EqBPyog/edit?usp=sharing


Joe Desbonnet, jdesbonnet@gmail.com
13 Feb 2015

Release History:

0.5.1, 6 Mar 2015
 * Add 'E' commend to en/disable echo of NMEA from GPS UART port to main UART port.
 * Add GPS fix type and HDOP to 'g' message
 
0.5.0, 1 Mar 2015
 * Support GPS module on second LPC812 UART (USART1). GPS UART TXD to pin 8 (PIO0_11). Reads NMEA 
 sentences from the GPS and echos to the main API UART (USART0).
 
0.4.1, 13 Feb 2015
 * Fix MCU unique ID retrieval bug
 * Use MCU unique ID to implement a hack to fix one faulty PCB board (will be removed later at some point)
 * Use MCU unique ID to assign node addresses from hard-coded table (can be over ridden with N command)
  
0.4.0, 11 Feb 2015
 * Supports saving settings in flash memory
 * Start to remove LPC810 legacy.
 * Display config at boot time
 * Display self-test status at boot time
 
 0.4.1, 12 Feb 2015
 * WDT timer set command working
 * MCU Unique ID working properly
 * Read MCU unique ID to auto implement V1B PCB hack
 
