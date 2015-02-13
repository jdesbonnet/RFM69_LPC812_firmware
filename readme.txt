LPC810/LPC812 firmware to drive RFM69HW radio modules
for WRSC2014.  This is an alpha-release 0.1.0.
Documentation available in shared Google Docs 
(ask for access links).

Built with LPCxpresso v7.3.0 (free IDE from NXP). 
GCC make file to  follow soon.

Draft documentation on the UART protocol is
here:
https://docs.google.com/document/d/1FBZINVb_g0gnWlEkQYNBEHTwiyHNY0fn5J37EqBPyog/edit?usp=sharing


Joe Desbonnet, jdesbonnet@gmail.com
7 Sep 2014

Release History:

0.4.0, 11 Feb 2014
 * Supports saving settings in flash memory
 * Start to remove LPC810 legacy.
 * Display config at boot time
 * Display self-test status at boot time
 
 0.4.1, 12 Feb 2014
 * WDT timer set command working
 * MCU Unique ID working properly
 * Read MCU unique ID to auto implement V1B PCB hack
 