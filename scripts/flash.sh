#!/bin/bash
#
# Script to program LPC8xx_RFMxx board through the UART. The device
# must be put into bootloader mode first. There are two ways of doing
# this.
# 1. Use the RESET and ENTER_ISP pins on the programming header.
# 2. Enter the 'O' UART command to enter bootloader.
#

LPC21ISP=/home/joe/Dropbox/Projects/LPC1114/lpc21isp_197/lpc21isp
BINFILE=./Release/LPC8xx_RFM69.bin

$LPC21ISP -control -bin $BINFILE  $1 115200  120000

