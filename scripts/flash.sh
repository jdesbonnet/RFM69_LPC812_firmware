#!/bin/bash
LPC21ISP=/home/joe/Dropbox/Projects/LPC1114/lpc21isp_197/lpc21isp
BINFILE=./Debug/LPC8xx_RFM69.bin

$LPC21ISP -control -bin $BINFILE  $1 115200  120000

