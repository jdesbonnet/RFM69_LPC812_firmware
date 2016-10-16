/**
 * RFM98 configuration parameters (application specific).
 *
 * Joe Desbonnet, jdesbonnet@gmail.com
 *
 */

#include "config.h"
#include "rfm98_rdl.h"

/*
 * RFM9x register configuration for packet operation on 433MHz
 */
const uint8_t RFM98_CONFIG[][2] = {


	// Power Amplifier Configuration
	{ RFM98_PACONFIG,
		RFM98_PACONFIG_PaSelect_VALUE(RFM98_PACONFIG_PaSelect_ON)
    	| RFM98_PACONFIG_MaxPower_VALUE(4)
		| RFM98_PACONFIG_OutputPower_VALUE(15)
	},

    // Configuration Terminator
    {255, 0}
  };
