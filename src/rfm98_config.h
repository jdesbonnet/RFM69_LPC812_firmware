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
	{ RFM98_MODEMCONFIG1,
		RFM98_MODEMCONFIG1_Bw_VALUE(7)
		| RFM98_MODEMCONFIG1_CodingRate_VALUE(4)
	},

	{ RFM98_MODEMCONFIG2,
		RFM98_MODEMCONFIG2_SpreadFactor_VALUE(12)
	},

#ifdef FEATURE_868MHZ
	{ RFM98_FRFMSB,
			0xD9
	},
	{ RFM98_FRFMID,
			0x00
	},
#endif
    // Configuration Terminator
    {255, 0}
  };
