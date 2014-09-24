#ifndef FLAGS_H_
#define FLAGS_H_

// Radio and MCU in sleep state
#define MODE_ALL_OFF (0)

// MCU on, radio in SLEEP (currently not supported)
#define MODE_RADIO_OFF (1)

// Radio and MCU in sleep state with period ping and listen
#define MODE_LOW_POWER_POLL (2)

// Radio in RX, MCU polling radio.
#define MODE_AWAKE (3)

// Forward all received frames to UART
#define FLAG_PROMISCUOUS_MODE (1<<4)


#endif
