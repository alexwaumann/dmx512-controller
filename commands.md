# Operation
On initial powerup, the hardware should default to device mode. Subsequent powerups will set the mode based
on the EEPROM value.

LED blinking, UART communications over virtual COM port, DMX512A bus transmission/reception must occur
simultaneously and support full-rate data.

# General Commands
  * device - switch to device mode (save in EEPROM)
  * controller - switch to controller mode (save in EEPROM)

# Controller Commands
  * clear - set data value of all 512 DMX addresses to zero
  * set A, V - set DMX address A to value V
  * get A - return the value at DMX address A
  * on - DMX stream is sent continuously
  * off - DMX stream NOT sent
  * max M - max of M addresses will be sent (default: 512)

LED NOTES:
  * transmit LED should glow to indicate that DMX data is being transmitted
  * receive LED should blink to indicate that a virtual COM port command has been received

# Device Commands
  * address A - set device address to A and use after next break (save in EEPROM)

LED NOTES:
  * transmit LED should be off
  * receive LED should glow to indicate that a DMX signal has been detected
  * receive LED should blink to indicate that a break is not detected in 2 seconds

# Transmit LED
  * transmit: onboard red LED
  * receive: onboard green LED
