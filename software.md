# Software Design

### DMX-512-A Implementation
#### UART1   -> on PC5,PC4
* 8 bit data frame, 2 stop bits, FIFOs enabled
* clocked to system clock
* Interrupt enabled for 1/8th TX FIFO empty
* baud rate 250 kHz
#### TIMER1
* 32 bit mode
* One-Shot mode count down
* 
	