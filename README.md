# Embedded II Project: DMX512 Controller/Device

In this repository you will find two source files containing main code:
1) main.c
	containing the final version of the source code for the project, refactored and improved from the initial 	source...
2) alejandro\_waumann-george\_hinkel.c
	containing the initial rough draft source code, while it compiles it does not function correctly
	
# Project Description

### DMX-512A Protocol
The DMX-512A protocol is often used to control lights. It can control the brightness of the light, servos that can move the lights, and other peripherals involved. Each controller can control up to 512 devices. It sends out the data at a rate of 250 Kbaud.

The protocol calls for a break of 92 us, at a minimum, followed by a mark after break with a minimum duration of 12 us. After this, a start code is sent (null byte) and then then data can be sent. The data consists of up to 512 bytes, each containing a start bit and 2 stop bits. The following break that is sent indicates the end of transmission.

### Theory of Operation
Our project implements the DMX-512A protocol for a controller and for a device. The device is able to switch between the modes to either control devices or be controlled by a controller. This is done via a serial interface using the virtual COMS port of the device.

In controller mode, the device is able to respond to various commands. Primarily, transmission can be turned on and off. When transmit is set to ’on’, a 176 us one-shot timer is triggered with the drive pin set ’low’ to send a break. In the timer interrupt, a 12 us one-shot timer is triggered with the drive pin set ’high’ to send a mark after break (MAB). When the interrupt is triggered again it will send a null byte (start code) followed by M data frames, where M is the max number of addresses to transmit (configureable in serial interface).

In device mode, the device is able to change its address via the serial interface. Once it recieves the start code, it will start to read the address values into a buffer. Upon receiving the break, the device will update turn its blue LED on/off to reflect the value recieved for its address.
