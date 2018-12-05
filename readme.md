# Libsensorimotor

Author(s):  

Matthias Kubisch
kubisch@informatik.hu-berlin.de
October 2018


## Building the shared library

The build system used to create the shared library from sources is 'scons'.
Check if scons is already installed on your system:

	scons -v

To install scons type
	
	sudo apt install scons

To build the shared library just type

	scons

the build process should terminate with the message

	'scons: done building targets.'

and you will find the freshly build library in the binary directory

	bin/libsensorimotor.so


## Using the sensorimotor python interface

Before you proceed with programming, make sure you have already set up your USB-to-Serial interface in low-latency mode (see below) and that each of the connected sensorimotors has a unique ID (see 'setting id')


## SETUP SERIAL

For setting up your usb serial device to best work with sensorimotors
you have to enable low latency for your serial (RS485) device(s).

Low latency mode is needed to have the best performance on higher baudrates and with multiple motors attached to the bus.  
In order to temporarily set your interface to low-latency mode type:

	setserial /dev/ttyUSB0 low_latency

when e.g. /dev/ttyUSB0 is your device. If setserial is not installed on your linux machine type 

	sudo apt install setserial
 
If that works out as expected you can make this permanent when creating a udev rule file, e.g

	sudo nano /etc/udev/rules.d/50-ttyusb.rules 

and put in

	KERNEL=="ttyUSB[0-9]*", MODE="0666", RUN+="/bin/setserial /dev/%k low_latency"
	KERNEL=="ttyACM[0-9]*", MODE="0666", RUN+="/bin/setserial /dev/%k low_latency"

The udev rules get activated when your reconnect your USB-to-serial device.

Further reading on how to write udev kernel rules see:
http://www.reactivated.net/writing_udev_rules.html


TODO:

describe modes
- setting IDs
- position
- velocity
- hold/break
- voltage control

- setting limits
	- voltage limit
	- position limit

