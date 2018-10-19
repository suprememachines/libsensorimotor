#Libsensorimotor

Author(s):  

Matthias Kubisch
kubisch@informatik.hu-berlin.de
October 2018


##Building the shared library

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


##Using the sensorimotor python interface

Before you proceed with programming, make sure you have already set up your USB-to-Serial interface in low-latency mode (link?) and that each of the connected sensorimotors has a unique ID (see 'setting id')

TODO:

describe modes
- position
- velocity
- hold/break
- voltage control

- setting limits
	- voltage limit
	- position limit

