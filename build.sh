# !/bin/bash

# A simple bash script to build the NuttX RTOS

#export ESP32 env
. $HOME/esp/esp-idf/export.sh


if [ $1 -eq 1 ]
then
	#distclean
	echo "Dist clean"
	make distclean
	
	#ESP32 configure
	echo "configuring ESP32 with NuttX RTOS"
	./tools/configure.sh esp32-devkitc:nsh
fi

#menuconfig
make menuconfig

#make and flash
make download ESPTOOL_PORT=/dev/ttyUSB0



