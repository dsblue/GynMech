GCCFLAGS=-g -O0 -Wall -mmcu=atmega168 
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt -lm
AVRDUDEFLAGS=-c avr109 -p m168 -b 115200 -P /dev/ttyUSB0
LINKOBJECTS=libnerdkits/delay.o libnerdkits/lcd.o libnerdkits/uart.o

all:	gynmech-upload

gynmech.hex:	gynmech.c
	make -C libnerdkits
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o gynmech.o gynmech.c ${LINKOBJECTS}
	avr-objcopy -j .text -O ihex gynmech.o gynmech.hex

gynmech.ass:	gynmech.hex
	avr-objdump -S -d gynmech.o > gynmech.ass

gynmech-upload:	gynmech.hex
	avrdude ${AVRDUDEFLAGS} -e
	sleep 0.1
	avrdude ${AVRDUDEFLAGS} -D -U flash:w:gynmech.hex:a
