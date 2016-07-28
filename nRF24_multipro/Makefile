ARD ?= /home/badger/prj/vrtulnicky/arduino-1.6.6/arduino
DISPLAY ?= ":1"

PROGRAM=build/nRF24_multipro.ino.hex

all: ${PROGRAM}

${PROGRAM}: build $(shell ls *.ino *.h Makefile | grep -v 'cfg\.h')
	rm -rf build/sketch
	if [ `hostname` = andree ]; then cp cfg_andree.h cfg.h ; \
	else cp cfg_default.h cfg.h; fi
	touch cfg.h
	DISPLAY=$(DISPLAY) $(ARD) --pref build.path=build --verify nRF24_multipro.ino

build:
	mkdir $@

clean:
	rm -r build

install: ${PROGRAM}
	ps aux|grep -v grep | grep ttyUSB0 | grep -q minicom && killall -9 minicom || true
	avrdude -c avrisp -p m328p -P /dev/ttyUSB0 -b 57600 -U flash:w:${PROGRAM}
