ifdef INSIDE_EMACS
	FILTER = 2>&1 | sed 's/src\///'
else
	FILTER = 
endif

upload:
	cd .. && (pio run --target upload) $(FILTER)

build:
	cd .. && (pio run) $(FILTER)

clean:
	cd .. && platformio run --target clean

monitor:
	cd .. && platformio device monitor

init:
	pio project init --ide emacs --board pro16MHzatmega328

