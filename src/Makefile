export PATH := $(HOME)/.platformio/penv/bin:$(PATH)

ifdef INSIDE_EMACS
	FILTER = 2>&1 | sed 's/src\///'
else
	FILTER = 
endif

upload:
	cd .. && (platformio run --target upload) $(FILTER)

build:
	cd .. && (platformio run) $(FILTER)

clean:
	cd .. && platformio run --target clean

monitor:
	cd .. && platformio device monitor

init:
	platformio project init --ide emacs --board pro16MHzatmega328

