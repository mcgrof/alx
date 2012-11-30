include target/linux/port.mk
#include target/freebsd/port.mk

all: help

help:
	@echo Possible build options:
	@echo
	@make -s linux-help
	@#make -s freebsd-help

.PHONY: help
