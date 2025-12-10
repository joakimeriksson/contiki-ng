CONTIKI_PROJECT = serial-radio-main
all: $(CONTIKI_PROJECT)

# Project source files
PROJECT_SOURCEFILES += serial-radio.c

# Include CRC16 from Contiki-NG lib
MODULES += os/lib

# Include nullnet and nullmac
MODULES += os/net/nullnet
MODULES += os/net/mac/nullmac

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
