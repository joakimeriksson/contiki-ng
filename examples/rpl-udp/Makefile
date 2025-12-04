CONTIKI_PROJECT = udp-client udp-server
all: $(CONTIKI_PROJECT)

CONTIKI=../..

# Enable shell for interactive debugging
MODULES += $(CONTIKI_NG_SERVICES_DIR)/shell

# Override CSMA ACK wait time for sub-GHz prop-mode (10ms instead of ~3.3ms)
# This helps with ACK reception on slower sub-GHz links
CFLAGS += -DCSMA_CONF_ACK_WAIT_TIME='(RTIMER_SECOND/100)'

include $(CONTIKI)/Makefile.include
