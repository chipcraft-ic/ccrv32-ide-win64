-include ../../common/legacy-env-var-support.mk

PROGNAME   := whetstone
COMMON_DIR := $(CHIPCRAFT_SDK_HOME)/common

ifdef LOOPS
	CPPFLAGS += -DLOOPS=$(LOOPS)
endif

ifdef PRINTOUT
	CPPFLAGS += -DPRINTOUT
endif

COMMON_SOURCES := syscalls.c uart.c

LDFLAGS += -u _printf_float
LDLIBS += -lm

include $(COMMON_DIR)/generic.mk
