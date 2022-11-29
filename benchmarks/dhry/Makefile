-include ../../common/legacy-env-var-support.mk

PROGNAME   := dhry
COMMON_DIR := $(CHIPCRAFT_SDK_HOME)/common

DEFAULT_COMMON_SOURCES ?= board.c
COMMON_SOURCES := syscalls.c libc_multicore.c uart.c

ifndef RUNS_NUMBER
	CPPFLAGS += -DRUNS_NUMBER=1000000
	CPPFLAGS += -DPRINT_ALL
else
	CPPFLAGS += -DRUNS_NUMBER=$(RUNS_NUMBER)
endif

#CPPFLAGS += -DREG=register
CPPFLAGS += -DHZ=1000000 # see _CLOCKS_PER_SEC_ definition in NewLib
CPPFLAGS += -DNO_DEFAULT_SBRK -DNO_DEFAULT_MULTICORE
CPPFLAGS += -D_STARTUP_STACK_SIZE=4096

CFLAGS_WARN := -Wno-implicit-function-declaration
CFLAGS_OPT  := -O3 -fno-inline-functions

LDFLAGS += -u _printf_float

include $(COMMON_DIR)/generic.mk

