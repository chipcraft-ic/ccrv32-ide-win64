-include ../../common/legacy-env-var-support.mk

PROGNAME      := flash-cache-test

COMMON_SOURCES := syscalls.c uart.c

CPPFLAGS += -DTSMC40ULPFMC_RAM_BUFFERS_DEBUG=0 -DTSMC40ULPFMC_RAM_BUFFERS=3U

#CFLAGS_OPT = -O0 -ggdb

include $(CHIPCRAFT_SDK_HOME)/common/generic.mk
