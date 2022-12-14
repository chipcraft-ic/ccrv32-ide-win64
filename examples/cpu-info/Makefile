-include ../../common/legacy-env-var-support.mk

PROGNAME      := cpu-info

COMMON_SOURCES := syscalls.c uart.c

LDFLAGS += -u _printf_float

include $(CHIPCRAFT_SDK_HOME)/common/generic.mk
