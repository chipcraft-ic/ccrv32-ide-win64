-include ../../common/legacy-env-var-support.mk

PROGNAME   := paranoia
COMMON_DIR := $(CHIPCRAFT_SDK_HOME)/common

COMMON_SOURCES := syscalls.c uart.c

LDFLAGS += -u _printf_float
LDLIBS += -lm

CPPFLAGS += -DNOPAUSE
CPPFLAGS += -D_STARTUP_STACK_SIZE=29696

include $(COMMON_DIR)/generic.mk
