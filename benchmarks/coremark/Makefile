-include ../../common/legacy-env-var-support.mk

PROGNAME   := coremark
COMMON_DIR := $(CHIPCRAFT_SDK_HOME)/common

CORE_FILES = core_list_join core_main core_matrix core_state core_util
SOURCES += $(addprefix coremark_v1.0/, $(addsuffix .c,$(CORE_FILES)))
SOURCES += $(wildcard ccrv32_port/*.c) $(wildcard ccrv32_port/*.S)

COMMON_SOURCES := syscalls.c uart.c core.S crc-verification.c

CFLAGS_WARN := -Wall -Wno-format
CFLAGS_OPT  := -O3 -fomit-frame-pointer -finline-functions -finline-limit=1000

CPPFLAGS += -I$(CURDIR)/coremark_v1.0 -I$(CURDIR)/ccrv32_port
CPPFLAGS += -DFLAGS_STR='"$(CFLAGS_OPT)"'

ifndef ITERATIONS
	CPPFLAGS += -DITERATIONS=3000
else
 CPPFLAGS += -DITERATIONS=$(ITERATIONS)
endif

CFLAGS += -Wno-strict-prototypes

CPPFLAGS += -D_STARTUP_STACK_SIZE=8192

LDFLAGS += -u _printf_float

ifndef MULTITHREAD
	ifeq ($(CHIPCRAFT_SDK_BOARD),ml605)
		CPPFLAGS += -DMULTITHREAD=3
	endif
    ifeq ($(CHIPCRAFT_SDK_BOARD),vcu108)
		CPPFLAGS += -DMULTITHREAD=4
	endif
else
	CPPFLAGS += -DMULTITHREAD=$(MULTITHREAD)
endif

include $(COMMON_DIR)/generic.mk
