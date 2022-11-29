CCPROG_OTHER_FLAGS    := -b $(CHIPCRAFT_SDK_DBG_BAUDRATE)

MAX2771_DRIVER        := max2771/max2771.c
ACQENG_DRIVER         := acqeng/acqeng.c
STREAMER_DRIVER       := streamer/streamer.c

COMMON_SOURCES        += $(MAX2771_DRIVER) $(ACQENG_DRIVER) $(STREAMER_DRIVER)

ram-write: $(PROGSREC)
	$(Q)$(CCPROG) $(CCPROG_FLAGS) $(PROGSREC)

flash-write: $(PROGSREC)
	$(Q)$(CCPROG) $(CCPROG_FLAGS) --data_flash $(PROGSREC)

flash-erase: $(PROGSREC)
	$(Q)$(CCPROG) $(CCPROG_FLAGS) --data_flash --erase $(PROGSREC)

