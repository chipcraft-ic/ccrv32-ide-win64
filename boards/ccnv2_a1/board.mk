CCPROG_OTHER_FLAGS    := -b $(CHIPCRAFT_SDK_DBG_BAUDRATE)
FLASH_DRIVER          := flash/tsmc40ulpfmc.c
ACQENG_DRIVER         := acqeng/acqeng.c
STREAMER_DRIVER       := streamer/streamer.c
COMMON_SOURCES        += $(FLASH_DRIVER) $(ACQENG_DRIVER) $(STREAMER_DRIVER)

ram-write: $(PROGSREC)
	$(Q)$(CCPROG) $(CCPROG_FLAGS) $(PROGSREC)

flash-write: $(PROGSREC)
	$(Q)$(CCPROG) $(CCPROG_FLAGS) --data_flash $(PROGSREC)

flash-erase: $(PROGSREC)
	$(Q)$(CCPROG) $(CCPROG_FLAGS) --data_flash --erase $(PROGSREC)

