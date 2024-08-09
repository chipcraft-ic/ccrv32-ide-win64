CCPROG_OTHER_FLAGS    := --jtag
FLASH_DRIVER          := flash/tsmc40ulpfmc.c
ACQENG_DRIVER         := acqeng/acqeng.c
STREAMER_DRIVER       := streamer/streamer.c
COMMON_SOURCES        += $(FLASH_DRIVER) $(ACQENG_DRIVER) $(STREAMER_DRIVER)

flash-write: $(PROGSREC)
	$(Q)$(CCPROG) $(CCPROG_FLAGS) --mcu ccnv2 --flash $(PROGSREC)

flash-erase: $(PROGSREC)
	$(Q)$(CCPROG) $(CCPROG_FLAGS) --mcu ccnv2 --flash --erase $(PROGSREC)

