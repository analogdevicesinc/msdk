# This file is used by the Makefile system to extend the functionality
# of certain recipes to integrate with the Secure Boot Tools (SBT).
# It adds a few Make goals specific to the SBTs.

# The SLA target.
# "make sla" will generate a .sbin file, and then generate scp packets
# using the "build_scp_session" tool.
.PHONY: sla
sla: release
	arm-none-eabi-size --format=berkeley $(BUILD_DIR)/$(PROJECT).elf
	$(CA_SIGN_BUILD) -c $(TARGET_SEC) key_file=$(TEST_KEY) ca=$(BUILD_DIR)/$(PROJECT).bin sca=$(BUILD_DIR)/$(PROJECT).sbin
	@echo " "
	arm-none-eabi-objcopy  $(BUILD_DIR)/$(PROJECT).elf --update-section .sig=$(BUILD_DIR)/$(PROJECT).sig
	$(BUILD_SESSION) -c $(TARGET_SEC) key_file=$(TEST_KEY) ${SCP_PACKETS} $(BUILD_DIR)/$(PROJECT).sbin

# The SCPA target.
# "make scpa" is a special rule for SCPA applet programs, which are
# special examples that load a program into RAM to extend the secure
# ROM functionality.  It is mostly the same as the sla rule, except
# some special modifications are made to the srec file and scp packets.
# It depends on an "scp_script.txt" file
.PHONY:scpa
scpa: release
	arm-none-eabi-size --format=berkeley $(BUILD_DIR)/$(PROJECT).elf
	@echo " "
	arm-none-eabi-objcopy -O srec -j .text -j .data -j.scpa_header -j.scpa_init -j.scpa_ops --srec-forceS3 --srec-len=128 $(BUILD_DIR)/$(PROJECT).elf  $(BUILD_DIR)/$(PROJECT).srec
	@echo " "
	@echo "Generating SCP Package"
	$(BUILD_SESSION) -c $(TARGET_SEC) key_file=$(TEST_KEY) script_file=scp_script.txt ${SCP_PACKETS}