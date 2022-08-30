# This file is used by the Makefile system to extend the functionality
# of certain recipes to integrate with the Secure Boot Tools (SBT).

# 	On release, a secure binary file (.sbin) file will be generated.  The .sbin file can
# be used to generate scpa packets.

# The SLA target.
.PHONY: sla
sla: release
	$(CA_SIGN_BUILD) -c $(TARGET_SEC) key_file=$(TEST_KEY) ca=$(BUILD_DIR)/$(PROJECT).bin sca=$(BUILD_DIR)/$(PROJECT).sbin
	@echo " "
	arm-none-eabi-objcopy  $(BUILD_DIR)/$(PROJECT).elf --update-section .sig=$(BUILD_DIR)/$(PROJECT).sig
	$(BUILD_SESSION) -c $(TARGET_SEC) key_file=$(TEST_KEY) ${SCP_PACKETS} $(BUILD_DIR)/$(PROJECT).sbin

.PHONY:scpa
scpa: release
	arm-none-eabi-size --format=berkeley $(BUILD_DIR)/$(PROJECT).elf
	@echo " "
	arm-none-eabi-objcopy -O srec -j .text -j .data -j.scpa_header -j.scpa_init -j.scpa_ops --srec-forceS3 --srec-len=128 $(BUILD_DIR)/$(PROJECT).elf  $(BUILD_DIR)/$(PROJECT).srec
	@echo " "
	@echo "Updating scp_script file (Rename output file name with project name)"
	$(shell sed -e 's/SCPA_OTP_Dump/$(PROJECT)/g' -i scp_script.txt)
	@echo "Generating SCP Package"
	$(BUILD_SESSION) -c $(TARGET_SEC) key_file=$(TEST_KEY) script_file=scp_script.txt ${SCP_PACKETS}