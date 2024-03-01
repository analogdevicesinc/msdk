#!/usr/bin/env python3
################################################################################
 #
 # Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 # (now owned by Analog Devices, Inc.),
 # Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 # is proprietary to Analog Devices, Inc. and its licensors.
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 #
 ###############################################################################

# This program is created to concatenate peripherals one by one into a consolidated SVD file
import sys 
import os
import copy
import re
import xml.etree.ElementTree as ET
import textwrap
import argparse

# Input Arugments
parser = argparse.ArgumentParser("python3 svd_add_peripheral_modified.py")
parser.add_argument("chip_periph_txt_path", type=str, help="Location of the device's chip_periph.txt file. This file lists all the individual peripheral SVD files that a device supports")
parser.add_argument("device_svd_path", type=str, help="Location of the device's SVD file (e.g. max32670.svd, max78002.svd)")
parser.add_argument("MAXIM_PATH", type=str, help="Location of the MaximSDK (msdk repo).")

args = parser.parse_args()

msdk_path=args.MAXIM_PATH
chip_periph=args.chip_periph_txt_path
device_svd=args.device_svd_path

revx_check=0
# Check array for the special arguments
for arg in sys.argv:
  if(arg == "-r"):
    revx_check = 1

with open(chip_periph) as f:
	file_directories = f.readlines()

# get file paths
file_path = [x.strip().split(';') for x in file_directories]

#set up device.svd xml tree
tree = ET.parse(device_svd)
root = tree.getroot() 
masterPeriph = root.find("./peripherals")

# go through every file path in chip_periph.txt
for i in range(len(file_path)):
	# Don't do anything if empty line
	if not ''.join(file_path[i]).strip():
		continue

	rev = ""
	if revx_check:
		revx_temp = file_path[i][0].strip().split('_')
		for x in revx_temp:
			if "rev" in x:
				revx = x.strip().split('.')
				rev = revx[0]
				break

	# Set up periphial SVD file paths to read for xml tree
	svd = msdk_path + "/" + file_path[i][0]
	svd = svd.replace("//", "/")
	svd = svd.replace("/./", "/")

	svd_tree = ET.parse(svd)
	svd_root = svd_tree.getroot()
	peripherals = svd_root.findall('./peripheral')

	print(svd, "added")
	
	count = 0
	per_name = ""
	per_description = ""
	for k in file_path[i]:
		#skip the first itteration of loop because this arg is the file we are accessing
		if (count == 0):
			count += 1
			continue

		#split up periphial arguments
		address = k.strip().split(' ')
		baseAddress = address[0]
		interruptVector = None
		if(len(address)>1):
			interruptVector = address[1]

		# check that it is the first itteration of adding the periphial
		if(count==1):
			#Go through peripherials in tree
			for added_peripheral in peripherals:
				if revx_check:
					per = added_peripheral.find('name')
					if(per.text.endswith('0') == True): # If base peripheral ends in 0, replace with nothing for better naming on the regs files
						per.text = re.sub('0', "", per.text)
					if rev: #if string is empty returns false
						per.text = per.text+ "_"+rev.upper()
						
				per_name = added_peripheral.find('name').text
				per_description = added_peripheral.find('description').text
				added_interupt = added_peripheral.find('interrupt')
				added_base = added_peripheral.find('baseAddress')
				#check base address and interupt for correct values otherwise fix them
				if(added_base != None):
					if(added_base.text != baseAddress):
						print("changed base address")
						added_base.text = baseAddress
				if(added_interupt != None):
					if(interruptVector != None):
						interrupt_value = added_interupt.find('value')
						if(interrupt_value.text != interruptVector):
							print("changed interrupt")
							interrupt_value.text = interruptVector
					else:
						print("Error missing Interupt Vector Argument")
						print(per_name)
						break
				# write new periphial to device_svd tree
				test = copy.copy(added_peripheral)
				masterPeriph.append(test)
				masterPeriph.append(ET.Comment(per_name + " " + per_description))
		# iterations of duplicate peripherials
		else: 
			if(per_name.endswith('0') == True): # If base peripheral ends in 0, replace with this instance number.
				per_name_iteration = re.sub('0', str(count-1), per_name)
			else:
				per_name_iteration = per_name + str(count-1)
			#create a peripheral tag derived from it base peripheral
			peripheral_tag = ET.SubElement(masterPeriph, 'peripheral')
			peripheral_tag.set('derivedFrom', per_name)
			name_tag = ET.SubElement(peripheral_tag, 'name')
			name_tag.text = per_name_iteration
			description_tag = ET.SubElement(peripheral_tag, 'description')
			description_tag.text = per_description +" " +str(count-1)
			baseAddress_tag = ET.SubElement(peripheral_tag, 'baseAddress')
			baseAddress_tag.text = baseAddress
			#add interupt if needed
			if(added_interupt != None):
				if(interruptVector != None):
					interrupt_tag = ET.SubElement(peripheral_tag, 'interrupt')
					interruptName_tag = ET.SubElement(interrupt_tag, 'name')
					interruptName_tag.text = per_name_iteration
					interruptDesc_tag = ET.SubElement(interrupt_tag, 'description')
					interruptDesc_tag.text = per_name_iteration + ' IRQ'
					interruptValue_tag = ET.SubElement(interrupt_tag, 'value')
					interruptValue_tag.text = interruptVector
				else:
					print("Error missing Interupt Vector Argument")
					print(per_name)
					break
			#add to device_svd tree
			masterPeriph.append(ET.Comment(per_name_iteration + " " + description_tag.text))
		#incrememt iterations
		count+=1

		#write to device_svd tree to device_svd file with encoding utf8 and add the XML declaration
		tree.write(device_svd, "utf-8", True)
