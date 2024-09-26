#! /usr/bin/bash

<<"CONFIGURATION"
The following is the configuration for hardware (max78002): 
Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
Install headers JP9 and JP10 to SDA and SCL respectively.
You must connect P0.10 to P0.16 (SCL) and P0.11 to P0.17 (SDA)
Connect P0.12 to P1.1. Connect P0.13 to P1.0. for UART test
Connect P0.21 (MOSI) to P0.22 (MISO).
Apply an input voltage between 0 and 1.25V to pin labeled 0 of the JH6 (Analog) header. for ADC
CONFIGURATION


# variable for configuration
baudRate=115200
timeLimit=5
timeLimitICC=30 #ICC needs around 20s to finish the test
boardVersion=max78002
#boardName=max78002-1
boardName=$1
uartPort=$(resource_manager -g $boardName.console_port)
target_uc=$(resource_manager -g $boardName.target)
Path=$MAXIM_PATH/Examples/$target_uc

# variable for testing purpose
tempFile=.temp.txt
result_UART_INTERRUPT='not tested'
result_UART_DMA='not tested'
result_HelloWorld='not tested'
result_HelloWorld_Cpp='not tested'
result_TRNG='not tested'
result_I2C='not tested'
result_SPI='not tested'
result_ICC='not tested'
result_DMA='not tested'
result_CRC='not tested'
result_ADC_DMA='not tested'
result_Lib_Gen='not tested'
result_Lib_Use='not tested'

function init() {
	# print the testcase
	echo "-------------------------------------------------------------------"
	echo "Start testing $1 Example:"
	# clean the temp file
	rm -rf $tempFile
	sleep 2

	# do initialization, compiler and flash the code
	testName=$1
	if [[ $testName != Library_Use ]]
	then
		make -C $Path/$testName distclean

		if [[ $1 == "ADC" || $1 == "UART" ]]; then
    		make -j -C "$Path/$testName" METHOD="$2"
		else
    		make -j -C "$Path/$testName"
		fi

	fi

	if [[ $testName != Library_Generate && $testName != Library_Use ]]
	then
		stty -F $uartPort $baudRate
		ocdflash $boardName $Path/$testName/build/$boardVersion.elf 
		if [[ $testName = ICC ]]
		then
			timeout $timeLimitICC cat $uartPort > $tempFile
		else
			timeout $timeLimit cat $uartPort > $tempFile
		fi
	fi
}

function test_Hello_World()  {
	init Hello_World

	# start testing the output
	grep "Hello World!" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_HelloWorld='pass'
	else
		result_HelloWorld='fail'
	fi

	printf "Test result for Hello_World: $result_HelloWorld\n"

}

function test_Hello_World_Cpp()  {
	init Hello_World_Cpp

	# start testing the output
	grep "C++ Hello World Example" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_HelloWorld_Cpp='pass'
	else
		result_HelloWorld_Cpp='fail'
	fi

	printf "Test result for Hello_World_Cpp: $result_HelloWorld_Cpp\n"

}

function test_UART() {
	init UART INTERRUPT

	# start testing the output
	grep "Example Succeeded" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_UART_INTERRUPT='pass'
	else
		result_UART_INTERRUPT='fail'
	fi
	
	printf "Test result for UART_INTERRUPT: $result_UART_INTERRUPT\n"
	init UART DMA

	# start testing the output
	grep "Example Succeeded" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_UART_DMA='pass'
	else
		result_UART_DMA='fail'
	fi
	
	printf "Test result for UART_DMA: $result_UART_DMA\n"
}

function test_TRNG() {
	init TRNG

	# start testing the output
	grep "Test Complete" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_TRNG='pass'
	else
		result_TRNG='fail'
	fi
	
	printf "Test result for TRNG: $result_TRNG\n"
	
}


function test_I2C() {
	init I2C

	# start testing the output
	grep "I2C Transaction Successful" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_I2C='pass'
	else
		result_I2C='fail'
	fi
	
	printf "Test result for I2C: $result_I2C\n"
	
}

function test_SPI() {
	init SPI

	# start testing the output
	grep "16 Bits Transaction Successful" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_SPI='pass'
	else
		result_SPI='fail'
	fi
	
	printf "Test result for SPI: $result_SPI\n"
	
	
}

function test_ICC() {
	init ICC

	# start testing the output
	grep "Example Succeeded" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_ICC='pass'
	else
		result_ICC='fail'
	fi
	
	printf "Test result for ICC: $result_ICC\n"
	
}


function test_DMA() {
	init DMA

	# start testing the output
	grep "Example Succeeded" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_DMA='pass'
	else
		result_DMA='fail'
	fi
	
	printf "Test result for DMA: $result_DMA\n"
	
}

function test_CRC() {
	init CRC

	# start testing the output
	grep "Example Succeeded" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_CRC='pass'
	else
		result_CRC='fail'
	fi
	
	printf "Test result for CRC: $result_CRC\n"
	
}


function test_ADC() {

	init ADC DMA

	# start testing the output
	result_ADC_DMA='pass'
	grep "Running Single Channel Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_DMA='fail'
	fi
	
	grep "Running Temperature Sensor Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_DMA='fail'
	fi

	grep "Running Multi Channel Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_DMA='fail'
	fi
	printf "Test result for ADC_DMA: $result_ADC_DMA\n"
}

function test_Lib_Gen() {
	init Library_Generate

	# start testing the output
	find $Path/$testName/build/$boardVersion.a
	if [[ $? -eq 0 ]];
	then
		result_Lib_Gen='pass'
	else
		result_Lib_Gen='fail'
	fi
	
	printf "Test result for Library_Generate: $result_Lib_Gen\n"
	
}

function test_Lib_Use() {
	init Library_Use

	# start testing the output
	make -C $Path/$testName distclean
	make -C $Path/$testName
	if [[ $? -eq 0 ]];
	then
		result_Lib_Use='pass'
	else
		result_Lib_Use='fail'
	fi
	
	printf "Test result for Library_Use: $result_Lib_Use\n"
	
}

function summary() {
	printf "*************************Result Summary****************************\n"
	printf "Test result for Hello_World: $result_HelloWorld\n"
	printf "Test result for Hello_World_Cpp: $result_HelloWorld_Cpp\n"
	printf "Test result for UART_INTERRUPT: $result_UART_INTERRUPT\n"
	printf "Test result for UART_DMA: $result_UART_DMA\n"
	printf "Test result for TRNG: $result_TRNG\n"
	printf "Test result for I2C: $result_I2C\n"
	printf "Test result for SPI: $result_SPI\n"
	printf "Test result for ICC: $result_ICC\n"
	printf "Test result for DMA: $result_DMA\n"
	printf "Test result for CRC: $result_CRC\n"
	printf "Test result for ADC_DMA: $result_ADC_DMA\n"
	printf "Test result for Library_Generate: $result_Lib_Gen\n"
	printf "Test result for Library_Use: $result_Lib_Use\n"
	
}

function main() {
	test_UART
	if [[ $result_UART_INTERRUPT = 'fail' ]]
	then
		printf "Since UART INTERRUPT test fails, example test stops. "
		return
	fi
	test_Hello_World
	test_Hello_World_Cpp
	test_TRNG
	test_I2C
	test_SPI
	test_DMA
	test_CRC
	test_ADC
	test_ICC
	test_Lib_Gen
	test_Lib_Use

	summary
	# clean the temp file
	rm -rf $tempFile
}

main
if [[ $result_UART_INTERRUPT = "fail" || 
	$result_UART_DMA = "fail" ||
	$result_HelloWorld = "fail" ||
    $result_HelloWorld_Cpp = "fail" ||
    $result_TRNG = "fail" ||
    $result_I2C = "fail" ||
 	$result_SPI = "fail" ||
    $result_ICC = "fail" ||
    $result_DMA = "fail" ||
    $result_CRC = "fail" ||
    $result_ADC_DMA = "fail" ||
    $result_Lib_Gen = "fail" ||
    $result_Lib_Use = "fail" ]]; then
	exit 2
	else
	exit 0
fi
