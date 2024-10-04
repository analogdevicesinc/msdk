#! /usr/bin/bash

<<"CONFIGURATION"
The following is the configuration for hardware (max32690): 
remove JP7(RX_EN) and install JP8(TX_EN) headers
Install headers JP9 and JP10 to SDA and SCL respectively.
You must connect P1.8->P2.8 (SCL) and P1.7->P2.7 (SCL).
Connect pins P2.12->P1.9 for UART test
connect MISO (P2.27) and MOSI (P2.28) pins
Apply an input voltage between 0 and 1.25V to pin labeled 0 of the JH6 (Analog) header. for ADC
CONFIGURATION


# variable for configuration
baudRate=115200
timeLimit=5
timeLimitICC=30 #ICC needs around 20s to finish the test
boardVersion=max32690
#boardName=max32690-1
boardName=$1
uartPort=$(resource_manager -g $boardName.console_port)
target_uc=$(resource_manager -g $boardName.target)
#MAXIM_PATH=/home/jcai/Workspace/msdk
Path=$MAXIM_PATH/Examples/$target_uc

# variable for testing purpose
tempFile=.temp.txt
result_UART_INTERRUPT='not tested'
result_UART_DMA='not tested'
result_HelloWorld='not tested'
result_HelloWorld_Cpp='not tested'
result_TRNG='not tested'
result_I2C='not tested'
result_SPI_POLLING='not tested'
result_SPI_INTERRUPT='not tested'
result_SPI_DMA='not tested'
result_SPI_V2_POLLING='not tested' #this is only for max32690
result_SPI_V2_INTERRUPT='not tested' #this is only for max32690
result_SPI_V2_DMA='not tested' #this is only for max32690
result_ICC='not tested'
result_Hash='not tested'
result_DMA='not tested'
result_CRC='not tested'
result_ADC_POLLING='not tested'
result_ADC_INTERRUPT='not tested'
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

		if [[ $1 = "SPI" || $1 = "SPI_v2" || $1 = "ADC" || $1 = "UART" ]];
		then
			make -j -C $Path/$testName METHOD=$2
		else
			make -j -C $Path/$testName
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
: '
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
'
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
	init SPI MASTERSYNC

	# start testing the output
	grep "16 Bits Transaction Successful" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_SPI_POLLING='pass'
	else
		result_SPI_POLLING='fail'
	fi
	
	printf "Test result for SPI_POLLING: $result_SPI_POLLING\n"

	init SPI MASTERASYNC

	# start testing the output
	grep "16 Bits Transaction Successful" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_SPI_INTERRUPT='pass'
	else
		result_SPI_INTERRUPT='fail'
	fi
	
	printf "Test result for SPI_INTERRUPT: $result_SPI_INTERRUPT\n"

	init SPI MASTERDMA

	# start testing the output
	grep "16 Bits Transaction Successful" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_SPI_DMA='pass'
	else
		result_SPI_DMA='fail'
	fi
	
	printf "Test result for SPI_DMA: $result_SPI_DMA\n"
	
}

function test_SPI_V2() {
	init SPI_v2 CONTROLLER_SYNC

	# start testing the output
	grep "16 Bits Transaction Successful" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_SPI_V2_POLLING='pass'
	else
		result_SPI_V2_POLLING='fail'
	fi
	
	printf "Test result for SPI_v2_POLLING: $result_SPI_V2_POLLING\n"

	init SPI_v2 CONTROLLER_ASYNC

	# start testing the output
	grep "16 Bits Transaction Successful" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_SPI_V2_INTERRUPT='pass'
	else
		result_SPI_V2_INTERRUPT='fail'
	fi
	
	printf "Test result for SPI_v2_INTERRUPT: $result_SPI_V2_INTERRUPT\n"

	init SPI_v2 CONTROLLER_DMA

	# start testing the output
	grep "16 Bits Transaction Successful" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_SPI_V2_DMA='pass'
	else
		result_SPI_V2_DMA='fail'
	fi
	
	printf "Test result for SPI_v2_DMA: $result_SPI_V2_DMA\n"
	
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

function test_Hash() {
	init Hash

	# start testing the output
	grep "Example Succeeded" $tempFile
	if [[ $? -eq 0 ]];
	then
		result_Hash='pass'
	else
		result_Hash='fail'
	fi
	
	printf "Test result for Hash: $result_Hash\n"
	
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

	init ADC POLLING

	# start testing the output
	result_ADC_POLLING='pass'
	grep "Running Single Channel Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_POLLING='fail'
	fi
	
	grep "Running Temperature Sensor Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_POLLING='fail'
	fi

	grep "Running Multi Channel Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_POLLING='fail'
	fi
	printf "Test result for ADC_POLLING: $result_ADC_POLLING\n"
	
	init ADC INTERRUPT

	# start testing the output
	result_ADC_INTERRUPT='pass'
	grep "Running Single Channel Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_INTERRUPT='fail'
	fi
	
	grep "Running Temperature Sensor Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_INTERRUPT='fail'
	fi

	grep "Running Multi Channel Example" $tempFile
	if [[ $? -ne 0 ]];
	then
		result_ADC_INTERRUPT='fail'
	fi
	printf "Test result for ADC_INTERRUPT: $result_ADC_INTERRUPT\n"

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
	printf "Test result for SPI_POLLING: $result_SPI_POLLING\n"
	printf "Test result for SPI_INTERRUPT: $result_SPI_INTERRUPT\n"
	printf "Test result for SPI_DMA: $result_SPI_DMA\n"
	if [[ $boardVersion = max32690 ]]
	then
		printf "Test result for SPI_v2_POLLING: $result_SPI_V2_POLLING\n"
		printf "Test result for SPI_v2_INTERRUPT: $result_SPI_V2_INTERRUPT\n"
		printf "Test result for SPI_v2_DMA: $result_SPI_V2_DMA\n"
	fi
	printf "Test result for ICC: $result_ICC\n"
	printf "Test result for Hash: $result_Hash\n"
	printf "Test result for DMA: $result_DMA\n"
	printf "Test result for CRC: $result_CRC\n"
	printf "Test result for ADC_POLLING: $result_ADC_POLLING\n"
	printf "Test result for ADC_INTERRUPT: $result_ADC_INTERRUPT\n"
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
	#test_I2C
	test_SPI
	if [[ $boardVersion = max32690 ]]
	then
		test_SPI_V2
	fi
	test_Hash
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
 	$result_SPI_POLLING = "fail" ||
    $result_SPI_INTERRUPT = "fail" ||
    $result_SPI_DMA = "fail" ||
    $result_SPI_V2_POLLING = "fail" ||   # Only for max32690
    $result_SPI_V2_INTERRUPT = "fail" || # Only for max32690
    $result_SPI_V2_DMA = "fail" ||       # Only for max32690
    $result_ICC = "fail" ||
    $result_Hash = "fail" ||
    $result_DMA = "fail" ||
    $result_CRC = "fail" ||
    $result_ADC_POLLING = "fail" ||
    $result_ADC_INTERRUPT = "fail" ||
    $result_ADC_DMA = "fail" ||
    $result_Lib_Gen = "fail" ||
    $result_Lib_Use = "fail" ]]; then
	exit 2
	else
	exit 0
fi
