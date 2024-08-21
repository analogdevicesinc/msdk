## Description
scp_to_c_array.py python script can be used to generate C array from scp files.
This script prepared to provide a way to manage SCP file in C code.
The scripts will convert each .packet file to a C array, the name of array will be name of the file, 
then each packet array will linked with a C structure.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Usage
```python scp_to_c_array.py [URL to packet.list file] <Target Board> <Image Name>```
Param 2 and 3 is optional. If you do not pass them, it will be 'MAX32520KIT' 'fw'

```
python scp_to_c_array.py "./SCP_MAX32520_Blinkled_P1_7/packet.list" MAX32520KIT blinkled
scp_MAX32520_blinkled.c generated.

```

## Output File

The output .c file content will be looks like:

```

...
...
static const unsigned char scp_0000033_disconnection_reply[] = { 0xbe, 0xef, 0xed, 0x04, 0x00, 0x00, 0x9e, 0x59 };

typedef struct {
	unsigned char type; // 1:hello_reply, 2:erase/del_mem
    unsigned char is_tx;// 1: From host to target, 0: From target to host
    unsigned short len;
    const unsigned char *data;
} scp_packet_struct;

const scp_packet_struct scp_MAX32520_fw[] = { 
{ 0 , 1 , 8       , scp_0000001_connection_request },
{ 0 , 0 , 8       , scp_0000002_connection_reply },
{ 0 , 1 , 8       , scp_0000003_ack },
{ 0 , 1 , 26      , scp_0000004_hello_request },
{ 0 , 0 , 8       , scp_0000005_ack },
{ 1 , 0 , 66      , scp_0000006_hello_reply },
{ 0 , 1 , 8       , scp_0000007_ack },
{ 2 , 1 , 90      , scp_0000008_del_mem },
{ 0 , 0 , 8       , scp_0000009_ack },
{ 0 , 0 , 20      , scp_0000010_del_mem_response },
{ 0 , 1 , 8       , scp_0000011_ack },
{ 0 , 1 , 8250    , scp_0000012_write_mem },
{ 0 , 0 , 8       , scp_0000013_ack },
{ 0 , 0 , 20      , scp_0000014_write_mem_response },
{ 0 , 1 , 8       , scp_0000015_ack },
{ 0 , 1 , 8250    , scp_0000016_write_mem },
{ 0 , 0 , 8       , scp_0000017_ack },
{ 0 , 0 , 20      , scp_0000018_write_mem_response },
{ 0 , 1 , 8       , scp_0000019_ack },
{ 0 , 1 , 8250    , scp_0000020_write_mem },
{ 0 , 0 , 8       , scp_0000021_ack },
{ 0 , 0 , 20      , scp_0000022_write_mem_response },
{ 0 , 1 , 8       , scp_0000023_ack },
{ 0 , 1 , 8250    , scp_0000024_write_mem },
{ 0 , 0 , 8       , scp_0000025_ack },
{ 0 , 0 , 20      , scp_0000026_write_mem_response },
{ 0 , 1 , 8       , scp_0000027_ack },
{ 0 , 1 , 1242    , scp_0000028_write_mem },
{ 0 , 0 , 8       , scp_0000029_ack },
{ 0 , 0 , 20      , scp_0000030_write_mem_response },
{ 0 , 1 , 8       , scp_0000031_ack },
{ 0 , 1 , 8       , scp_0000032_disconnection_request },
{ 0 , 0 , 8       , scp_0000033_disconnection_reply },
{ 0 , 0 , 0       , 0 } 
};

```