# BLE_periph

This is the simplest application that should be used when getting started. It will advertise as "Periph" and accepts connection requests.

## Board Setup

Before building firmware you must select the correct value for BOARD in project.mk, e.g. "EvKit_V1".

### LEDs

The red LED will indicate that an error assertion has occurred.  

The green LED indicates CPU activity. When the LED is on, the CPU is active, when the LED
is off, the CPU is in sleep mode.

### Expected Output

On startup:
```
terminal: init
PeriphHandlerInit
Calculating database hash
Periph got evt 119
Periph got evt 32
>>> Reset complete <<<
dmAdvActConfig: state: 0
dmAdvActSetData: state: 0
dmAdvActStart: state: 0
HCI_LE_ADV_ENABLE_CMD_CMPL_CBACK_EVT: state: 3
dmDevPassEvtToDevPriv: event: 12, param: 33, advHandle: 0
Periph got evt 33
>>> Advertising started <<<

```

When a connection has been made.
```
dmConnIdByBdAddr not found
dmConnCcbAlloc 1
dmConnSmExecute event=28 state=0
dmAdvConnected: state: 1
dmDevPassEvtToDevPriv: event: 13, param: 34, advHandle: 0
AttsCccInitTable connId=1
smpDbGetRecord: connId: 1 type: 1
smpDbAddDevice
SmpDbGetFailureCount: connId: 1 count: 0
smpDbGetRecord: connId: 1 type: 1
smpDbAddDevice
SmpDbGetPairingDisabledTime: connId: 1 period: 0 attemptMult: 0
Periph got evt 39
>>> Connection opened <<<
Periph got evt 65
Periph got evt 87
attsProcMtuReq features 0x00
hciCoreTxAclStart len=7
Periph got evt 22
connId=1 idleMask=0x0004
hciCoreTxAclStart len=18
connId=1 idleMask=0x0004
hciCoreTxAclStart len=26
hciCoreTxAclStart len=34                                                                                              
connId=1 idleMask=0x0004                                                                                              
hciCoreTxAclStart len=34                                                                                              
attsCccMainCback connId=1 handle=19                                                                                   
hciCoreTxAclStart len=5                                                                                               
hciCoreTxAclStart len=27                                                                                              
hciCoreTxAclStart len=9                                                                                               
connId=1 idleMask=0x0004                                                                                              
hciCoreTxAclStart len=10                                                                                              
connId=1 idleMask=0x0004                                                                                              
hciCoreTxAclStart len=9                                                                                               
attsCccMainCback connId=1 handle=515                                                                                  
hciCoreTxAclStart len=7                                                                                               
hciCoreTxAclStart len=14
```

### Commands
Type the desired command and parameter (if applicable) and press enter to execute the command.  

__help__  Displays the available commands.  
__echo (on|off)__  Enables or disables the input echo. On by default.  
__btn (ID) (s|m|l|x)__  Simulates button presses. Example: "btn 1 s" for a short button press on button 1.  
__pin (ConnID) (Pin Code)__  Used to input the pairing pin code.  

### Push buttons
Push buttons are not implemented in this example.

## Stack Initialization

## GAP Peripheral / Slave Role

### Advertising interval
The advertising interval is configurable in this structure. We can define multiple interverals, each with their own duration. In this case we will advertise at a short interval (96\*0.625ms = 60ms) for 30 seconds. We will then transition to a long interval (1600\*0.625 ms = 1000 ms) indefinetly. Longer advertising intervals will conserve power, but increase the latency when communicating with scanning devices or creating connections.

```c
/*! configurable parameters for advertising */
static const appAdvCfg_t periphAdvCfg = {
    { 30000, 0,    0 }, /*! Advertising durations in ms, 0 corresponds to infinite */
    { 96,    1600, 0 }  /*! Advertising intervals in 0.625 ms units */
};
```

Applications can also set a definite advertising duration with will cause the device to stop advertising at the end of the duation. The application can restart advertising by calling AppAdvStart().

### Advertising data

Applications can define the advertising data with this structure. This information will be broadcast in every advertising event. Each portion of the advertising data is defined by a length byte, a type byte, and the data. In this case we're advertising the flags that the device is discoverable and BR/EDR (Bluetooth Classic) is not supported. We're also advertising the device name "Periph". 

```c
/*! advertising data, discoverable mode */
static const uint8_t periphAdvDataDisc[] = {
    /*! flags */
    2, /*! length */
    DM_ADV_TYPE_FLAGS, /*! AD type */
    DM_FLAG_LE_GENERAL_DISC | /*! flags */
        DM_FLAG_LE_BREDR_NOT_SUP,
    /*! device name */
    7, /*! length */
    DM_ADV_TYPE_LOCAL_NAME, /*! AD type */
    'P', 'e', 'r', 'i', 'p', 'h'
};
```

## GATT Server

## MTU size and Throughput

Each layer of the stack has parameters that will bottleneck the throughput of the system. The ATT layer defines a Maximum Transmission Unit (MTU) to indiate the maximum length of an ATT packet. 

``` c
/*! ATT configurable parameters (increase MTU) 
 * ATT_MAX_TRANS_TIMEOUT = 30 seconds
 */
static const attCfg_t periphAttCfg = {
    15, /* ATT server service discovery connection idle timeout in seconds */
    241, /* desired ATT MTU */
    ATT_MAX_TRANS_TIMEOUT, /* transcation timeout in seconds */
    4 /* number of queued prepare writes supported by server */
};
```

This MaxRxAclLen defines the maximum reassembled RX Asynchronous Connection-Orientated Logical(ACL) packet length. Packets received at the HCI layer must be buffered until the entire ACL packet has been received. Once the entire ACL packet has been received, the HCI layer will send the packet to the L2CAP layer.
```c
/*************************************************************************************************/
/*!
 *  \brief  Set the maximum reassembled RX ACL packet length.  Minimum value is 27.
 *
 *  \param  len     ACL packet length.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciSetMaxRxAclLen(uint16_t len);
```

The MTU must be less than or equal to the MaxRxAclLen - L2C Header Length (4 bytes).
``` c
  /* if configured MTU size is larger than maximum RX PDU length */
  if (pAttCfg->mtu > (HciGetMaxRxAclLen() - L2C_HDR_LEN))
  {
    /* notify app about MTU misconfiguration */
    attExecCallback(0, DM_ERROR_IND, 0, DM_ERR_ATT_RX_PDU_LEN_EXCEEDED, 0);
  }
```

## Callbacks

## Adding WSF events and handlers


