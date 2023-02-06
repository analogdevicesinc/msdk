"""
Class to read from Analog Front End (AFE) Registers 
and Digital Base Band (DBB) Registers 

"""
import BLE_hci
from BLE_hci import Namespace

MXC_BASE_BTLE=0x40050000

class AFE:
    def __init__(self) -> None:
        pass

class DBB:
    def __init__(self,hciInterface,ctrlReg=None, rxReg=None, txReg=None, rffeReg=None):
        self.ctrlReg = ctrlReg
        self.rxReg = rxReg
        self.txReg = txReg
        self.rffeReg = rffeReg
        self.hciInterface : BLE_hci.BLE_hci = hciInterface
        
        #put radio into good state for DBB
        self.hciInterface.resetFunc(None)
        self.hciInterface.txTestFunc(Namespace(channel=0, phy=1, packetLength=0, payload=3))

    def __del__(self):
        # close out of hci
        self.hciInterface.endTestFunc(None)

    def readRegs(self, start, sizeBytes):
        """
        Read length sizeBytes of memory starting at address start
        NOTE: Function does not check for unmapped regions
        """
        regReadSize = "0x%02X" %(sizeBytes)
        addr =  "0x%08X" % (start)

        print(f'Reading {sizeBytes} from address {addr}')
        
        return self.hciInterface.readRegFunc(Namespace(addr=addr,length=regReadSize))
        
            
        
    def readCtrlReg(self):
        """
        Read and return the DBB Ctrl Reg
        """

        # DBB has a reserved region from Offset 0x96 to 0xff 
        # Offset 0x108 is also reserved
        # Attempting to read them causes a hardfault
    
        CTRL_REG_ADDR = MXC_BASE_BTLE + 0x1000

        ctrlReg = self.readRegs(CTRL_REG_ADDR, 0x96)

        # assume reserved registers are 0x00
        ctrlReg.extend(['00'] * (0xff - 0x96 + 1))

        next = self.readRegs(CTRL_REG_ADDR + 0x100, 0x108 - 0x100)
        ctrlReg.extend(next)
        
        #Add reserved 0x104 regoion
        ctrlReg.extend(['00'] * 4)


        
        #The last bit is the AES information including the key 
        # which is protected so we have to stop before or we get a hardfault
        next = self.readRegs(CTRL_REG_ADDR + 0x10C, 0x110 - 0x10C)
        ctrlReg.extend(next)
        ctrlReg.extend(['00']*(0x120-0x100))
        
        print('Ctrl Reg Read', len(ctrlReg))
        

        return ctrlReg


    
    def readRxReg(self):
        MXC_BASE_BTLE_DBB_RX = MXC_BASE_BTLE + 0x3000
        
        #Reserved Offsets 
        RESERVED1_START=0x76    + MXC_BASE_BTLE_DBB_RX
        RESERVED2_START=0x13a   + MXC_BASE_BTLE_DBB_RX
        RESERVED3_START=0x2dc   + MXC_BASE_BTLE_DBB_RX
        RESERVED4_START=0x40c   + MXC_BASE_BTLE_DBB_RX
        RESERVED5_START=0x47b   + MXC_BASE_BTLE_DBB_RX
        RESERVED6_START=0x484   + MXC_BASE_BTLE_DBB_RX
        END_OF_RX_REG = 0x584   + 2 + MXC_BASE_BTLE_DBB_RX

    
        #readable region starts
        REGION1_START = MXC_BASE_BTLE_DBB_RX
        REGION2_START = 0x78  + MXC_BASE_BTLE_DBB_RX
        REGION3_START = 0x13c + MXC_BASE_BTLE_DBB_RX
        REGION4_START = 0x400 + MXC_BASE_BTLE_DBB_RX
        REGION5_START = 0x420 + MXC_BASE_BTLE_DBB_RX
        REGION6_START = 0x47c + MXC_BASE_BTLE_DBB_RX
        REGION7_START = 0x584 + MXC_BASE_BTLE_DBB_RX

        #reserved region lengths
        RESERVED1_LEN = 2
        RESERVED2_LEN = 2
        RESERVED3_LEN = 73 * 4
        RESERVED4_LEN = 4  * 5
        RESERVED5_LEN = 1
        RESERVED6_LEN = 2
        RESERVED7_LEN = 0
        
        # Used to iterate through sections of memory getting/appending data
        regionMap = [(RESERVED1_START, REGION1_START, RESERVED1_LEN),
                     (RESERVED2_START, REGION2_START, RESERVED2_LEN),
                     (RESERVED3_START, REGION3_START, RESERVED3_LEN),
                     (RESERVED4_START, REGION4_START, RESERVED4_LEN),
                     (RESERVED5_START, REGION5_START, RESERVED5_LEN),
                     (RESERVED6_START, REGION6_START, RESERVED6_LEN),
                     (END_OF_RX_REG,   REGION7_START, RESERVED7_LEN),
                     ]
        ctrlReg = []

        readout = self.readRegs(regionMap[1][1], regionMap[1][0] - regionMap[0][1] )
        # for count, region in enumerate(regionMap):

        #     print('Reading Region ', count)

        #     #get the data from the device 

        #     print('Start', hex(region[1]))
        #     print('Reserved', hex(region[0]))
        #     regionLength =  region[0] - region[1]
        #     print('Region Length', regionLength)
        #     readout = self.readRegs(region[1], regionLength)
        #     ctrlReg.extend(readout)

        #     #insert reserved regions as zeros
        #     reserved = ['00'] * region[2]
        #     ctrlReg.extend(reserved)
         
        
        # return ctrlReg
        
        # read up to reserved region 1
        ctrlReg = self.readRegs(MXC_BASE_BTLE_DBB_RX, RESERVED1_START - REGION1_START)

        
        # #read region 2
        
        # next = self.readRegs(REGION2_START, RESERVED2_START - REGION2_START)
        # ctrlReg.extend(next)
        
        # #read region 3
        
        # next = self.readRegs(REGION3_START, RESERVED3_START - REGION3_START)
        # ctrlReg.extend(next)
        
        # #read region 4
        # REGION4_START=0x400
        # next = self.readRegs(0x400, 0x40c - 0x400)
        # ctrlReg.extend(next)
        

        # #read region 4
        # REGION5_START=0x420
        # next = self.readRegs(0x420, 0x47b - 0x420)
        # ctrlReg.extend(next)

        # #read region 5
        # REGION6_START=0x47c
        # next = self.readRegs(0x47c, 0x484 - 0x47c)
        # ctrlReg.extend(next)
        
        # #last but not least
        # #last place is a 2 element 32bit array so 8 bytes
        # next = self.readRegs(0x584, 8)
        # ctrlReg.extend(next)
        
        
        return ctrlReg

        
    def readTxReg(self):
        MXC_BASE_BTLE_DBB_TX = MXC_BASE_BTLE + 0x2000
    def readRffeReg(self):
        MXC_BASE_BTLE_DBB_EXT_RFFE  = MXC_BASE_BTLE + 0x8000
    def readAll(self):
        self.ctrlReg = self.readCtrlReg()
        self.rxReg = self.readRxReg()
        self.txReg = self.readTxReg()
        self.rffeReg = self.readRffeReg()

        return {'ctrl' : self.ctrlReg, 
                'rx' : self.rxReg, 
                'tx' : self.txReg, 
                'rffe' : self.rffeReg }
        
    def dump(self):
        dumpRead = self.readAll()
        print(dumpRead)
    
