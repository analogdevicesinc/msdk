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
        # #anything bigger than 128 bytes seems to cause a problem with the fromhex function
        # regReadSize = "0x%02X" %(32 * 4)
        # regBase = reg
        # ctrlReg = []
        
        # #size of the ctrl reg is 304 bytes so just go  through more than that
        # while reg <  regBase + 128 * 3:
        #     evtBytes = self.hciInterface.readRegFunc(Namespace(addr=MXC_BASE_BTLE_DBB_CTRL,length=regReadSize))
        #     ctrlReg.extend(evtBytes)
        #     reg += 128
        
        # #read too much so delete until we get to the size (probably not needed)
        # while len(ctrlReg) != CTRL_REG_SIZE:
        #     ctrlReg.pop()
        # print(ctrlReg)
        # print('length', len(ctrlReg))
    
    

    
    def readRxReg(self):
        MXC_BASE_BTLE_DBB_RX = MXC_BASE_BTLE + 0x3000
        
    def readTxReg(self):
        MXC_BASE_BTLE_DBB_TX = MXC_BASE_BTLE + 0x2000
    def readRffeReg(self):
        MXC_BASE_BTLE_DBB_EXT_RFFE  = MXC_BASE_BTLE + 0x8000
    def readAll(self):
        pass
    def dump(self):
        dumpRead = self.readAll()
        print(dumpRead)
    
