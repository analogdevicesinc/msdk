"""
Class to read from Analog Front End (AFE) Registers 
and Digital Base Band (DBB) Registers 

"""
import BLE_hci
from BLE_hci import Namespace

from time import sleep
MXC_BASE_BTLE = 0x40050000

BoardBaseRegs = {
    "max32690": 0x40050000,
    "max32655": 0x40050000,
    "max32665": 0x40050000,
}


class DBB:
    """
    Class Used to Read registers of DBB
    All offsets and data taken from datasheet
    """

    def __init__(self, hciInterface, board, ctrlReg=None, rxReg=None, txReg=None, rffeReg=None):

        self.ctrlReg = ctrlReg
        self.rxReg = rxReg
        self.txReg = txReg
        self.rffeReg = rffeReg

        assert (board in BoardBaseRegs)

        self.board = board

        self.hciInterface: BLE_hci.BLE_hci = hciInterface

        # put radio into good state for DBB
        self.hciInterface.resetFunc(None)

    def __del__(self):
        # close out of hci
        print('Resetting Board')
        self.hciInterface.resetFunc(None)

    def readRegs(self, start, sizeBytes):
        """
        Read length sizeBytes of memory starting at address start
        NOTE: Function does not check for unmapped regions
        """

        # There seems to be a problem when lengths are greater than 251.
        # I believe it is because the header packet is 3 bytes and
        # so if you have 252 then the total length is 255 and the format is wrong
        # Just gonna assert for now
        assert (sizeBytes <= 251)

        regReadSize = "0x%02X" % (sizeBytes)
        addr = "0x%08X" % (start)

        print(f'Reading {sizeBytes} from address {addr}')
        return self.hciInterface.readReg(addr=addr, length=regReadSize)

    def readRegion(self, start, stop):
        """
        Read region from start to stop, non inclusive of stop
        returns region as list
        """
        region = []

        totalLen = stop - start
        READ_LEN_MAX = 251

        while totalLen > 0:

            if totalLen <= READ_LEN_MAX:
                readout = self.readRegs(start, totalLen)
                amtRead = totalLen
            else:
                readout = self.readRegs(start, READ_LEN_MAX)
                amtRead = READ_LEN_MAX
            region.extend(readout)
            totalLen -= amtRead

        print('Length', len(region))
        assert (len(region) == stop - start)

        return region

    def readRegions(self, baseAddr, offsetLut: dict):
        """
        Reads multiple regions give region map 
        and returns data as list
        """
        regions = []
        for count, region in enumerate(offsetLut):

            # unpack to make readable
            region_start, reserved_start, reserved_len = region

            # add base to offset address
            region_start += baseAddr
            reserved_start += baseAddr

            print('Reading Region', count)

            regionLength = reserved_start - region_start
            print('Expected Region Length', regionLength)

            readout = self.readRegion(region_start, reserved_start)

            if len(readout) != (regionLength):
                print('Error occurred during readout. Aborting operation')
                return []

            regions.extend(readout)

            # add reserved region to the register read
            regions.extend(['00'] * reserved_len)
            sleep(0.1)

        return regions

    def readCtrlReg(self):
        """
        Read and return the DBB Ctrl Reg
        """

        # DBB has a reserved region from Offset 0x96 to 0xff

        # Offset 0x108 is also reserved
        # Attempting to read them causes a hardfault

        CTRL_REG_ADDR = BoardBaseRegs[self.board] + 0x1000

        ctrlReg = self.readRegs(CTRL_REG_ADDR, 0x96)

        # assume reserved registers are 0x00
        ctrlReg.extend(['00'] * (0xff - 0x96 + 1))

        next = self.readRegs(CTRL_REG_ADDR + 0x100, 0x108 - 0x100)
        ctrlReg.extend(next)

        # Add reserved 0x104 regoion
        ctrlReg.extend(['00'] * 4)

        # The last bit is the AES information including the key

        # which is protected so we have to stop before or we get a hardfault
        next = self.readRegs(CTRL_REG_ADDR + 0x10C, 0x110 - 0x10C)
        ctrlReg.extend(next)
        ctrlReg.extend(['00']*(0x120-0x100))

        print('Ctrl Reg Read', len(ctrlReg))

        return ctrlReg

    def readTxReg(self):
        """
        Read out the TX register of the DBB and return as list
        Reserved regions will be set as '00'
        """

        MXC_BASE_BTLE_DBB_TX = BoardBaseRegs[self.board] + 0x2000

        offsetLut = [
            (0x00, 0x70, 0x180 - 0x70),
            # any address offset past 0x18c causes a hardfault
            (0x180, 0x18c, 0x194 + 76 - 0x18c)
        ]

        # Channel 0
        # Phy 1MB
        # Packet Length Default
        # Payload PRBS15

        self.hciInterface.txTestFunc(
            Namespace(channel=0, phy=1, packetLength=0, payload=3))

        txRegs = self.readRegions(
            baseAddr=MXC_BASE_BTLE_DBB_TX, offsetLut=offsetLut)

        self.hciInterface.endTestFunc(None)

        return txRegs

    def readRxReg(self):
        """
        Reads the contents of the rx register and returns data as a list
        All reserved regions initialized as '00'
        """
        MXC_BASE_BTLE_DBB_RX = BoardBaseRegs[self.board] + 0x3000

        # Offset Lookup for RX registers
        # There are a lot of traps and so some of these offsets were found by trial and error
        # Start of region, Start of reserved region, reserved region len
        offsetLut = [(0x00,    0x76, 2),
                     (0x78,    0x13a, 2),
                     (0x13c,   0x2dc, 73*4),
                     (0x400,   0x404, 4),
                     (0x408,   0x40c, 4*5),
                     # The entire region with CTE values causes hardfaults
                     (0x420, 0x424, 0x586 - 0x424),
                     ]

        self.hciInterface.rxTestFunc(Namespace(channel=0, phy=1))

        rxRegs = self.readRegions(MXC_BASE_BTLE_DBB_RX, offsetLut)

        self.hciInterface.endTestFunc(None)

        return rxRegs

    def readRffeReg(self):

        MXC_BASE_BTLE_DBB_EXT_RFFE = BoardBaseRegs[self.board] + 0x8000

        offsetLut = [(0x00, 0x11e, 2),
                     (0x120, 0x146, 0x164 - 0x146),
                     (0x164, 0x168 + 2, 0x168 + 2 - 0x164)
                     ]

        rffe = self.readRegions(
            baseAddr=MXC_BASE_BTLE_DBB_EXT_RFFE, offsetLut=offsetLut)

        return rffe

    def readAll(self):
        """
        Read all registers associated with dbb
        Return dict of register readouts
        Keys:
        ctrl: ctrl registers
        rx: rx registers
        tx : tx registers
        rffe: rffe registers 
        """

        self.ctrlReg = self.readCtrlReg()
        self.rxReg = self.readRxReg()
        self.txReg = self.readTxReg()
        self.rffeReg = self.readRffeReg()

        return {'ctrl': self.ctrlReg,
                'rx': self.rxReg,
                'tx': self.txReg,
                'rffe': self.rffeReg}

    def dump(self):
        dumpRead = self.readAll()
        print(dumpRead)
