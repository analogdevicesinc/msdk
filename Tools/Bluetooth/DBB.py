
class RegisterOffsets:
    CTRL = 0x1000
    TX = 0x2000
    RX = 0x3000
    RFFE = 0x8000


class CtrlReg:
    dbb_ctrl_version = 0x0000
    dbb_ctrl_rst = 0x0002
    tx_ctrl_dbb_en_dly = 0x0004
    tx_ctrl_rffe_en_dly = 0x0008
    tx_ctrl_dbb_dis_dly = 0x0010
    tx_ctrl_rffe_dis_dly = 0x0014
    tx_ctrl_cmd = 0x001c
    tx_ctrl_debug_en_tx_on_sfd_to = 0x001e
    rx_ctrl_dbb_en_dly = 0x0020
    rx_ctrl_rffe_en_dly = 0x0024
    rx_ctrl_dbb_dis_dly = 0x002c
    rx_ctrl_rffe_dis_dly = 0x0030
    rx_ctrl_cmd = 0x0038
    tx_pmu_wake_up_dly = 0x003c
    tx_pmu_ctrl = 0x003e
    rffe_pmu_wake_up_dly = 0x0040
    rffe_pmu_ctrl = 0x0042
    rx_pmu_wake_up_dly = 0x0044
    rx_pmu_ctrl = 0x0046
    gen_pmu_status = 0x0048
    event_timing_cntr_clk_mult_p = 0x004c
    event_timing_cntr_clk_div_q = 0x004e
    event_timing_cntr_val = 0x0050
    event_timing_tx_enable_time = 0x0054
    event_timing_rx_enable_time = 0x0058
    event_timing_gp_event_time = 0x005c
    event_timing_tx_enable_delta_time = 0x0060
    event_timing_rx_enable_delta_time = 0x0064
    event_timing_gp_event_delta_time = 0x0068
    event_timing_ctrl = 0x006c
    event_timing_timestamp_tx_done = 0x0070
    event_timing_timestamp_rx_received = 0x0074
    event_timing_timestamp_rx_sfd_det = 0x0078
    event_timing_timestamp_rx_sfd_to = 0x007c
    event_timing_timestamp_rx_energy_det = 0x0080
    events_status = 0x0084
    events_irq_test = 0x0086
    cmu_gating_on = 0x0088
    cmu_main_mult_p = 0x008a
    cmu_main_div_q = 0x008c
    cmu_phy_mult_p = 0x008e
    cmu_phy_div_q = 0x0090
    cmu_dl_mult_p = 0x0092
    cmu_dl_div_q = 0x0094
    rsv_0x96 = 0x96
    b2b_ctrl = 0x0098
    misc_intr_ctrl = 0x009c
    rsv_0xa0_0xff = 0xa0
    aes_st = 0x0100
    aes_aad = 0x0104
    rsv_0x108 = 0x108
    aes_ctrl = 0x010C
    aes_key = 0x0110
    aes_ctr_blk = 0x0120


class Spim:
    def __init__(self, pyOcdTarget):
        pass

    def read(self, address):
        pass

    def write(self, address):
        pass


class DBB:
    DBBBaseAddr = 0x40050000
    CtrlAddrBase = DBBBaseAddr + RegisterOffsets.CTRL
    TxBaseAddr = DBBBaseAddr + RegisterOffsets.TX
    RxBaseAddr = DBBBaseAddr + RegisterOffsets.RX
    RffeBaseAddr = DBBBaseAddr + RegisterOffsets.RFFE

    RegSize8 = 0x1
    RegSize16 = 0x2
    RegSize32 = 0x3

    def __init__(self, pyOcdTarget):
        self.target = pyOcdTarget
        self.spim = Spim(self.target)

    def read8(self, addr):
        return self.target.read8(addr)
    def read16(self,addr):
        return self.target.read16(addr)
    def read32(self,addr):
        return self.target.read32(addr)
            

    def readRange8(self, baseAddr, len):
        data = []
        for i in range(len):
            data.append(self.target.read8(baseAddr + i))
        return data

    def readRange16(self, baseAddr, len):
        data = []
        for i in range(len):
            data.append(self.target.read16(baseAddr + i))
        return data

    def readRange32(self, baseAddr, len):
        data = []
        for i in range(len):
            data.append(self.target.read16(baseAddr + i))
        return data

    def write8(self, addr,value):
        self.target.write8(addr, value)

    def write16(self, addr,value):
        self.target.write8(addr, value)

    def write32(self, addr,value):
        self.target.write8(addr, value)

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

            readout = self.readRange8(
                region_start, reserved_start - region_start)

            if len(readout) != (regionLength):
                print('Error occurred during readout.')
                return []

            regions.extend(readout)

            # add reserved region to the register read
            regions.extend(['00'] * reserved_len)

        return regions

    def ctrlAddr(self, offset):
        return self.CtrlAddrBase + offset
    def rxAddr(self, offset):
        return self.RxBaseAddr + offset
    def txAddr(self, offset):
        return self.TxBaseAddr + offset
    def rffeAddr(self, offset):
        return self.RffeBaseAddr + offset

    def getCtrlVersionInfo(self):
        DBB_CTRL = 0x40051000
        version = self.target.read16(DBB_CTRL, 1)
        version_minor = version & 0xff
        version_major = (version >> 8) * 0xff

        return (version_major, version_minor)

    def getCtrlAll(self):
        len0 = 0x96
        LEN = 0x120 + 16
        ctrl = self.readRange8(self.CtrlAddrBase, len0)
        ctrl.extend(['00'] * (0xff - 0x96 + 1))
        ctrl.extend(self.readRange8(self.CtrlAddrBase + 0x100, 0x108 - 0x100))
        ctrl.extend(['00'] * 4)
        ctrl.extend(self.readRange8(self.CtrlAddrBase + 0x10C, 0x110 - 0x10C))
        ctrl.extend(['00']*(0x120-0x100))

        return ctrl

    def getTxAll(self):
        """
        Read out the TX register of the DBB and return as list
        Reserved regions will be set as '00'
        """

        MXC_BASE_BTLE_DBB_TX = self.TxBaseAddr

        offsetLut = [
            (0x00, 0x70, 0x180 - 0x70),
            # any address offset past 0x18c causes a hardfault
            (0x180, 0x18c, 0x194 + 76 - 0x18c)
        ]

        return self.readRegions(
            baseAddr=MXC_BASE_BTLE_DBB_TX, offsetLut=offsetLut)

    def getRxAll(self):
        """
        Reads the contents of the rx register and returns data as a list
        All reserved regions initialized as '00'
        """
        MXC_BASE_BTLE_DBB_RX = self.RxBaseAddr

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

        return self.readRegions(MXC_BASE_BTLE_DBB_RX, offsetLut)

    def getAll(self):

        ctrl = self.getCtrlAll()
        rx = self.getRxAll()
        tx = self.getTxAll()
        rffe = self.getRffeAll()

        return {
            'ctrl': ctrl,
            'rx': rx,
            'tx': tx,
            'rffe': rffe
        }

    def getRffeAll(self):
        MXC_BASE_BTLE_DBB_EXT_RFFE = self.RffeBaseAddr

        offsetLut = [(0x00, 0x11e, 2),
                     (0x120, 0x146, 0x164 - 0x146),
                     (0x164, 0x168 + 2, 0x168 + 2 - 0x164)
                     ]

        return self.readRegions(
            baseAddr=MXC_BASE_BTLE_DBB_EXT_RFFE, offsetLut=offsetLut)

    def getAll(self):

        ctrl = self.getCtrlAll()
        rx = self.getRxAll()
        tx = self.getTxAll()
        rffe = self.getRffeAll()

        return {
            'ctrl': ctrl,
            'rx': rx,
            'tx': tx,
            'rffe': rffe
        }

    def enableRffePmu(self, enable=True):
        if enable:
            self.write16(self.ctrlAddr(CtrlReg.rffe_pmu_ctrl),(1 << 1))
        else:
            self.write16(self.ctrlAddr(CtrlReg.rffe_pmu_ctrl), (1 << 2))

    def rffePmuIsEnabled(self):
        return self.read16(self.ctrlAddr(CtrlReg.rffe_pmu_ctrl)) & (1 << 1)
