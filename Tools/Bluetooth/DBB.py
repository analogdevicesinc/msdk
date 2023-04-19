
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


class RffeReg:
    rffe_ifc_version = 0x0000
    rffe_ifc_rffe_version = 0x0002
    general_param = 0x0004
    general_delay_freq = 0x0006
    general_delay_amp = 0x0008
    general_spi_invert_csn = 0x000a
    general_skip_sdmod = 0x000c
    tx_seq_ena_seq_lngth_total = 0x0010
    tx_seq_ena_seq_lngth_part = 0x0012
    tx_seq_ena_cmd = 0x0014
    tx_seq_dis_seq_lngth_total = 0x0034
    tx_seq_dis_seq_lngth_part = 0x0036
    tx_seq_dis_cmd = 0x0038
    tx_seq_spi = 0x0058
    rx_seq_ena_seq_lngth_total = 0x005c
    rx_seq_ena_seq_lngth_part = 0x005e
    rx_seq_ena_cmd = 0x0060
    rx_seq_dis_seq_lngth_total = 0x0080
    rx_seq_dis_seq_lngth_part = 0x0082
    rx_seq_dis_cmd = 0x0084
    rx_seq_spi = 0x00a4
    rffe_spim_cfg = 0x00a8
    rffe_spim_data_out = 0x00aa
    rffe_spim_start_transaction = 0x00ac
    rffe_spim_data_in = 0x00ae
    agc_spi_cfg = 0x00b0
    agc_enc5_gain_addr = 0x00b4
    agc_enc5_gain_table = 0x00b6
    agc_enc5_gain_table_db = 0x00c6
    agc_enc5_ready_tmr = 0x00d6
    agc_enc5_offs_i_addr = 0x00d8
    agc_enc5_offs_q_addr = 0x00da
    agc_enc5_dc_offs = 0x00dc
    agc_enc5_offs_i_bypass = 0x00de
    agc_enc5_offs_q_bypass = 0x00ee
    agc_enc5_curr_dc_offs_i = 0x00fe
    agc_enc5_curr_dc_offs_q = 0x010e
    rsv_0x11e = 0x11e
    general2_skip_fir_amp = 0x0120
    general2_fir_coefficients = 0x0122
    general2_fir_scalingfactor = 0x012a
    anti_alias_am_skip_fir = 0x012c
    anti_alias_am_coeff = 0x012e
    anti_alias_am_scalingfactor = 0x0136
    anti_alias_fm_skip_fir = 0x0138
    anti_alias_fm_coeff = 0x013a
    anti_alias_fm_scalingfactor = 0x0142
    cdc_clk_div_en = 0x0144
    rsv_0x146_0x163 = 0x146
    iq_ctrl = 0x0164
    iq_data = 0x0168


class OffsetLuts:

    max32665 = {
        'ctrl': [
            (0x00, 0x96, 0xff - 0x96 + 1),
            (0x100, 0x108, 4),
            (0x10c, 0x110, 0x120 - 0x100)
        ],
        'tx': [
            (0x00, 0x70, 0x180 - 0x70),
            # any address offset past 0x18c causes a hardfault
            (0x180, 0x18c, 0x194 + 76 - 0x18c)
        ],

        'rx': [(0x00,    0x76, 2),
               (0x78,    0x13a, 2),
               (0x13c,   0x2dc, 73*4),
               (0x400,   0x404, 4),
               (0x408,   0x40c, 4*5),
               # The entire region with CTE values causes hardfaults
               (0x420, 0x424, 0x586 - 0x424),
               ],
        'rffe': [(0x00, 0x11e, 2),
                 (0x120, 0x146, 0x164 - 0x146),
                 (0x164, 0x168 + 2, 0x168 + 2 - 0x164)
                 ]}

    max32655 = {
        'ctrl': [
            (0x00, 0x96, 0xff - 0x96 + 1),
            (0x100, 0x108, 4),
            (0x10c, 0x110, 0x120 - 0x100)
        ],
        'tx': [
            (0x00, 0x70, 0x180 - 0x70),
            # any address offset past 0x18c causes a hardfault
            (0x180, 0x18c, 0x194 + 76 - 0x18c)
        ],

        'rx': [(0x00,    0x76, 2),
               (0x78,    0x13a, 2),
               (0x13c,   0x2dc, 73*4),
               (0x400,   0x404, 4),
               (0x408,   0x40c, 4*5),
               # The entire region with CTE values causes hardfaults
               (0x420, 0x424, 0x586 - 0x424),
               ],
        'rffe': [(0x00, 0x11e, 2),
                 (0x120, 0x146, 0x164 - 0x146),
                 (0x164, 0x168 + 2, 0x168 + 2 - 0x164)
                 ]}


    #event timing region skipped
    max32690 = {
        'ctrl': [
            (0x00, 0x50, 0x84 - 0x50 + 1),
            (0x84, 0x96, 0xff - 0x96 + 1),
            (0x100, 0x108, 4),
            (0x10c, 0x110, 0x120 - 0x100)
        ],
        'tx': [
            (0x00, 0x70, 0x180 - 0x70),
            # any address offset past 0x18c causes a hardfault
            (0x180, 0x18c, 0x194 + 76 - 0x18c)
        ],

        'rx': [(0x00,    0x76, 2),
               (0x78,    0x13a, 2),
               (0x13c,   0x2dc, 73*4),
               (0x400,   0x404, 4),
               (0x408,   0x40c, 4*5),
               # The entire region with CTE values causes hardfaults
               (0x420, 0x424, 0x586 - 0x424),
               ],
        'rffe': [(0x00, 0x11e, 2),
                 (0x120, 0x146, 0x164 - 0x146),
                 (0x164, 0x168 + 2, 0x168 + 2 - 0x164)
                 ]}

    @staticmethod
    def getOffsetLut(chip):
        chip = chip.lower()
        if chip == 'max32655':
            return OffsetLuts.max32655
        elif chip == 'max32665':
            return OffsetLuts.max32665
        elif chip == 'max32690':
            return OffsetLuts.max32690
        else:
            msg = f'Chip {chip} is not valid'
            raise Exception(msg)


class DBB:
    DBBBaseAddr = 0x40050000
    CtrlAddrBase = DBBBaseAddr + RegisterOffsets.CTRL
    TxBaseAddr = DBBBaseAddr + RegisterOffsets.TX
    RxBaseAddr = DBBBaseAddr + RegisterOffsets.RX
    RffeBaseAddr = DBBBaseAddr + RegisterOffsets.RFFE

    RegSize8 = 0x1
    RegSize16 = 0x2
    RegSize32 = 0x3

    def __init__(self, pyOcdTarget, chip='max32655'):
        self.target = pyOcdTarget
        self.chip = chip
        self.offsetLuts = OffsetLuts.getOffsetLut(self.chip)

    def read8(self, addr):
        return self.target.read8(addr)

    def read16(self, addr):
        return self.target.read16(addr)

    def read32(self, addr):
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

    def write8(self, addr, value):
        self.target.write8(addr, value)

    def write16(self, addr, value):
        self.target.write8(addr, value)

    def write32(self, addr, value):
        self.target.write8(addr, value)

    def read(self, addr, size=0x1):
        if size == self.RegSize8:
            return self.read8(addr)
        elif size == self.RegSize16:
            return self.read16(addr)
        elif size == self.RegSize32:
            return self.read32(addr)

    def write(self, addr, value):

        if value <= 255:
            self.write8(addr, value)
        elif value <= 65535:
            self.write16(addr, value)
        else:
            self.write32(addr, value)

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

            # readout = [hex(i) for i in readout]

            if len(readout) != (regionLength):
                print('Error occurred during readout.')
                return []

            regions.extend(readout)

            # add reserved region to the register read
            regions.extend([0x00] * reserved_len)

        return regions

    def spim_setup(self):
        cpha = 0
        cpol = 1
        endianess = 0
        clk_rate = 2

        spi_setup = (
            (clk_rate << 0) |
            (endianess << 4) |
            (cpol << 5) |
            (cpha << 6) |
            1 << 7 |
            0)

        self.write8(self.rffeAddr(RffeReg.rffe_spim_cfg), spi_setup)
        self.write8(self.rffeAddr(RffeReg.tx_seq_spi), spi_setup)
        self.write8(self.rffeAddr(RffeReg.rx_seq_spi), spi_setup)
        self.write8(self.rffeAddr(RffeReg.agc_spi_cfg), spi_setup)

    def spim_write(self, addr, value):

        self.enableRffePmu(True)

        while not (self.readCtrl(CtrlReg.gen_pmu_status, self.RegSize16) & (1 << 2)):
            pass

        for i in range(0xf):
            # wait a little extra
            pass

        addr |= (1 << 7)

        self.writeCtrl(CtrlReg.events_status, (1 << 5))
        self.writeRffe(RffeReg.rffe_spim_data_out, (addr << 8) + value)
        self.writeRffe(RffeReg.rffe_spim_start_transaction, (1 << 0))

        while not (self.readCtrl(CtrlReg.events_status) & (1 << 5), self.RegSize16):
            pass

        self.enableRffePmu(False)

    def spim_read(self, addr):
        self.enableRffePmu(True)

        while not (self.readCtrl(CtrlReg.gen_pmu_status, self.RegSize16) & (1 << 2)):
            pass

        for i in range(0xf):
            # wait a little extra
            pass

        addr &= ~(1 << 7)

        self.writeCtrl(CtrlReg.events_status, (1 << 5))
        self.writeRffe(RffeReg.rffe_spim_data_out, (addr << 8) + 0)
        self.writeRffe(RffeReg.rffe_spim_start_transaction, (1 << 0))

        while not self.readCtrl(CtrlReg.events_status, self.RegSize16) & (1 << 5):
            pass

        self.enableRffePmu(False)

        return self.readRffe(RffeReg.rffe_spim_data_in, self.RegSize8)

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

    def readCtrl(self, register, size):
        return self.read(self.ctrlAddr(register), size)

    def writeCtrl(self, register, value):
        self.write(self.ctrlAddr(register), value)

    def readRffe(self, register, size):
        return self.read(self.rffeAddr(register), size)

    def writeRffe(self, register, value):
        self.write(self.rffeAddr(register), value)

    def getCtrlAll(self):

        offsetLut = self.offsetLuts['ctrl']
        ctrl = self.readRegions(self.CtrlAddrBase, offsetLut)

        return ctrl

    def getTxAll(self):
        """
        Read out the TX register of the DBB and return as list
        Reserved regions will be set as '00'
        """

        MXC_BASE_BTLE_DBB_TX = self.TxBaseAddr

        
        offsetLut = self.offsetLuts['tx']

        return self.readRegions(
            baseAddr=MXC_BASE_BTLE_DBB_TX, offsetLut=offsetLut)

    def getRxAll(self):
        """
        Reads the contents of the rx register and returns data as a list
        All reserved regions initialized as '00'
        """
        MXC_BASE_BTLE_DBB_RX = self.RxBaseAddr

        offsetLut = self.offsetLuts['rx']

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


        offsetLut = self.offsetLuts['rffe']

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
            self.write16(self.ctrlAddr(CtrlReg.rffe_pmu_ctrl), (1 << 1))
        else:
            self.write16(self.ctrlAddr(CtrlReg.rffe_pmu_ctrl), (1 << 2))

    def getPmuStatus(self):
        return self.readCtrl(CtrlReg.gen_pmu_status, self.RegSize32)

    def txPmuIsEnable(self):
        return bool(self.getPmuStatus() & 0x1)

    def rffePmuIsEnabled(self):
        return bool(self.getPmuStatus() & (1 << 1))

    def rxPmuIsEnabled(self):
        return bool(self.getPmuStatus() & (1 << 2))
