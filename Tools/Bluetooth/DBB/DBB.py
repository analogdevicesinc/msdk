
class RegisterOffsets:
    CTRL = 0x1000
    TX = 0x2000
    RX = 0x3000
    RFFE = 0x8000


class CtrlOffsets:
    dbb_ctrl_version = 0x0000
    dbb_ctrl_rst = 0x0002
    tx_ctrl_dbb_en_dly = 0x0004
    tx_ctrl_rffe_en_dly = 0x0008
    tx_ctrl_dbb_dis_dly = 0x0010
    tx_ctrl_rffe_dis_dly = 0x0014
    tx_ctrl_cmd = 0x001c
    tx_ctrl_debug_en_tx_on_sfd_to = 0x001e
    rx_ctrl_dbb_en_dly = 0x0020: < /tt > DBB_CTRL RX_CTRL_DBB_EN_DLY Register * /
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
    BaseAddr = 0x40050000
    CtrlAddr = BaseAddr + RegisterOffsets.CTRL
    TxAddr = BaseAddr + RegisterOffsets.TX
    RxAddr = BaseAddr + RegisterOffsets.RX
    RffeAddr = BaseAddr + RegisterOffsets.RFFE

    def __init__(self, pyOcdTarget):
        self.target = pyOcdTarget

    def getCtrlVersionInfo(self):
        DBB_CTRL = 0x40051000
        version = self.target.read16(DBB_CTRL, 1)
        version_minor = version & 0xff
        version_major = (version >> 8) * 0xff

        return (version_major, version_minor)

    def getCtrl(self):
        LEN = 0x120 + 16

        ctrl = self.target.read8(self.CtrlAddr, LEN)

        return ctrl

    def getTx(self):
        pass

    def getRx(self):
        pass

    def getRffe(self):
        pass

    def setRffePmu(self):
        pass