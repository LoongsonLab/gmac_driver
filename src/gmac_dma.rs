#![allow(dead_code, unused_assignments, unused_mut)]

use crate::gmac_const::*;
use crate::platform::*;

use core::ptr::{null_mut, read_volatile, write_volatile};

use crate::net_device;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct DmaDesc {
    pub status: u32,
    pub length: u32,
    pub buffer1: u32,
    pub buffer2: u32,
}

pub fn eth_mac_read_reg(base: u64, offset: u32) -> u32 {
    let mut addr: u64 = 0;
    let mut data: u32 = 0;
    addr = base + offset as u64;
    unsafe { data = read_volatile(addr as *mut u32) };
    return data;
}

pub fn eth_mac_write_reg(mut base: u64, mut offset: u32, mut data: u32) {
    let mut addr: u64;
    addr = base + offset as u64;
    unsafe { write_volatile(addr as *mut u32, data) };
}

pub fn eth_mac_set_bits(base: u64, offset: u32, pos: u32) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(base, offset);
    data |= pos;
    eth_mac_write_reg(base, offset, data);
}

pub fn eth_mac_clear_bits(base: u64, offset: u32, pos: u32) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(base, offset);
    data &= !pos;
    eth_mac_write_reg(base, offset, data);
}

pub fn eth_mdio_read(regbase: u64, phybase: u32, offset: u32) -> u16 {
    let mut addr: u32 = 0;
    addr = (phybase << GmiiDevShift) & GmiiDevMask | (offset << GmiiRegShift) & GmiiRegMask;
    addr |= GmiiCsrClk4 | GmiiBusy;

    eth_mac_write_reg(regbase, GmacGmiiAddr, addr);
    while eth_mac_read_reg(regbase, GmacGmiiAddr) & GmiiBusy != 0 {}

    return (eth_mac_read_reg(regbase, GmacGmiiData) & 0xffff) as u16;
}

pub fn eth_mdio_write(regbase: u64, phybase: u32, offset: u32, data: u16) {
    eth_mac_write_reg(regbase, GmacGmiiData, data as u32);

    let mut addr: u32 = 0;
    addr = phybase << GmiiDevShift & GmiiDevMask | offset << GmiiRegShift & GmiiRegMask;
    addr |= GmiiWrite | GmiiCsrClk4 | GmiiBusy;

    eth_mac_write_reg(regbase, GmacGmiiAddr, addr);
    while eth_mac_read_reg(regbase, GmacGmiiAddr) & GmiiBusy != 0 {}
}

pub fn eth_mac_set_addr(gmacdev: &net_device) {
    let addr: [u8; 6] = gmacdev.mac_addr;
    let mut data: u32;

    data = ((addr[5] as u32) << 8) | (addr[4] as u32);
    eth_mac_write_reg(gmacdev.mac_base, GmacAddr0High, data);

    data = ((addr[3] as u32) << 24)
        | ((addr[2] as u32) << 16)
        | ((addr[1] as u32) << 8)
        | (addr[0] as u32);
    eth_mac_write_reg(gmacdev.mac_base, GmacAddr0Low, data);
}

pub fn eth_gmac_get_mac_addr(gmacdev: &net_device, addr: &mut [u8; 6]) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.mac_base, GmacAddr0High);
    addr[5] = ((data >> 8) & 0xff) as u8;
    addr[4] = (data & 0xff) as u8;

    data = eth_mac_read_reg(gmacdev.mac_base, GmacAddr0Low);
    addr[3] = ((data >> 24) & 0xff) as u8;
    addr[2] = ((data >> 16) & 0xff) as u8;
    addr[1] = ((data >> 8) & 0xff) as u8;
    addr[0] = (data & 0xff) as u8;
}

pub fn eth_dma_reset(gmacdev: &net_device) {
    let mut data: u32 = 0;

    eth_mac_write_reg(gmacdev.dma_base, DmaBusMode, DmaResetOn);

    loop {
        data = eth_mac_read_reg(gmacdev.dma_base, DmaBusMode);
        if (data & 1) == 0 {
            break;
        }
    }
}

pub fn eth_gmac_resume_dma_rx(gmacdev: &net_device) {
    eth_mac_write_reg(gmacdev.dma_base, DmaRxPollDemand, 0);
}

pub fn eth_gmac_resume_dma_tx(gmacdev: &net_device) {
    eth_mac_write_reg(gmacdev.dma_base, DmaTxPollDemand, 0);
}

pub fn eth_dma_enable_rx(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.dma_base, DmaControl);
    data |= DmaRxStart;
    eth_mac_write_reg(gmacdev.dma_base, DmaControl, data);
}

pub fn eth_dma_enable_tx(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.dma_base, DmaControl);
    data |= DmaTxStart;
    eth_mac_write_reg(gmacdev.dma_base, DmaControl, data);
}

pub fn eth_gmac_disable_dma_tx(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.dma_base, DmaControl);
    data &= !DmaTxStart;
    eth_mac_write_reg(gmacdev.dma_base, DmaControl, data);
}

pub fn eth_gmac_disable_dma_rx(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.dma_base, DmaControl);
    data &= !DmaRxStart;
    eth_mac_write_reg(gmacdev.dma_base, DmaControl, data);
}

pub fn eth_gmac_enable_rx(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.mac_base, GmacConfig);
    data |= GmacRx;
    eth_mac_write_reg(gmacdev.mac_base, GmacConfig, data);
}

pub fn eth_gmac_enable_tx(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.mac_base, GmacConfig);
    data |= GmacTx;
    eth_mac_write_reg(gmacdev.mac_base, GmacConfig, data);
}

pub fn eth_gmac_disable_rx(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.mac_base, GmacConfig);
    data &= !GmacRx;
    eth_mac_write_reg(gmacdev.mac_base, GmacConfig, data);
}

pub fn eth_gmac_disable_tx(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.mac_base, GmacConfig);
    data &= !GmacTx;
    eth_mac_write_reg(gmacdev.mac_base, GmacConfig, data);
}

pub fn eth_dma_clear_curr_irq(gmacdev: &net_device) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.dma_base, DmaStatus);
    eth_mac_write_reg(gmacdev.dma_base, DmaStatus, data);
}

pub fn eth_dma_clear_irq(gmacdev: &net_device, value: u32) {
    eth_mac_write_reg(gmacdev.dma_base, DmaStatus, value);
}

pub fn eth_dma_enable_interrupt(gmacdev: &net_device, value: u32) {
    eth_mac_write_reg(gmacdev.dma_base, DmaInterrupt, value);
}

pub fn eth_dma_disable_interrupt_all(gmacdev: &net_device) {
    eth_mac_write_reg(gmacdev.dma_base, DmaInterrupt, DmaIntDisable);
}

pub fn eth_dma_disable_interrupt(gmacdev: &net_device, value: u32) {
    eth_mac_clear_bits(gmacdev.dma_base, DmaInterrupt, value);
}

pub fn eth_gmac_disable_mmc_irq(gmacdev: &net_device) {
    eth_mac_write_reg(gmacdev.mac_base, GmacMmcIntrMaskTx, 0xffffffff);
    eth_mac_write_reg(gmacdev.mac_base, GmacMmcIntrMaskRx, 0xffffffff);
    eth_mac_write_reg(gmacdev.mac_base, GmacMmcRxIpcIntrMask, 0xffffffff);
}

pub fn eth_dma_bus_mode_init(gmacdev: &net_device) {
    let mut value: u32 = 0;
    value |= DmaMixedBurstEnable;
    value |= DmaBurstLengthx8 | DmaBurstLength32;
    value |= DmaDescriptor4DWords | DmaDescriptorSkip0;
    eth_mac_write_reg(gmacdev.dma_base, DmaBusMode, value);
}

pub fn eth_dma_control_init(gmacdev: &net_device) {
    let mut value: u32 = 0;
    value |= DmaStoreAndForward | DmaTxSecondFrame;
    eth_mac_write_reg(gmacdev.dma_base, DmaControl, value);
}

pub fn eth_dma_axi_bus_mode_init(gmacdev: &net_device) {
    let mut value: u32 = 0;
    value |= 0xff;
    value |= 0x77 << 16;
    eth_mac_write_reg(gmacdev.dma_base, 0x28, value);
}

pub fn eth_dma_reg_init(gmacdev: &net_device) {
    eth_dma_bus_mode_init(gmacdev);
    eth_dma_control_init(gmacdev);
    eth_dma_axi_bus_mode_init(gmacdev);
}

pub fn eth_gmac_back_off_limit(gmacdev: &net_device, value: u32) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.mac_base, GmacConfig);
    data &= !GmacBackoffLimit;
    data |= value;
    eth_mac_write_reg(gmacdev.mac_base, GmacConfig, data);
}

pub fn eth_gmac_config_init(gmacdev: &net_device) {
    eth_mac_set_bits(gmacdev.mac_base, GmacConfig, GmacTxConfig);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacWatchdog);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacJabber);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacFrameBurst);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacJumboFrame);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacRxOwn);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacLoopback);
    eth_mac_set_bits(gmacdev.mac_base, GmacConfig, GmacDuplex);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacRetry);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacPadCrcStrip);
    eth_mac_clear_bits(gmacdev.mac_base, GmacConfig, GmacDeferralCheck);
    eth_gmac_back_off_limit(gmacdev, GmacBackoffLimit0);
}

pub fn eth_gmac_set_pass_control(gmacdev: &net_device, value: u32) {
    let mut data: u32 = 0;
    data = eth_mac_read_reg(gmacdev.mac_base, GmacFrameFilter);
    data &= !GmacPassControl;
    data |= value;
    eth_mac_write_reg(gmacdev.mac_base, GmacFrameFilter, data);
}

pub fn eth_gmac_frame_filter(gmacdev: &net_device) {
    eth_mac_clear_bits(gmacdev.mac_base, GmacFrameFilter, GmacSrcAddrFilter);
    eth_gmac_set_pass_control(gmacdev, GmacPassControl0);
    eth_mac_clear_bits(gmacdev.mac_base, GmacFrameFilter, GmacBroadcast);
    eth_mac_clear_bits(gmacdev.mac_base, GmacFrameFilter, GmacMulticastFilter);
    eth_mac_clear_bits(gmacdev.mac_base, GmacFrameFilter, GmacDestAddrFilter);
    eth_mac_clear_bits(gmacdev.mac_base, GmacFrameFilter, GmacMcastHashFilter);
    eth_mac_clear_bits(gmacdev.mac_base, GmacFrameFilter, GmacUcastHashFilter);
    eth_mac_clear_bits(gmacdev.mac_base, GmacFrameFilter, GmacPromiscuousMode);
    eth_mac_set_bits(gmacdev.mac_base, GmacFrameFilter, GmacFilter);
}

pub fn eth_gmac_flow_control(gmacdev: &net_device) {
    let mut dma_ctrl: u32 = 0;
    dma_ctrl = eth_mac_read_reg(gmacdev.dma_base, DmaControl);
    dma_ctrl &= !(DmaRxFlowCtrlAct | DmaRxFlowCtrlDeact);
    dma_ctrl &= !DmaEnHwFlowCtrl;
    eth_mac_write_reg(gmacdev.dma_base, DmaControl, dma_ctrl);

    let mut flow_ctrl: u32 = 0;
    flow_ctrl |= GmacPauseTimeMask;
    flow_ctrl &= !(GmacRxFlowControl | GmacTxFlowControl);
    eth_mac_write_reg(gmacdev.mac_base, GmacFlowControl, flow_ctrl);
}

pub fn eth_gmac_reg_init(gmacdev: &net_device) {
    eth_gmac_config_init(gmacdev);
    eth_gmac_frame_filter(gmacdev);
    eth_gmac_flow_control(gmacdev);
}

pub fn eth_setup_tx_desc_queue(gmacdev: &mut net_device, desc_num: u32) {
    let mut desc: *mut DmaDesc = null_mut();
    let mut dma_addr: u32 = 0;
    let mut buffer: u64 = 0;

    desc = eth_malloc_align((size_of::<DmaDesc>() * (desc_num as usize)) as u64, 64)
        as *mut DmaDesc;
    dma_addr = eth_virt_to_phys(desc as u64);

    gmacdev.tx_next = 0;
    gmacdev.tx_busy = 0;

    eth_mac_write_reg(gmacdev.dma_base, DmaTxBaseAddr, dma_addr);

    for i in 0..desc_num {
        buffer = eth_malloc_align(2048, 64);
        gmacdev.tx_desc[i as usize] = desc;
        gmacdev.tx_buffer[i as usize] = buffer;

        let is_last = i == desc_num - 1;
        unsafe {
            (*desc).status = if is_last { TxDescEndOfRing } else { 0 };
            (*desc).length = 0;
            (*desc).buffer1 = 0;
            (*desc).buffer2 = 0;
            desc = desc.offset(1);
        }
    }
}

pub fn eth_setup_rx_desc_queue(gmacdev: &mut net_device, desc_num: u32) {
    let mut desc: *mut DmaDesc = null_mut();
    let mut dma_addr: u32 = 0;
    let mut buffer: u64 = 0;

    desc = eth_malloc_align((size_of::<DmaDesc>() * (desc_num as usize)) as u64, 16)
        as *mut DmaDesc;
    dma_addr = unsafe { eth_virt_to_phys(desc as u64) };

    gmacdev.rx_busy = 0;

    eth_mac_write_reg(gmacdev.dma_base, DmaRxBaseAddr, dma_addr);

    for i in 0..desc_num {
        buffer = eth_malloc_align(2048, 16);
        dma_addr = eth_virt_to_phys(buffer);
        gmacdev.rx_desc[i as usize] = desc;
        gmacdev.rx_buffer[i as usize] = buffer;

        let is_last = i == desc_num - 1;
        unsafe {
            (*desc).status = DescOwnByDma;
            (*desc).length = if is_last { RxDescEndOfRing } else { 0 };
            (*desc).length |= ((2048 << DescSize1Shift) & DescSize1Mask) as u32;
            (*desc).buffer1 = dma_addr;
            (*desc).buffer2 = 0;
            desc = desc.offset(1);
        }
    }
}

pub fn eth_get_desc_owner(desc: &DmaDesc) -> bool {
    return (desc.status & DescOwnByDma) == DescOwnByDma;
}

pub fn eth_get_rx_length(desc: &DmaDesc) -> u32 {
    return (desc.status & DescFrameLengthMask) >> DescFrameLengthShift;
}

pub fn eth_is_tx_desc_valid(desc: &DmaDesc) -> bool {
    return (desc.status & DescError) == 0;
}

pub fn eth_is_desc_empty(desc: &DmaDesc) -> bool {
    return (desc.length & DescSize1Mask == 0) && (desc.length & DescSize2Mask == 0);
}

pub fn eth_is_rx_desc_valid(desc: &DmaDesc) -> bool {
    return (desc.status & DescError == 0)
        && (desc.status & DescRxFirst == DescRxFirst)
        && (desc.status & DescRxLast == DescRxLast);
}

pub fn eth_is_last_rx_desc(desc: &DmaDesc) -> bool {
    return desc.length & RxDescEndOfRing == RxDescEndOfRing;
}

pub fn eth_is_last_tx_desc(desc: &DmaDesc) -> bool {
    return desc.status & TxDescEndOfRing == TxDescEndOfRing;
}
