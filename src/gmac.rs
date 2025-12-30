#![allow(dead_code, unused_assignments, unused_mut)]

use core::slice::from_raw_parts_mut;
use alloc::vec::Vec;

use crate::gmac_const::*;
use crate::gmac_dma::*;
use crate::platform::*;


#[derive(Copy, Clone)]
#[repr(C)]
pub struct net_device {
    pub parent: *mut u8,
    pub iobase: u64,
    pub mac_addr: [u8; 6],
    pub mac_base: u64,
    pub dma_base: u64,
    pub phy_base: u64,
    pub version: u32,
    pub tx_busy: u32,
    pub tx_next: u32,
    pub rx_busy: u32,
    pub tx_desc: [*mut DmaDesc; 128],
    pub rx_desc: [*mut DmaDesc; 128],
    pub tx_buffer: [u64; 128],
    pub rx_buffer: [u64; 128],
    pub rx_packets: u64,
    pub tx_packets: u64,
    pub rx_bytes: u64,
    pub tx_bytes: u64,
    pub rx_errors: u64,
    pub tx_errors: u64,
    pub advertising: u32,
    pub link_status: u32,
    pub duplex_mode: u32,
    pub speed: u32,
    pub rx_ready: Option<u32>
}

impl Default for net_device {
    fn default() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

impl net_device {
    pub const fn new() -> Self {
        unsafe { core::mem::zeroed() }
    }
}


unsafe impl Sync for net_device {}
unsafe impl Send for net_device {}

// 检查rgmii链路状态
// eth_update_linkstate通知操作系统链路状态
pub fn eth_phy_rgsmii_check(gmacdev: &mut net_device) {
    let mut value: u32 = 0;
    let mut status: u32 = 0;

    value = eth_mac_read_reg(gmacdev.mac_base, GmacRgsmiiStatus);
    status = value & (MacLinkStatus >> MacLinkStatusOff);

    if gmacdev.link_status != status {
        eth_update_linkstate(gmacdev, status);
    }

    if status != 0 {
        gmacdev.link_status = 1;
        gmacdev.duplex_mode = value & MacLinkMode;
        let mut speed: u32 = value & MacLinkSpeed;
        if speed == MacLinkSpeed_125 {
            gmacdev.speed = 1000;
        } else if speed == MacLinkSpeed_25 {
            gmacdev.speed = 100;
        } else {
            gmacdev.speed = 10;
        }
        eth_printf(
            b"Link is Up - %u Mpbs / %s Duplex\n\0" as *const u8,
            gmacdev.speed,
            if gmacdev.duplex_mode != 0 {
                b"Full\0" as *const u8
            } else {
                b"Half\0" as *const u8
            },
        );
    } else {
        gmacdev.link_status = 0;
        eth_printf(b"Link is Down\n\0" as *const u8);
    };
}

// 初始化phy
pub fn eth_phy_init(gmacdev: &mut net_device) {
    let mut phy: u32 = 0;
    let mut data: u32 = 0;

    data = eth_mdio_read(gmacdev.mac_base, gmacdev.phy_base as u32, 2) as u32;
    phy |= data << 16;
    data = eth_mdio_read(gmacdev.mac_base, gmacdev.phy_base as u32, 3) as u32;
    phy |= data;

    match phy {
        0x0000010a => {
            eth_printf(
                b"probed ethernet phy YT8511H/C, id 0x%08x\n\0" as *const u8,
                phy,
            );
        }
        _ => {
            eth_printf(
                b"probed unknown ethernet phy, id 0x%08x\n\0" as *const u8,
                phy,
            );
        }
    };
}

pub fn eth_handle_tx_over(gmacdev: &mut net_device) {
    loop {
        let mut desc_idx: u32 = gmacdev.tx_busy;
        let mut txdesc: DmaDesc = unsafe { gmacdev.tx_desc[desc_idx as usize].read() } as DmaDesc;

        if eth_get_desc_owner(&txdesc) || eth_is_desc_empty(&txdesc) {
            break;
        }

        if eth_is_tx_desc_valid(&txdesc) {
            let mut length: u32 = (txdesc.length & DescSize1Mask) >> DescSize1Shift;
            gmacdev.tx_bytes += length as u64;
            gmacdev.tx_packets += 1;
        } else {
            gmacdev.tx_errors += 1;
        }

        let is_last: bool = eth_is_last_tx_desc(&txdesc);
        txdesc.status = if is_last { TxDescEndOfRing } else { 0 };
        txdesc.length = 0;
        txdesc.buffer1 = 0;
        txdesc.buffer2 = 0;
        unsafe {
            gmacdev.tx_desc[desc_idx as usize].write(txdesc);
        }

        gmacdev.tx_busy = if is_last { 0 } else { desc_idx + 1 };
    }
}

// 操作系统传递接收数据的单元pbuf给驱动
// pbuf可能是操作系统自定义结构
// 返回接收到的数据字节数
pub fn eth_tx(gmacdev: &mut net_device, pbuf: Vec<u8>) -> i32 {
    let mut buffer: u64 = 0;
    let mut length: u32 = 0;
    let mut dma_addr: u32 = 0;
    let mut desc_idx: u32 = gmacdev.tx_next;
    let mut txdesc: DmaDesc = unsafe { gmacdev.tx_desc[desc_idx as usize].read() } as DmaDesc;
    let mut is_last: bool = eth_is_last_tx_desc(&txdesc);

    if eth_get_desc_owner(&txdesc) {
        return -1;
    }

    buffer = gmacdev.tx_buffer[desc_idx as usize];
    length = pbuf.len() as u32;

    let dmapbuf = unsafe { from_raw_parts_mut(buffer as *mut u8, length as usize) };
    dmapbuf.copy_from_slice(pbuf.as_slice());

    dma_addr = eth_virt_to_phys(buffer);

    txdesc.status |= DescOwnByDma | DescTxIntEnable | DescTxLast | DescTxFirst;
    txdesc.length = length << DescSize1Shift & DescSize1Mask;
    txdesc.buffer1 = dma_addr;
    txdesc.buffer2 = 0;
    unsafe {
        gmacdev.tx_desc[desc_idx as usize].write(txdesc);
    }

    gmacdev.tx_next = if is_last { 0 } else { desc_idx + 1 };

    eth_sync_dcache();
    eth_gmac_resume_dma_tx(gmacdev);

    // eth_printf(b"gmac send packet: %lx, len: %d\n\0" as *const u8, buffer, length);
    // eth_mdelay(1);

    return 0;
}

// pbuf是返回给操作系统的数据单元
// 可能是操作系统自定义结构
pub fn eth_rx(gmacdev: &mut net_device) -> Option<Vec<Vec<u8>>> {
    let mut recv_packets = Vec::new();
    let mut desc_idx: u32 = gmacdev.rx_busy;
    let mut rxdesc: DmaDesc = unsafe { gmacdev.rx_desc[desc_idx as usize].read() } as DmaDesc;
    let mut is_last: bool = eth_is_last_rx_desc(&rxdesc);

    if eth_is_desc_empty(&rxdesc) || eth_get_desc_owner(&rxdesc) {
        eth_dma_enable_interrupt(gmacdev, DmaIntEnable);
        return None;
    }

    let mut dma_addr = rxdesc.buffer1;

    if eth_is_rx_desc_valid(&rxdesc) {
        let mut length: u32 = eth_get_rx_length(&rxdesc);
        let mut buffer: u64 = eth_phys_to_virt(dma_addr);

        eth_sync_dcache();
        let mbuf = unsafe { from_raw_parts_mut(buffer as *mut u8, length as usize) };
        recv_packets.push(mbuf.to_vec());
        gmacdev.rx_bytes += length as u64;
        gmacdev.rx_packets += 1;
    } else {
        gmacdev.rx_errors += 1;
    }

    rxdesc.status = DescOwnByDma;
    rxdesc.length = if is_last { RxDescEndOfRing } else { 0 };
    rxdesc.length |= (2048) << DescSize1Shift & DescSize1Mask;
    rxdesc.buffer1 = dma_addr;
    rxdesc.buffer2 = 0;
    unsafe {
        gmacdev.rx_desc[desc_idx as usize].write(rxdesc);
    }

    gmacdev.rx_busy = if is_last { 0 } else { desc_idx + 1 };

    if recv_packets.len() > 0 {
        Some(recv_packets)
    } else {
        None
    }
}

// 中断处理程序
// eth_rx_ready通知操作系统可以接收数据
// eth_handle_tx_over用于处理已经发送完的描述符
#[unsafe(no_mangle)]
pub fn eth_irq(gmacdev: &mut net_device) {
    let mut dma_status: u32 = 0;
    let mut dma_int_enable: u32 = DmaIntEnable;

    gmacdev.rx_ready = None;

    dma_status = eth_mac_read_reg(gmacdev.dma_base, DmaStatus);
    if dma_status == 0 {
        return;
    }

    eth_dma_disable_interrupt_all(gmacdev);

    if dma_status & GmacPmtIntr != 0 {
        eth_printf(b"gmac pmt interrupt\n\0" as *const u8);
    }
    if dma_status & GmacMmcIntr != 0 {
        eth_printf(b"gmac mmc interrupt\n\0" as *const u8);
    }
    if dma_status & GmacLineIntfIntr != 0 {
        eth_mac_read_reg(gmacdev.mac_base, GmacInterruptStatus);
        eth_mac_read_reg(gmacdev.mac_base, GmacInterruptMask);
        if eth_mac_read_reg(gmacdev.mac_base, GmacInterruptStatus) & GmacRgmiiIntSts != 0 {
            eth_mac_read_reg(gmacdev.mac_base, GmacRgsmiiStatus);
        }
        eth_phy_rgsmii_check(gmacdev);
    }

    eth_mac_write_reg(gmacdev.dma_base, DmaStatus, dma_status);

    if dma_status & DmaIntBusError != 0 {
        eth_printf(b"gmac fatal bus error interrupt\n\0" as *const u8);
    }
    if dma_status & DmaIntRxStopped != 0 {
        eth_printf(b"gmac receive process stopped\n\0" as *const u8);
        eth_dma_enable_rx(gmacdev);
    }
    if dma_status & DmaIntRxNoBuffer != 0 {
        // eth_printf(b"gmac receive buffer unavailable\n\0" as *const u8);
        dma_int_enable &= !DmaIntRxNoBuffer;
        eth_gmac_resume_dma_rx(gmacdev);
        gmacdev.rx_ready = Some(1 as u32);
        eth_rx_ready(gmacdev);
    }
    if dma_status & DmaIntRxCompleted != 0 {
        dma_int_enable &= !DmaIntRxCompleted;
        gmacdev.rx_ready = Some(1 as u32);
        eth_rx_ready(gmacdev);
        // eth_printf(b"gmac transmit receive success!\n\0" as *const u8);
    }
    if dma_status & DmaIntTxUnderflow != 0 {
        eth_printf(b"gmac transmit underflow\n\0" as *const u8);
    }
    if dma_status & DmaIntRcvOverflow != 0 {
        eth_printf(b"gmac receive underflow\n\0" as *const u8);
    }
    if dma_status & DmaIntTxNoBuffer != 0 {}
    if dma_status & DmaIntTxStopped != 0 {
        eth_printf(b"gmac transmit process stopped\n\0" as *const u8);
    }
    if dma_status & DmaIntTxCompleted != 0 {
        eth_handle_tx_over(gmacdev);
        // eth_printf(b"gmac transmit process over!\n\0" as *const u8);
    }
    eth_dma_enable_interrupt(gmacdev, dma_int_enable);
}

// 初始化
pub fn eth_init(gmacdev: &mut net_device) -> i32 {
    // 在eth_init内或外，利用uncached地址初始化结构体的iobase
    // gmacdev.iobase = eth_phys_to_uncached(0x40040000);
    gmacdev.mac_base = gmacdev.iobase + 0x0000;
    gmacdev.dma_base = gmacdev.iobase + 0x1000;
    gmacdev.phy_base = 0;
    gmacdev.version = eth_mac_read_reg(gmacdev.mac_base, GmacVersion);

    eth_printf(b"gmac eth_init...\n\0" as *const u8);

    eth_dma_reset(gmacdev);
    eth_mac_set_addr(gmacdev);
    eth_phy_init(gmacdev);

    eth_setup_rx_desc_queue(gmacdev, 128);
    eth_setup_tx_desc_queue(gmacdev, 128);

    eth_dma_reg_init(gmacdev);
    eth_gmac_reg_init(gmacdev);

    eth_sync_dcache();

    eth_gmac_disable_mmc_irq(gmacdev);
    eth_dma_clear_curr_irq(gmacdev);

    eth_dma_enable_interrupt(gmacdev, DmaIntEnable);

    eth_gmac_enable_rx(gmacdev);
    eth_gmac_enable_tx(gmacdev);
    eth_dma_enable_rx(gmacdev);
    eth_dma_enable_tx(gmacdev);

    eth_isr_install();
    eth_phy_rgsmii_check(gmacdev);

    return 0;
}
