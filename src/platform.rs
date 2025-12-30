use core::{alloc::Layout, arch::asm};

use alloc::alloc::alloc;

use crate::gmac_const::*;
use crate::net_device;

unsafe extern "C" {
    pub safe fn eth_printf(fmt: *const u8, _: ...) -> i32;

    // 物理地址转换为uncached虚拟地址
    pub safe fn eth_phys_to_uncached(pa: u64) -> u64;

    // cached虚拟地址转换为物理地址
    // ahci dma可以接受64位的物理地址
    pub safe fn eth_virt_to_phys(va: u64) -> u32;

    pub safe fn eth_phys_to_virt(pa: u32) -> u64;

    pub safe fn eth_mdelay(ms: u64);

    pub safe fn eth_handle_rx_buffer(buffer: u64, length: u32) -> u64;
    pub safe fn eth_handle_tx_buffer(p: u64, buffer: u64, len: u64) -> u32;
    pub safe fn eth_isr_install();
}

pub fn eth_malloc_align(size: u64, align: u32) -> u64 {
    unsafe { alloc(Layout::from_size_align_unchecked(size as _, align as _)) as u64 }
}

// 同步dcache中所有cached和uncached访存请求
pub fn eth_sync_dcache() {
    unsafe {
        asm!("dbar 0");
    }
}


// 中断isr通知OS可以调用rx函数
pub fn eth_rx_ready(gmacdev: *mut net_device) {}

// 中断isr通知链路状态发生变化，status - 1表示up，0表示down
// 链路目前仅支持1000Mbps duplex
pub fn eth_update_linkstate(gmacdev: *mut net_device, _status: u32) {}

// OS注册中断，中断号为12

