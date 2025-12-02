#![no_std]
#![allow(dead_code, unused_assignments, unused_mut)]

extern crate alloc;

mod drv_eth;
mod eth_const;
mod eth_dma;
mod platform;

pub use drv_eth::*;
pub use eth_const::*;
pub use drv_eth::*;

