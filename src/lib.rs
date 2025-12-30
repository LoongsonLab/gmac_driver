#![no_std]
#![allow(dead_code, unused_assignments, unused_mut)]

extern crate alloc;

mod gmac;
mod gmac_const;
mod gmac_dma;
mod platform;

pub use gmac::*;
pub use gmac_const::*;

