#![no_std]
#![allow(dead_code, unused_imports)]
#![no_main]

use core::prelude::*;
use embedded_hal::prelude::*;
use stm32f1xx_hal::prelude::*;

use defmt;
use defmt_rtt as _;

use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

#[defmt::timestamp]
fn timestamp() -> u64 {
    cortex_m::peripheral::DWT::get_cycle_count() as u64
}

#[app(device=stm32f1::stm32f103, peripherals=true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {};
