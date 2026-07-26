#![doc = include_str!("../README.md")]
#![no_std]
#![warn(missing_docs)]

#[cfg(feature = "sdmmc-concurrency-test")]
extern crate alloc;

mod cmd;
mod regs;
mod sdmmc;
mod utils;
mod dma;

pub use self::sdmmc::{SdMmc, SdMmcError, SdMmcResult};
pub use self::utils::{R1CardStatus, R1CurrentState};
