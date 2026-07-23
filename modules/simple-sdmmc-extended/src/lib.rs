#![doc = include_str!("../README.md")]
#![no_std]
#![warn(missing_docs)]

#[cfg(feature = "multi-block-test")]
extern crate alloc;

mod cmd;
mod regs;
mod sdmmc;
mod utils;
mod dma;

pub use self::sdmmc::SdMmc;
