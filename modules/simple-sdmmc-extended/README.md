# Simple SD/MMC Driver

[![Crates.io](https://img.shields.io/crates/v/simple-sdmmc?style=flat-square)](https://crates.io/crates/simple-sdmmc)
[![Docs.rs](https://img.shields.io/docsrs/simple-sdmmc?style=flat-square)](https://docs.rs/simple-sdmmc)

A simple SD/MMC driver that just works. Pure Rust, `#![no_std]` and no `alloc`.

The current block I/O implementation supports SDHC and SDXC cards using block
addressing. SDSC cards (`OCR.CCS = 0` or CSD v1) require byte-addressed commands
and are rejected during initialization.

Asynchronous writes wait for card programming completion with CMD13 status
queries and cooperative scheduler yields. Synchronous writes retain the direct
DAT/controller-busy polling path.

*Experimental*
