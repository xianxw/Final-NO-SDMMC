//! SD/MMC driver based on SDIO.

use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use log::info;
use simple_sdmmc::SdMmc;

use crate::BlockDriverOps;

/// A SD/MMC driver.
pub struct SdMmcDriver(SdMmc, Option<usize>);

impl SdMmcDriver {
    /// Creates a new [`SdMmcDriver`] from the given base address.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `base` is a valid pointer to the SD/MMC controller's
    /// register block and that no other code is concurrently accessing the same hardware.
    pub unsafe fn new(base: usize, irq_register: impl FnOnce() -> bool, irq_num: Option<usize>) -> Self {
        Self(SdMmc::new(base, irq_register), irq_num)
    }

    pub fn irq_handler() {
        info!("SDMMC IRQ handler invoked");
        info!("SDMMC IRQ handler entering dma_irq_handler");
        SdMmc::dma_irq_handler();
        info!("SDMMC IRQ handler returned from dma_irq_handler");
    }
}

impl BaseDriverOps for SdMmcDriver {
    fn device_type(&self) -> DeviceType {
        DeviceType::Block
    }

    fn device_name(&self) -> &str {
        "sdmmc"
    }

    fn irq_num(&self) -> Option<usize> {
        self.1
    }
}

impl BlockDriverOps for SdMmcDriver {
    fn num_blocks(&self) -> u64 {
        self.0.num_blocks()
    }

    fn block_size(&self) -> usize {
        SdMmc::BLOCK_SIZE
    }

    fn read_block(&mut self, block_id: u64, buf: &mut [u8]) -> DevResult {
        let (blocks, remainder) = buf.as_chunks_mut::<{ SdMmc::BLOCK_SIZE }>();

        if !remainder.is_empty() {
            return Err(DevError::InvalidParam);
        }

        for (i, block) in blocks.iter_mut().enumerate() {
            self.0.read_block(block_id as u32 + i as u32, block);
        }

        Ok(())
    }

    fn write_block(&mut self, block_id: u64, buf: &[u8]) -> DevResult {
        let (blocks, remainder) = buf.as_chunks::<{ SdMmc::BLOCK_SIZE }>();

        if !remainder.is_empty() {
            return Err(DevError::InvalidParam);
        }

        for (i, block) in blocks.iter().enumerate() {
            self.0.write_block(block_id as u32 + i as u32, block);
        }

        Ok(())
    }

    fn flush(&mut self) -> DevResult {
        Ok(())
    }
}
