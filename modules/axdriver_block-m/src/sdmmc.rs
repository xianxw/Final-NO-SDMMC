//! SD/MMC driver based on SDIO.

use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use log::error;
use simple_sdmmc::{SdMmc, SdMmcError};

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
    pub unsafe fn new(
        base: usize,
        irq_register: impl FnOnce() -> bool,
        irq_num: Option<usize>,
    ) -> Result<Self, SdMmcError> {
        Ok(Self(unsafe { SdMmc::new(base, irq_register) }?, irq_num))
    }

    pub fn irq_handler() {
        SdMmc::dma_irq_handler();
    }

    #[cfg(feature = "sdmmc-concurrency-test")]
    pub fn run_concurrency_test(&mut self) -> ! {
        self.0.run_concurrency_test()
    }

    fn map_error(error: SdMmcError) -> DevError {
        error!("SD/MMC block operation failed: {error:?}");
        match error {
            SdMmcError::UnsupportedCard => DevError::Unsupported,
            SdMmcError::InvalidParameter | SdMmcError::OutOfRange => DevError::InvalidParam,
            SdMmcError::DmaAllocation => DevError::NoMemory,
            SdMmcError::CommandBusy | SdMmcError::DataBusy => DevError::ResourceBusy,
            SdMmcError::DmaUnavailable
            | SdMmcError::DmaAddress
            | SdMmcError::DriverFaulted
            | SdMmcError::RecoveryFailed => DevError::BadState,
            SdMmcError::DescriptorPublication
            | SdMmcError::InitializationFailed
            | SdMmcError::CardStatus(_)
            | SdMmcError::AppCommandRejected(_)
            | SdMmcError::CommandStartTimeout
            | SdMmcError::CommandTimeout
            | SdMmcError::DataTimeout
            | SdMmcError::Hardware
            | SdMmcError::TerminalValidation
            | SdMmcError::CardBusyTimeout => DevError::Io,
        }
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
        if buf.is_empty() || !buf.len().is_multiple_of(SdMmc::BLOCK_SIZE) {
            return Err(DevError::InvalidParam);
        }
        let block_count = (buf.len() / SdMmc::BLOCK_SIZE) as u64;
        let Some(end_block) = block_id.checked_add(block_count) else {
            return Err(DevError::InvalidParam);
        };
        if block_id > u32::MAX as u64
            || end_block > u32::MAX as u64 + 1
            || end_block > self.0.num_blocks()
        {
            return Err(DevError::InvalidParam);
        }

        self.0
            .read_blocks(block_id as u32, buf)
            .map_err(Self::map_error)
    }

    fn write_block(&mut self, block_id: u64, buf: &[u8]) -> DevResult {
        if buf.is_empty() || !buf.len().is_multiple_of(SdMmc::BLOCK_SIZE) {
            return Err(DevError::InvalidParam);
        }
        let block_count = (buf.len() / SdMmc::BLOCK_SIZE) as u64;
        let Some(end_block) = block_id.checked_add(block_count) else {
            return Err(DevError::InvalidParam);
        };
        if block_id > u32::MAX as u64
            || end_block > u32::MAX as u64 + 1
            || end_block > self.0.num_blocks()
        {
            return Err(DevError::InvalidParam);
        }

        self.0
            .write_blocks(block_id as u32, buf)
            .map_err(Self::map_error)
    }

    fn flush(&mut self) -> DevResult {
        Ok(())
    }
}
