use alloc::vec::Vec;
use core::sync::atomic::Ordering;

use log::warn;

use super::{
    ASYNC_WRITE_BUSY_SEQUENCE, IDMAC_COMPLETION, MAX_DMA_BLOCKS, SdMmc, SdMmcError, SdMmcResult,
};

const TEST_START_LBA: u32 = 2_099_200;
const TEST_REGION_BLOCKS: usize = 64;
const TEST_REQUEST_BLOCKS: [usize; 6] = [1, 2, 8, 32, 33, 64];

impl SdMmc {
    fn allocate_test_buffer(bytes: usize) -> SdMmcResult<Vec<u8>> {
        let mut buffer = Vec::new();
        buffer
            .try_reserve_exact(bytes)
            .map_err(|_| SdMmcError::DmaAllocation)?;
        buffer.resize(bytes, 0);
        Ok(buffer)
    }

    fn fill_async_write_test_pattern(buffer: &mut [u8], start_lba: u32, tag: u8) {
        debug_assert!(buffer.len().is_multiple_of(Self::BLOCK_SIZE));
        for (block_offset, block) in buffer.chunks_mut(Self::BLOCK_SIZE).enumerate() {
            let lba = start_lba + block_offset as u32;
            let lba_bytes = lba.to_le_bytes();
            for (byte_offset, byte) in block.iter_mut().enumerate() {
                *byte = lba_bytes[byte_offset % lba_bytes.len()]
                    .wrapping_add((byte_offset as u8).wrapping_mul(29))
                    .wrapping_add(tag);
            }
        }
    }

    fn async_write_test_command_plan(request_blocks: usize) -> &'static str {
        match request_blocks {
            1 => "CMD24",
            2..=32 => "CMD25",
            33 => "CMD25(32)+CMD24(1)",
            64 => "CMD25(32)+CMD25(32)",
            _ => "chunked",
        }
    }

    fn restore_async_write_test_region(&mut self, backup: &[u8], verify: &mut [u8]) -> SdMmcResult {
        if self.idmac_faulted {
            return Err(SdMmcError::DriverFaulted);
        }

        self.write_blocks(TEST_START_LBA, backup)?;
        verify.fill(0);
        self.read_blocks(TEST_START_LBA, verify)?;
        if verify != backup {
            return Err(SdMmcError::TerminalValidation);
        }
        Ok(())
    }

    pub(super) fn run_async_write_busy_test(&mut self) -> SdMmcResult {
        let region_bytes = TEST_REGION_BLOCKS * Self::BLOCK_SIZE;
        let test_end_lba = u64::from(TEST_START_LBA) + TEST_REGION_BLOCKS as u64;
        if test_end_lba > self.num_blocks {
            warn!(
                "SDMMC_ASYNC_WRITE_BUSY_TEST refused: region_end_lba={} num_blocks={}",
                test_end_lba, self.num_blocks,
            );
            return Err(SdMmcError::OutOfRange);
        }

        warn!(
            "SDMMC_ASYNC_WRITE_BUSY_TEST begin destructive_raw_lba=true start_lba={} blocks={} \
             power_loss_can_corrupt_region=true requires_spare_card_or_reserved_region=true",
            TEST_START_LBA, TEST_REGION_BLOCKS,
        );

        let mut backup = Self::allocate_test_buffer(region_bytes)?;
        let mut backup_confirmation = Self::allocate_test_buffer(region_bytes)?;
        let mut work = Self::allocate_test_buffer(region_bytes)?;

        self.read_blocks(TEST_START_LBA, &mut backup)?;
        self.read_blocks(TEST_START_LBA, &mut backup_confirmation)?;
        if backup_confirmation != backup {
            warn!(
                "SDMMC_ASYNC_WRITE_BUSY_TEST refused: backup read was unstable; no sectors written"
            );
            return Err(SdMmcError::TerminalValidation);
        }
        warn!(
            "SDMMC_ASYNC_WRITE_BUSY_TEST backup_verified start_lba={} blocks={}",
            TEST_START_LBA, TEST_REGION_BLOCKS,
        );

        for (case_index, request_blocks) in TEST_REQUEST_BLOCKS.into_iter().enumerate() {
            let request_bytes = request_blocks * Self::BLOCK_SIZE;
            let pattern = &mut work[..request_bytes];
            Self::fill_async_write_test_pattern(
                pattern,
                TEST_START_LBA,
                0x60u8.wrapping_add(case_index as u8),
            );

            warn!(
                "SDMMC_ASYNC_WRITE_BUSY_TEST case_begin request_blocks={} chunks={} \
                 command_plan={} lba={}..={}",
                request_blocks,
                request_blocks.div_ceil(MAX_DMA_BLOCKS),
                Self::async_write_test_command_plan(request_blocks),
                TEST_START_LBA,
                TEST_START_LBA + request_blocks as u32 - 1,
            );

            let expected_chunks = request_blocks.div_ceil(MAX_DMA_BLOCKS);
            let started_nanos = axhal::time::monotonic_time_nanos();
            let generation_before = IDMAC_COMPLETION.generation.load(Ordering::Acquire);
            let irq_transfer_count_before =
                IDMAC_COMPLETION.irq_transfer_count.load(Ordering::Acquire);
            let busy_sequence_before = ASYNC_WRITE_BUSY_SEQUENCE.load(Ordering::Acquire);
            let write_result =
                axtask::future::block_on(self.write_blocks_async(TEST_START_LBA, pattern));
            let elapsed_us =
                axhal::time::monotonic_time_nanos().saturating_sub(started_nanos) / 1_000;
            let generation_after = IDMAC_COMPLETION.generation.load(Ordering::Acquire);
            let irq_generation = IDMAC_COMPLETION.snapshot_generation.load(Ordering::Acquire);
            let irq_transfer_count_after =
                IDMAC_COMPLETION.irq_transfer_count.load(Ordering::Acquire);
            let busy_sequence_after = ASYNC_WRITE_BUSY_SEQUENCE.load(Ordering::Acquire);
            let expected_generation = generation_before.wrapping_add(expected_chunks);
            let expected_irq_transfer_count =
                irq_transfer_count_before.wrapping_add(expected_chunks);
            let expected_busy_sequence = busy_sequence_before.wrapping_add(expected_chunks);
            let dma_irq_seen = generation_after == expected_generation
                && irq_generation == expected_generation
                && irq_transfer_count_after == expected_irq_transfer_count;
            let cmd13_per_chunk = busy_sequence_after == expected_busy_sequence;

            let mut operation_error = write_result.err();
            if operation_error.is_none() && (!dma_irq_seen || !cmd13_per_chunk) {
                operation_error = Some(SdMmcError::TerminalValidation);
            }
            let mut data_ok = false;
            if operation_error.is_none() {
                let actual = &mut backup_confirmation[..request_bytes];
                actual.fill(0);
                match self.read_blocks(TEST_START_LBA, actual) {
                    Ok(()) => {
                        data_ok = actual == pattern;
                        if !data_ok {
                            operation_error = Some(SdMmcError::TerminalValidation);
                        }
                    }
                    Err(error) => operation_error = Some(error),
                }
            }

            let restore_result =
                self.restore_async_write_test_region(&backup, &mut backup_confirmation);
            let restore_ok = restore_result.is_ok();
            warn!(
                "SDMMC_ASYNC_WRITE_BUSY_TEST case_end request_blocks={} expected_chunks={} \
                 elapsed_us={} write_result={:?} dma_irq_seen={} cmd13_per_chunk={} \
                 generation_before={} generation_after={} irq_generation={} \
                 irq_transfer_count_before={} irq_transfer_count_after={} busy_sequence_before={} \
                 busy_sequence_after={} data_ok={} restore_result={:?} driver_faulted={}",
                request_blocks,
                expected_chunks,
                elapsed_us,
                write_result,
                dma_irq_seen,
                cmd13_per_chunk,
                generation_before,
                generation_after,
                irq_generation,
                irq_transfer_count_before,
                irq_transfer_count_after,
                busy_sequence_before,
                busy_sequence_after,
                data_ok,
                restore_result,
                self.idmac_faulted,
            );

            if let Err(error) = restore_result {
                warn!(
                    "SDMMC_ASYNC_WRITE_BUSY_TEST FAILED request_blocks={} restoration_failed=true \
                     test_region_may_be_modified=true error={:?}",
                    request_blocks, error,
                );
                return Err(error);
            }
            if let Some(error) = operation_error {
                warn!(
                    "SDMMC_ASYNC_WRITE_BUSY_TEST FAILED request_blocks={} data_ok={} \
                     region_restored={} error={:?}",
                    request_blocks, data_ok, restore_ok, error,
                );
                return Err(error);
            }
        }

        warn!(
            "SDMMC_ASYNC_WRITE_BUSY_TEST PASS cases={} region_restored=true start_lba={} blocks={}",
            TEST_REQUEST_BLOCKS.len(),
            TEST_START_LBA,
            TEST_REGION_BLOCKS,
        );
        Ok(())
    }
}
