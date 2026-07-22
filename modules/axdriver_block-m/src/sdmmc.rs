//! SD/MMC driver based on SDIO.

#[cfg(feature = "sdmmc-async")]
use alloc::{vec, vec::Vec};

use axdriver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
#[cfg(feature = "sdmmc-async")]
use axtask::future::block_on;
use log::{info, warn};
#[cfg(feature = "sdmmc-async")]
use simple_sdmmc::IdmacDiagnostics;
use simple_sdmmc::SdMmc;

use crate::BlockDriverOps;

#[cfg(feature = "sdmmc-async")]
const DATA_TEST_START_LBA: u32 = 2_099_200;
#[cfg(feature = "sdmmc-async")]
const IRQ_SMOKE_READS: usize = 8;
#[cfg(feature = "sdmmc-async")]
const FIXED_READ_REPEATS: usize = 64;
#[cfg(feature = "sdmmc-async")]
const SEQUENTIAL_READ_BLOCKS: u32 = 256;
#[cfg(feature = "sdmmc-async")]
const PERF_BLOCKS: usize = 256;
#[cfg(feature = "sdmmc-async")]
const PERF_ROUNDS: usize = 5;
#[cfg(feature = "sdmmc-async")]
const PERF_REQUEST_BLOCKS: [usize; 4] = [1, 2, 4, 8];
#[cfg(feature = "sdmmc-async")]
const MAX_REQUEST_BLOCKS: usize = 8;
#[cfg(feature = "sdmmc-async")]
const MAX_REQUEST_BYTES: usize = MAX_REQUEST_BLOCKS * SdMmc::BLOCK_SIZE;

#[cfg(feature = "sdmmc-async")]
#[derive(Clone, Copy, Default)]
struct PerfDiagnostics {
    completion_irqs: usize,
    error_irqs: usize,
    invalid_irqs: usize,
    timeouts: usize,
}

#[cfg(feature = "sdmmc-async")]
struct PerfSample {
    request_blocks: usize,
    requests: usize,
    elapsed_io_ns: u64,
    throughput_mib_s_milli: u64,
    request_iops_milli: u64,
    block_iops_milli: u64,
    avg_latency_ns: u64,
    avg_per_block_ns: u64,
    p50_latency_ns: u64,
    p95_latency_ns: u64,
    p99_latency_ns: u64,
    max_latency_ns: u64,
    diagnostics: PerfDiagnostics,
    validation_failures: usize,
}

#[cfg(feature = "sdmmc-async")]
#[derive(Clone, Copy)]
struct PerfSummary {
    throughput_mean_milli: u64,
    avg_latency_mean_ns: u64,
}

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
    ) -> Self {
        let driver = Self(SdMmc::new(base, irq_register), irq_num);

        #[cfg(feature = "sdmmc-async")]
        let mut driver = driver;
        #[cfg(feature = "sdmmc-async")]
        driver.run_async_dma_validation();
        #[cfg(feature = "sdmmc-async")]
        driver.run_dma_performance_benchmark();
        #[cfg(not(feature = "sdmmc-async"))]
        warn!("SDMMC DMA mode: synchronous CPU polling");

        driver
    }

    #[cfg(feature = "sdmmc-async")]
    fn read_async_checked(&mut self, lba: u32, buf: &mut [u8; SdMmc::BLOCK_SIZE]) {
        block_on(self.0.read_block_async(lba, buf));
    }

    #[cfg(feature = "sdmmc-async")]
    fn read_blocks_async_checked(&mut self, lba: u32, buf: &mut [u8]) {
        block_on(self.0.read_blocks_async(lba, buf));
    }

    #[cfg(feature = "sdmmc-async")]
    fn write_blocks_async_checked(&mut self, lba: u32, buf: &[u8]) {
        block_on(self.0.write_blocks_async(lba, buf));
    }

    #[cfg(feature = "sdmmc-async")]
    fn fill_test_pattern(buf: &mut [u8], start_lba: u32, tag: u8) {
        assert_eq!(buf.len() % SdMmc::BLOCK_SIZE, 0);
        for (block_offset, block) in buf.chunks_mut(SdMmc::BLOCK_SIZE).enumerate() {
            let lba_bytes = (start_lba + block_offset as u32).to_le_bytes();
            for (index, byte) in block.iter_mut().enumerate() {
                *byte = lba_bytes[index % lba_bytes.len()]
                    .wrapping_add((index as u8).wrapping_mul(31))
                    .wrapping_add(tag);
            }
        }
    }

    #[cfg(feature = "sdmmc-async")]
    fn count_block_mismatches(actual: &[u8], expected: &[u8]) -> usize {
        assert_eq!(actual.len(), expected.len());
        assert_eq!(actual.len() % SdMmc::BLOCK_SIZE, 0);
        actual
            .chunks(SdMmc::BLOCK_SIZE)
            .zip(expected.chunks(SdMmc::BLOCK_SIZE))
            .filter(|(actual, expected)| actual != expected)
            .count()
    }

    #[cfg(feature = "sdmmc-async")]
    fn run_async_dma_validation(&mut self) {
        let read_end = DATA_TEST_START_LBA as u64 + SEQUENTIAL_READ_BLOCKS as u64;
        assert!(
            read_end <= self.0.num_blocks(),
            "SDMMC validation range exceeds card capacity"
        );

        warn!(
            "SDMMC async DMA stage 1: completion batch smoke test at LBA {}, iterations={}",
            DATA_TEST_START_LBA, IRQ_SMOKE_READS,
        );
        let mut sync_buf = [0u8; SdMmc::BLOCK_SIZE];
        self.0.read_block(DATA_TEST_START_LBA, &mut sync_buf);

        let diagnostics_before = SdMmc::idmac_diagnostics();
        let mut async_buf = [0u8; SdMmc::BLOCK_SIZE];
        for iteration in 0..IRQ_SMOKE_READS {
            self.read_async_checked(DATA_TEST_START_LBA, &mut async_buf);
            assert!(
                sync_buf == async_buf,
                "SDMMC async DMA smoke test returned different data at iteration {}",
                iteration,
            );
        }
        let diagnostics_after = SdMmc::idmac_diagnostics();
        let completion_irqs = diagnostics_after
            .completion_irqs
            .saturating_sub(diagnostics_before.completion_irqs);
        let error_irqs = diagnostics_after
            .error_irqs
            .saturating_sub(diagnostics_before.error_irqs);
        let timeouts = diagnostics_after
            .timeouts
            .saturating_sub(diagnostics_before.timeouts);
        assert!(
            error_irqs == 0 && timeouts == 0,
            "SDMMC async DMA IRQ batch failed: completion_irqs={}, error_irqs={}, timeouts={}",
            completion_irqs,
            error_irqs,
            timeouts,
        );
        warn!(
            "SDMMC async DMA stage 1: passed; completion_irqs={}, completion before IRQ dispatch \
             is allowed",
            completion_irqs,
        );

        warn!(
            "SDMMC async DMA stage 2: repeat-read validation, iterations={}",
            FIXED_READ_REPEATS
        );
        for iteration in 0..FIXED_READ_REPEATS {
            self.read_async_checked(DATA_TEST_START_LBA, &mut async_buf);
            assert!(
                sync_buf == async_buf,
                "SDMMC repeat-read data mismatch at iteration {}",
                iteration
            );
        }

        warn!(
            "SDMMC async DMA stage 2: building single-block reference for LBA {}..={}",
            DATA_TEST_START_LBA,
            DATA_TEST_START_LBA + SEQUENTIAL_READ_BLOCKS - 1,
        );
        let mut single_block_reference =
            vec![0u8; SEQUENTIAL_READ_BLOCKS as usize * SdMmc::BLOCK_SIZE];
        for (offset, block) in single_block_reference
            .chunks_mut(SdMmc::BLOCK_SIZE)
            .enumerate()
        {
            self.0
                .read_blocks(DATA_TEST_START_LBA + offset as u32, block);
        }

        let mut sync_blocks = vec![0u8; MAX_REQUEST_BYTES];
        let mut async_blocks = vec![0u8; MAX_REQUEST_BYTES];
        for &request_blocks in &PERF_REQUEST_BLOCKS {
            let request_bytes = request_blocks * SdMmc::BLOCK_SIZE;
            warn!(
                "SDMMC async DMA stage 2: sequential-read validation request_blocks={} \
                 LBA {}..={}",
                request_blocks,
                DATA_TEST_START_LBA,
                DATA_TEST_START_LBA + SEQUENTIAL_READ_BLOCKS - 1,
            );
            for offset in (0..SEQUENTIAL_READ_BLOCKS as usize).step_by(request_blocks) {
                let lba = DATA_TEST_START_LBA + offset as u32;
                let sync_buf = &mut sync_blocks[..request_bytes];
                let async_buf = &mut async_blocks[..request_bytes];
                let byte_offset = offset * SdMmc::BLOCK_SIZE;
                let expected = &single_block_reference[byte_offset..byte_offset + request_bytes];
                self.0.read_blocks(lba, sync_buf);
                self.read_blocks_async_checked(lba, async_buf);
                assert!(
                    sync_buf == expected && async_buf == expected,
                    "SDMMC sequential read data mismatch against single-block reference at LBA {} \
                     request_blocks={}",
                    lba,
                    request_blocks,
                );
            }
        }

        let mut pattern = vec![0u8; MAX_REQUEST_BYTES];
        let mut verify = vec![0u8; MAX_REQUEST_BYTES];
        for (size_index, &request_blocks) in PERF_REQUEST_BLOCKS.iter().enumerate() {
            let request_bytes = request_blocks * SdMmc::BLOCK_SIZE;
            let original = &single_block_reference[..request_bytes];
            let pattern = &mut pattern[..request_bytes];
            let verify = &mut verify[..request_bytes];
            warn!(
                "SDMMC async DMA stage 2: write/read/restore validation request_blocks={} \
                 LBA {}..={}",
                request_blocks,
                DATA_TEST_START_LBA,
                DATA_TEST_START_LBA + request_blocks as u32 - 1,
            );

            Self::fill_test_pattern(pattern, DATA_TEST_START_LBA, 0x50 + size_index as u8);
            self.write_blocks_async_checked(DATA_TEST_START_LBA, pattern);
            self.0.read_blocks(DATA_TEST_START_LBA, verify);
            let async_write_data_ok = verify == pattern;

            Self::fill_test_pattern(pattern, DATA_TEST_START_LBA, 0xa0 + size_index as u8);
            self.0.write_blocks(DATA_TEST_START_LBA, pattern);
            self.read_blocks_async_checked(DATA_TEST_START_LBA, verify);
            let async_read_data_ok = verify == pattern;

            for block_offset in 0..request_blocks {
                let byte_offset = block_offset * SdMmc::BLOCK_SIZE;
                let block_end = byte_offset + SdMmc::BLOCK_SIZE;
                let lba = DATA_TEST_START_LBA + block_offset as u32;
                self.0
                    .write_blocks(lba, &original[byte_offset..block_end]);
                self.0
                    .read_blocks(lba, &mut verify[byte_offset..block_end]);
            }
            let restore_ok = verify == original;

            assert!(
                async_write_data_ok && async_read_data_ok && restore_ok,
                "SDMMC write validation failed at LBA {} request_blocks={}: \
                 async_write_data_ok={}, async_read_data_ok={}, restore_ok={}",
                DATA_TEST_START_LBA,
                request_blocks,
                async_write_data_ok,
                async_read_data_ok,
                restore_ok,
            );
        }

        warn!(
            "SDMMC async DMA stage 2: passed; tested sectors restored, filesystem path remains \
             synchronous"
        );
    }

    #[cfg(feature = "sdmmc-async")]
    fn diagnostics_delta(before: IdmacDiagnostics, after: IdmacDiagnostics) -> PerfDiagnostics {
        PerfDiagnostics {
            completion_irqs: after.completion_irqs.saturating_sub(before.completion_irqs),
            error_irqs: after.error_irqs.saturating_sub(before.error_irqs),
            invalid_irqs: after.invalid_irqs.saturating_sub(before.invalid_irqs),
            timeouts: after.timeouts.saturating_sub(before.timeouts),
        }
    }

    #[cfg(feature = "sdmmc-async")]
    fn finish_perf_sample(
        mut latencies_ns: Vec<u64>,
        diagnostics: PerfDiagnostics,
        validation_failures: usize,
        request_blocks: usize,
    ) -> PerfSample {
        assert!(PERF_REQUEST_BLOCKS.contains(&request_blocks));
        assert_eq!(PERF_BLOCKS % request_blocks, 0);
        let requests = PERF_BLOCKS / request_blocks;
        assert_eq!(latencies_ns.len(), requests);
        let elapsed_io_ns = latencies_ns.iter().copied().sum::<u64>().max(1);
        latencies_ns.sort_unstable();
        let bytes = (PERF_BLOCKS * SdMmc::BLOCK_SIZE) as u128;
        let throughput_mib_s_milli =
            (bytes * 1_000_000_000 * 1_000 / (elapsed_io_ns as u128 * 1_048_576)) as u64;
        let request_iops_milli =
            (requests as u128 * 1_000_000_000 * 1_000 / elapsed_io_ns as u128) as u64;
        let block_iops_milli =
            (PERF_BLOCKS as u128 * 1_000_000_000 * 1_000 / elapsed_io_ns as u128) as u64;

        PerfSample {
            request_blocks,
            requests,
            elapsed_io_ns,
            throughput_mib_s_milli,
            request_iops_milli,
            block_iops_milli,
            avg_latency_ns: elapsed_io_ns / requests as u64,
            avg_per_block_ns: elapsed_io_ns / PERF_BLOCKS as u64,
            p50_latency_ns: Self::percentile(&latencies_ns, 50),
            p95_latency_ns: Self::percentile(&latencies_ns, 95),
            p99_latency_ns: Self::percentile(&latencies_ns, 99),
            max_latency_ns: *latencies_ns.last().unwrap(),
            diagnostics,
            validation_failures,
        }
    }

    #[cfg(feature = "sdmmc-async")]
    fn percentile(sorted_values: &[u64], percentile: usize) -> u64 {
        let rank = (sorted_values.len() * percentile).div_ceil(100);
        sorted_values[rank.saturating_sub(1)]
    }

    #[cfg(feature = "sdmmc-async")]
    fn measure_sync_read(&mut self, originals: &[u8], request_blocks: usize) -> PerfSample {
        assert_eq!(originals.len(), PERF_BLOCKS * SdMmc::BLOCK_SIZE);
        let request_bytes = request_blocks * SdMmc::BLOCK_SIZE;
        let requests = PERF_BLOCKS / request_blocks;
        let before = SdMmc::idmac_diagnostics();
        let mut latencies_ns = Vec::with_capacity(requests);
        let mut validation_failures = 0;
        let mut buf = [0u8; MAX_REQUEST_BYTES];

        for request in 0..requests {
            let block_offset = request * request_blocks;
            let byte_offset = request * request_bytes;
            let actual = &mut buf[..request_bytes];
            let start = axhal::time::monotonic_time_nanos();
            self.0
                .read_blocks(DATA_TEST_START_LBA + block_offset as u32, actual);
            latencies_ns.push(
                axhal::time::monotonic_time_nanos()
                    .saturating_sub(start)
                    .max(1),
            );
            validation_failures += Self::count_block_mismatches(
                actual,
                &originals[byte_offset..byte_offset + request_bytes],
            );
        }

        let diagnostics = Self::diagnostics_delta(before, SdMmc::idmac_diagnostics());
        Self::finish_perf_sample(
            latencies_ns,
            diagnostics,
            validation_failures,
            request_blocks,
        )
    }

    #[cfg(feature = "sdmmc-async")]
    fn measure_async_read(&mut self, originals: &[u8], request_blocks: usize) -> PerfSample {
        assert_eq!(originals.len(), PERF_BLOCKS * SdMmc::BLOCK_SIZE);
        let request_bytes = request_blocks * SdMmc::BLOCK_SIZE;
        let requests = PERF_BLOCKS / request_blocks;
        let before = SdMmc::idmac_diagnostics();
        let (latencies_ns, validation_failures) = block_on(async {
            let mut latencies_ns = Vec::with_capacity(requests);
            let mut validation_failures = 0;
            let mut buf = [0u8; MAX_REQUEST_BYTES];

            for request in 0..requests {
                let block_offset = request * request_blocks;
                let byte_offset = request * request_bytes;
                let actual = &mut buf[..request_bytes];
                let start = axhal::time::monotonic_time_nanos();
                self.0
                    .read_blocks_async(DATA_TEST_START_LBA + block_offset as u32, actual)
                    .await;
                latencies_ns.push(
                    axhal::time::monotonic_time_nanos()
                        .saturating_sub(start)
                        .max(1),
                );
                validation_failures += Self::count_block_mismatches(
                    actual,
                    &originals[byte_offset..byte_offset + request_bytes],
                );
            }
            (latencies_ns, validation_failures)
        });

        let diagnostics = Self::diagnostics_delta(before, SdMmc::idmac_diagnostics());
        Self::finish_perf_sample(
            latencies_ns,
            diagnostics,
            validation_failures,
            request_blocks,
        )
    }

    #[cfg(feature = "sdmmc-async")]
    fn measure_sync_write(&mut self, tag: u8, request_blocks: usize) -> PerfSample {
        let request_bytes = request_blocks * SdMmc::BLOCK_SIZE;
        let requests = PERF_BLOCKS / request_blocks;
        let before = SdMmc::idmac_diagnostics();
        let mut latencies_ns = Vec::with_capacity(requests);
        let mut pattern = [0u8; MAX_REQUEST_BYTES];

        for request in 0..requests {
            let lba = DATA_TEST_START_LBA + (request * request_blocks) as u32;
            let pattern = &mut pattern[..request_bytes];
            Self::fill_test_pattern(pattern, lba, tag);
            let start = axhal::time::monotonic_time_nanos();
            self.0.write_blocks(lba, pattern);
            latencies_ns.push(
                axhal::time::monotonic_time_nanos()
                    .saturating_sub(start)
                    .max(1),
            );
        }

        let diagnostics = Self::diagnostics_delta(before, SdMmc::idmac_diagnostics());
        let validation_failures = self.validate_pattern(tag, request_blocks);
        Self::finish_perf_sample(
            latencies_ns,
            diagnostics,
            validation_failures,
            request_blocks,
        )
    }

    #[cfg(feature = "sdmmc-async")]
    fn measure_async_write(&mut self, tag: u8, request_blocks: usize) -> PerfSample {
        let request_bytes = request_blocks * SdMmc::BLOCK_SIZE;
        let requests = PERF_BLOCKS / request_blocks;
        let before = SdMmc::idmac_diagnostics();
        let latencies_ns = block_on(async {
            let mut latencies_ns = Vec::with_capacity(requests);
            let mut pattern = [0u8; MAX_REQUEST_BYTES];

            for request in 0..requests {
                let lba = DATA_TEST_START_LBA + (request * request_blocks) as u32;
                let pattern = &mut pattern[..request_bytes];
                Self::fill_test_pattern(pattern, lba, tag);
                let start = axhal::time::monotonic_time_nanos();
                self.0.write_blocks_async(lba, pattern).await;
                latencies_ns.push(
                    axhal::time::monotonic_time_nanos()
                        .saturating_sub(start)
                        .max(1),
                );
            }
            latencies_ns
        });

        let diagnostics = Self::diagnostics_delta(before, SdMmc::idmac_diagnostics());
        let validation_failures = self.validate_pattern(tag, request_blocks);
        Self::finish_perf_sample(
            latencies_ns,
            diagnostics,
            validation_failures,
            request_blocks,
        )
    }

    #[cfg(feature = "sdmmc-async")]
    fn validate_pattern(&mut self, tag: u8, request_blocks: usize) -> usize {
        let request_bytes = request_blocks * SdMmc::BLOCK_SIZE;
        let requests = PERF_BLOCKS / request_blocks;
        let mut expected = [0u8; MAX_REQUEST_BYTES];
        let mut actual = [0u8; MAX_REQUEST_BYTES];
        let mut failures = 0;

        for request in 0..requests {
            let lba = DATA_TEST_START_LBA + (request * request_blocks) as u32;
            let expected = &mut expected[..request_bytes];
            let actual = &mut actual[..request_bytes];
            Self::fill_test_pattern(expected, lba, tag);
            self.0.read_blocks(lba, actual);
            failures += Self::count_block_mismatches(actual, expected);
        }
        failures
    }

    #[cfg(feature = "sdmmc-async")]
    fn sample_failed(sample: &PerfSample) -> bool {
        sample.diagnostics.error_irqs != 0
            || sample.diagnostics.timeouts != 0
            || sample.validation_failures != 0
    }

    #[cfg(feature = "sdmmc-async")]
    fn print_perf_sample(mode: &str, operation: &str, round: usize, sample: &PerfSample) {
        let io_failures = sample.diagnostics.error_irqs + sample.diagnostics.timeouts;
        let error_rate_ppm = io_failures as u128 * 1_000_000 / sample.requests as u128;
        let dma_error_rate_ppm = sample.diagnostics.error_irqs as u128 * 1_000_000
            / sample.requests as u128;
        let timeout_rate_ppm =
            sample.diagnostics.timeouts as u128 * 1_000_000 / sample.requests as u128;
        let validation_failure_rate_ppm =
            sample.validation_failures as u128 * 1_000_000 / PERF_BLOCKS as u128;
        warn!(
            "SDMMC PERF mode={} op={} request_blocks={} requests={} round={} blocks={} bytes={} \
             elapsed_io_ms={}.{:03} throughput_mib_s={}.{:03} block_iops={}.{:03} \
             request_iops={}.{:03}",
            mode,
            operation,
            sample.request_blocks,
            sample.requests,
            round,
            PERF_BLOCKS,
            PERF_BLOCKS * SdMmc::BLOCK_SIZE,
            sample.elapsed_io_ns / 1_000_000,
            sample.elapsed_io_ns % 1_000_000 / 1_000,
            sample.throughput_mib_s_milli / 1_000,
            sample.throughput_mib_s_milli % 1_000,
            sample.block_iops_milli / 1_000,
            sample.block_iops_milli % 1_000,
            sample.request_iops_milli / 1_000,
            sample.request_iops_milli % 1_000,
        );
        warn!(
            "SDMMC PERF LATENCY mode={} op={} request_blocks={} round={} avg_request_us={}.{:03} \
             avg_per_block_us={}.{:03} p50_request_us={}.{:03} p95_request_us={}.{:03} \
             p99_request_us={}.{:03} max_request_us={}.{:03}",
            mode,
            operation,
            sample.request_blocks,
            round,
            sample.avg_latency_ns / 1_000,
            sample.avg_latency_ns % 1_000,
            sample.avg_per_block_ns / 1_000,
            sample.avg_per_block_ns % 1_000,
            sample.p50_latency_ns / 1_000,
            sample.p50_latency_ns % 1_000,
            sample.p95_latency_ns / 1_000,
            sample.p95_latency_ns % 1_000,
            sample.p99_latency_ns / 1_000,
            sample.p99_latency_ns % 1_000,
            sample.max_latency_ns / 1_000,
            sample.max_latency_ns % 1_000,
        );
        warn!(
            "SDMMC PERF HEALTH mode={} op={} request_blocks={} requests={} round={} \
             irq_completion={} irq_error={} irq_invalid={} timeouts={} validation_failures={} \
             dma_error_rate_ppm={} timeout_rate_ppm={} validation_failure_rate_ppm={} \
             error_rate_ppm={}",
            mode,
            operation,
            sample.request_blocks,
            sample.requests,
            round,
            sample.diagnostics.completion_irqs,
            sample.diagnostics.error_irqs,
            sample.diagnostics.invalid_irqs,
            sample.diagnostics.timeouts,
            sample.validation_failures,
            dma_error_rate_ppm,
            timeout_rate_ppm,
            validation_failure_rate_ppm,
            error_rate_ppm,
        );
    }

    #[cfg(feature = "sdmmc-async")]
    fn integer_sqrt(value: u128) -> u64 {
        if value < 2 {
            return value as u64;
        }
        let mut current = value;
        let mut next = (current + value / current) / 2;
        while next < current {
            current = next;
            next = (current + value / current) / 2;
        }
        current as u64
    }

    #[cfg(feature = "sdmmc-async")]
    fn mean_min_max_stddev(values: &[u64]) -> (u64, u64, u64, u64) {
        assert!(!values.is_empty());
        let mean =
            (values.iter().map(|&value| value as u128).sum::<u128>() / values.len() as u128) as u64;
        let variance = values
            .iter()
            .map(|&value| {
                let delta = value.abs_diff(mean) as u128;
                delta * delta
            })
            .sum::<u128>()
            / values.len() as u128;
        (
            mean,
            *values.iter().min().unwrap(),
            *values.iter().max().unwrap(),
            Self::integer_sqrt(variance),
        )
    }

    #[cfg(feature = "sdmmc-async")]
    fn print_perf_summary(
        mode: &str,
        operation: &str,
        request_blocks: usize,
        samples: &[PerfSample],
    ) -> PerfSummary {
        assert!(
            samples
                .iter()
                .all(|sample| sample.request_blocks == request_blocks)
        );
        let throughput = samples
            .iter()
            .map(|sample| sample.throughput_mib_s_milli)
            .collect::<Vec<_>>();
        let avg_latency = samples
            .iter()
            .map(|sample| sample.avg_latency_ns)
            .collect::<Vec<_>>();
        let (throughput_mean, throughput_min, throughput_max, throughput_stddev) =
            Self::mean_min_max_stddev(&throughput);
        let (latency_mean, latency_min, latency_max, latency_stddev) =
            Self::mean_min_max_stddev(&avg_latency);

        warn!(
            "SDMMC PERF STABILITY mode={} op={} request_blocks={} runs={} \
             throughput_mean_mib_s={}.{:03} throughput_min={}.{:03} throughput_max={}.{:03} \
             throughput_stddev={}.{:03}",
            mode,
            operation,
            request_blocks,
            samples.len(),
            throughput_mean / 1_000,
            throughput_mean % 1_000,
            throughput_min / 1_000,
            throughput_min % 1_000,
            throughput_max / 1_000,
            throughput_max % 1_000,
            throughput_stddev / 1_000,
            throughput_stddev % 1_000,
        );
        warn!(
            "SDMMC PERF STABILITY_LATENCY mode={} op={} request_blocks={} runs={} \
             avg_request_mean_us={}.{:03} avg_request_min={}.{:03} avg_request_max={}.{:03} \
             avg_request_stddev={}.{:03}",
            mode,
            operation,
            request_blocks,
            samples.len(),
            latency_mean / 1_000,
            latency_mean % 1_000,
            latency_min / 1_000,
            latency_min % 1_000,
            latency_max / 1_000,
            latency_max % 1_000,
            latency_stddev / 1_000,
            latency_stddev % 1_000,
        );

        PerfSummary {
            throughput_mean_milli: throughput_mean,
            avg_latency_mean_ns: latency_mean,
        }
    }

    #[cfg(feature = "sdmmc-async")]
    fn print_perf_ratio(
        operation: &str,
        request_blocks: usize,
        sync: PerfSummary,
        asynchronous: PerfSummary,
    ) {
        let throughput_ratio = asynchronous.throughput_mean_milli as u128 * 1_000
            / sync.throughput_mean_milli.max(1) as u128;
        let latency_ratio = asynchronous.avg_latency_mean_ns as u128 * 1_000
            / sync.avg_latency_mean_ns.max(1) as u128;
        warn!(
            "SDMMC PERF RATIO op={} request_blocks={} async_over_sync_throughput={}.{:03}x \
             async_over_sync_avg_latency={}.{:03}x",
            operation,
            request_blocks,
            throughput_ratio / 1_000,
            throughput_ratio % 1_000,
            latency_ratio / 1_000,
            latency_ratio % 1_000,
        );
    }

    #[cfg(feature = "sdmmc-async")]
    fn run_dma_performance_benchmark(&mut self) {
        let range_end = DATA_TEST_START_LBA as u64 + PERF_BLOCKS as u64;
        assert!(
            range_end <= self.0.num_blocks(),
            "SDMMC performance range exceeds card capacity"
        );
        warn!(
            "SDMMC PERF START lba={}..={} blocks={} rounds={} request_blocks=1,2,4,8 \
             single_request_in_flight=true workload=sd_io_only",
            DATA_TEST_START_LBA,
            DATA_TEST_START_LBA + PERF_BLOCKS as u32 - 1,
            PERF_BLOCKS,
            PERF_ROUNDS,
        );

        warn!("SDMMC PERF backing up all test sectors in memory");
        let mut originals = vec![0u8; PERF_BLOCKS * SdMmc::BLOCK_SIZE];
        for (offset, block) in originals.chunks_mut(SdMmc::BLOCK_SIZE).enumerate() {
            self.0
                .read_blocks(DATA_TEST_START_LBA + offset as u32, block);
        }

        for &request_blocks in &PERF_REQUEST_BLOCKS {
            let mut sync_read_samples = Vec::with_capacity(PERF_ROUNDS);
            let mut async_read_samples = Vec::with_capacity(PERF_ROUNDS);
            for round in 0..PERF_ROUNDS {
                if round % 2 == 0 {
                    let sample = self.measure_sync_read(&originals, request_blocks);
                    Self::print_perf_sample("sync", "read", round + 1, &sample);
                    assert!(
                        !Self::sample_failed(&sample),
                        "SDMMC sync read benchmark failed: request_blocks={} round={}",
                        request_blocks,
                        round + 1,
                    );
                    sync_read_samples.push(sample);

                    let sample = self.measure_async_read(&originals, request_blocks);
                    Self::print_perf_sample("async", "read", round + 1, &sample);
                    assert!(
                        !Self::sample_failed(&sample),
                        "SDMMC async read benchmark failed: request_blocks={} round={}",
                        request_blocks,
                        round + 1,
                    );
                    async_read_samples.push(sample);
                } else {
                    let sample = self.measure_async_read(&originals, request_blocks);
                    Self::print_perf_sample("async", "read", round + 1, &sample);
                    assert!(
                        !Self::sample_failed(&sample),
                        "SDMMC async read benchmark failed: request_blocks={} round={}",
                        request_blocks,
                        round + 1,
                    );
                    async_read_samples.push(sample);

                    let sample = self.measure_sync_read(&originals, request_blocks);
                    Self::print_perf_sample("sync", "read", round + 1, &sample);
                    assert!(
                        !Self::sample_failed(&sample),
                        "SDMMC sync read benchmark failed: request_blocks={} round={}",
                        request_blocks,
                        round + 1,
                    );
                    sync_read_samples.push(sample);
                }
            }

            let sync_read =
                Self::print_perf_summary("sync", "read", request_blocks, &sync_read_samples);
            let async_read =
                Self::print_perf_summary("async", "read", request_blocks, &async_read_samples);
            Self::print_perf_ratio("read", request_blocks, sync_read, async_read);
        }

        let mut write_failed = false;
        'write_sizes: for (size_index, &request_blocks) in PERF_REQUEST_BLOCKS.iter().enumerate() {
            let mut sync_write_samples = Vec::with_capacity(PERF_ROUNDS);
            let mut async_write_samples = Vec::with_capacity(PERF_ROUNDS);
            for round in 0..PERF_ROUNDS {
                let sync_tag = 0x20u8
                    .wrapping_add((size_index as u8).wrapping_mul(0x10))
                    .wrapping_add(round as u8);
                let async_tag = 0xa0u8
                    .wrapping_add((size_index as u8).wrapping_mul(0x10))
                    .wrapping_add(round as u8);
                if round % 2 == 0 {
                    let sample = self.measure_sync_write(sync_tag, request_blocks);
                    Self::print_perf_sample("sync", "write", round + 1, &sample);
                    write_failed = Self::sample_failed(&sample);
                    sync_write_samples.push(sample);
                    if !write_failed {
                        let sample = self.measure_async_write(async_tag, request_blocks);
                        Self::print_perf_sample("async", "write", round + 1, &sample);
                        write_failed = Self::sample_failed(&sample);
                        async_write_samples.push(sample);
                    }
                } else {
                    let sample = self.measure_async_write(async_tag, request_blocks);
                    Self::print_perf_sample("async", "write", round + 1, &sample);
                    write_failed = Self::sample_failed(&sample);
                    async_write_samples.push(sample);
                    if !write_failed {
                        let sample = self.measure_sync_write(sync_tag, request_blocks);
                        Self::print_perf_sample("sync", "write", round + 1, &sample);
                        write_failed = Self::sample_failed(&sample);
                        sync_write_samples.push(sample);
                    }
                }
                if write_failed {
                    break 'write_sizes;
                }
            }

            let sync_write =
                Self::print_perf_summary("sync", "write", request_blocks, &sync_write_samples);
            let async_write =
                Self::print_perf_summary("async", "write", request_blocks, &async_write_samples);
            Self::print_perf_ratio("write", request_blocks, sync_write, async_write);
        }

        warn!("SDMMC PERF restoring all test sectors");
        for (offset, block) in originals.chunks(SdMmc::BLOCK_SIZE).enumerate() {
            self.0
                .write_blocks(DATA_TEST_START_LBA + offset as u32, block);
        }
        let mut restored = vec![0u8; PERF_BLOCKS * SdMmc::BLOCK_SIZE];
        for (offset, block) in restored.chunks_mut(SdMmc::BLOCK_SIZE).enumerate() {
            self.0
                .read_blocks(DATA_TEST_START_LBA + offset as u32, block);
        }
        let restore_failures = Self::count_block_mismatches(&restored, &originals);
        warn!(
            "SDMMC PERF RESTORE blocks={} validation_failures={}",
            PERF_BLOCKS, restore_failures
        );
        assert_eq!(
            restore_failures, 0,
            "SDMMC performance test sector restoration failed"
        );
        assert!(
            !write_failed,
            "SDMMC write benchmark failed; test sectors were restored"
        );

        warn!(
            "SDMMC PERF PASSED request_blocks=1,2,4,8 all timed data validated and test sectors \
             restored"
        );
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
        if buf.len() % SdMmc::BLOCK_SIZE != 0 {
            return Err(DevError::InvalidParam);
        }
        let block_count = (buf.len() / SdMmc::BLOCK_SIZE) as u64;
        let Some(end_block) = block_id.checked_add(block_count) else {
            return Err(DevError::InvalidParam);
        };
        if end_block > self.0.num_blocks()
            || block_id > u32::MAX as u64
            || end_block > u32::MAX as u64 + 1
        {
            return Err(DevError::InvalidParam);
        }

        self.0.read_blocks(block_id as u32, buf);
        Ok(())
    }

    fn write_block(&mut self, block_id: u64, buf: &[u8]) -> DevResult {
        if buf.len() % SdMmc::BLOCK_SIZE != 0 {
            return Err(DevError::InvalidParam);
        }
        let block_count = (buf.len() / SdMmc::BLOCK_SIZE) as u64;
        let Some(end_block) = block_id.checked_add(block_count) else {
            return Err(DevError::InvalidParam);
        };
        if end_block > self.0.num_blocks()
            || block_id > u32::MAX as u64
            || end_block > u32::MAX as u64 + 1
        {
            return Err(DevError::InvalidParam);
        }

        self.0.write_blocks(block_id as u32, buf);
        Ok(())
    }

    fn flush(&mut self) -> DevResult {
        Ok(())
    }
}
