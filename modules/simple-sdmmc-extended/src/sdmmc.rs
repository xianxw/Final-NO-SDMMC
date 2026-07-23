#[cfg(feature = "multi-block-test")]
use alloc::{vec, vec::Vec};
use core::{
    alloc::Layout,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering},
    time::Duration,
};

use axtask::WaitQueue;
use log::{debug, info, trace, warn};
use volatile::VolatilePtr;

use crate::{
    cmd::{Command, DataXfer},
    dma::{DMABuffer, DMAInfo, IdmacDescriptor, alloc_coherent, dealloc_coherent},
    regs::{ClkDiv, ClkEna, RegisterBlock, RegisterBlockVolatileFieldAccess},
    utils::{Cid, CsdV2},
};

// VisionFive 2 firmware configures SDIO1 CIU as PLL2 / 3 / 8 = 49.5 MHz.
// For CLKDIV=n, DW-MMC outputs CIU / (2*n) to the card.
const VISIONFIVE2_SDIO_CIU_CLOCK_HZ: u32 = 49_500_000;
const IDENTIFICATION_CLOCK_DIVIDER: u8 = 100;
const DEFAULT_SPEED_CLOCK_DIVIDER: u8 = 1;
const DMA_BUFFER_SIZE: usize = 32 * 512;
const IDMAC_DESCRIPTOR_BUFFER_SIZE: usize = 8 * 512;

#[cfg(feature = "multi-block-test")]
const MULTI_BLOCK_TEST_START_LBA: u32 = 2_099_200;
#[cfg(feature = "multi-block-test")]
const MULTI_BLOCK_TEST_BLOCKS: usize = 256;
#[cfg(feature = "multi-block-test")]
const MULTI_BLOCK_TEST_ROUNDS: usize = 5;
#[cfg(feature = "multi-block-test")]
const MULTI_BLOCK_REQUEST_SIZES: [usize; 6] = [1, 2, 4, 8, 16, 32];

#[cfg(feature = "multi-block-test")]
#[derive(Clone, Copy)]
enum MultiBlockOperation {
    Read,
    Write,
}

#[cfg(feature = "multi-block-test")]
impl MultiBlockOperation {
    const fn name(self) -> &'static str {
        match self {
            Self::Read => "read",
            Self::Write => "write",
        }
    }
}

#[cfg(feature = "multi-block-test")]
#[derive(Clone, Copy)]
#[repr(usize)]
enum MultiBlockPhase {
    Total,
    TransactionConfig,
    BounceCopy,
    IdleWait,
    StatusClear,
    DescriptorAlloc,
    DescriptorPublish,
    CommandIssue,
    CommandAccept,
    PostStartChecks,
    CommandResponse,
    DataTransfer,
    AutoStopTail,
    TerminalValidate,
    FinishCleanup,
    CardBusy,
    Unaccounted,
}

#[cfg(feature = "multi-block-test")]
impl MultiBlockPhase {
    const COUNT: usize = 17;
    const ALL: [Self; Self::COUNT] = [
        Self::Total,
        Self::TransactionConfig,
        Self::BounceCopy,
        Self::IdleWait,
        Self::StatusClear,
        Self::DescriptorAlloc,
        Self::DescriptorPublish,
        Self::CommandIssue,
        Self::CommandAccept,
        Self::PostStartChecks,
        Self::CommandResponse,
        Self::DataTransfer,
        Self::AutoStopTail,
        Self::TerminalValidate,
        Self::FinishCleanup,
        Self::CardBusy,
        Self::Unaccounted,
    ];

    const fn name(self) -> &'static str {
        match self {
            Self::Total => "total",
            Self::TransactionConfig => "transaction_config",
            Self::BounceCopy => "bounce_copy",
            Self::IdleWait => "previous_idle_wait",
            Self::StatusClear => "status_clear",
            Self::DescriptorAlloc => "descriptor_alloc",
            Self::DescriptorPublish => "descriptor_publish",
            Self::CommandIssue => "command_issue",
            Self::CommandAccept => "command_accept",
            Self::PostStartChecks => "post_start_checks",
            Self::CommandResponse => "command_response",
            Self::DataTransfer => "data_transfer",
            Self::AutoStopTail => "auto_stop_tail",
            Self::TerminalValidate => "terminal_validate",
            Self::FinishCleanup => "finish_cleanup",
            Self::CardBusy => "card_busy",
            Self::Unaccounted => "unaccounted",
        }
    }
}

#[cfg(feature = "multi-block-test")]
#[derive(Clone, Copy)]
struct MultiBlockPhaseStats {
    count: u64,
    total_ns: u64,
    min_ns: u64,
    max_ns: u64,
}

#[cfg(feature = "multi-block-test")]
impl MultiBlockPhaseStats {
    const fn new() -> Self {
        Self {
            count: 0,
            total_ns: 0,
            min_ns: u64::MAX,
            max_ns: 0,
        }
    }

    fn record(&mut self, duration_ns: u64) {
        self.count = self.count.saturating_add(1);
        self.total_ns = self.total_ns.saturating_add(duration_ns);
        self.min_ns = self.min_ns.min(duration_ns);
        self.max_ns = self.max_ns.max(duration_ns);
    }

    fn average_ns(self) -> u64 {
        if self.count == 0 {
            0
        } else {
            self.total_ns / self.count
        }
    }
}

#[cfg(feature = "multi-block-test")]
#[derive(Clone, Copy)]
struct MultiBlockProfileStats {
    phases: [MultiBlockPhaseStats; MultiBlockPhase::COUNT],
}

#[cfg(feature = "multi-block-test")]
impl MultiBlockProfileStats {
    const fn new() -> Self {
        Self {
            phases: [MultiBlockPhaseStats::new(); MultiBlockPhase::COUNT],
        }
    }

    fn record(&mut self, sample: &[u64; MultiBlockPhase::COUNT]) {
        for phase in MultiBlockPhase::ALL {
            self.phases[phase as usize].record(sample[phase as usize]);
        }
    }
}

#[cfg(feature = "multi-block-test")]
struct MultiBlockProfiler {
    enabled: bool,
    operation: MultiBlockOperation,
    request_start_ns: u64,
    current: [u64; MultiBlockPhase::COUNT],
    read: MultiBlockProfileStats,
    write: MultiBlockProfileStats,
}

#[cfg(feature = "multi-block-test")]
impl MultiBlockProfiler {
    const fn new() -> Self {
        Self {
            enabled: false,
            operation: MultiBlockOperation::Read,
            request_start_ns: 0,
            current: [0; MultiBlockPhase::COUNT],
            read: MultiBlockProfileStats::new(),
            write: MultiBlockProfileStats::new(),
        }
    }

    fn begin(&mut self, operation: MultiBlockOperation) {
        if !self.enabled {
            return;
        }
        self.operation = operation;
        self.current.fill(0);
        self.request_start_ns = axhal::time::monotonic_time_nanos();
    }

    fn add(&mut self, phase: MultiBlockPhase, duration_ns: u64) {
        if self.enabled {
            self.current[phase as usize] = self.current[phase as usize].saturating_add(duration_ns);
        }
    }

    fn finish(&mut self) {
        if !self.enabled {
            return;
        }
        let total_ns = axhal::time::monotonic_time_nanos()
            .saturating_sub(self.request_start_ns)
            .max(1);
        self.current[MultiBlockPhase::Total as usize] = total_ns;
        let accounted_ns = MultiBlockPhase::ALL
            .iter()
            .copied()
            .filter(|phase| !matches!(phase, MultiBlockPhase::Total | MultiBlockPhase::Unaccounted))
            .map(|phase| self.current[phase as usize] as u128)
            .sum::<u128>();
        self.current[MultiBlockPhase::Unaccounted as usize] =
            total_ns.saturating_sub(accounted_ns.min(u64::MAX as u128) as u64);

        match self.operation {
            MultiBlockOperation::Read => self.read.record(&self.current),
            MultiBlockOperation::Write => self.write.record(&self.current),
        }
    }

    fn take(&mut self, operation: MultiBlockOperation) -> MultiBlockProfileStats {
        match operation {
            MultiBlockOperation::Read => {
                let stats = self.read;
                self.read = MultiBlockProfileStats::new();
                stats
            }
            MultiBlockOperation::Write => {
                let stats = self.write;
                self.write = MultiBlockProfileStats::new();
                stats
            }
        }
    }
}

fn wait_until<F>(mut f: F)
where
    F: FnMut() -> bool,
{
    while !f() {
        core::hint::spin_loop();
    }
}

static IDMAC_WAIT_QUEUE: WaitQueue = WaitQueue::new();
static IDMAC_DONE_FLAG: AtomicBool = AtomicBool::new(false);
static IDMAC_ERROR_FLAG: AtomicBool = AtomicBool::new(false);
static IDMAC_START_LOGGED: AtomicBool = AtomicBool::new(false);
static SDMMC_REGS_BASE: AtomicUsize = AtomicUsize::new(0);
static IDMAC_COMPLETION: IdmacCompletion = IdmacCompletion::new();

struct IdmacCompletion {
    generation: AtomicUsize,
    snapshot_generation: AtomicUsize,
    rintsts_bits: AtomicU32,
    idsts_bits: AtomicU32,
}

impl IdmacCompletion {
    const fn new() -> Self {
        Self {
            generation: AtomicUsize::new(0),
            snapshot_generation: AtomicUsize::new(0),
            rintsts_bits: AtomicU32::new(0),
            idsts_bits: AtomicU32::new(0),
        }
    }

    fn begin_transfer(&self) -> usize {
        self.snapshot_generation.store(0, Ordering::Relaxed);
        self.rintsts_bits.store(0, Ordering::Relaxed);
        self.idsts_bits.store(0, Ordering::Relaxed);
        self.generation
            .fetch_add(1, Ordering::AcqRel)
            .wrapping_add(1)
    }

    fn record_irq(&self, rintsts: crate::regs::RIntSts, idsts: crate::regs::IdSts) {
        let generation = self.generation.load(Ordering::Acquire);
        if generation == 0 {
            return;
        }

        self.rintsts_bits
            .fetch_or(rintsts.into_bits(), Ordering::Relaxed);
        self.idsts_bits
            .fetch_or(idsts.into_bits(), Ordering::Relaxed);
        self.snapshot_generation
            .store(generation, Ordering::Release);
    }

    fn snapshot_bits(&self, generation: usize) -> Option<(u32, u32)> {
        if self.snapshot_generation.load(Ordering::Acquire) != generation {
            return None;
        }

        let rintsts_bits = self.rintsts_bits.load(Ordering::Relaxed);
        let idsts_bits = self.idsts_bits.load(Ordering::Relaxed);
        if self.snapshot_generation.load(Ordering::Acquire) == generation {
            Some((rintsts_bits, idsts_bits))
        } else {
            None
        }
    }
}

#[inline(always)]
fn dma_io_fence() {
    #[cfg(target_arch = "riscv64")]
    unsafe {
        core::arch::asm!("fence iorw, iorw", options(nostack, preserves_flags));
    }

    #[cfg(not(target_arch = "riscv64"))]
    core::sync::atomic::fence(Ordering::SeqCst);
}

struct IdmacTransferContext {
    cmd: crate::regs::Cmd,
    arg: u32,
    generation: usize,
    dma_desc_info: DMAInfo,
    layout: Layout,
    desc_ptr: *mut IdmacDescriptor,
    descriptor_count: usize,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum IdmacWaitError {
    CommandTimeout,
    DataTimeout,
    Hardware,
}

/// Data width for SD/MMC data transfer, used to configure the CTYPE register of the controller.
/// Will decide alignment requirements for DMA buffer and data in FIFO.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AHBDataWidth {
    Bits16,
    Bits32,
    Bits64,
}

impl AHBDataWidth {
    // Returns the alignment requirement in bytes for the given data width.
    pub fn align_value(&self) -> usize {
        match self {
            AHBDataWidth::Bits16 => 2,
            AHBDataWidth::Bits32 => 4,
            AHBDataWidth::Bits64 => 8,
        }
    }
}

/// SD/MMC driver.
pub struct SdMmc {
    /// Register block for the SD/MMC controller, accessed through volatile reads/writes.
    regs: VolatilePtr<'static, RegisterBlock>,

    /// Number of blocks on the SD/MMC card, determined during initialization from the CSD register.
    num_blocks: u64,

    /// Indicates whether the Internal DMA (IDMAC) is enabled for data transfer.
    ahb_data_width: AHBDataWidth,

    /// Coherent buffer used for DMA transfers.
    dma_buffer: Option<DMABuffer>,

    /// Once set, no further benchmark transfer may be submitted.
    idmac_faulted: bool,

    /// The DMA buffer must be retained if hardware could not be stopped.
    idmac_reset_failed: bool,

    #[cfg(feature = "multi-block-test")]
    multi_block_profiler: MultiBlockProfiler,
}

struct ActiveIdmacTransfer<'a> {
    sdmmc: &'a mut SdMmc,
    context: Option<IdmacTransferContext>,
}

impl<'a> ActiveIdmacTransfer<'a> {
    fn new(sdmmc: &'a mut SdMmc, context: IdmacTransferContext) -> Self {
        Self {
            sdmmc,
            context: Some(context),
        }
    }

    fn context(&self) -> &IdmacTransferContext {
        self.context.as_ref().unwrap()
    }

    fn wait_sync(&mut self) -> Result<(), IdmacWaitError> {
        let context = self.context.as_ref().unwrap();
        self.sdmmc.wait_transfer_sync(context)
    }

    async fn wait_async(&self) -> Result<(), IdmacWaitError> {
        self.sdmmc.wait_transfer_async(self.context()).await
    }

    fn validate(&mut self) -> bool {
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.sdmmc.multi_block_phase_start();
        let valid = self.sdmmc.validate_idmac_terminal(self.context());
        #[cfg(feature = "multi-block-test")]
        self.sdmmc
            .multi_block_phase_finish(MultiBlockPhase::TerminalValidate, phase_start);
        valid
    }

    fn response(&self) -> [u32; 4] {
        self.sdmmc.regs.resp().read()
    }

    fn fault(&mut self) {
        self.sdmmc.idmac_faulted = true;
    }

    fn finish(mut self, recover: bool) -> bool {
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.sdmmc.multi_block_phase_start();
        let context = self.context.take().unwrap();
        let finished = self.sdmmc.finish_idmac_transfer(context, recover);
        #[cfg(feature = "multi-block-test")]
        self.sdmmc
            .multi_block_phase_finish(MultiBlockPhase::FinishCleanup, phase_start);
        finished
    }
}

impl Drop for ActiveIdmacTransfer<'_> {
    fn drop(&mut self) {
        let Some(context) = self.context.take() else {
            return;
        };

        warn!("active IDMAC future dropped; aborting the transfer and faulting the driver");
        self.sdmmc.idmac_faulted = true;
        let _ = self.sdmmc.finish_idmac_transfer(context, true);
    }
}

impl SdMmc {
    /// The offset of the FIFO register from the base address of the SD/MMC controller's register block.
    const FIFO: usize = 0x200;

    /// Creates a new `SdMmc` instance from the given base address.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `base` is a valid pointer to the SD/MMC controller's
    /// register block and that no other code is concurrently accessing the same hardware.
    pub unsafe fn new(base: usize, register_irq: impl FnOnce() -> bool) -> Self {
        let regs = unsafe { VolatilePtr::new(NonNull::new_unchecked(base as *mut _)) };
        SDMMC_REGS_BASE.store(base, Ordering::Release);

        let mut this = Self {
            regs,
            num_blocks: 0,
            ahb_data_width: AHBDataWidth::Bits32,
            dma_buffer: None,
            idmac_faulted: false,
            idmac_reset_failed: false,
            #[cfg(feature = "multi-block-test")]
            multi_block_profiler: MultiBlockProfiler::new(),
        };
        this.init();
        this.try_enable_idmac(DMA_BUFFER_SIZE, AHBDataWidth::Bits32, register_irq);
        #[cfg(feature = "multi-block-test")]
        this.run_multi_block_test();

        this
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_phase_start(&self) -> Option<u64> {
        self.multi_block_profiler
            .enabled
            .then(axhal::time::monotonic_time_nanos)
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_phase_finish(&mut self, phase: MultiBlockPhase, started_ns: Option<u64>) {
        if let Some(started_ns) = started_ns {
            self.multi_block_profiler.add(
                phase,
                axhal::time::monotonic_time_nanos().saturating_sub(started_ns),
            );
        }
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_test_read_single(&mut self, buffer: &mut [u8]) {
        assert_eq!(buffer.len(), MULTI_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE);
        for (block_index, block) in buffer.chunks_mut(Self::BLOCK_SIZE).enumerate() {
            self.read_block(
                MULTI_BLOCK_TEST_START_LBA + block_index as u32,
                block.try_into().unwrap(),
            );
        }
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_test_write_single(&mut self, buffer: &[u8]) {
        assert_eq!(buffer.len(), MULTI_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE);
        for (block_index, block) in buffer.chunks(Self::BLOCK_SIZE).enumerate() {
            self.write_block(
                MULTI_BLOCK_TEST_START_LBA + block_index as u32,
                block.try_into().unwrap(),
            );
        }
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_fill_pattern(buffer: &mut [u8], tag: u8) {
        for (block_index, block) in buffer.chunks_mut(Self::BLOCK_SIZE).enumerate() {
            let lba = MULTI_BLOCK_TEST_START_LBA + block_index as u32;
            let lba_bytes = lba.to_le_bytes();
            for (byte_index, byte) in block.iter_mut().enumerate() {
                *byte = lba_bytes[byte_index % lba_bytes.len()]
                    .wrapping_add((byte_index as u8).wrapping_mul(31))
                    .wrapping_add(tag);
            }
        }
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_first_mismatch(actual: &[u8], expected: &[u8]) -> Option<(usize, u8, u8)> {
        assert_eq!(actual.len(), expected.len());
        actual
            .iter()
            .copied()
            .zip(expected.iter().copied())
            .enumerate()
            .find_map(|(index, (actual, expected))| {
                (actual != expected).then_some((index, expected, actual))
            })
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_percentile(sorted: &[u64], percentile: usize) -> u64 {
        let rank = (sorted.len() * percentile).div_ceil(100);
        sorted[rank.saturating_sub(1)]
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_print_round(
        operation: MultiBlockOperation,
        request_blocks: usize,
        round: usize,
        mut latencies_ns: Vec<u64>,
        validation_ok: bool,
    ) {
        let requests = MULTI_BLOCK_TEST_BLOCKS / request_blocks;
        assert_eq!(latencies_ns.len(), requests);
        let elapsed_ns = latencies_ns.iter().copied().sum::<u64>().max(1);
        let bytes = (MULTI_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE) as u128;
        let throughput_milli =
            (bytes * 1_000_000_000 * 1_000 / (elapsed_ns as u128 * 1_048_576)) as u64;
        let average_ns = elapsed_ns / requests as u64;
        latencies_ns.sort_unstable();
        let p50_ns = Self::multi_block_percentile(&latencies_ns, 50);
        let p95_ns = Self::multi_block_percentile(&latencies_ns, 95);
        let p99_ns = Self::multi_block_percentile(&latencies_ns, 99);
        let max_ns = *latencies_ns.last().unwrap();
        let command = if request_blocks == 1 {
            match operation {
                MultiBlockOperation::Read => "CMD17",
                MultiBlockOperation::Write => "CMD24",
            }
        } else {
            match operation {
                MultiBlockOperation::Read => "CMD18+AutoCMD12",
                MultiBlockOperation::Write => "CMD25+AutoCMD12",
            }
        };
        let descriptor_count =
            (request_blocks * Self::BLOCK_SIZE).div_ceil(IDMAC_DESCRIPTOR_BUFFER_SIZE);

        warn!(
            "MULTI_BLOCK_ROUND operation={} request_blocks={} round={} command={} descriptors={} \
             requests={} bytes={} elapsed_ns={} throughput_mib_s={}.{:03} average_request_ns={} \
             average_block_ns={} p50_ns={} p95_ns={} p99_ns={} max_ns={} validation={}",
            operation.name(),
            request_blocks,
            round,
            command,
            descriptor_count,
            requests,
            bytes,
            elapsed_ns,
            throughput_milli / 1_000,
            throughput_milli % 1_000,
            average_ns,
            elapsed_ns / MULTI_BLOCK_TEST_BLOCKS as u64,
            p50_ns,
            p95_ns,
            p99_ns,
            max_ns,
            if validation_ok { "ok" } else { "failed" },
        );
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_print_profile(
        operation: MultiBlockOperation,
        request_blocks: usize,
        stats: MultiBlockProfileStats,
    ) {
        let total_ns = stats.phases[MultiBlockPhase::Total as usize]
            .total_ns
            .max(1);
        for phase in MultiBlockPhase::ALL {
            let phase_stats = stats.phases[phase as usize];
            if phase_stats.count == 0 {
                continue;
            }
            let percent_milli = (phase_stats.total_ns as u128 * 100_000 / total_ns as u128) as u64;
            warn!(
                "MULTI_BLOCK_PHASE operation={} request_blocks={} phase={} count={} total_ns={} \
                 average_ns={} min_ns={} max_ns={} percent={}.{:03}",
                operation.name(),
                request_blocks,
                phase.name(),
                phase_stats.count,
                phase_stats.total_ns,
                phase_stats.average_ns(),
                if phase_stats.min_ns == u64::MAX {
                    0
                } else {
                    phase_stats.min_ns
                },
                phase_stats.max_ns,
                percent_milli / 1_000,
                percent_milli % 1_000,
            );
        }
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_measure_read(
        &mut self,
        request_blocks: usize,
        expected: &[u8],
        transfer: &mut [u8],
    ) -> bool {
        let request_bytes = request_blocks * Self::BLOCK_SIZE;
        let requests = MULTI_BLOCK_TEST_BLOCKS / request_blocks;
        let _ = self.multi_block_profiler.take(MultiBlockOperation::Read);

        let mut all_valid = true;
        for round in 1..=MULTI_BLOCK_TEST_ROUNDS {
            transfer.fill(0);
            let mut latencies_ns = Vec::with_capacity(requests);
            self.multi_block_profiler.enabled = true;
            for request in 0..requests {
                let block_offset = request * request_blocks;
                let byte_offset = request * request_bytes;
                let start_ns = axhal::time::monotonic_time_nanos();
                self.read_blocks(
                    MULTI_BLOCK_TEST_START_LBA + block_offset as u32,
                    &mut transfer[byte_offset..byte_offset + request_bytes],
                );
                latencies_ns.push(
                    axhal::time::monotonic_time_nanos()
                        .saturating_sub(start_ns)
                        .max(1),
                );
            }
            self.multi_block_profiler.enabled = false;
            let mismatch = Self::multi_block_first_mismatch(transfer, expected);
            if let Some((offset, expected, actual)) = mismatch {
                all_valid = false;
                warn!(
                    "MULTI_BLOCK_MISMATCH operation=read request_blocks={} round={} offset={} \
                     lba={} byte_in_block={} expected=0x{:02x} actual=0x{:02x}",
                    request_blocks,
                    round,
                    offset,
                    MULTI_BLOCK_TEST_START_LBA + (offset / Self::BLOCK_SIZE) as u32,
                    offset % Self::BLOCK_SIZE,
                    expected,
                    actual,
                );
            }
            Self::multi_block_print_round(
                MultiBlockOperation::Read,
                request_blocks,
                round,
                latencies_ns,
                mismatch.is_none(),
            );
        }

        let profile = self.multi_block_profiler.take(MultiBlockOperation::Read);
        Self::multi_block_print_profile(MultiBlockOperation::Read, request_blocks, profile);
        all_valid
    }

    #[cfg(feature = "multi-block-test")]
    fn multi_block_measure_write(
        &mut self,
        size_index: usize,
        request_blocks: usize,
        expected: &mut [u8],
        verify: &mut [u8],
    ) -> bool {
        let request_bytes = request_blocks * Self::BLOCK_SIZE;
        let requests = MULTI_BLOCK_TEST_BLOCKS / request_blocks;
        let _ = self.multi_block_profiler.take(MultiBlockOperation::Write);

        let mut all_valid = true;
        for round in 1..=MULTI_BLOCK_TEST_ROUNDS {
            Self::multi_block_fill_pattern(
                expected,
                0x40u8
                    .wrapping_add((size_index as u8).wrapping_mul(16))
                    .wrapping_add(round as u8),
            );
            let mut latencies_ns = Vec::with_capacity(requests);
            self.multi_block_profiler.enabled = true;
            for request in 0..requests {
                let block_offset = request * request_blocks;
                let byte_offset = request * request_bytes;
                let start_ns = axhal::time::monotonic_time_nanos();
                self.write_blocks(
                    MULTI_BLOCK_TEST_START_LBA + block_offset as u32,
                    &expected[byte_offset..byte_offset + request_bytes],
                );
                latencies_ns.push(
                    axhal::time::monotonic_time_nanos()
                        .saturating_sub(start_ns)
                        .max(1),
                );
            }
            self.multi_block_profiler.enabled = false;

            self.multi_block_test_read_single(verify);
            let mismatch = Self::multi_block_first_mismatch(verify, expected);
            if let Some((offset, expected, actual)) = mismatch {
                all_valid = false;
                warn!(
                    "MULTI_BLOCK_MISMATCH operation=write request_blocks={} round={} offset={} \
                     lba={} byte_in_block={} expected=0x{:02x} actual=0x{:02x}",
                    request_blocks,
                    round,
                    offset,
                    MULTI_BLOCK_TEST_START_LBA + (offset / Self::BLOCK_SIZE) as u32,
                    offset % Self::BLOCK_SIZE,
                    expected,
                    actual,
                );
            }
            Self::multi_block_print_round(
                MultiBlockOperation::Write,
                request_blocks,
                round,
                latencies_ns,
                mismatch.is_none(),
            );
        }

        let profile = self.multi_block_profiler.take(MultiBlockOperation::Write);
        Self::multi_block_print_profile(MultiBlockOperation::Write, request_blocks, profile);
        all_valid
    }

    #[cfg(feature = "multi-block-test")]
    fn run_multi_block_test(&mut self) {
        let test_end_lba = MULTI_BLOCK_TEST_START_LBA + MULTI_BLOCK_TEST_BLOCKS as u32 - 1;
        assert!(
            test_end_lba as u64 + 1 <= self.num_blocks,
            "multi-block-test region exceeds the detected card capacity"
        );
        assert!(
            self.dma_buffer
                .as_ref()
                .is_some_and(|buffer| buffer.size >= DMA_BUFFER_SIZE),
            "multi-block-test requires a 16 KiB IDMAC bounce buffer"
        );
        for request_blocks in MULTI_BLOCK_REQUEST_SIZES {
            assert_eq!(MULTI_BLOCK_TEST_BLOCKS % request_blocks, 0);
            assert!(request_blocks * Self::BLOCK_SIZE <= DMA_BUFFER_SIZE);
        }

        warn!(
            "MULTI_BLOCK_TEST begin start_lba={} end_lba={} blocks={} bytes={} rounds={} \
             request_blocks=1,2,4,8,16,32 bus_width_bits=1 card_clock_hz={} destructive=true \
             restore_on_normal_completion=true",
            MULTI_BLOCK_TEST_START_LBA,
            test_end_lba,
            MULTI_BLOCK_TEST_BLOCKS,
            MULTI_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE,
            MULTI_BLOCK_TEST_ROUNDS,
            VISIONFIVE2_SDIO_CIU_CLOCK_HZ / (2 * DEFAULT_SPEED_CLOCK_DIVIDER as u32),
        );

        let region_bytes = MULTI_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE;
        let mut backup = vec![0u8; region_bytes];
        let mut expected = vec![0u8; region_bytes];
        let mut transfer = vec![0u8; region_bytes];

        self.multi_block_profiler.enabled = false;
        warn!("MULTI_BLOCK_TEST stage=backup status=begin method=CMD17");
        self.multi_block_test_read_single(&mut backup);
        self.multi_block_test_read_single(&mut transfer);
        assert!(
            Self::multi_block_first_mismatch(&transfer, &backup).is_none(),
            "multi-block-test backup could not be reproduced with CMD17"
        );
        warn!("MULTI_BLOCK_TEST stage=backup status=verified");

        let mut test_failed = false;
        for request_blocks in MULTI_BLOCK_REQUEST_SIZES {
            self.multi_block_profiler.enabled = false;
            for request in 0..(MULTI_BLOCK_TEST_BLOCKS / request_blocks) {
                let block_offset = request * request_blocks;
                let byte_offset = block_offset * Self::BLOCK_SIZE;
                let request_bytes = request_blocks * Self::BLOCK_SIZE;
                self.read_blocks(
                    MULTI_BLOCK_TEST_START_LBA + block_offset as u32,
                    &mut transfer[byte_offset..byte_offset + request_bytes],
                );
            }
            if Self::multi_block_first_mismatch(&transfer, &backup).is_some() {
                warn!(
                    "MULTI_BLOCK_TEST read warmup failed request_blocks={}",
                    request_blocks
                );
                test_failed = true;
            }
            test_failed |= !self.multi_block_measure_read(request_blocks, &backup, &mut transfer);
        }

        for (size_index, request_blocks) in MULTI_BLOCK_REQUEST_SIZES.into_iter().enumerate() {
            test_failed |= !self.multi_block_measure_write(
                size_index,
                request_blocks,
                &mut expected,
                &mut transfer,
            );
        }

        self.multi_block_profiler.enabled = false;
        warn!("MULTI_BLOCK_TEST stage=restore status=begin method=CMD24");
        self.multi_block_test_write_single(&backup);
        self.multi_block_test_read_single(&mut transfer);
        if let Some((offset, expected, actual)) =
            Self::multi_block_first_mismatch(&transfer, &backup)
        {
            panic!(
                "multi-block-test RESTORE FAILED at byte {} (LBA {}, byte {}): expected=0x{:02x}, \
                 actual=0x{:02x}",
                offset,
                MULTI_BLOCK_TEST_START_LBA + (offset / Self::BLOCK_SIZE) as u32,
                offset % Self::BLOCK_SIZE,
                expected,
                actual,
            );
        }
        warn!("MULTI_BLOCK_TEST stage=restore status=verified");
        assert!(!test_failed, "multi-block-test detected data mismatches");
        warn!("MULTI_BLOCK_TEST complete status=ok original_region_restored=true");
    }

    fn can_send_cmd(&self) -> bool {
        !self.regs.cmd().read().start_cmd()
    }

    fn can_send_data(&self) -> bool {
        let status = self.regs.status().read();
        !status.data_busy() && !status.data_state_mc_busy()
    }

    fn command_finished(&self) -> bool {
        let rintsts = self.regs.rintsts().read();
        rintsts.command_done() || rintsts.error()
    }

    fn clear_idsts(&self) {
        let idsts = self.regs.idsts().read();
        if idsts.ais()
            || idsts.nis()
            || idsts.ces()
            || idsts.du()
            || idsts.fbe()
            || idsts.ri()
            || idsts.ti()
        {
            debug!("Clearing IDSTS: {:?}", idsts);
            self.regs.idsts().write(idsts);
        }
    }

    fn reset_idmac(&self) -> bool {
        self.regs.bmod().update(|r| r.with_de(false).with_swr(true));
        self.regs
            .ctrl()
            .update(|r| r.with_dma_reset(true).with_use_internal_dmac(false));
        dma_io_fence();

        let deadline = axhal::time::monotonic_time() + Duration::from_millis(100);
        loop {
            let bmod = self.regs.bmod().read();
            let ctrl = self.regs.ctrl().read();
            if !bmod.swr() && !ctrl.dma_reset() {
                return true;
            }
            if axhal::time::monotonic_time() >= deadline {
                warn!(
                    "IDMAC reset timeout: BMOD={:?}, CTRL={:?}, IDSTS={:?}",
                    bmod,
                    ctrl,
                    self.regs.idsts().read(),
                );
                return false;
            }
            core::hint::spin_loop();
        }
    }

    fn finish_idmac_transfer(&mut self, context: IdmacTransferContext, recover: bool) -> bool {
        if recover && !self.reset_idmac() {
            warn!("IDMAC recovery failed; retaining the descriptor to avoid DMA use-after-free");
            self.idmac_faulted = true;
            self.idmac_reset_failed = true;
            let rintsts = self.regs.rintsts().read();
            let idsts = self.regs.idsts().read();
            self.regs.rintsts().write(rintsts);
            self.regs.idsts().write(idsts);
            return false;
        }

        let rintsts = self.regs.rintsts().read();
        let idsts = self.regs.idsts().read();
        self.regs.rintsts().write(rintsts);
        self.regs.idsts().write(idsts);
        dma_io_fence();
        unsafe { dealloc_coherent(context.dma_desc_info, context.layout) };

        if recover {
            self.regs
                .bmod()
                .update(|r| r.with_de(true).with_dsl(0).with_fb(true));
            self.regs
                .ctrl()
                .update(|r| r.with_use_internal_dmac(true).with_int_enable(true));
            self.regs.idinten().write(
                crate::regs::IdIntEn::new()
                    .with_ai(true)
                    .with_ni(true)
                    .with_ces(true)
                    .with_du(true)
                    .with_fbe(true)
                    .with_ri(true)
                    .with_ti(true),
            );
            self.regs
                .intmask()
                .update(|r| r.with_acd(true).with_dto(true));
            dma_io_fence();
        }

        true
    }

    fn fifo_cnt(&self) -> usize {
        self.regs.status().read().fifo_count() as usize
    }

    fn set_transaction_size(&self, blk_size: u16, byte_cnt: u32) {
        self.regs.blksiz().update(|r| r.with_block_size(blk_size));
        self.regs.bytcnt().write(byte_cnt);
    }

    fn program_card_clock_divider(&self, divider: u8) -> bool {
        self.regs.clkena().write(ClkEna::new());
        if self.send_cmd(Command::ResetClock).is_none() {
            warn!("failed to latch disabled card clock before setting CLKDIV={divider}");
            return false;
        }

        self.regs
            .clkdiv()
            .write(ClkDiv::new().with_clk_divider0(divider));
        if self.send_cmd(Command::ResetClock).is_none() {
            warn!("failed to latch CLKDIV={divider} while card clock was disabled");
            return false;
        }

        self.regs.clkena().write(ClkEna::new().with_cclk_enable(1));
        if self.send_cmd(Command::ResetClock).is_none() {
            warn!("failed to re-enable card clock after setting CLKDIV={divider}");
            return false;
        }

        let actual_divider = self.regs.clkdiv().read().clk_divider0();
        let clock_enabled = self.regs.clkena().read().cclk_enable() & 1 != 0;
        if actual_divider != divider || !clock_enabled {
            warn!(
                "card clock register verification failed: requested_divider={}, \
                 actual_divider={}, enabled={}",
                divider, actual_divider, clock_enabled,
            );
            return false;
        }
        true
    }

    fn switch_card_clock_divider(&self, divider: u8) -> bool {
        let idle_deadline = axhal::time::monotonic_time() + Duration::from_secs(1);
        while !self.can_send_cmd() || !self.can_send_data() {
            if axhal::time::monotonic_time() >= idle_deadline {
                warn!(
                    "card did not become idle before clock switch: CMD={:?}, STATUS={:?}",
                    self.regs.cmd().read(),
                    self.regs.status().read(),
                );
                return false;
            }
            core::hint::spin_loop();
        }

        let previous_divider = self.regs.clkdiv().read().clk_divider0();
        if self.program_card_clock_divider(divider) {
            return true;
        }

        warn!(
            "card clock switch to CLKDIV={} failed; attempting rollback to CLKDIV={}",
            divider, previous_divider,
        );
        if !self.program_card_clock_divider(previous_divider) {
            warn!("card clock rollback failed; further transfers are unsafe");
        }
        false
    }

    fn send_cmd(&self, command: Command<'_>) -> Option<[u32; 4]> {
        let is_reset_clock = matches!(command, Command::ResetClock);
        let expects_busy = matches!(command, Command::SelectCard(_));
        trace!("send_cmd {command:#x?}");

        let (cmd, arg, xfer) = command.build();
        assert_eq!(cmd.data_expected(), xfer.is_some());

        trace!("send_cmd {cmd:?} {arg:#x?}");

        // Wait for command to be sendable (with timeout counter)
        let mut cmd_wait_count = 0u64;
        let cmd_max_wait = 1_000_000u64; // ~1M iterations = few seconds on modern CPU
        while !self.can_send_cmd() {
            core::hint::spin_loop();
            cmd_wait_count += 1;
            if cmd_wait_count > cmd_max_wait {
                break;
            }
        }
        if !self.can_send_cmd() {
            warn!(
                "cmd {} cannot be submitted: controller stayed busy; CMD={:?}, STATUS={:?}, \
                 RINTSTS={:?}",
                cmd.cmd_index(),
                self.regs.cmd().read(),
                self.regs.status().read(),
                self.regs.rintsts().read(),
            );
            return None;
        }
        if cmd.data_expected() {
            while !self.can_send_data() {
                core::hint::spin_loop();
            }
        }

        // RINTSTS is write-1-to-clear. Clear completion/error bits left by the
        // preceding command before submitting a new command.
        let stale_rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(stale_rintsts);

        self.regs.cmdarg().write(arg);
        self.regs.cmd().write(cmd);

        // Wait for command to complete (with timeout counter)
        let mut start_cmd_wait_count = 0u64;
        while !self.can_send_cmd() {
            core::hint::spin_loop();
            start_cmd_wait_count += 1;
            if start_cmd_wait_count > cmd_max_wait {
                break;
            }
        }
        if !self.can_send_cmd() {
            let rintsts = self.regs.rintsts().read();
            self.regs.rintsts().write(rintsts);
            warn!(
                "cmd {} was not accepted before timeout; CMD={:?}, STATUS={:?}, RINTSTS={:?}",
                cmd.cmd_index(),
                self.regs.cmd().read(),
                self.regs.status().read(),
                rintsts,
            );
            return None;
        }
        trace!("cmd {} sent", cmd.cmd_index());

        let mut command_timed_out = false;
        if !is_reset_clock {
            // Every real command, including response-less CMD0, completes by
            // setting command_done or an error bit. Clock-update commands are
            // the only exception and complete when start_cmd clears.
            let mut completion_wait_count = 0u64;
            let completion_deadline = axhal::time::monotonic_time() + Duration::from_secs(1);
            while !self.command_finished() {
                core::hint::spin_loop();
                completion_wait_count += 1;
                if axhal::time::monotonic_time() >= completion_deadline {
                    command_timed_out = true;
                    warn!(
                        "cmd {} completion timeout after {} iterations; STATUS={:?}, RINTSTS={:?}",
                        cmd.cmd_index(),
                        completion_wait_count,
                        self.regs.status().read(),
                        self.regs.rintsts().read(),
                    );
                    break;
                }
            }

            trace!("cmd {} completed", cmd.cmd_index());
        }

        if command_timed_out {
            let rintsts = self.regs.rintsts().read();
            self.regs.rintsts().write(rintsts);
            return None;
        }

        let command_status = self.regs.rintsts().read();
        if command_status.error() {
            let resp = self.regs.resp().read();
            self.regs.rintsts().write(command_status);
            warn!(
                "cmd {} failed before data/busy phase: rintsts={command_status:?}, resp={resp:?}",
                cmd.cmd_index(),
            );
            return None;
        }

        let mut busy_timed_out = false;
        if expects_busy {
            let busy_deadline = axhal::time::monotonic_time() + Duration::from_secs(1);
            while !self.can_send_data() {
                if axhal::time::monotonic_time() >= busy_deadline {
                    busy_timed_out = true;
                    break;
                }
                core::hint::spin_loop();
            }
        }

        if let Some(xfer) = xfer {
            let fifo_base = unsafe { self.regs.as_raw_ptr().byte_add(Self::FIFO) }.cast::<u64>();
            let mut offset = 0;
            match xfer {
                DataXfer::Read(buf) => {
                    wait_until(|| {
                        let rintsts = self.regs.rintsts().read();

                        if rintsts.receive_fifo_data_request() {
                            trace!("rxdr");
                            while self.fifo_cnt() >= 2 && offset + 8 <= buf.len() {
                                let data = unsafe { fifo_base.read_volatile() };
                                buf[offset..offset + 8].copy_from_slice(&data.to_le_bytes());
                                offset += 8;
                            }
                        }

                        rintsts.data_transfer_over() || rintsts.error()
                    });
                    trace!("received {offset} bytes");
                }
                DataXfer::Write(buf) => {
                    wait_until(|| {
                        let rintsts = self.regs.rintsts().read();

                        if rintsts.transmit_fifo_data_request() {
                            trace!("txdr");
                            // Leave eight entries below the 128-entry FIFO limit.
                            while self.fifo_cnt() < 120 && offset + 8 <= buf.len() {
                                let data =
                                    u64::from_le_bytes(buf[offset..offset + 8].try_into().unwrap());
                                unsafe { fifo_base.write_volatile(data) };
                                offset += 8;
                            }
                        }

                        rintsts.data_transfer_over() || rintsts.error()
                    });
                    trace!("sent {offset} bytes");
                }
            }
        }

        let resp = self.regs.resp().read();

        let rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(rintsts);

        if busy_timed_out {
            warn!("cmd {} card busy timeout", cmd.cmd_index());
            return None;
        }

        if rintsts.error() {
            warn!(
                "cmd {} error - rintsts: {rintsts:?}, resp: {resp:?}",
                cmd.cmd_index()
            );
            warn!(
                "  response_timeout: {}, data_read_timeout: {}, start_bit_error: {}, \
                 end_bit_error: {}",
                rintsts.response_timeout(),
                rintsts.data_read_timeout(),
                rintsts.start_bit_error(),
                rintsts.end_bit_error()
            );
            warn!(
                "  data_crc_error: {}, response_crc_error: {}, response_error: {}, \
                 hardware_locked_write: {}",
                rintsts.data_crc_error(),
                rintsts.response_crc_error(),
                rintsts.response_error(),
                rintsts.hardware_locked_write()
            );
            return None;
        }
        Some(resp)
    }

    fn init(&mut self) {
        info!("Initializing SD/MMC driver at {:?}", self.regs);

        // U-Boot leaves the controller configured, but the driver needs a clean status baseline.
        let rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(rintsts);

        // Reconfigure the card clock while it is disabled.
        self.regs.clkena().write(ClkEna::new());
        if self.send_cmd(Command::ResetClock).is_none() {
            warn!("ResetClock failed while disabling card clock; continuing");
        }
        self.regs
            .clkdiv()
            .write(ClkDiv::new().with_clk_divider0(IDENTIFICATION_CLOCK_DIVIDER));
        self.regs.clkena().write(ClkEna::new().with_cclk_enable(1));
        if self.send_cmd(Command::ResetClock).is_none() {
            warn!("ResetClock failed while enabling card clock; continuing");
        }

        for _ in 0..10000 {
            core::hint::spin_loop();
        }

        self.regs.pwren().write(1u32.into());

        for _ in 0..100000 {
            core::hint::spin_loop();
        }

        self.regs.ctype().write(0.into());

        self.regs.bmod().update(|r| r.with_de(false).with_swr(true));
        self.regs
            .ctrl()
            .update(|r| r.with_dma_reset(true).with_use_internal_dmac(false));

        if self.send_cmd(Command::GoIdleState).is_none() {
            warn!("GoIdleState timed out during initialization; continuing");
        }

        let has_valid_resp = match self.send_cmd(Command::SendIfCond(0x1aa)) {
            Some(resp) => {
                if resp[0] & 0xff != 0xaa {
                    warn!("Unexpected SendIfCond response: {:?}", resp);
                    false
                } else {
                    true
                }
            }
            None => {
                warn!("SendIfCond FAILED - card not responding or unsupported");
                false
            }
        };

        if !has_valid_resp {
            warn!("SD card not responding properly - continuing anyway");
        }

        let mut attempt = 0;
        let mut card_initialized = false;
        let acmd41_deadline = axhal::time::monotonic_time() + Duration::from_secs(2);
        while axhal::time::monotonic_time() < acmd41_deadline {
            attempt += 1;
            if self.send_cmd(Command::AppCmd(0)).is_some() {
                match self.send_cmd(Command::SdSendOpCond(0x40FF_8000)) {
                    Some(resp) => {
                        let ocr = resp[0];
                        if ocr & 0x8000_0000 != 0 {
                            info!(
                                "SD card is ready after {} attempts, OCR={ocr:#010x}",
                                attempt
                            );
                            card_initialized = true;
                            if ocr & 0x4000_0000 != 0 {
                                debug!("SD card supports high capacity");
                            } else {
                                debug!("SD card is standard capacity");
                            }
                            break;
                        }
                    }
                    None => warn!("SdSendOpCond failed on attempt {}", attempt),
                }
            } else {
                warn!("AppCmd failed on attempt {}", attempt);
            }

            axhal::time::busy_wait(Duration::from_millis(10));
        }
        if !card_initialized {
            warn!("ACMD41 timed out after {} attempts", attempt);
        }

        if !card_initialized {
            warn!("Card initialization failed - continuing anyway");
            return;
        }

        match self.send_cmd(Command::AllSendCid) {
            Some(resp) => {
                let cid = unsafe { core::mem::transmute::<[u32; 4], Cid>(resp) };
                info!("cid: {cid:?}");
            }
            None => {
                warn!("AllSendCid failed - cannot determine card ID");
                return;
            }
        }

        let rca = match self.send_cmd(Command::SendRelativeAddr) {
            Some(resp) => {
                let rca = (resp[0] >> 16) & 0xffff;
                debug!("rca: {rca:#x}");
                rca
            }
            None => {
                warn!("SendRelativeAddr failed - cannot get card address");
                return;
            }
        };

        match self.send_cmd(Command::SendCsd(rca << 16)) {
            Some(resp) => {
                let csd = unsafe { core::mem::transmute::<[u32; 4], CsdV2>(resp) };
                debug!("csd: {csd:?}");
                self.num_blocks = csd.num_blocks();
                info!("SD card capacity: {:#x} blocks", self.num_blocks);
            }
            None => {
                warn!("SendCsd failed - cannot determine card capacity");
                self.num_blocks = 0;
            }
        }

        if self.send_cmd(Command::SelectCard(rca << 16)).is_none() {
            warn!("SelectCard failed");
        }

        if self.send_cmd(Command::AppCmd(rca << 16)).is_none() {
            warn!("AppCmd failed");
        }

        // A block-sized buffer keeps the short SCR transfer DMA-aligned.
        self.set_transaction_size(8, 8);
        let mut buf = [0u8; 512];
        match self.send_cmd(Command::SendScr(&mut buf)) {
            Some(_) => {
                let scr = u64::from_be_bytes(buf[..8].try_into().unwrap());
                debug!("Bus width supported: {:#x?}", (scr >> 48) & 0xf);
            }
            None => warn!("SendScr failed"),
        }

        let rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(rintsts);

        if !self.switch_card_clock_divider(DEFAULT_SPEED_CLOCK_DIVIDER) {
            warn!("failed to enter SD Default Speed; disabling block transfers");
            self.num_blocks = 0;
            return;
        }
        let card_clock_hz =
            VISIONFIVE2_SDIO_CIU_CLOCK_HZ / (2 * DEFAULT_SPEED_CLOCK_DIVIDER as u32);
        warn!(
            "SD/MMC card clock switched to Default Speed: ciu_clock_hz={}, CLKDIV={}, \
             card_clock_hz={}",
            VISIONFIVE2_SDIO_CIU_CLOCK_HZ, DEFAULT_SPEED_CLOCK_DIVIDER, card_clock_hz,
        );

        info!("SD/MMC driver initialized");
    }

    fn validate_block_buffer(&self, block: u32, len: usize) -> usize {
        assert!(len != 0, "SD/MMC transfer buffer must not be empty");
        assert_eq!(
            len % Self::BLOCK_SIZE,
            0,
            "SD/MMC transfer length must be block aligned"
        );
        let blocks = len / Self::BLOCK_SIZE;
        let end = block as u64 + blocks as u64;
        assert!(
            end <= self.num_blocks,
            "SD/MMC transfer range exceeds card capacity"
        );
        blocks
    }

    fn read_dma_chunk(&mut self, block: u32, buf: &mut [u8]) {
        debug_assert!(buf.len() <= DMA_BUFFER_SIZE);
        #[cfg(feature = "multi-block-test")]
        self.multi_block_profiler.begin(MultiBlockOperation::Read);
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        self.set_transaction_size(Self::BLOCK_SIZE as u16, buf.len() as u32);
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::TransactionConfig, phase_start);

        let dma_buf_info = self
            .dma_buffer
            .as_ref()
            .expect("synchronous DMA read requested without an IDMAC buffer");
        let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
        let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
            .expect("DMA buffer address exceeds the IDMAC 32-bit address range");
        let dma_buf = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
        let command = if buf.len() == Self::BLOCK_SIZE {
            Command::ReadSingleBlock(block, dma_buf)
        } else {
            Command::ReadMultipleBlocks(block, dma_buf)
        };
        self.send_cmd_idmac(command, dma_bus_addr)
            .expect("synchronous IDMAC read failed");

        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        let dma_usr_slice = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
        buf.copy_from_slice(dma_usr_slice);
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::BounceCopy, phase_start);
        #[cfg(feature = "multi-block-test")]
        self.multi_block_profiler.finish();
    }

    async fn read_dma_chunk_async(&mut self, block: u32, buf: &mut [u8]) {
        debug_assert!(buf.len() <= DMA_BUFFER_SIZE);
        self.set_transaction_size(Self::BLOCK_SIZE as u16, buf.len() as u32);

        let dma_buf_info = self
            .dma_buffer
            .as_ref()
            .expect("asynchronous DMA read requested without an IDMAC buffer");
        let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
        let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
            .expect("DMA buffer address exceeds the IDMAC 32-bit address range");
        let dma_buf = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
        let command = if buf.len() == Self::BLOCK_SIZE {
            Command::ReadSingleBlock(block, dma_buf)
        } else {
            Command::ReadMultipleBlocks(block, dma_buf)
        };
        self.send_cmd_idmac_async(command, dma_bus_addr)
            .await
            .expect("asynchronous IDMAC read failed");

        let dma_usr_slice = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
        buf.copy_from_slice(dma_usr_slice);
    }

    fn wait_card_ready_after_write(&mut self) {
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        let deadline = axhal::time::monotonic_time() + Duration::from_secs(5);
        while !self.can_send_data() {
            if axhal::time::monotonic_time() >= deadline {
                #[cfg(feature = "multi-block-test")]
                self.multi_block_phase_finish(MultiBlockPhase::CardBusy, phase_start);
                self.idmac_faulted = true;
                panic!("SD/MMC card stayed busy after write");
            }
            core::hint::spin_loop();
        }
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::CardBusy, phase_start);
    }

    fn write_dma_chunk(&mut self, block: u32, buf: &[u8]) {
        debug_assert!(buf.len() <= DMA_BUFFER_SIZE);
        #[cfg(feature = "multi-block-test")]
        self.multi_block_profiler.begin(MultiBlockOperation::Write);
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        self.set_transaction_size(Self::BLOCK_SIZE as u16, buf.len() as u32);
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::TransactionConfig, phase_start);

        let dma_buf_info = self
            .dma_buffer
            .as_ref()
            .expect("synchronous DMA write requested without an IDMAC buffer");
        let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
        let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
            .expect("DMA buffer address exceeds the IDMAC 32-bit address range");
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        let dma_usr_slice = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
        dma_usr_slice.copy_from_slice(buf);
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::BounceCopy, phase_start);

        let dma_buf = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
        let command = if buf.len() == Self::BLOCK_SIZE {
            Command::WriteSingleBlock(block, dma_buf)
        } else {
            Command::WriteMultipleBlocks(block, dma_buf)
        };
        self.send_cmd_idmac(command, dma_bus_addr)
            .expect("synchronous IDMAC write failed");
        self.wait_card_ready_after_write();
        #[cfg(feature = "multi-block-test")]
        self.multi_block_profiler.finish();
    }

    async fn write_dma_chunk_async(&mut self, block: u32, buf: &[u8]) {
        debug_assert!(buf.len() <= DMA_BUFFER_SIZE);
        self.set_transaction_size(Self::BLOCK_SIZE as u16, buf.len() as u32);

        let dma_buf_info = self
            .dma_buffer
            .as_ref()
            .expect("asynchronous DMA write requested without an IDMAC buffer");
        let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
        let dma_usr_slice = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
        dma_usr_slice.copy_from_slice(buf);

        let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
            .expect("DMA buffer address exceeds the IDMAC 32-bit address range");
        let dma_buf = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
        let command = if buf.len() == Self::BLOCK_SIZE {
            Command::WriteSingleBlock(block, dma_buf)
        } else {
            Command::WriteMultipleBlocks(block, dma_buf)
        };
        self.send_cmd_idmac_async(command, dma_bus_addr)
            .await
            .expect("asynchronous IDMAC write failed");
        self.wait_card_ready_after_write();
    }

    /// Reads one or more contiguous blocks from the SD/MMC card.
    pub fn read_blocks(&mut self, mut block: u32, buf: &mut [u8]) {
        let mut remaining_blocks = self.validate_block_buffer(block, buf.len());
        for chunk in buf.chunks_mut(DMA_BUFFER_SIZE) {
            self.read_dma_chunk(block, chunk);
            let chunk_blocks = chunk.len() / Self::BLOCK_SIZE;
            remaining_blocks -= chunk_blocks;
            if remaining_blocks != 0 {
                block = block
                    .checked_add(chunk_blocks as u32)
                    .expect("SD/MMC read block address overflow");
            }
        }
    }

    /// Reads one or more contiguous blocks and asynchronously waits for each DMA chunk.
    pub async fn read_blocks_async(&mut self, mut block: u32, buf: &mut [u8]) {
        let mut remaining_blocks = self.validate_block_buffer(block, buf.len());
        for chunk in buf.chunks_mut(DMA_BUFFER_SIZE) {
            self.read_dma_chunk_async(block, chunk).await;
            let chunk_blocks = chunk.len() / Self::BLOCK_SIZE;
            remaining_blocks -= chunk_blocks;
            if remaining_blocks != 0 {
                block = block
                    .checked_add(chunk_blocks as u32)
                    .expect("SD/MMC asynchronous read block address overflow");
            }
        }
    }

    /// Writes one or more contiguous blocks to the SD/MMC card.
    pub fn write_blocks(&mut self, mut block: u32, buf: &[u8]) {
        let mut remaining_blocks = self.validate_block_buffer(block, buf.len());
        for chunk in buf.chunks(DMA_BUFFER_SIZE) {
            self.write_dma_chunk(block, chunk);
            let chunk_blocks = chunk.len() / Self::BLOCK_SIZE;
            remaining_blocks -= chunk_blocks;
            if remaining_blocks != 0 {
                block = block
                    .checked_add(chunk_blocks as u32)
                    .expect("SD/MMC write block address overflow");
            }
        }
    }

    /// Writes one or more contiguous blocks and asynchronously waits for each DMA chunk.
    pub async fn write_blocks_async(&mut self, mut block: u32, buf: &[u8]) {
        let mut remaining_blocks = self.validate_block_buffer(block, buf.len());
        for chunk in buf.chunks(DMA_BUFFER_SIZE) {
            self.write_dma_chunk_async(block, chunk).await;
            let chunk_blocks = chunk.len() / Self::BLOCK_SIZE;
            remaining_blocks -= chunk_blocks;
            if remaining_blocks != 0 {
                block = block
                    .checked_add(chunk_blocks as u32)
                    .expect("SD/MMC asynchronous write block address overflow");
            }
        }
    }

    /// Reads a single block from the SD/MMC card.
    pub fn read_block(&mut self, block: u32, buf: &mut [u8; 512]) {
        self.read_blocks(block, buf);
    }

    /// Reads a single block using IDMAC and asynchronously waits for completion.
    pub async fn read_block_async(&mut self, block: u32, buf: &mut [u8; 512]) {
        self.read_blocks_async(block, buf).await;
    }

    /// Writes a single block to the SD/MMC card.
    pub fn write_block(&mut self, block: u32, buf: &[u8; 512]) {
        self.write_blocks(block, buf);
    }

    /// Writes a single block using IDMAC and asynchronously waits for completion.
    pub async fn write_block_async(&mut self, block: u32, buf: &[u8; 512]) {
        self.write_blocks_async(block, buf).await;
    }

    /// Returns the number of blocks.
    pub fn num_blocks(&self) -> u64 {
        self.num_blocks
    }

    /// Enables the Internal DMA (IDMAC) for DMA transfers.
    pub fn try_enable_idmac(
        &mut self,
        buf_size: usize,
        ahb_data_width: AHBDataWidth,
        register_irq: impl FnOnce() -> bool,
    ) {
        let hcon = self.regs.hcon().read();
        let hardware_data_width = match hcon.h_data_width() {
            0 => AHBDataWidth::Bits16,
            1 => AHBDataWidth::Bits32,
            2 => AHBDataWidth::Bits64,
            value => {
                warn!(
                    "Unsupported IDMAC H_DATA_WIDTH value {} in HCON={:?}",
                    value, hcon
                );
                return;
            }
        };
        if hardware_data_width != ahb_data_width {
            warn!(
                "IDMAC data width corrected from {:?} to HCON-reported {:?}",
                ahb_data_width, hardware_data_width
            );
        }
        self.ahb_data_width = hardware_data_width;

        if !self.reset_idmac() {
            warn!("Failed to reset IDMAC before enabling it");
            return;
        }

        let layout = Layout::from_size_align(buf_size, hardware_data_width.align_value())
            .expect("Invalid layout for DMA buffer");
        match unsafe { alloc_coherent(layout) } {
            Ok(dma_info) => {
                self.dma_buffer = Some(DMABuffer {
                    addr: dma_info,
                    size: buf_size,
                });
            }
            Err(e) => {
                warn!(
                    "Failed to allocate DMA buffer: {:?}, use PIO mode instead",
                    e
                );
                return;
            }
        }

        let rintsts_before_enable = self.regs.rintsts().read();
        let idsts_before_enable = self.regs.idsts().read();
        if rintsts_before_enable.error()
            || rintsts_before_enable.auto_command_done()
            || rintsts_before_enable.data_transfer_over()
            || rintsts_before_enable.receive_fifo_data_request()
            || rintsts_before_enable.transmit_fifo_data_request()
        {
            self.regs.rintsts().write(rintsts_before_enable);
        }
        if idsts_before_enable.ais()
            || idsts_before_enable.nis()
            || idsts_before_enable.ces()
            || idsts_before_enable.du()
            || idsts_before_enable.fbe()
            || idsts_before_enable.ri()
            || idsts_before_enable.ti()
        {
            self.clear_idsts();
        }

        self.regs
            .bmod()
            .update(|r| r.with_de(true).with_dsl(0).with_fb(true));
        dma_io_fence();
        let bmod_after = self.regs.bmod().read();
        let idsts_after_bmod = self.regs.idsts().read();
        if idsts_after_bmod.du() || idsts_after_bmod.fbe() || idsts_after_bmod.ais() {
            warn!(
                "try_enable_idmac: abnormal IDSTS after BMOD enable: {:?}",
                idsts_after_bmod
            );
        }
        if !bmod_after.de() || bmod_after.dsl() != 0 || !bmod_after.fb() {
            warn!(
                "Failed to set BMOD register for IDMAC, use PIO mode instead; actual: de={}, \
                 dsl={}, fb={}, pbl={}",
                bmod_after.de(),
                bmod_after.dsl(),
                bmod_after.fb(),
                bmod_after.pbl(),
            );
            unsafe {
                dealloc_coherent(self.dma_buffer.as_ref().unwrap().addr, layout);
            }
            self.dma_buffer = None;
            return;
        }

        self.regs
            .ctrl()
            .update(|r| r.with_use_internal_dmac(true).with_int_enable(true));
        dma_io_fence();
        let ctrl_after = self.regs.ctrl().read();
        let idsts_after_ctrl = self.regs.idsts().read();
        if !ctrl_after.use_internal_dmac() || !ctrl_after.int_enable() {
            warn!(
                "Failed to set CTRL register for IDMAC and interrupt output, use PIO mode \
                 instead; expected use_internal_dmac=true, int_enable=true. actual: \
                 use_internal_dmac={}, int_enable={}. IDSTS={:?}",
                ctrl_after.use_internal_dmac(),
                ctrl_after.int_enable(),
                idsts_after_ctrl
            );
            unsafe {
                dealloc_coherent(self.dma_buffer.as_ref().unwrap().addr, layout);
            }
            self.dma_buffer = None;
            return;
        }
        if idsts_after_ctrl.du() || idsts_after_ctrl.fbe() || idsts_after_ctrl.ais() {
            warn!(
                "try_enable_idmac: abnormal IDSTS after CTRL enable; disabling IDMAC path: {:?}",
                idsts_after_ctrl
            );
            unsafe {
                dealloc_coherent(self.dma_buffer.as_ref().unwrap().addr, layout);
            }
            self.dma_buffer = None;
            return;
        }

        // Enable IDMAC completion/error interrupts and controller ACD/DTO.
        self.regs.idinten().write(
            crate::regs::IdIntEn::new()
                .with_ai(true)
                .with_ni(true)
                .with_ces(true)
                .with_du(true)
                .with_fbe(true)
                .with_ri(true)
                .with_ti(true),
        );
        self.regs
            .intmask()
            .write(crate::regs::IntMask::new().with_acd(true).with_dto(true));

        let idinten_after = self.regs.idinten().read();
        let intmask_after = self.regs.intmask().read();
        let idsts_after_enable = self.regs.idsts().read();
        if !idinten_after.ai()
            || !idinten_after.ni()
            || !idinten_after.ces()
            || !idinten_after.du()
            || !idinten_after.fbe()
            || !idinten_after.ri()
            || !idinten_after.ti()
        {
            warn!(
                "try_enable_idmac: IDINTEN mismatch after write; verify hardware support and \
                 register access"
            );
        }
        if !intmask_after.acd()
            || !intmask_after.dto()
            || intmask_after.cmd()
            || intmask_after.rxdr()
            || intmask_after.txdr()
        {
            warn!(
                "try_enable_idmac: INTMASK mismatch after write; acd={}, dto={}, cmd={}, rxdr={}, \
                 txdr={}",
                intmask_after.acd(),
                intmask_after.dto(),
                intmask_after.cmd(),
                intmask_after.rxdr(),
                intmask_after.txdr(),
            );
        }
        if idsts_after_enable.du() || idsts_after_enable.fbe() || idsts_after_enable.ais() {
            warn!(
                "try_enable_idmac: abnormal post-enable IDSTS detected: {:?}",
                idsts_after_enable
            );
        }
        let irq_registered = register_irq();
        if !irq_registered {
            let idsts_on_irq_fail = self.regs.idsts().read();
            let idinten_on_irq_fail = self.regs.idinten().read();
            let rintsts_on_irq_fail = self.regs.rintsts().read();
            warn!(
                "Failed to register IRQ for IDMAC, use PIO mode instead; RINTSTS={:?}, \
                 IDSTS={:?}, IDINTEN={:?}, DBADDR=0x{:08x}",
                rintsts_on_irq_fail,
                idsts_on_irq_fail,
                idinten_on_irq_fail,
                self.regs.dbaddr().read(),
            );
            unsafe {
                dealloc_coherent(self.dma_buffer.as_ref().unwrap().addr, layout);
            }
            self.dma_buffer = None;

            self.regs.bmod().update(|r| r.with_de(false).with_swr(true));
            self.regs
                .ctrl()
                .update(|r| r.with_dma_reset(true).with_use_internal_dmac(false));
            return;
        }
    }

    fn prepare_idmac_transfer(
        &mut self,
        command: Command<'_>,
        dma_bus_addr: u32,
    ) -> Option<IdmacTransferContext> {
        if self.idmac_faulted {
            warn!("refusing IDMAC transfer because the driver is faulted");
            return None;
        }

        let (cmd, arg, xfer) = command.build();
        assert!(
            cmd.data_expected(),
            "send_cmd_idmac should only be used for commands that require data transfer"
        );
        assert!(
            xfer.is_some(),
            "send_cmd_idmac requires a data buffer for transfer"
        );

        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        let cmd_idle_deadline = axhal::time::monotonic_time() + Duration::from_secs(1);
        while !self.can_send_cmd() {
            if axhal::time::monotonic_time() >= cmd_idle_deadline {
                warn!(
                    "send_cmd_idmac: can_send_cmd timeout; CMD={:?}, STATUS={:?}, RINTSTS={:?}, \
                     IDSTS={:?}",
                    self.regs.cmd().read(),
                    self.regs.status().read(),
                    self.regs.rintsts().read(),
                    self.regs.idsts().read(),
                );
                self.idmac_faulted = true;
                return None;
            }
            core::hint::spin_loop();
        }
        let data_idle_deadline = axhal::time::monotonic_time() + Duration::from_secs(1);
        while !self.can_send_data() {
            if axhal::time::monotonic_time() >= data_idle_deadline {
                warn!(
                    "send_cmd_idmac: can_send_data timeout; CMD={:?}, STATUS={:?}, RINTSTS={:?}, \
                     IDSTS={:?}",
                    self.regs.cmd().read(),
                    self.regs.status().read(),
                    self.regs.rintsts().read(),
                    self.regs.idsts().read(),
                );
                self.idmac_faulted = true;
                return None;
            }
            core::hint::spin_loop();
        }
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::IdleWait, phase_start);

        // Establish a clean W1C status baseline for the new transaction.
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        let stale_rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(stale_rintsts);

        let stale_idsts = self.regs.idsts().read();
        if stale_idsts.ais()
            || stale_idsts.nis()
            || stale_idsts.ces()
            || stale_idsts.du()
            || stale_idsts.fbe()
            || stale_idsts.ri()
            || stale_idsts.ti()
        {
            self.clear_idsts();
        }
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::StatusClear, phase_start);

        let xfer = xfer.unwrap();

        IDMAC_DONE_FLAG.store(false, Ordering::Release);
        IDMAC_ERROR_FLAG.store(false, Ordering::Release);

        let buf_len = match xfer {
            DataXfer::Read(buf) => buf.len(),
            DataXfer::Write(buf) => buf.len(),
        };

        assert!(buf_len != 0, "IDMAC transfer buffer must not be empty");
        assert!(
            buf_len <= DMA_BUFFER_SIZE,
            "IDMAC transfer exceeds the DMA bounce buffer: {buf_len}"
        );

        let descriptor_count = buf_len.div_ceil(IDMAC_DESCRIPTOR_BUFFER_SIZE);
        let layout = Layout::array::<IdmacDescriptor>(descriptor_count)
            .expect("Invalid IDMAC descriptor chain layout");
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        let dma_desc_info =
            unsafe { alloc_coherent(layout) }.expect("Failed to allocate DMA descriptor");
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::DescriptorAlloc, phase_start);
        let desc_ptr = dma_desc_info.cpu_addr.as_ptr() as *mut IdmacDescriptor;
        let desc_phy_addr = u32::try_from(dma_desc_info.bus_addr.as_u64())
            .expect("DMA descriptor address exceeds the IDMAC 32-bit address range");

        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        for index in 0..descriptor_count {
            let offset = index * IDMAC_DESCRIPTOR_BUFFER_SIZE;
            let segment_len = (buf_len - offset).min(IDMAC_DESCRIPTOR_BUFFER_SIZE);
            let last = index + 1 == descriptor_count;
            let buffer_addr = dma_bus_addr
                .checked_add(offset as u32)
                .expect("DMA data buffer crosses the IDMAC 32-bit address boundary");
            let next_descriptor_addr = if last {
                0
            } else {
                desc_phy_addr
                    .checked_add(((index + 1) * core::mem::size_of::<IdmacDescriptor>()) as u32)
                    .expect("DMA descriptor chain crosses the IDMAC 32-bit address boundary")
            };

            let mut descriptor = IdmacDescriptor::new();
            descriptor.set_desc0_control_descriptor(
                true,
                false,
                false,
                !last,
                index == 0,
                last,
                !last,
            );
            descriptor.set_des1_buffer1_size(segment_len as u16);
            descriptor.set_des2_buffer1_address(buffer_addr);
            descriptor.set_des3_next_descriptor_address(next_descriptor_addr);
            unsafe { core::ptr::write_volatile(desc_ptr.add(index), descriptor) };
        }

        dma_io_fence();
        self.regs.bytcnt().write(buf_len as u32);
        self.regs.dbaddr().write(desc_phy_addr);
        dma_io_fence();
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::DescriptorPublish, phase_start);

        let context = IdmacTransferContext {
            cmd,
            arg,
            generation: 0,
            dma_desc_info,
            layout,
            desc_ptr,
            descriptor_count,
        };
        let programmed_dbaddr = self.regs.dbaddr().read();
        let programmed_byte_count = self.regs.bytcnt().read();
        let owned_descriptors = Self::descriptor_owned_count(&context);
        if programmed_dbaddr != desc_phy_addr
            || programmed_byte_count != buf_len as u32
            || owned_descriptors != descriptor_count
        {
            warn!(
                "IDMAC descriptor publication failed before cmd={}: expected_DBADDR=0x{:08x}, \
                 actual_DBADDR=0x{:08x}, expected_BYTCNT={}, actual_BYTCNT={}, expected_OWN={}, \
                 actual_OWN={}",
                cmd.cmd_index(),
                desc_phy_addr,
                programmed_dbaddr,
                buf_len,
                programmed_byte_count,
                descriptor_count,
                owned_descriptors,
            );
            self.idmac_faulted = true;
            let _ = self.finish_idmac_transfer(context, true);
            return None;
        }

        Some(context)
    }

    fn start_idmac_transfer(
        &mut self,
        mut context: IdmacTransferContext,
    ) -> Option<IdmacTransferContext> {
        let cmd = context.cmd;
        context.generation = IDMAC_COMPLETION.begin_transfer();

        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        self.regs.cmdarg().write(context.arg);
        dma_io_fence();
        self.regs.cmd().write(cmd);
        dma_io_fence();
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::CommandIssue, phase_start);

        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        let mut start_cmd_wait_count = 0u64;
        let start_cmd_deadline = axhal::time::monotonic_time() + Duration::from_millis(100);
        while self.regs.cmd().read().start_cmd() {
            core::hint::spin_loop();
            start_cmd_wait_count += 1;
            if axhal::time::monotonic_time() >= start_cmd_deadline {
                warn!(
                    "send_cmd_idmac: start_cmd timeout after {} iterations; CMD={:?}, \
                     STATUS={:?}, RINTSTS={:?}, IDSTS={:?}, BMOD={:?}, CTRL={:?}, \
                     DBADDR=0x{:08x}, desc_own={}",
                    start_cmd_wait_count,
                    self.regs.cmd().read(),
                    self.regs.status().read(),
                    self.regs.rintsts().read(),
                    self.regs.idsts().read(),
                    self.regs.bmod().read(),
                    self.regs.ctrl().read(),
                    self.regs.dbaddr().read(),
                    Self::descriptor_owned(&context),
                );
                self.idmac_faulted = true;
                let _ = self.finish_idmac_transfer(context, true);
                return None;
            }
        }
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::CommandAccept, phase_start);

        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        let idsts_before_pldmnd = self.regs.idsts().read();
        if idsts_before_pldmnd.du() {
            warn!(
                "send_cmd_idmac: IDSTS indicates Descriptor Unavailable after CMD; resuming IDMAC"
            );
            self.clear_idsts();
            self.regs.pldmnd().write(1);
            dma_io_fence();
        }
        let idsts_after_pldmnd = self.regs.idsts().read();
        if idsts_after_pldmnd.du() {
            warn!(
                "send_cmd_idmac: IDSTS still indicates Descriptor Unavailable after PLDMND; \
                 disabling IDMAC path"
            );
            self.idmac_faulted = true;
            let _ = self.finish_idmac_transfer(context, true);
            return None;
        }
        if idsts_after_pldmnd.ais() || idsts_after_pldmnd.fbe() {
            warn!("send_cmd_idmac: IDMAC abnormal status after CMD+PLDMND; disabling IDMAC path");
            self.idmac_faulted = true;
            let _ = self.finish_idmac_transfer(context, true);
            return None;
        }

        let fsm = self.regs.idsts().read().fsm();
        let desc_own = Self::descriptor_owned(&context);
        let dbaddr = self.regs.dbaddr().read();
        if IDMAC_START_LOGGED.swap(true, Ordering::AcqRel) {
            debug!(
                "IDMAC DMA started: cmd={}, fsm={}, desc_own={}, DBADDR=0x{:08x}",
                cmd.cmd_index(),
                fsm,
                desc_own,
                dbaddr,
            );
        } else {
            warn!(
                "IDMAC DMA started: cmd={}, fsm={}, desc_own={}, DBADDR=0x{:08x}",
                cmd.cmd_index(),
                fsm,
                desc_own,
                dbaddr,
            );
        }
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::PostStartChecks, phase_start);

        Some(context)
    }

    fn idmac_completion_status(
        &self,
        generation: usize,
    ) -> (crate::regs::RIntSts, crate::regs::IdSts) {
        let current_rintsts = self.regs.rintsts().read().into_bits();
        let current_idsts = self.regs.idsts().read().into_bits();
        let (irq_rintsts, irq_idsts) = IDMAC_COMPLETION.snapshot_bits(generation).unwrap_or((0, 0));

        (
            crate::regs::RIntSts::from_bits(current_rintsts | irq_rintsts),
            crate::regs::IdSts::from_bits(current_idsts | irq_idsts),
        )
    }

    fn idmac_status_has_error(rintsts: &crate::regs::RIntSts, idsts: &crate::regs::IdSts) -> bool {
        IDMAC_ERROR_FLAG.load(Ordering::Acquire)
            || rintsts.error()
            || idsts.ais()
            || idsts.ces()
            || idsts.du()
            || idsts.fbe()
    }

    fn idmac_command_done_or_error(&self, context: &IdmacTransferContext) -> bool {
        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        rintsts.command_done() || Self::idmac_status_has_error(&rintsts, &idsts)
    }

    fn idmac_terminal_events_or_error(&self, context: &IdmacTransferContext) -> bool {
        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            return true;
        }

        let command_done = !context.cmd.response_expect() || rintsts.command_done();
        let auto_stop_done = !context.cmd.send_auto_stop() || rintsts.auto_command_done();
        let dma_done = if context.cmd.read_write() {
            idsts.ti()
        } else {
            idsts.ri()
        };
        command_done && auto_stop_done && dma_done && rintsts.data_transfer_over()
    }

    fn descriptor_owned_count(context: &IdmacTransferContext) -> usize {
        (0..context.descriptor_count)
            .filter(|&index| {
                let descriptor = unsafe { context.desc_ptr.add(index) };
                let des0 =
                    unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*descriptor).des0)) };
                des0.own()
            })
            .count()
    }

    fn descriptor_owned(context: &IdmacTransferContext) -> bool {
        Self::descriptor_owned_count(context) != 0
    }

    fn descriptor_card_error(context: &IdmacTransferContext) -> bool {
        (0..context.descriptor_count).any(|index| {
            let descriptor = unsafe { context.desc_ptr.add(index) };
            let des0 = unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*descriptor).des0)) };
            des0.ces()
        })
    }

    fn validate_idmac_terminal(&self, context: &IdmacTransferContext) -> bool {
        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        let has_error = Self::idmac_status_has_error(&rintsts, &idsts);
        let command_done = !context.cmd.response_expect() || rintsts.command_done();
        let auto_stop_done = !context.cmd.send_auto_stop() || rintsts.auto_command_done();
        let dma_done = if context.cmd.read_write() {
            idsts.ti()
        } else {
            idsts.ri()
        };
        let controller_done = rintsts.data_transfer_over();

        dma_io_fence();
        let descriptor_owned = Self::descriptor_owned(context);
        let descriptor_card_error = Self::descriptor_card_error(context);
        let complete = !has_error
            && command_done
            && auto_stop_done
            && dma_done
            && controller_done
            && !descriptor_owned
            && !descriptor_card_error;

        if !complete {
            warn!(
                "IDMAC terminal validation failed: cmd={}, RINTSTS={rintsts:?}, IDSTS={idsts:?}, \
                 command_done={command_done}, auto_stop_done={auto_stop_done}, \
                 dma_done={dma_done}, controller_done={controller_done}, \
                 desc_own={descriptor_owned}, desc_ces={descriptor_card_error}, descriptors={}",
                context.cmd.cmd_index(),
                context.descriptor_count,
            );
        }

        complete
    }

    fn wait_transfer_sync(&mut self, context: &IdmacTransferContext) -> Result<(), IdmacWaitError> {
        #[cfg(feature = "multi-block-test")]
        let phase_start = self.multi_block_phase_start();
        if context.cmd.response_expect() {
            let deadline = axhal::time::wall_time() + Duration::from_secs(2);
            while !self.idmac_command_done_or_error(context) {
                if axhal::time::wall_time() >= deadline {
                    #[cfg(feature = "multi-block-test")]
                    self.multi_block_phase_finish(MultiBlockPhase::CommandResponse, phase_start);
                    return Err(IdmacWaitError::CommandTimeout);
                }
                core::hint::spin_loop();
            }
        }
        #[cfg(feature = "multi-block-test")]
        self.multi_block_phase_finish(MultiBlockPhase::CommandResponse, phase_start);

        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            return Err(IdmacWaitError::Hardware);
        }

        #[cfg(feature = "multi-block-test")]
        let data_phase_start = self.multi_block_phase_start();
        #[cfg(feature = "multi-block-test")]
        let mut data_done_ns = None;
        let deadline = axhal::time::wall_time() + Duration::from_secs(5);
        loop {
            let terminal = self.idmac_terminal_events_or_error(context);
            #[cfg(feature = "multi-block-test")]
            if data_phase_start.is_some() && data_done_ns.is_none() {
                let (rintsts, idsts) = self.idmac_completion_status(context.generation);
                let dma_done = if context.cmd.read_write() {
                    idsts.ti()
                } else {
                    idsts.ri()
                };
                if dma_done && rintsts.data_transfer_over() {
                    data_done_ns = Some(axhal::time::monotonic_time_nanos());
                }
            }
            if terminal {
                break;
            }
            if axhal::time::wall_time() >= deadline {
                #[cfg(feature = "multi-block-test")]
                {
                    let terminal_ns = axhal::time::monotonic_time_nanos();
                    let started_ns = data_phase_start.unwrap_or(terminal_ns);
                    self.multi_block_profiler.add(
                        MultiBlockPhase::DataTransfer,
                        data_done_ns
                            .unwrap_or(terminal_ns)
                            .saturating_sub(started_ns),
                    );
                    self.multi_block_profiler.add(
                        MultiBlockPhase::AutoStopTail,
                        terminal_ns.saturating_sub(data_done_ns.unwrap_or(terminal_ns)),
                    );
                }
                return Err(IdmacWaitError::DataTimeout);
            }
            core::hint::spin_loop();
        }
        #[cfg(feature = "multi-block-test")]
        {
            let terminal_ns = axhal::time::monotonic_time_nanos();
            let started_ns = data_phase_start.unwrap_or(terminal_ns);
            let data_done_ns = data_done_ns.unwrap_or(terminal_ns);
            self.multi_block_profiler.add(
                MultiBlockPhase::DataTransfer,
                data_done_ns.saturating_sub(started_ns),
            );
            self.multi_block_profiler.add(
                MultiBlockPhase::AutoStopTail,
                terminal_ns.saturating_sub(data_done_ns),
            );
        }

        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            Err(IdmacWaitError::Hardware)
        } else {
            Ok(())
        }
    }

    async fn wait_transfer_async(
        &self,
        context: &IdmacTransferContext,
    ) -> Result<(), IdmacWaitError> {
        if context.cmd.response_expect() {
            let command_timed_out = IDMAC_WAIT_QUEUE
                .wait_timeout_until_async(Duration::from_secs(2), || {
                    self.idmac_command_done_or_error(context)
                })
                .await;
            if command_timed_out {
                return Err(IdmacWaitError::CommandTimeout);
            }
        }

        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            return Err(IdmacWaitError::Hardware);
        }

        let data_timed_out = IDMAC_WAIT_QUEUE
            .wait_timeout_until_async(Duration::from_secs(5), || {
                self.idmac_terminal_events_or_error(context)
            })
            .await;
        if data_timed_out {
            return Err(IdmacWaitError::DataTimeout);
        }

        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            Err(IdmacWaitError::Hardware)
        } else {
            Ok(())
        }
    }

    async fn send_cmd_idmac_async(
        &mut self,
        command: Command<'_>,
        dma_bus_addr: u32,
    ) -> Option<[u32; 4]> {
        let context = self.prepare_idmac_transfer(command, dma_bus_addr)?;
        let context = self.start_idmac_transfer(context)?;
        let mut transfer = ActiveIdmacTransfer::new(self, context);

        trace!(
            "send_cmd_idmac_async: Async DMA transfer started for command index {}",
            transfer.context().cmd.cmd_index()
        );

        if let Err(error) = transfer.wait_async().await {
            warn!(
                "send_cmd_idmac_async: transfer failed for command index {}: {:?}",
                transfer.context().cmd.cmd_index(),
                error,
            );
            transfer.fault();
            let _ = transfer.finish(true);
            return None;
        }

        if !transfer.validate() {
            transfer.fault();
            let _ = transfer.finish(true);
            return None;
        }

        let resp = transfer.response();
        if !transfer.finish(false) {
            return None;
        }
        Some(resp)
    }

    fn send_cmd_idmac(&mut self, command: Command<'_>, dma_bus_addr: u32) -> Option<[u32; 4]> {
        let context = self.prepare_idmac_transfer(command, dma_bus_addr)?;
        let context = self.start_idmac_transfer(context)?;
        let mut transfer = ActiveIdmacTransfer::new(self, context);

        if let Err(error) = transfer.wait_sync() {
            warn!(
                "send_cmd_idmac: transfer failed for command index {}: {:?}",
                transfer.context().cmd.cmd_index(),
                error,
            );
            transfer.fault();
            let _ = transfer.finish(true);
            return None;
        }

        if !transfer.validate() {
            transfer.fault();
            let _ = transfer.finish(true);
            return None;
        }

        let resp = transfer.response();
        if !transfer.finish(false) {
            return None;
        }
        Some(resp)
    }

    /// The interrupt handler for the IDMAC DMA transfer completion.
    pub fn dma_irq_handler() {
        let regs_base = SDMMC_REGS_BASE.load(Ordering::Acquire);
        let mut should_notify = false;
        if regs_base != 0 {
            let regs = unsafe { VolatilePtr::new(NonNull::new_unchecked(regs_base as *mut _)) };
            let rintsts = regs.rintsts().read();
            let idsts = regs.idsts().read();
            let has_rintsts = rintsts.sdio() != 0
                || rintsts.end_bit_error()
                || rintsts.auto_command_done()
                || rintsts.start_bit_error()
                || rintsts.hardware_locked_write()
                || rintsts.fifo_under_over_run()
                || rintsts.host_timeout()
                || rintsts.data_read_timeout()
                || rintsts.response_timeout()
                || rintsts.data_crc_error()
                || rintsts.response_crc_error()
                || rintsts.receive_fifo_data_request()
                || rintsts.transmit_fifo_data_request()
                || rintsts.data_transfer_over()
                || rintsts.command_done()
                || rintsts.response_error()
                || rintsts.card_detect();
            let has_idsts = idsts.ais()
                || idsts.nis()
                || idsts.ces()
                || idsts.du()
                || idsts.fbe()
                || idsts.ri()
                || idsts.ti();
            let idmac_error =
                idsts.ais() || idsts.ces() || idsts.du() || idsts.fbe() || rintsts.error();
            let transfer_done = idsts.ri() || idsts.ti() || rintsts.data_transfer_over();
            let transfer_event = transfer_done || rintsts.auto_command_done();

            if idmac_error {
                IDMAC_ERROR_FLAG.store(true, Ordering::Release);
                log::error!("SDMMC DMA error: RINTSTS={:?}, IDSTS={:?}", rintsts, idsts);
            }

            if has_idsts {
                regs.idsts().write(idsts);
            }

            if transfer_event
                || idmac_error
                || rintsts.receive_fifo_data_request()
                || rintsts.transmit_fifo_data_request()
            {
                let mut clear_rintsts = crate::regs::RIntSts::new();
                clear_rintsts = clear_rintsts
                    .with_end_bit_error(rintsts.end_bit_error())
                    .with_auto_command_done(rintsts.auto_command_done())
                    .with_start_bit_error(rintsts.start_bit_error())
                    .with_hardware_locked_write(rintsts.hardware_locked_write())
                    .with_fifo_under_over_run(rintsts.fifo_under_over_run())
                    .with_host_timeout(rintsts.host_timeout())
                    .with_data_read_timeout(rintsts.data_read_timeout())
                    .with_response_timeout(rintsts.response_timeout())
                    .with_data_crc_error(rintsts.data_crc_error())
                    .with_response_crc_error(rintsts.response_crc_error())
                    .with_response_error(rintsts.response_error())
                    .with_data_transfer_over(rintsts.data_transfer_over())
                    .with_receive_fifo_data_request(rintsts.receive_fifo_data_request())
                    .with_transmit_fifo_data_request(rintsts.transmit_fifo_data_request());
                regs.rintsts().write(clear_rintsts);
            }

            IDMAC_COMPLETION.record_irq(rintsts, idsts);
            should_notify = transfer_event || idmac_error;

            if !has_rintsts && !has_idsts {
                debug!(
                    "SDMMC IRQ without pending status: RINTSTS={:?}, IDSTS={:?}",
                    rintsts, idsts
                );
            }
        } else {
            warn!("SdMmc::dma_irq_handler: no SDMMC register base available to clear IDSTS");
        }

        if should_notify {
            IDMAC_DONE_FLAG.store(true, Ordering::Release);
            IDMAC_WAIT_QUEUE.notify_one(false);
        }
    }

    /// The size of a block in bytes.
    pub const BLOCK_SIZE: usize = 512;
}

impl Drop for SdMmc {
    fn drop(&mut self) {
        if self.idmac_reset_failed {
            warn!("retaining the DMA buffer because IDMAC reset did not complete");
            return;
        }

        if let Some(dma_buf) = &self.dma_buffer {
            info!(
                "Deallocating DMA buffer: virt=0x{:08x}, phys=0x{:08x}, size={}",
                dma_buf.addr.cpu_addr.as_ptr() as u64,
                dma_buf.addr.bus_addr.as_u64(),
                dma_buf.size
            );
            let layout = Layout::from_size_align(dma_buf.size, self.ahb_data_width.align_value())
                .expect("Invalid layout for DMA buffer");
            unsafe {
                dealloc_coherent(dma_buf.addr, layout);
            }
        }
    }
}

// SAFETY: all externally reachable methods that mutate controller state require
// exclusive `&mut self`; the IRQ handler only accesses MMIO and atomic snapshots.
unsafe impl Send for SdMmc {}
unsafe impl Sync for SdMmc {}
