use core::{
    alloc::Layout,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering},
    time::Duration,
};

#[cfg(feature = "one-block-test")]
use alloc::vec;
use axtask::WaitQueue;
use log::{debug, info, trace, warn};
use volatile::VolatilePtr;

use crate::{
    cmd::{Command, DataXfer},
    dma::{DMABuffer, DMAInfo, IdmacDescriptor, alloc_coherent, dealloc_coherent},
    regs::{ClkDiv, ClkEna, RegisterBlock, RegisterBlockVolatileFieldAccess},
    utils::{Cid, CsdV2},
};

// Existing production assumption. The diagnostic build uses the measured JH7110 clock tree.
#[cfg(not(feature = "one-block-test"))]
const VISIONFIVE2_SDIO_CIU_CLOCK_HZ: u32 = 100_000_000;
#[cfg(feature = "one-block-test")]
const VISIONFIVE2_SDIO_CIU_CLOCK_HZ: u32 = 49_500_000;
const IDENTIFICATION_CLOCK_DIVIDER: u8 = 100;
#[cfg(not(feature = "one-block-test"))]
const DEFAULT_SPEED_CLOCK_DIVIDER: u8 = 2;
#[cfg(feature = "one-block-test")]
const DEFAULT_SPEED_CLOCK_DIVIDER: u8 = 1;

#[cfg(feature = "one-block-test")]
const ONE_BLOCK_TEST_START_LBA: u32 = 2_099_200;
#[cfg(feature = "one-block-test")]
const ONE_BLOCK_TEST_BLOCKS: usize = 256;
#[cfg(feature = "one-block-test")]
const ONE_BLOCK_TEST_ROUNDS: usize = 5;
#[cfg(feature = "one-block-test")]
const ONE_BLOCK_TIMER_SAMPLES: usize = 4_096;
#[cfg(feature = "one-block-test")]
const ONE_BLOCK_TRANSFER_BYTES: u32 = 512;

#[cfg(feature = "one-block-test")]
const JH7110_SYSCRG_BASE_PADDR: usize = 0x1302_0000;
#[cfg(feature = "one-block-test")]
const JH7110_SYS_SYSCON_BASE_PADDR: usize = 0x1303_0000;
#[cfg(feature = "one-block-test")]
const JH7110_OSC_CLOCK_HZ: u64 = 24_000_000;
#[cfg(feature = "one-block-test")]
const JH7110_SYSCLK_BUS_ROOT: usize = 5;
#[cfg(feature = "one-block-test")]
const JH7110_SYSCLK_AXI_CFG0: usize = 7;
#[cfg(feature = "one-block-test")]
const JH7110_SYSCLK_SDIO1_SDCARD: usize = 94;

#[cfg(feature = "one-block-test")]
#[derive(Clone, Copy)]
enum OneBlockOperation {
    Read,
    Write,
}

#[cfg(feature = "one-block-test")]
impl OneBlockOperation {
    const fn name(self) -> &'static str {
        match self {
            Self::Read => "read",
            Self::Write => "write",
        }
    }
}

#[cfg(feature = "one-block-test")]
#[derive(Clone, Copy)]
#[repr(usize)]
enum OneBlockPhase {
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
    TerminalValidate,
    FinishCleanup,
    CardBusy,
    Unaccounted,
}

#[cfg(feature = "one-block-test")]
impl OneBlockPhase {
    const COUNT: usize = 16;
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
            Self::TerminalValidate => "terminal_validate",
            Self::FinishCleanup => "finish_cleanup",
            Self::CardBusy => "card_busy",
            Self::Unaccounted => "unaccounted",
        }
    }
}

#[cfg(feature = "one-block-test")]
#[derive(Clone, Copy)]
struct OneBlockPhaseStats {
    count: u64,
    total_ns: u64,
    min_ns: u64,
    max_ns: u64,
}

#[cfg(feature = "one-block-test")]
impl OneBlockPhaseStats {
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

    fn merge(&mut self, other: Self) {
        if other.count == 0 {
            return;
        }
        self.count = self.count.saturating_add(other.count);
        self.total_ns = self.total_ns.saturating_add(other.total_ns);
        self.min_ns = self.min_ns.min(other.min_ns);
        self.max_ns = self.max_ns.max(other.max_ns);
    }

    fn average_ns(self) -> u64 {
        if self.count == 0 {
            0
        } else {
            self.total_ns / self.count
        }
    }
}

#[cfg(feature = "one-block-test")]
#[derive(Clone, Copy)]
struct OneBlockTcbcntSample {
    in_transfer_readable: bool,
    response_start_ns: u64,
    initial_count: u32,
    first_nonzero_count: Option<u32>,
    first_nonzero_ns: Option<u64>,
    reached_block_count: Option<u32>,
    reached_block_ns: Option<u64>,
    terminal_count: u32,
    poll_count: u64,
    transition_count: u64,
    max_step_bytes: u32,
}

#[cfg(feature = "one-block-test")]
impl OneBlockTcbcntSample {
    fn new(
        in_transfer_readable: bool,
        response_start_ns: u64,
        initial_observed_ns: u64,
        initial_count: u32,
    ) -> Self {
        let mut sample = Self {
            in_transfer_readable,
            response_start_ns,
            initial_count,
            first_nonzero_count: None,
            first_nonzero_ns: None,
            reached_block_count: None,
            reached_block_ns: None,
            terminal_count: initial_count,
            poll_count: 1,
            transition_count: 0,
            max_step_bytes: 0,
        };
        sample.record_observation(initial_observed_ns, initial_count);
        sample
    }

    fn record_observation(&mut self, observed_ns: u64, count: u32) {
        if count != self.terminal_count {
            self.transition_count = self.transition_count.saturating_add(1);
            self.max_step_bytes = self
                .max_step_bytes
                .max(count.saturating_sub(self.terminal_count));
        }
        if self.first_nonzero_count.is_none() && count != 0 {
            self.first_nonzero_count = Some(count);
            self.first_nonzero_ns = Some(observed_ns);
        }
        if self.reached_block_count.is_none() && count >= ONE_BLOCK_TRANSFER_BYTES {
            self.reached_block_count = Some(count);
            self.reached_block_ns = Some(observed_ns);
        }
        self.terminal_count = count;
    }

    fn observe(&mut self, observed_ns: u64, count: u32) {
        self.poll_count = self.poll_count.saturating_add(1);
        self.record_observation(observed_ns, count);
    }
}

#[cfg(feature = "one-block-test")]
#[derive(Clone, Copy)]
struct OneBlockTcbcntStats {
    requests: u64,
    in_transfer_readable_requests: u64,
    reached_block_requests: u64,
    slope_samples: u64,
    all_zero_requests: u64,
    initial_count: OneBlockPhaseStats,
    first_nonzero_count: OneBlockPhaseStats,
    reached_block_count: OneBlockPhaseStats,
    terminal_count: OneBlockPhaseStats,
    poll_count: OneBlockPhaseStats,
    transition_count: OneBlockPhaseStats,
    max_step_bytes: OneBlockPhaseStats,
    bus_width_bits: OneBlockPhaseStats,
    response_to_block_ns: OneBlockPhaseStats,
    observed_payload_ns: OneBlockPhaseStats,
    estimated_sdclk_hz: OneBlockPhaseStats,
}

#[cfg(feature = "one-block-test")]
impl OneBlockTcbcntStats {
    const fn new() -> Self {
        Self {
            requests: 0,
            in_transfer_readable_requests: 0,
            reached_block_requests: 0,
            slope_samples: 0,
            all_zero_requests: 0,
            initial_count: OneBlockPhaseStats::new(),
            first_nonzero_count: OneBlockPhaseStats::new(),
            reached_block_count: OneBlockPhaseStats::new(),
            terminal_count: OneBlockPhaseStats::new(),
            poll_count: OneBlockPhaseStats::new(),
            transition_count: OneBlockPhaseStats::new(),
            max_step_bytes: OneBlockPhaseStats::new(),
            bus_width_bits: OneBlockPhaseStats::new(),
            response_to_block_ns: OneBlockPhaseStats::new(),
            observed_payload_ns: OneBlockPhaseStats::new(),
            estimated_sdclk_hz: OneBlockPhaseStats::new(),
        }
    }

    fn record(&mut self, sample: OneBlockTcbcntSample, bus_width_bits: u64) {
        self.requests = self.requests.saturating_add(1);
        if sample.in_transfer_readable {
            self.in_transfer_readable_requests =
                self.in_transfer_readable_requests.saturating_add(1);
        }
        self.initial_count.record(sample.initial_count as u64);
        self.terminal_count.record(sample.terminal_count as u64);
        self.poll_count.record(sample.poll_count);
        self.transition_count.record(sample.transition_count);
        self.max_step_bytes.record(sample.max_step_bytes as u64);
        self.bus_width_bits.record(bus_width_bits);

        let Some(first_count) = sample.first_nonzero_count else {
            self.all_zero_requests = self.all_zero_requests.saturating_add(1);
            return;
        };
        self.first_nonzero_count.record(first_count as u64);

        let (Some(first_ns), Some(reached_count), Some(reached_ns)) = (
            sample.first_nonzero_ns,
            sample.reached_block_count,
            sample.reached_block_ns,
        ) else {
            return;
        };
        self.reached_block_requests = self.reached_block_requests.saturating_add(1);
        self.reached_block_count.record(reached_count as u64);
        self.response_to_block_ns
            .record(reached_ns.saturating_sub(sample.response_start_ns));

        let elapsed_ns = reached_ns.saturating_sub(first_ns);
        let transferred_bytes = reached_count.saturating_sub(first_count) as u64;
        if !sample.in_transfer_readable
            || elapsed_ns == 0
            || transferred_bytes == 0
            || bus_width_bits == 0
        {
            return;
        }

        self.slope_samples = self.slope_samples.saturating_add(1);
        self.observed_payload_ns.record(elapsed_ns);
        let sdclk_hz = ((transferred_bytes as u128 * 8 * 1_000_000_000u128)
            / (elapsed_ns as u128 * bus_width_bits as u128))
            .min(u64::MAX as u128) as u64;
        self.estimated_sdclk_hz.record(sdclk_hz);
    }

    fn merge(&mut self, other: Self) {
        self.requests = self.requests.saturating_add(other.requests);
        self.in_transfer_readable_requests = self
            .in_transfer_readable_requests
            .saturating_add(other.in_transfer_readable_requests);
        self.reached_block_requests = self
            .reached_block_requests
            .saturating_add(other.reached_block_requests);
        self.slope_samples = self.slope_samples.saturating_add(other.slope_samples);
        self.all_zero_requests = self
            .all_zero_requests
            .saturating_add(other.all_zero_requests);
        self.initial_count.merge(other.initial_count);
        self.first_nonzero_count.merge(other.first_nonzero_count);
        self.reached_block_count.merge(other.reached_block_count);
        self.terminal_count.merge(other.terminal_count);
        self.poll_count.merge(other.poll_count);
        self.transition_count.merge(other.transition_count);
        self.max_step_bytes.merge(other.max_step_bytes);
        self.bus_width_bits.merge(other.bus_width_bits);
        self.response_to_block_ns
            .merge(other.response_to_block_ns);
        self.observed_payload_ns.merge(other.observed_payload_ns);
        self.estimated_sdclk_hz
            .merge(other.estimated_sdclk_hz);
    }
}

#[cfg(feature = "one-block-test")]
#[derive(Clone, Copy)]
struct OneBlockProfileStats {
    phases: [OneBlockPhaseStats; OneBlockPhase::COUNT],
    tcbcnt: OneBlockTcbcntStats,
}

#[cfg(feature = "one-block-test")]
impl OneBlockProfileStats {
    const fn new() -> Self {
        Self {
            phases: [OneBlockPhaseStats::new(); OneBlockPhase::COUNT],
            tcbcnt: OneBlockTcbcntStats::new(),
        }
    }

    fn record(&mut self, sample: &[u64; OneBlockPhase::COUNT]) {
        for phase in OneBlockPhase::ALL {
            self.phases[phase as usize].record(sample[phase as usize]);
        }
    }

    fn merge(&mut self, other: Self) {
        for phase in OneBlockPhase::ALL {
            self.phases[phase as usize].merge(other.phases[phase as usize]);
        }
        self.tcbcnt.merge(other.tcbcnt);
    }
}

#[cfg(feature = "one-block-test")]
struct OneBlockProfiler {
    enabled: bool,
    operation: OneBlockOperation,
    request_start_ns: u64,
    current: [u64; OneBlockPhase::COUNT],
    read: OneBlockProfileStats,
    write: OneBlockProfileStats,
}

#[cfg(feature = "one-block-test")]
impl OneBlockProfiler {
    const fn new() -> Self {
        Self {
            enabled: false,
            operation: OneBlockOperation::Read,
            request_start_ns: 0,
            current: [0; OneBlockPhase::COUNT],
            read: OneBlockProfileStats::new(),
            write: OneBlockProfileStats::new(),
        }
    }

    fn begin(&mut self, operation: OneBlockOperation) {
        if !self.enabled {
            return;
        }
        self.operation = operation;
        self.current.fill(0);
        self.request_start_ns = axhal::time::monotonic_time_nanos();
    }

    fn add(&mut self, phase: OneBlockPhase, duration_ns: u64) {
        if self.enabled {
            let value = &mut self.current[phase as usize];
            *value = value.saturating_add(duration_ns);
        }
    }

    fn record_tcbcnt(&mut self, sample: OneBlockTcbcntSample, bus_width_bits: u64) {
        if self.enabled {
            self.write.tcbcnt.record(sample, bus_width_bits);
        }
    }

    fn finish(&mut self) {
        if !self.enabled {
            return;
        }
        let total_ns = axhal::time::monotonic_time_nanos()
            .saturating_sub(self.request_start_ns)
            .max(1);
        self.current[OneBlockPhase::Total as usize] = total_ns;

        let accounted_ns = OneBlockPhase::ALL
            .iter()
            .copied()
            .filter(|phase| {
                !matches!(phase, OneBlockPhase::Total | OneBlockPhase::Unaccounted)
            })
            .map(|phase| self.current[phase as usize] as u128)
            .sum::<u128>();
        self.current[OneBlockPhase::Unaccounted as usize] =
            total_ns.saturating_sub(accounted_ns.min(u64::MAX as u128) as u64);

        match self.operation {
            OneBlockOperation::Read => self.read.record(&self.current),
            OneBlockOperation::Write => self.write.record(&self.current),
        }
    }

    fn take(&mut self, operation: OneBlockOperation) -> OneBlockProfileStats {
        match operation {
            OneBlockOperation::Read => {
                let stats = self.read;
                self.read = OneBlockProfileStats::new();
                stats
            }
            OneBlockOperation::Write => {
                let stats = self.write;
                self.write = OneBlockProfileStats::new();
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

    #[cfg(feature = "one-block-test")]
    one_block_profiler: OneBlockProfiler,
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
        #[cfg(feature = "one-block-test")]
        let phase_start = self.sdmmc.one_block_phase_start();
        let context = self.context.as_ref().unwrap();
        let complete = self.sdmmc.validate_idmac_terminal(context);
        #[cfg(feature = "one-block-test")]
        self.sdmmc
            .one_block_phase_finish(OneBlockPhase::TerminalValidate, phase_start);
        complete
    }

    fn response(&self) -> [u32; 4] {
        self.sdmmc.regs.resp().read()
    }

    fn fault(&mut self) {
        self.sdmmc.idmac_faulted = true;
    }

    fn finish(mut self, recover: bool) -> bool {
        let context = self.context.take().unwrap();
        #[cfg(feature = "one-block-test")]
        let phase_start = self.sdmmc.one_block_phase_start();
        let complete = self.sdmmc.finish_idmac_transfer(context, recover);
        #[cfg(feature = "one-block-test")]
        self.sdmmc
            .one_block_phase_finish(OneBlockPhase::FinishCleanup, phase_start);
        complete
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
            #[cfg(feature = "one-block-test")]
            one_block_profiler: OneBlockProfiler::new(),
        };
        this.init();
        this.try_enable_idmac(512, AHBDataWidth::Bits32, register_irq);

        #[cfg(feature = "one-block-test")]
        this.run_one_block_test();

        this
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_phase_start(&self) -> Option<u64> {
        self.one_block_profiler
            .enabled
            .then(axhal::time::monotonic_time_nanos)
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_phase_finish(&mut self, phase: OneBlockPhase, started_ns: Option<u64>) {
        if let Some(started_ns) = started_ns {
            self.one_block_profiler.add(
                phase,
                axhal::time::monotonic_time_nanos().saturating_sub(started_ns),
            );
        }
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_wait_card_idle(&mut self) {
        if !self.one_block_profiler.enabled {
            return;
        }

        let phase_start = self.one_block_phase_start();
        let deadline = axhal::time::monotonic_time() + Duration::from_secs(1);
        while !self.can_send_data() {
            if axhal::time::monotonic_time() >= deadline {
                self.one_block_phase_finish(OneBlockPhase::CardBusy, phase_start);
                self.idmac_faulted = true;
                panic!("one-block-test: card stayed busy for more than one second after CMD24");
            }
            core::hint::spin_loop();
        }
        self.one_block_phase_finish(OneBlockPhase::CardBusy, phase_start);
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_mib_per_sec_milli(bytes: u64, elapsed_ns: u64) -> u64 {
        if elapsed_ns == 0 {
            return 0;
        }
        ((bytes as u128 * 1_000_000_000u128 * 1_000u128)
            / (elapsed_ns as u128 * 1_048_576u128))
            .min(u64::MAX as u128) as u64
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_print_round(
        operation: OneBlockOperation,
        round: usize,
        stats: OneBlockProfileStats,
        validation_ok: bool,
    ) {
        let total = stats.phases[OneBlockPhase::Total as usize];
        let bytes = total.count.saturating_mul(Self::BLOCK_SIZE as u64);
        let throughput = Self::one_block_mib_per_sec_milli(bytes, total.total_ns);
        warn!(
            "ONE_BLOCK_ROUND operation={} round={} requests={} bytes={} elapsed_ns={} \
             average_request_ns={} throughput_mib_s={}.{:03} validation={}",
            operation.name(),
            round,
            total.count,
            bytes,
            total.total_ns,
            total.average_ns(),
            throughput / 1_000,
            throughput % 1_000,
            if validation_ok { "ok" } else { "failed" },
        );
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_print_profile(operation: OneBlockOperation, stats: OneBlockProfileStats) {
        let request_total_ns = stats.phases[OneBlockPhase::Total as usize].total_ns;
        for phase in OneBlockPhase::ALL {
            let phase_stats = stats.phases[phase as usize];
            let percentage_x100 = if request_total_ns == 0 {
                0
            } else {
                ((phase_stats.total_ns as u128 * 10_000u128) / request_total_ns as u128)
                    .min(u64::MAX as u128) as u64
            };
            let min_ns = if phase_stats.count == 0 {
                0
            } else {
                phase_stats.min_ns
            };
            warn!(
                "ONE_BLOCK_PHASE operation={} phase={} count={} total_ns={} average_ns={} \
                 min_ns={} max_ns={} request_total_percent={}.{:02}",
                operation.name(),
                phase.name(),
                phase_stats.count,
                phase_stats.total_ns,
                phase_stats.average_ns(),
                min_ns,
                phase_stats.max_ns,
                percentage_x100 / 100,
                percentage_x100 % 100,
            );
        }

        if matches!(operation, OneBlockOperation::Write) {
            Self::one_block_print_tcbcnt(stats.tcbcnt);
        }
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_stats_min(stats: OneBlockPhaseStats) -> u64 {
        if stats.count == 0 { 0 } else { stats.min_ns }
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_print_tcbcnt(stats: OneBlockTcbcntStats) {
        warn!(
            "ONE_BLOCK_TCBCNT_SUMMARY command=CMD24 requests={} in_transfer_readable={} \
             reached_512={} slope_samples={} all_zero={} bus_width_bits_avg={} \
             bus_width_bits_min={} bus_width_bits_max={}",
            stats.requests,
            stats.in_transfer_readable_requests,
            stats.reached_block_requests,
            stats.slope_samples,
            stats.all_zero_requests,
            stats.bus_width_bits.average_ns(),
            Self::one_block_stats_min(stats.bus_width_bits),
            stats.bus_width_bits.max_ns,
        );
        warn!(
            "ONE_BLOCK_TCBCNT_COUNTS initial_avg={} initial_min={} initial_max={} \
             first_nonzero_avg={} first_nonzero_min={} first_nonzero_max={} \
             reached_avg={} reached_min={} reached_max={} terminal_avg={} \
             terminal_min={} terminal_max={}",
            stats.initial_count.average_ns(),
            Self::one_block_stats_min(stats.initial_count),
            stats.initial_count.max_ns,
            stats.first_nonzero_count.average_ns(),
            Self::one_block_stats_min(stats.first_nonzero_count),
            stats.first_nonzero_count.max_ns,
            stats.reached_block_count.average_ns(),
            Self::one_block_stats_min(stats.reached_block_count),
            stats.reached_block_count.max_ns,
            stats.terminal_count.average_ns(),
            Self::one_block_stats_min(stats.terminal_count),
            stats.terminal_count.max_ns,
        );
        warn!(
            "ONE_BLOCK_TCBCNT_SAMPLING samples_avg={} samples_min={} samples_max={} \
             transitions_avg={} transitions_min={} transitions_max={} max_step_bytes_avg={} \
             max_step_bytes_min={} max_step_bytes_max={}",
            stats.poll_count.average_ns(),
            Self::one_block_stats_min(stats.poll_count),
            stats.poll_count.max_ns,
            stats.transition_count.average_ns(),
            Self::one_block_stats_min(stats.transition_count),
            stats.transition_count.max_ns,
            stats.max_step_bytes.average_ns(),
            Self::one_block_stats_min(stats.max_step_bytes),
            stats.max_step_bytes.max_ns,
        );
        warn!(
            "ONE_BLOCK_TCBCNT_TIMING response_to_512_avg_ns={} response_to_512_min_ns={} \
             response_to_512_max_ns={} observed_payload_avg_ns={} observed_payload_min_ns={} \
             observed_payload_max_ns={} estimated_sdclk_avg_hz={} estimated_sdclk_min_hz={} \
             estimated_sdclk_max_hz={} method=endpoint_slope",
            stats.response_to_block_ns.average_ns(),
            Self::one_block_stats_min(stats.response_to_block_ns),
            stats.response_to_block_ns.max_ns,
            stats.observed_payload_ns.average_ns(),
            Self::one_block_stats_min(stats.observed_payload_ns),
            stats.observed_payload_ns.max_ns,
            stats.estimated_sdclk_hz.average_ns(),
            Self::one_block_stats_min(stats.estimated_sdclk_hz),
            stats.estimated_sdclk_hz.max_ns,
        );
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_bus_width_bits(&self) -> u64 {
        let ctype = self.regs.ctype().read().into_bits();
        if ctype & (1 << 16) != 0 {
            8
        } else if ctype & 1 != 0 {
            4
        } else {
            1
        }
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_tcbcnt_begin(
        &self,
        context: &IdmacTransferContext,
    ) -> Option<OneBlockTcbcntSample> {
        if !self.one_block_profiler.enabled || context.cmd.cmd_index() != 24 {
            return None;
        }

        let in_transfer_readable = !self.regs.hcon().read().area_opt();
        let response_start_ns = axhal::time::monotonic_time_nanos();
        let initial_count = self.regs.tcbcnt().read();
        let initial_observed_ns = axhal::time::monotonic_time_nanos();
        Some(OneBlockTcbcntSample::new(
            in_transfer_readable,
            response_start_ns,
            initial_observed_ns,
            initial_count,
        ))
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_tcbcnt_observe(&self, sample: &mut Option<OneBlockTcbcntSample>) {
        let Some(sample) = sample.as_mut() else {
            return;
        };
        if !sample.in_transfer_readable || sample.reached_block_ns.is_some() {
            return;
        }

        let count = self.regs.tcbcnt().read();
        let observed_ns = axhal::time::monotonic_time_nanos();
        sample.observe(observed_ns, count);
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_tcbcnt_finish(&mut self, sample: &mut Option<OneBlockTcbcntSample>) {
        let Some(mut sample) = sample.take() else {
            return;
        };

        let terminal_count = self.regs.tcbcnt().read();
        let terminal_observed_ns = axhal::time::monotonic_time_nanos();
        sample.observe(terminal_observed_ns, terminal_count);
        let bus_width_bits = self.one_block_bus_width_bits();
        self.one_block_profiler
            .record_tcbcnt(sample, bus_width_bits);
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_mmio_read_u32(paddr: usize) -> u32 {
        let vaddr = axhal::mem::phys_to_virt(paddr.into()).as_usize();
        // The platform MMIO map contains both SYSCRG and SYS_SYSCON.
        unsafe { core::ptr::read_volatile(vaddr as *const u32) }
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_pll2_rate_hz(cfg0: u32, cfg1: u32, cfg2: u32) -> Option<u64> {
        let dacpd = (cfg0 >> 15) & 1;
        let dsmpd = (cfg0 >> 16) & 1;
        let fbdiv = (cfg0 >> 17) & 0x0fff;
        let frac = cfg1 & 0x00ff_ffff;
        let postdiv1 = (cfg1 >> 28) & 0x3;
        let prediv = cfg2 & 0x3f;
        if prediv == 0 || dacpd != dsmpd {
            return None;
        }

        let effective_frac = if dacpd == 0 { frac } else { 0 };
        let feedback_x_2_24 = ((fbdiv as u128) << 24) | effective_frac as u128;
        let denominator = (prediv as u128) * (1u128 << postdiv1) * (1u128 << 24);
        Some(
            ((JH7110_OSC_CLOCK_HZ as u128 * feedback_x_2_24) / denominator)
                .min(u64::MAX as u128) as u64,
        )
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_print_clock_tree(&self) {
        let bus_root_addr = JH7110_SYSCRG_BASE_PADDR + 4 * JH7110_SYSCLK_BUS_ROOT;
        let axi_cfg0_addr = JH7110_SYSCRG_BASE_PADDR + 4 * JH7110_SYSCLK_AXI_CFG0;
        let sdio1_addr = JH7110_SYSCRG_BASE_PADDR + 4 * JH7110_SYSCLK_SDIO1_SDCARD;
        let pll2_cfg0_addr = JH7110_SYS_SYSCON_BASE_PADDR + 0x2c;
        let pll2_cfg1_addr = JH7110_SYS_SYSCON_BASE_PADDR + 0x30;
        let pll2_cfg2_addr = JH7110_SYS_SYSCON_BASE_PADDR + 0x34;

        let bus_root = Self::one_block_mmio_read_u32(bus_root_addr);
        let axi_cfg0 = Self::one_block_mmio_read_u32(axi_cfg0_addr);
        let sdio1_sdcard = Self::one_block_mmio_read_u32(sdio1_addr);
        let pll2_cfg0 = Self::one_block_mmio_read_u32(pll2_cfg0_addr);
        let pll2_cfg1 = Self::one_block_mmio_read_u32(pll2_cfg1_addr);
        let pll2_cfg2 = Self::one_block_mmio_read_u32(pll2_cfg2_addr);

        warn!(
            "ONE_BLOCK_CLOCK_RAW BUS_ROOT[0x{:08x}]=0x{:08x} \
             AXI_CFG0[0x{:08x}]=0x{:08x} SDIO1_SDCARD[0x{:08x}]=0x{:08x} \
             PLL2_CFG0[0x{:08x}]=0x{:08x} PLL2_CFG1[0x{:08x}]=0x{:08x} \
             PLL2_CFG2[0x{:08x}]=0x{:08x}",
            bus_root_addr,
            bus_root,
            axi_cfg0_addr,
            axi_cfg0,
            sdio1_addr,
            sdio1_sdcard,
            pll2_cfg0_addr,
            pll2_cfg0,
            pll2_cfg1_addr,
            pll2_cfg1,
            pll2_cfg2_addr,
            pll2_cfg2,
        );

        let pll2_hz = Self::one_block_pll2_rate_hz(pll2_cfg0, pll2_cfg1, pll2_cfg2);
        let bus_root_mux = (bus_root >> 24) & 0xf;
        let (bus_root_parent, bus_root_parent_name) = match bus_root_mux {
            0 => (Some(JH7110_OSC_CLOCK_HZ), "osc"),
            1 => (pll2_hz, "pll2"),
            _ => (None, "invalid"),
        };
        let axi_cfg0_divider = (axi_cfg0 & 0x00ff_ffff) as u64;
        let axi_cfg0_hz = bus_root_parent
            .filter(|_| axi_cfg0_divider != 0)
            .map(|parent| parent / axi_cfg0_divider);
        let sdio1_enabled = sdio1_sdcard & (1 << 31) != 0;
        let sdio1_divider = (sdio1_sdcard & 0x00ff_ffff) as u64;
        let ciu_hz = axi_cfg0_hz
            .filter(|_| sdio1_enabled && sdio1_divider != 0)
            .map(|parent| parent / sdio1_divider);
        let dw_divider = self.regs.clkdiv().read().clk_divider0() as u64;
        let sdclk_hz = ciu_hz.map(|ciu| {
            if dw_divider == 0 {
                ciu
            } else {
                ciu / (2 * dw_divider)
            }
        });
        let root_source_valid = match bus_root_mux {
            0 => true,
            1 => pll2_hz.is_some(),
            _ => false,
        };
        let valid = root_source_valid
            && bus_root_parent.is_some()
            && axi_cfg0_hz.is_some()
            && ciu_hz.is_some()
            && sdclk_hz.is_some();

        warn!(
            "ONE_BLOCK_CLOCK_TREE osc_hz={} pll2_hz={} bus_root_mux={} \
             bus_root_parent={} bus_root_hz={} axi_cfg0_divider={} axi_cfg0_hz={} \
             sdio1_gate={} sdio1_divider={} ciu_hz={} dw_clkdiv={} sdclk_hz={} status={}",
            JH7110_OSC_CLOCK_HZ,
            pll2_hz.unwrap_or(0),
            bus_root_mux,
            bus_root_parent_name,
            bus_root_parent.unwrap_or(0),
            axi_cfg0_divider,
            axi_cfg0_hz.unwrap_or(0),
            sdio1_enabled,
            sdio1_divider,
            ciu_hz.unwrap_or(0),
            dw_divider,
            sdclk_hz.unwrap_or(0),
            if valid { "ok" } else { "invalid_register_chain" },
        );
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_read_region(&mut self, buffer: &mut [u8]) {
        assert_eq!(buffer.len(), ONE_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE);
        for (block_index, block) in buffer
            .chunks_exact_mut(Self::BLOCK_SIZE)
            .enumerate()
        {
            self.read_block(
                ONE_BLOCK_TEST_START_LBA + block_index as u32,
                block.try_into().unwrap(),
            );
        }
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_write_region(&mut self, buffer: &[u8]) {
        assert_eq!(buffer.len(), ONE_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE);
        for (block_index, block) in buffer.chunks_exact(Self::BLOCK_SIZE).enumerate() {
            self.write_block(
                ONE_BLOCK_TEST_START_LBA + block_index as u32,
                block.try_into().unwrap(),
            );
        }
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_fill_pattern(buffer: &mut [u8], round: usize) {
        for (block_index, block) in buffer.chunks_exact_mut(Self::BLOCK_SIZE).enumerate() {
            let lba = ONE_BLOCK_TEST_START_LBA + block_index as u32;
            for (byte_index, byte) in block.iter_mut().enumerate() {
                let mut value = ((lba as u64) << 32)
                    ^ ((round as u64) << 16)
                    ^ byte_index as u64
                    ^ 0x9e37_79b9_7f4a_7c15;
                value ^= value >> 30;
                value = value.wrapping_mul(0xbf58_476d_1ce4_e5b9);
                value ^= value >> 27;
                value = value.wrapping_mul(0x94d0_49bb_1331_11eb);
                value ^= value >> 31;
                *byte = value as u8;
            }
        }
    }

    #[cfg(feature = "one-block-test")]
    fn one_block_first_mismatch(actual: &[u8], expected: &[u8]) -> Option<(usize, u8, u8)> {
        actual
            .iter()
            .zip(expected)
            .enumerate()
            .find_map(|(offset, (&actual, &expected))| {
                (actual != expected).then_some((offset, expected, actual))
            })
    }

    #[cfg(feature = "one-block-test")]
    fn run_one_block_test(&mut self) {
        let test_end_lba = ONE_BLOCK_TEST_START_LBA + ONE_BLOCK_TEST_BLOCKS as u32 - 1;
        assert!(
            self.num_blocks > test_end_lba as u64,
            "one-block-test region exceeds the detected card capacity"
        );
        assert!(
            self.dma_buffer.is_some(),
            "one-block-test requires the IDMAC bounce buffer"
        );

        warn!(
            "ONE_BLOCK_TEST begin start_lba={} end_lba={} blocks={} bytes={} rounds={} \
             warning=raw_region_is_temporarily_overwritten",
            ONE_BLOCK_TEST_START_LBA,
            test_end_lba,
            ONE_BLOCK_TEST_BLOCKS,
            ONE_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE,
            ONE_BLOCK_TEST_ROUNDS,
        );
        warn!(
            "ONE_BLOCK_REGISTERS CLKDIV=0x{:08x} CLKSRC=0x{:08x} CLKENA=0x{:08x} \
             CTYPE=0x{:08x} FIFOTH=0x{:08x} BMOD=0x{:08x} HCON=0x{:08x} UHS=0x{:08x}",
            self.regs.clkdiv().read().into_bits(),
            self.regs.clksrc().read().into_bits(),
            self.regs.clkena().read().into_bits(),
            self.regs.ctype().read().into_bits(),
            self.regs.fifoth().read().into_bits(),
            self.regs.bmod().read().into_bits(),
            self.regs.hcon().read().into_bits(),
            self.regs.uhs().read().into_bits(),
        );
        self.one_block_print_clock_tree();
        let hcon_area_optimized = self.regs.hcon().read().area_opt();
        warn!(
            "ONE_BLOCK_TCBCNT_CAPABILITY hcon_area_optimized={} \
             in_transfer_reads_supported={} counter_register_offset=0x005c",
            hcon_area_optimized,
            !hcon_area_optimized,
        );

        let mut timer_stats = OneBlockPhaseStats::new();
        for _ in 0..ONE_BLOCK_TIMER_SAMPLES {
            let started_ns = axhal::time::monotonic_time_nanos();
            timer_stats.record(
                axhal::time::monotonic_time_nanos().saturating_sub(started_ns),
            );
        }
        warn!(
            "ONE_BLOCK_TIMER samples={} average_ns={} min_ns={} max_ns={}",
            timer_stats.count,
            timer_stats.average_ns(),
            timer_stats.min_ns,
            timer_stats.max_ns,
        );

        let region_bytes = ONE_BLOCK_TEST_BLOCKS * Self::BLOCK_SIZE;
        let mut backup = vec![0u8; region_bytes];
        let mut transfer_buffer = vec![0u8; region_bytes];
        let mut expected = vec![0u8; region_bytes];

        self.one_block_profiler.enabled = false;
        warn!("ONE_BLOCK_TEST stage=backup status=begin");
        self.one_block_read_region(&mut backup);
        warn!("ONE_BLOCK_TEST stage=backup status=complete");

        // The backup pass warms the complete sequential read range. This extra pass
        // verifies that the reference is stable before any write is attempted.
        self.one_block_read_region(&mut transfer_buffer);
        if let Some((offset, expected_byte, actual_byte)) =
            Self::one_block_first_mismatch(&transfer_buffer, &backup)
        {
            panic!(
                "one-block-test read warm-up mismatch at byte {}: expected=0x{:02x}, actual=0x{:02x}",
                offset, expected_byte, actual_byte
            );
        }
        warn!("ONE_BLOCK_TEST stage=read_warmup status=ok");

        let _ = self.one_block_profiler.take(OneBlockOperation::Read);
        let mut read_total = OneBlockProfileStats::new();
        for round in 1..=ONE_BLOCK_TEST_ROUNDS {
            self.one_block_profiler.enabled = true;
            self.one_block_read_region(&mut transfer_buffer);
            self.one_block_profiler.enabled = false;
            let round_stats = self.one_block_profiler.take(OneBlockOperation::Read);
            let mismatch = Self::one_block_first_mismatch(&transfer_buffer, &backup);
            Self::one_block_print_round(
                OneBlockOperation::Read,
                round,
                round_stats,
                mismatch.is_none(),
            );
            read_total.merge(round_stats);
            if let Some((offset, expected_byte, actual_byte)) = mismatch {
                panic!(
                    "one-block-test measured read mismatch in round {} at byte {}: \
                     expected=0x{:02x}, actual=0x{:02x}",
                    round, offset, expected_byte, actual_byte
                );
            }
        }
        Self::one_block_print_profile(OneBlockOperation::Read, read_total);

        // Warm the write path before measuring it. From this point onward every exit
        // must restore the original region before reporting a data mismatch.
        Self::one_block_fill_pattern(&mut expected, 0);
        self.one_block_write_region(&expected);
        self.one_block_read_region(&mut transfer_buffer);
        let mut write_failure = Self::one_block_first_mismatch(&transfer_buffer, &expected)
            .map(|(offset, expected_byte, actual_byte)| {
                (0usize, offset, expected_byte, actual_byte)
            });
        warn!(
            "ONE_BLOCK_TEST stage=write_warmup status={}",
            if write_failure.is_none() {
                "ok"
            } else {
                "failed"
            }
        );

        let _ = self.one_block_profiler.take(OneBlockOperation::Write);
        let mut write_total = OneBlockProfileStats::new();
        if write_failure.is_none() {
            for round in 1..=ONE_BLOCK_TEST_ROUNDS {
                Self::one_block_fill_pattern(&mut expected, round);
                self.one_block_profiler.enabled = true;
                self.one_block_write_region(&expected);
                self.one_block_profiler.enabled = false;
                let round_stats = self.one_block_profiler.take(OneBlockOperation::Write);

                self.one_block_read_region(&mut transfer_buffer);
                let mismatch = Self::one_block_first_mismatch(&transfer_buffer, &expected);
                Self::one_block_print_round(
                    OneBlockOperation::Write,
                    round,
                    round_stats,
                    mismatch.is_none(),
                );
                write_total.merge(round_stats);
                if let Some((offset, expected_byte, actual_byte)) = mismatch {
                    write_failure = Some((round, offset, expected_byte, actual_byte));
                    break;
                }
            }
        }
        Self::one_block_print_profile(OneBlockOperation::Write, write_total);

        self.one_block_profiler.enabled = false;
        warn!("ONE_BLOCK_TEST stage=restore status=begin");
        self.one_block_write_region(&backup);
        self.one_block_read_region(&mut transfer_buffer);
        if let Some((offset, expected_byte, actual_byte)) =
            Self::one_block_first_mismatch(&transfer_buffer, &backup)
        {
            panic!(
                "one-block-test RESTORE FAILED at byte {} (LBA {}, byte {}): \
                 expected=0x{:02x}, actual=0x{:02x}",
                offset,
                ONE_BLOCK_TEST_START_LBA + (offset / Self::BLOCK_SIZE) as u32,
                offset % Self::BLOCK_SIZE,
                expected_byte,
                actual_byte,
            );
        }
        warn!("ONE_BLOCK_TEST stage=restore status=verified");

        if let Some((round, offset, expected_byte, actual_byte)) = write_failure {
            panic!(
                "one-block-test write validation failed in round {} at byte {} (LBA {}, byte {}): \
                 expected=0x{:02x}, actual=0x{:02x}; original data was restored",
                round,
                offset,
                ONE_BLOCK_TEST_START_LBA + (offset / Self::BLOCK_SIZE) as u32,
                offset % Self::BLOCK_SIZE,
                expected_byte,
                actual_byte,
            );
        }

        warn!("ONE_BLOCK_TEST complete status=ok original_region_restored=true");
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
            self.regs.intmask().update(|r| r.with_dto(true));
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

    /// Reads a single block from the SD/MMC card.
    pub fn read_block(&mut self, block: u32, buf: &mut [u8; 512]) {
        #[cfg(feature = "one-block-test")]
        self.one_block_profiler.begin(OneBlockOperation::Read);
        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
        self.set_transaction_size(512, 512);
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::TransactionConfig, phase_start);

        if let Some(dma_buf_info) = &self.dma_buffer {
            let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
            let dma_buf = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
            let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
                .expect("DMA buffer address exceeds the IDMAC 32-bit address range");

            self.send_cmd_idmac(Command::ReadSingleBlock(block, dma_buf), dma_bus_addr)
                .unwrap();

            #[cfg(feature = "one-block-test")]
            let phase_start = self.one_block_phase_start();
            let dma_usr_slice = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
            buf.copy_from_slice(dma_usr_slice);
            #[cfg(feature = "one-block-test")]
            self.one_block_phase_finish(OneBlockPhase::BounceCopy, phase_start);
        } else {
            panic!("synchronous DMA read requested without an IDMAC buffer");
        }
        #[cfg(feature = "one-block-test")]
        self.one_block_profiler.finish();
    }

    /// Reads a single block using IDMAC and asynchronously waits for completion.
    pub async fn read_block_async(&mut self, block: u32, buf: &mut [u8; 512]) {
        self.set_transaction_size(512, 512);

        if let Some(dma_buf_info) = &self.dma_buffer {
            let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
            let dma_buf = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
            let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
                .expect("DMA buffer address exceeds the IDMAC 32-bit address range");

            self.send_cmd_idmac_async(Command::ReadSingleBlock(block, dma_buf), dma_bus_addr)
                .await
                .expect("asynchronous IDMAC read failed; benchmark must stop");

            let dma_usr_slice = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
            buf.copy_from_slice(dma_usr_slice);
        } else {
            panic!("asynchronous DMA read requested without an IDMAC buffer");
        }
    }

    /// Writes a single block to the SD/MMC card.
    pub fn write_block(&mut self, block: u32, buf: &[u8; 512]) {
        #[cfg(feature = "one-block-test")]
        self.one_block_profiler.begin(OneBlockOperation::Write);
        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
        self.set_transaction_size(512, 512);
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::TransactionConfig, phase_start);

        if let Some(dma_buf_info) = &self.dma_buffer {
            let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
            let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
                .expect("DMA buffer address exceeds the IDMAC 32-bit address range");
            #[cfg(feature = "one-block-test")]
            let phase_start = self.one_block_phase_start();
            let dma_usr_slice =
                unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
            dma_usr_slice.copy_from_slice(buf);
            #[cfg(feature = "one-block-test")]
            self.one_block_phase_finish(OneBlockPhase::BounceCopy, phase_start);

            let dma_buf = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
            self.send_cmd_idmac(Command::WriteSingleBlock(block, dma_buf), dma_bus_addr)
                .unwrap();
            #[cfg(feature = "one-block-test")]
            self.one_block_wait_card_idle();
        } else {
            panic!("synchronous DMA write requested without an IDMAC buffer");
        }
        #[cfg(feature = "one-block-test")]
        self.one_block_profiler.finish();
    }

    /// Writes a single block using IDMAC and asynchronously waits for completion.
    pub async fn write_block_async(&mut self, block: u32, buf: &[u8; 512]) {
        self.set_transaction_size(512, 512);

        if let Some(dma_buf_info) = &self.dma_buffer {
            let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
            let dma_usr_slice =
                unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
            dma_usr_slice.copy_from_slice(buf);

            let dma_buf = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
            let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
                .expect("DMA buffer address exceeds the IDMAC 32-bit address range");
            self.send_cmd_idmac_async(Command::WriteSingleBlock(block, dma_buf), dma_bus_addr)
                .await
                .expect("asynchronous IDMAC write failed; benchmark must stop");
        } else {
            panic!("asynchronous DMA write requested without an IDMAC buffer");
        }
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

        // Enable IDMAC completion/error interrupts and controller DTO.
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
            .write(crate::regs::IntMask::new().with_dto(true));

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
        if !intmask_after.dto()
            || intmask_after.cmd()
            || intmask_after.rxdr()
            || intmask_after.txdr()
        {
            warn!(
                "try_enable_idmac: INTMASK mismatch after write; dto={}, cmd={}, rxdr={}, txdr={}",
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

        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
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
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::IdleWait, phase_start);

        // Establish a clean W1C status baseline for the new transaction.
        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
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

        let xfer = xfer.unwrap();

        IDMAC_DONE_FLAG.store(false, Ordering::Release);
        IDMAC_ERROR_FLAG.store(false, Ordering::Release);
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::StatusClear, phase_start);

        let buf_len = match xfer {
            DataXfer::Read(buf) => buf.len(),
            DataXfer::Write(buf) => buf.len(),
        };

        assert!(
            buf_len <= 0x1fff,
            "IDMAC single descriptor buffer too large: {buf_len}"
        );

        // One descriptor covers the contiguous single-block buffer.
        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
        let layout = Layout::new::<IdmacDescriptor>();
        let dma_desc_info =
            unsafe { alloc_coherent(layout) }.expect("Failed to allocate DMA descriptor");
        let desc_ptr = dma_desc_info.cpu_addr.as_ptr() as *mut IdmacDescriptor;
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::DescriptorAlloc, phase_start);

        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
        let mut descriptor = IdmacDescriptor::new();
        descriptor.set_desc0_control_descriptor(true, false, false, false, true, true, false);
        descriptor.set_des1_buffer1_size(buf_len as u16);
        descriptor.set_des2_buffer1_address(dma_bus_addr);
        descriptor.set_des3_next_descriptor_address(0);
        unsafe { core::ptr::write_volatile(desc_ptr, descriptor) };

        let desc_phy_addr = u32::try_from(dma_desc_info.bus_addr.as_u64())
            .expect("DMA descriptor address exceeds the IDMAC 32-bit address range");
        dma_io_fence();
        self.regs.bytcnt().write(buf_len as u32);
        self.regs.dbaddr().write(desc_phy_addr);
        dma_io_fence();
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::DescriptorPublish, phase_start);

        Some(IdmacTransferContext {
            cmd,
            arg,
            generation: 0,
            dma_desc_info,
            layout,
            desc_ptr,
        })
    }

    fn start_idmac_transfer(
        &mut self,
        mut context: IdmacTransferContext,
    ) -> Option<IdmacTransferContext> {
        let cmd = context.cmd;
        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
        context.generation = IDMAC_COMPLETION.begin_transfer();

        self.regs.cmdarg().write(context.arg);
        dma_io_fence();
        self.regs.cmd().write(cmd);
        dma_io_fence();
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::CommandIssue, phase_start);

        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
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
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::CommandAccept, phase_start);

        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
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
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::PostStartChecks, phase_start);

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
        let dma_done = if context.cmd.read_write() {
            idsts.ti()
        } else {
            idsts.ri()
        };
        command_done && dma_done && rintsts.data_transfer_over()
    }

    fn descriptor_owned(context: &IdmacTransferContext) -> bool {
        let des0 = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!((*context.desc_ptr).des0))
        };
        des0.own()
    }

    fn validate_idmac_terminal(&self, context: &IdmacTransferContext) -> bool {
        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        let has_error = Self::idmac_status_has_error(&rintsts, &idsts);
        let command_done = !context.cmd.response_expect() || rintsts.command_done();
        let dma_done = if context.cmd.read_write() {
            idsts.ti()
        } else {
            idsts.ri()
        };
        let controller_done = rintsts.data_transfer_over();

        dma_io_fence();
        let descriptor_owned = Self::descriptor_owned(context);
        let complete = !has_error
            && command_done
            && dma_done
            && controller_done
            && !descriptor_owned;

        if !complete {
            warn!(
                "IDMAC terminal validation failed: cmd={}, RINTSTS={rintsts:?}, \
                 IDSTS={idsts:?}, command_done={command_done}, dma_done={dma_done}, \
                 controller_done={controller_done}, desc_own={descriptor_owned}",
                context.cmd.cmd_index(),
            );
        }

        complete
    }

    fn wait_transfer_sync(
        &mut self,
        context: &IdmacTransferContext,
    ) -> Result<(), IdmacWaitError> {
        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
        if context.cmd.response_expect() {
            let deadline = axhal::time::wall_time() + Duration::from_secs(2);
            while !self.idmac_command_done_or_error(context) {
                if axhal::time::wall_time() >= deadline {
                    #[cfg(feature = "one-block-test")]
                    self.one_block_phase_finish(OneBlockPhase::CommandResponse, phase_start);
                    return Err(IdmacWaitError::CommandTimeout);
                }
                core::hint::spin_loop();
            }
        }
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::CommandResponse, phase_start);

        #[cfg(feature = "one-block-test")]
        let phase_start = self.one_block_phase_start();
        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            #[cfg(feature = "one-block-test")]
            self.one_block_phase_finish(OneBlockPhase::DataTransfer, phase_start);
            return Err(IdmacWaitError::Hardware);
        }

        #[cfg(feature = "one-block-test")]
        let mut tcbcnt_sample = self.one_block_tcbcnt_begin(context);
        let deadline = axhal::time::wall_time() + Duration::from_secs(5);
        while !self.idmac_terminal_events_or_error(context) {
            #[cfg(feature = "one-block-test")]
            self.one_block_tcbcnt_observe(&mut tcbcnt_sample);
            if axhal::time::wall_time() >= deadline {
                #[cfg(feature = "one-block-test")]
                {
                    self.one_block_tcbcnt_finish(&mut tcbcnt_sample);
                    self.one_block_phase_finish(OneBlockPhase::DataTransfer, phase_start);
                }
                return Err(IdmacWaitError::DataTimeout);
            }
            core::hint::spin_loop();
        }

        #[cfg(feature = "one-block-test")]
        self.one_block_tcbcnt_finish(&mut tcbcnt_sample);
        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        #[cfg(feature = "one-block-test")]
        self.one_block_phase_finish(OneBlockPhase::DataTransfer, phase_start);
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

            if idmac_error {
                IDMAC_ERROR_FLAG.store(true, Ordering::Release);
                log::error!("SDMMC DMA error: RINTSTS={:?}, IDSTS={:?}", rintsts, idsts);
            }

            if has_idsts {
                regs.idsts().write(idsts);
            }

            if transfer_done
                || idmac_error
                || rintsts.receive_fifo_data_request()
                || rintsts.transmit_fifo_data_request()
            {
                let mut clear_rintsts = crate::regs::RIntSts::new();
                clear_rintsts = clear_rintsts
                    .with_end_bit_error(rintsts.end_bit_error())
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
            should_notify = transfer_done || idmac_error;

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
