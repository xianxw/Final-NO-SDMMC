// Test-only read/compute concurrency benchmark. Enabled only by the
// `sdmmc-concurrency-test` feature on the dedicated final-test branch.

use alloc::{sync::Arc, vec, vec::Vec};
use core::{
    hint::black_box,
    sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering},
    time::Duration,
};

use log::warn;

use super::*;

const TEST_START_LBA: u32 = 2_099_200;
const TEST_ROUNDS: usize = 5;
const LEGACY_CONTROL_BLOCKS: usize = 256;
const RANGE_DIAGNOSTIC_WIDE_BLOCKS: usize = 2 * 1024 * 1024 / SdMmc::BLOCK_SIZE;
const RANGE_DIAGNOSTIC_REQUEST_BLOCKS: [usize; 3] = [1, 8, 32];
const SUSTAINED_DIAGNOSTIC_BLOCKS: usize = 8 * 1024 * 1024 / SdMmc::BLOCK_SIZE;
const WARMUP_REQUESTS: usize = 64;
const MATRIX_LATENCY_SAMPLE_STRIDE: usize = 16;
const COMPUTE_ITERATIONS_PER_UNIT: usize = 4_096;
const COMPUTE_BASELINE_DURATION: Duration = Duration::from_secs(2);
const COMPUTE_CASE_DURATION: Duration = Duration::from_secs(2);
const CHECKSUM_SEED: u64 = 0xcbf2_9ce4_8422_2325;

const WORKLOADS: [ReadWorkload; 3] = [
    ReadWorkload {
        name: "cmd17_control",
        request_blocks: 1,
        total_blocks: 2 * 1024 * 1024 / SdMmc::BLOCK_SIZE,
    },
    ReadWorkload {
        name: "cmd18_8block",
        request_blocks: 8,
        total_blocks: 16 * 1024 * 1024 / SdMmc::BLOCK_SIZE,
    },
    ReadWorkload {
        name: "cmd18_32block",
        request_blocks: 32,
        total_blocks: 16 * 1024 * 1024 / SdMmc::BLOCK_SIZE,
    },
];

static OBSERVATION_GENERATION: AtomicUsize = AtomicUsize::new(0);
static OBSERVATION_ENABLED: AtomicBool = AtomicBool::new(false);
static OBSERVATION_READY: AtomicBool = AtomicBool::new(false);
static OBSERVATION_CLOSED: AtomicBool = AtomicBool::new(true);
static IRQ_ENTRY_NS: AtomicU64 = AtomicU64::new(0);
static IRQ_NOTIFY_NS: AtomicU64 = AtomicU64::new(0);
static IRQ_EVENT_COUNT: AtomicUsize = AtomicUsize::new(0);
static IRQ_ENTRY_TO_RESUME_NS: AtomicU64 = AtomicU64::new(0);
static IRQ_NOTIFY_TO_RESUME_NS: AtomicU64 = AtomicU64::new(0);
static COMMAND_WAIT_ELAPSED_NS: AtomicU64 = AtomicU64::new(0);
static DATA_WAIT_ELAPSED_NS: AtomicU64 = AtomicU64::new(0);
static DEADLINE_FALLBACK: AtomicBool = AtomicBool::new(false);
static TERMINAL_RINTSTS: AtomicU32 = AtomicU32::new(0);
static TERMINAL_IDSTS: AtomicU32 = AtomicU32::new(0);

static STAGE_OBSERVATION_ENABLED: AtomicBool = AtomicBool::new(false);
static STAGE_OBSERVATION_CLOSED: AtomicBool = AtomicBool::new(true);
static STAGE_GENERATION: AtomicUsize = AtomicUsize::new(0);
static STAGE_API_START_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_API_END_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_PREPARE_START_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_PREPARE_END_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_CMD_WRITE_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_SUBMIT_END_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_COMMAND_SEEN_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_TERMINAL_SEEN_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_VALIDATION_END_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_CLEANUP_START_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_CLEANUP_END_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_FIRST_IRQ_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_IRQ_COMMAND_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_IRQ_DATA_DONE_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_IRQ_ACD_NS: AtomicU64 = AtomicU64::new(0);
static STAGE_IRQ_RINTSTS: AtomicU32 = AtomicU32::new(0);
static STAGE_IRQ_IDSTS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Debug)]
pub(super) struct DriverStageObservation {
    generation: usize,
    api_start_ns: u64,
    api_end_ns: u64,
    prepare_start_ns: u64,
    prepare_end_ns: u64,
    cmd_write_ns: u64,
    submit_end_ns: u64,
    command_seen_ns: u64,
    terminal_seen_ns: u64,
    validation_end_ns: u64,
    cleanup_start_ns: u64,
    cleanup_end_ns: u64,
    first_irq_ns: u64,
    irq_command_ns: u64,
    irq_data_done_ns: u64,
    irq_acd_ns: u64,
    irq_rintsts_bits: u32,
    irq_idsts_bits: u32,
}

fn reset_stage_observation() {
    STAGE_GENERATION.store(0, Ordering::Relaxed);
    STAGE_API_START_NS.store(0, Ordering::Relaxed);
    STAGE_API_END_NS.store(0, Ordering::Relaxed);
    STAGE_PREPARE_START_NS.store(0, Ordering::Relaxed);
    STAGE_PREPARE_END_NS.store(0, Ordering::Relaxed);
    STAGE_CMD_WRITE_NS.store(0, Ordering::Relaxed);
    STAGE_SUBMIT_END_NS.store(0, Ordering::Relaxed);
    STAGE_COMMAND_SEEN_NS.store(0, Ordering::Relaxed);
    STAGE_TERMINAL_SEEN_NS.store(0, Ordering::Relaxed);
    STAGE_VALIDATION_END_NS.store(0, Ordering::Relaxed);
    STAGE_CLEANUP_START_NS.store(0, Ordering::Relaxed);
    STAGE_CLEANUP_END_NS.store(0, Ordering::Relaxed);
    STAGE_FIRST_IRQ_NS.store(0, Ordering::Relaxed);
    STAGE_IRQ_COMMAND_NS.store(0, Ordering::Relaxed);
    STAGE_IRQ_DATA_DONE_NS.store(0, Ordering::Relaxed);
    STAGE_IRQ_ACD_NS.store(0, Ordering::Relaxed);
    STAGE_IRQ_RINTSTS.store(0, Ordering::Relaxed);
    STAGE_IRQ_IDSTS.store(0, Ordering::Relaxed);
}

fn stage_observation_active() -> bool {
    STAGE_OBSERVATION_ENABLED.load(Ordering::Acquire)
        && !STAGE_OBSERVATION_CLOSED.load(Ordering::Acquire)
}

fn stage_generation_active(generation: usize) -> bool {
    stage_observation_active() && STAGE_GENERATION.load(Ordering::Acquire) == generation
}

fn record_stage_now(slot: &AtomicU64) {
    if stage_observation_active() {
        slot.store(axhal::time::monotonic_time_nanos(), Ordering::Release);
    }
}

fn record_stage_now_for_generation(slot: &AtomicU64, generation: usize) {
    if stage_generation_active(generation) {
        slot.store(axhal::time::monotonic_time_nanos(), Ordering::Release);
    }
}

fn record_first_stage_time(slot: &AtomicU64, value: u64) {
    let _ = slot.compare_exchange(0, value, Ordering::AcqRel, Ordering::Acquire);
}

fn begin_driver_stage_observation() {
    STAGE_OBSERVATION_ENABLED.store(false, Ordering::Release);
    STAGE_OBSERVATION_CLOSED.store(true, Ordering::Release);
    reset_stage_observation();
    STAGE_API_START_NS.store(axhal::time::monotonic_time_nanos(), Ordering::Relaxed);
    STAGE_OBSERVATION_CLOSED.store(false, Ordering::Release);
    STAGE_OBSERVATION_ENABLED.store(true, Ordering::Release);
}

fn finish_driver_stage_observation() -> DriverStageObservation {
    STAGE_API_END_NS.store(axhal::time::monotonic_time_nanos(), Ordering::Release);
    STAGE_OBSERVATION_CLOSED.store(true, Ordering::Release);
    STAGE_OBSERVATION_ENABLED.store(false, Ordering::Release);

    DriverStageObservation {
        generation: STAGE_GENERATION.load(Ordering::Acquire),
        api_start_ns: STAGE_API_START_NS.load(Ordering::Acquire),
        api_end_ns: STAGE_API_END_NS.load(Ordering::Acquire),
        prepare_start_ns: STAGE_PREPARE_START_NS.load(Ordering::Acquire),
        prepare_end_ns: STAGE_PREPARE_END_NS.load(Ordering::Acquire),
        cmd_write_ns: STAGE_CMD_WRITE_NS.load(Ordering::Acquire),
        submit_end_ns: STAGE_SUBMIT_END_NS.load(Ordering::Acquire),
        command_seen_ns: STAGE_COMMAND_SEEN_NS.load(Ordering::Acquire),
        terminal_seen_ns: STAGE_TERMINAL_SEEN_NS.load(Ordering::Acquire),
        validation_end_ns: STAGE_VALIDATION_END_NS.load(Ordering::Acquire),
        cleanup_start_ns: STAGE_CLEANUP_START_NS.load(Ordering::Acquire),
        cleanup_end_ns: STAGE_CLEANUP_END_NS.load(Ordering::Acquire),
        first_irq_ns: STAGE_FIRST_IRQ_NS.load(Ordering::Acquire),
        irq_command_ns: STAGE_IRQ_COMMAND_NS.load(Ordering::Acquire),
        irq_data_done_ns: STAGE_IRQ_DATA_DONE_NS.load(Ordering::Acquire),
        irq_acd_ns: STAGE_IRQ_ACD_NS.load(Ordering::Acquire),
        irq_rintsts_bits: STAGE_IRQ_RINTSTS.load(Ordering::Acquire),
        irq_idsts_bits: STAGE_IRQ_IDSTS.load(Ordering::Acquire),
    }
}

pub(super) fn stage_observation_enabled() -> bool {
    stage_observation_active()
}

pub(super) fn record_stage_prepare_start() {
    record_stage_now(&STAGE_PREPARE_START_NS);
}

pub(super) fn record_stage_prepare_end() {
    record_stage_now(&STAGE_PREPARE_END_NS);
}

pub(super) fn record_stage_generation(generation: usize) {
    if stage_observation_active() {
        STAGE_GENERATION.store(generation, Ordering::Release);
    }
}

pub(super) fn record_stage_cmd_write(generation: usize) {
    record_stage_now_for_generation(&STAGE_CMD_WRITE_NS, generation);
}

pub(super) fn record_stage_submit_end(generation: usize) {
    record_stage_now_for_generation(&STAGE_SUBMIT_END_NS, generation);
}

pub(super) fn record_stage_command_seen(generation: usize) {
    record_stage_now_for_generation(&STAGE_COMMAND_SEEN_NS, generation);
}

pub(super) fn record_stage_terminal_seen(generation: usize) {
    record_stage_now_for_generation(&STAGE_TERMINAL_SEEN_NS, generation);
}

pub(super) fn record_stage_validation_end(generation: usize) {
    record_stage_now_for_generation(&STAGE_VALIDATION_END_NS, generation);
}

pub(super) fn record_stage_cleanup_start(generation: usize) {
    record_stage_now_for_generation(&STAGE_CLEANUP_START_NS, generation);
}

pub(super) fn record_stage_cleanup_end(generation: usize) {
    record_stage_now_for_generation(&STAGE_CLEANUP_END_NS, generation);
}

pub(super) fn record_stage_irq(
    generation: usize,
    irq_entry_ns: u64,
    rintsts_bits: u32,
    idsts_bits: u32,
) {
    if !stage_generation_active(generation) {
        return;
    }

    let rintsts = crate::regs::RIntSts::from_bits(rintsts_bits);
    let idsts = crate::regs::IdSts::from_bits(idsts_bits);
    record_first_stage_time(&STAGE_FIRST_IRQ_NS, irq_entry_ns);
    if rintsts.command_done() {
        record_first_stage_time(&STAGE_IRQ_COMMAND_NS, irq_entry_ns);
    }
    if rintsts.data_transfer_over() || idsts.ri() || idsts.ti() {
        record_first_stage_time(&STAGE_IRQ_DATA_DONE_NS, irq_entry_ns);
    }
    if rintsts.auto_command_done() {
        record_first_stage_time(&STAGE_IRQ_ACD_NS, irq_entry_ns);
    }
    STAGE_IRQ_RINTSTS.fetch_or(rintsts_bits, Ordering::AcqRel);
    STAGE_IRQ_IDSTS.fetch_or(idsts_bits, Ordering::AcqRel);
}

pub(super) fn observation_enabled() -> bool {
    OBSERVATION_ENABLED.load(Ordering::Acquire)
}

pub(super) fn begin_transfer_observation(generation: usize) {
    OBSERVATION_CLOSED.store(true, Ordering::Release);
    OBSERVATION_GENERATION.store(0, Ordering::Release);
    OBSERVATION_READY.store(false, Ordering::Release);
    if !OBSERVATION_ENABLED.load(Ordering::Acquire) {
        return;
    }
    IRQ_ENTRY_NS.store(0, Ordering::Relaxed);
    IRQ_NOTIFY_NS.store(0, Ordering::Relaxed);
    IRQ_EVENT_COUNT.store(0, Ordering::Relaxed);
    IRQ_ENTRY_TO_RESUME_NS.store(0, Ordering::Relaxed);
    IRQ_NOTIFY_TO_RESUME_NS.store(0, Ordering::Relaxed);
    COMMAND_WAIT_ELAPSED_NS.store(0, Ordering::Relaxed);
    DATA_WAIT_ELAPSED_NS.store(0, Ordering::Relaxed);
    DEADLINE_FALLBACK.store(false, Ordering::Relaxed);
    TERMINAL_RINTSTS.store(0, Ordering::Relaxed);
    TERMINAL_IDSTS.store(0, Ordering::Relaxed);
    OBSERVATION_GENERATION.store(generation, Ordering::Release);
    OBSERVATION_CLOSED.store(false, Ordering::Release);
}

pub(super) fn record_irq_wake(generation: usize, entry_ns: u64, notify_ns: u64) {
    if !OBSERVATION_ENABLED.load(Ordering::Acquire)
        || OBSERVATION_GENERATION.load(Ordering::Acquire) != generation
        || OBSERVATION_CLOSED.load(Ordering::Acquire)
        || OBSERVATION_READY.load(Ordering::Acquire)
    {
        return;
    }

    IRQ_ENTRY_NS.store(entry_ns, Ordering::Relaxed);
    IRQ_NOTIFY_NS.store(notify_ns, Ordering::Release);
    IRQ_EVENT_COUNT.fetch_add(1, Ordering::AcqRel);
}

pub(super) fn record_async_command_wait(
    generation: usize,
    timed_out: bool,
    command_wait_elapsed_ns: u64,
) {
    if !OBSERVATION_ENABLED.load(Ordering::Acquire)
        || OBSERVATION_GENERATION.load(Ordering::Acquire) != generation
    {
        return;
    }

    let fallback_threshold_ns = (IDMAC_COMMAND_TIMEOUT.as_nanos() as u64) * 3 / 4;
    COMMAND_WAIT_ELAPSED_NS.store(command_wait_elapsed_ns, Ordering::Relaxed);
    if timed_out
        || (IRQ_EVENT_COUNT.load(Ordering::Acquire) == 0
            && command_wait_elapsed_ns >= fallback_threshold_ns)
    {
        DEADLINE_FALLBACK.store(true, Ordering::Release);
    }
}

pub(super) fn record_async_resume(
    generation: usize,
    timed_out: bool,
    data_wait_elapsed_ns: u64,
    rintsts: crate::regs::RIntSts,
    idsts: crate::regs::IdSts,
) {
    if !OBSERVATION_ENABLED.load(Ordering::Acquire)
        || OBSERVATION_GENERATION.load(Ordering::Acquire) != generation
    {
        return;
    }
    OBSERVATION_CLOSED.store(true, Ordering::Release);

    let resumed_ns = axhal::time::monotonic_time_nanos();
    let entry_ns = IRQ_ENTRY_NS.load(Ordering::Acquire);
    let notify_ns = IRQ_NOTIFY_NS.load(Ordering::Acquire);
    let irq_events = IRQ_EVENT_COUNT.load(Ordering::Acquire);
    let fallback_threshold_ns = (IDMAC_DATA_WATCHDOG_TIMEOUT.as_nanos() as u64) * 3 / 4;

    IRQ_ENTRY_TO_RESUME_NS.store(
        if entry_ns == 0 {
            0
        } else {
            resumed_ns.saturating_sub(entry_ns)
        },
        Ordering::Relaxed,
    );
    IRQ_NOTIFY_TO_RESUME_NS.store(
        if notify_ns == 0 {
            0
        } else {
            resumed_ns.saturating_sub(notify_ns)
        },
        Ordering::Relaxed,
    );
    DATA_WAIT_ELAPSED_NS.store(data_wait_elapsed_ns, Ordering::Relaxed);
    DEADLINE_FALLBACK.fetch_or(
        timed_out || (irq_events == 0 && data_wait_elapsed_ns >= fallback_threshold_ns),
        Ordering::AcqRel,
    );
    TERMINAL_RINTSTS.store(rintsts.into_bits(), Ordering::Relaxed);
    TERMINAL_IDSTS.store(idsts.into_bits(), Ordering::Relaxed);
    OBSERVATION_READY.store(true, Ordering::Release);
}

fn set_observation_enabled(enabled: bool) {
    if !enabled {
        OBSERVATION_CLOSED.store(true, Ordering::Release);
    }
    OBSERVATION_ENABLED.store(enabled, Ordering::Release);
}

#[derive(Clone, Copy, Debug)]
struct AsyncObservation {
    ready: bool,
    generation: usize,
    irq_events: usize,
    entry_to_resume_ns: u64,
    notify_to_resume_ns: u64,
    command_wait_elapsed_ns: u64,
    data_wait_elapsed_ns: u64,
    deadline_fallback: bool,
    rintsts_bits: u32,
    idsts_bits: u32,
}

fn async_observation() -> AsyncObservation {
    let ready = OBSERVATION_READY.load(Ordering::Acquire);
    let generation = OBSERVATION_GENERATION.load(Ordering::Acquire);
    let observation = AsyncObservation {
        ready,
        generation,
        irq_events: IRQ_EVENT_COUNT.load(Ordering::Acquire),
        entry_to_resume_ns: IRQ_ENTRY_TO_RESUME_NS.load(Ordering::Relaxed),
        notify_to_resume_ns: IRQ_NOTIFY_TO_RESUME_NS.load(Ordering::Relaxed),
        command_wait_elapsed_ns: COMMAND_WAIT_ELAPSED_NS.load(Ordering::Relaxed),
        data_wait_elapsed_ns: DATA_WAIT_ELAPSED_NS.load(Ordering::Relaxed),
        deadline_fallback: DEADLINE_FALLBACK.load(Ordering::Relaxed),
        rintsts_bits: TERMINAL_RINTSTS.load(Ordering::Relaxed),
        idsts_bits: TERMINAL_IDSTS.load(Ordering::Relaxed),
    };

    if ready
        && OBSERVATION_READY.load(Ordering::Acquire)
        && OBSERVATION_GENERATION.load(Ordering::Acquire) == generation
    {
        observation
    } else {
        AsyncObservation {
            ready: false,
            ..observation
        }
    }
}

#[derive(Clone, Copy)]
struct ReadWorkload {
    name: &'static str,
    request_blocks: usize,
    total_blocks: usize,
}

impl ReadWorkload {
    fn request_bytes(self) -> usize {
        self.request_blocks * SdMmc::BLOCK_SIZE
    }

    fn total_bytes(self) -> usize {
        self.total_blocks * SdMmc::BLOCK_SIZE
    }

    fn request_count(self) -> usize {
        self.total_blocks / self.request_blocks
    }

    fn with_total_blocks(self, total_blocks: usize) -> Self {
        Self {
            total_blocks,
            ..self
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum IoMode {
    Sync,
    Async,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RangeScope {
    Short,
    Wide,
}

impl RangeScope {
    const fn name(self) -> &'static str {
        match self {
            Self::Short => "short_128k",
            Self::Wide => "wide_2m",
        }
    }

    const fn total_blocks(self) -> usize {
        match self {
            Self::Short => LEGACY_CONTROL_BLOCKS,
            Self::Wide => RANGE_DIAGNOSTIC_WIDE_BLOCKS,
        }
    }
}

impl IoMode {
    const fn name(self) -> &'static str {
        match self {
            Self::Sync => "sync",
            Self::Async => "async",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ComputeMode {
    NoYield,
    YieldPerUnit,
}

impl ComputeMode {
    const fn name(self) -> &'static str {
        match self {
            Self::NoYield => "no_yield",
            Self::YieldPerUnit => "yield_per_unit",
        }
    }
}

#[derive(Debug)]
enum TestFailure {
    InvalidEnvironment {
        cpu_count: usize,
        card_blocks: u64,
        required_end_lba: u64,
        dma_ready: bool,
    },
    Io {
        mode: IoMode,
        request: usize,
        error: SdMmcError,
    },
    DataMismatch {
        workload: &'static str,
        expected: u64,
        actual: u64,
    },
    AsyncTerminal {
        request: usize,
        reason: &'static str,
        observation: AsyncObservation,
    },
    StageObservation {
        workload: &'static str,
        mode: IoMode,
        request: usize,
        reason: &'static str,
        observation: DriverStageObservation,
    },
}

impl TestFailure {
    fn log(&self) {
        match self {
            Self::InvalidEnvironment {
                cpu_count,
                card_blocks,
                required_end_lba,
                dma_ready,
            } => warn!(
                "SDMMC_CONCURRENCY_FAILURE kind=invalid_environment cpu_count={} card_blocks={} \
                 required_end_lba={} dma_ready={}",
                cpu_count, card_blocks, required_end_lba, dma_ready,
            ),
            Self::Io {
                mode,
                request,
                error,
            } => warn!(
                "SDMMC_CONCURRENCY_FAILURE kind=io mode={} request={} error={:?}",
                mode.name(),
                request,
                error,
            ),
            Self::DataMismatch {
                workload,
                expected,
                actual,
            } => warn!(
                "SDMMC_CONCURRENCY_FAILURE kind=data_mismatch workload={} expected=0x{:016x} \
                 actual=0x{:016x}",
                workload, expected, actual,
            ),
            Self::AsyncTerminal {
                request,
                reason,
                observation,
            } => warn!(
                "SDMMC_CONCURRENCY_FAILURE kind=async_terminal request={} reason={} \
                 generation={} ready={} irq_events={} entry_to_resume_ns={} \
                 notify_to_resume_ns={} command_wait_elapsed_ns={} data_wait_elapsed_ns={} \
                 deadline_fallback={} \
                 RINTSTS=0x{:08x} IDSTS=0x{:08x}",
                request,
                reason,
                observation.generation,
                observation.ready,
                observation.irq_events,
                observation.entry_to_resume_ns,
                observation.notify_to_resume_ns,
                observation.command_wait_elapsed_ns,
                observation.data_wait_elapsed_ns,
                observation.deadline_fallback,
                observation.rintsts_bits,
                observation.idsts_bits,
            ),
            Self::StageObservation {
                workload,
                mode,
                request,
                reason,
                observation,
            } => warn!(
                "SDMMC_CONCURRENCY_FAILURE kind=stage_observation workload={} io={} \
                 request={} reason={} \
                 generation={} api={}..{} prepare={}..{} cmd_write={} submit_end={} \
                 command_seen={} terminal_seen={} validation_end={} cleanup={}..{} \
                 first_irq={} irq_command={} irq_data_done={} irq_acd={} \
                 RINTSTS=0x{:08x} IDSTS=0x{:08x}",
                workload,
                mode.name(),
                request,
                reason,
                observation.generation,
                observation.api_start_ns,
                observation.api_end_ns,
                observation.prepare_start_ns,
                observation.prepare_end_ns,
                observation.cmd_write_ns,
                observation.submit_end_ns,
                observation.command_seen_ns,
                observation.terminal_seen_ns,
                observation.validation_end_ns,
                observation.cleanup_start_ns,
                observation.cleanup_end_ns,
                observation.first_irq_ns,
                observation.irq_command_ns,
                observation.irq_data_done_ns,
                observation.irq_acd_ns,
                observation.irq_rintsts_bits,
                observation.irq_idsts_bits,
            ),
        }
    }
}

struct ComputeControl {
    ready: AtomicBool,
    run: AtomicBool,
    stop: AtomicBool,
    started_ns: AtomicU64,
    deadline_ns: AtomicU64,
    finished_ns: AtomicU64,
    units: AtomicU64,
    digest: AtomicU64,
    window_completed: AtomicBool,
}

impl ComputeControl {
    fn new() -> Self {
        Self {
            ready: AtomicBool::new(false),
            run: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            started_ns: AtomicU64::new(0),
            deadline_ns: AtomicU64::new(0),
            finished_ns: AtomicU64::new(0),
            units: AtomicU64::new(0),
            digest: AtomicU64::new(0),
            window_completed: AtomicBool::new(false),
        }
    }
}

struct ComputeWorker {
    control: Arc<ComputeControl>,
    task: axtask::AxTaskRef,
}

#[derive(Clone, Copy)]
struct ComputeResult {
    elapsed_ns: u64,
    units: u64,
    digest: u64,
    work_per_s_milli: u64,
    window_completed: bool,
}

impl ComputeWorker {
    fn spawn(mode: ComputeMode) -> Self {
        let control = Arc::new(ComputeControl::new());
        let task_control = control.clone();
        let task = axtask::spawn_with_name(
            move || {
                task_control.ready.store(true, Ordering::Release);
                while !task_control.run.load(Ordering::Acquire) {
                    axtask::yield_now();
                }

                let deadline_ns = task_control.deadline_ns.load(Ordering::Acquire);
                let mut units = 0u64;
                let mut state = 0x243f_6a88_85a3_08d3u64;
                loop {
                    if task_control.stop.load(Ordering::Acquire) {
                        break;
                    }
                    if axhal::time::monotonic_time_nanos() >= deadline_ns {
                        task_control.window_completed.store(true, Ordering::Release);
                        break;
                    }

                    state = compute_work_unit(state, units);
                    units = units.saturating_add(1);
                    if mode == ComputeMode::YieldPerUnit {
                        axtask::yield_now();
                    }
                }

                task_control.units.store(units, Ordering::Release);
                task_control.digest.store(state, Ordering::Release);
                task_control
                    .finished_ns
                    .store(axhal::time::monotonic_time_nanos(), Ordering::Release);
            },
            alloc::format!("sdmmc-compute-{}", mode.name()),
        );

        let worker = Self { control, task };
        while !worker.control.ready.load(Ordering::Acquire) {
            axtask::yield_now();
        }
        worker
    }

    fn start(&self, started_ns: u64, duration: Duration) {
        self.control.started_ns.store(started_ns, Ordering::Release);
        self.control
            .deadline_ns
            .store(started_ns + duration.as_nanos() as u64, Ordering::Release);
        self.control.run.store(true, Ordering::Release);
    }

    fn stop_and_join(self) -> ComputeResult {
        self.control.stop.store(true, Ordering::Release);
        self.join()
    }

    fn join(self) -> ComputeResult {
        self.task.join();
        let started_ns = self.control.started_ns.load(Ordering::Acquire);
        let finished_ns = self.control.finished_ns.load(Ordering::Acquire);
        let elapsed_ns = finished_ns.saturating_sub(started_ns);
        let units = self.control.units.load(Ordering::Acquire);
        ComputeResult {
            elapsed_ns,
            units,
            digest: self.control.digest.load(Ordering::Acquire),
            work_per_s_milli: rate_per_second_milli(units, elapsed_ns),
            window_completed: self.control.window_completed.load(Ordering::Acquire),
        }
    }
}

#[inline(never)]
fn compute_work_unit(mut state: u64, unit: u64) -> u64 {
    for iteration in 0..COMPUTE_ITERATIONS_PER_UNIT as u64 {
        state ^= unit.wrapping_mul(0x9e37_79b9_7f4a_7c15) ^ iteration;
        state = state.rotate_left(17).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        state ^= state >> 29;
    }
    black_box(state)
}

struct IoMeasurements {
    checksum: u64,
    request_latencies_ns: Vec<u64>,
    irq_entry_latencies_ns: Vec<u64>,
    irq_notify_latencies_ns: Vec<u64>,
    irq_completions: usize,
    fast_path_completions: usize,
    deadline_fallbacks: usize,
}

impl IoMeasurements {
    fn new(requests: usize, asynchronous: bool, latency_stride: Option<usize>) -> Self {
        let latency_samples = latency_stride
            .map(|stride| requests.div_ceil(stride.max(1)))
            .unwrap_or(0);
        Self {
            checksum: CHECKSUM_SEED,
            request_latencies_ns: Vec::with_capacity(latency_samples),
            irq_entry_latencies_ns: Vec::with_capacity(if asynchronous {
                latency_samples
            } else {
                0
            }),
            irq_notify_latencies_ns: Vec::with_capacity(if asynchronous {
                latency_samples
            } else {
                0
            }),
            irq_completions: 0,
            fast_path_completions: 0,
            deadline_fallbacks: 0,
        }
    }
}

struct IoRun {
    elapsed_ns: u64,
    measurements: IoMeasurements,
    compute: Option<ComputeResult>,
}

struct CaseAggregate {
    name: &'static str,
    io_mode: IoMode,
    compute_mode: Option<ComputeMode>,
    throughput_mib_s_milli: Vec<u64>,
    work_per_s_milli: Vec<u64>,
    request_latencies_ns: Vec<u64>,
    irq_entry_latencies_ns: Vec<u64>,
    irq_notify_latencies_ns: Vec<u64>,
    irq_completions: usize,
    fast_path_completions: usize,
    deadline_fallbacks: usize,
    compute_window_completions: usize,
}

impl CaseAggregate {
    fn new(name: &'static str, io_mode: IoMode, compute_mode: Option<ComputeMode>) -> Self {
        Self {
            name,
            io_mode,
            compute_mode,
            throughput_mib_s_milli: Vec::with_capacity(TEST_ROUNDS),
            work_per_s_milli: Vec::with_capacity(TEST_ROUNDS),
            request_latencies_ns: Vec::new(),
            irq_entry_latencies_ns: Vec::new(),
            irq_notify_latencies_ns: Vec::new(),
            irq_completions: 0,
            fast_path_completions: 0,
            deadline_fallbacks: 0,
            compute_window_completions: 0,
        }
    }

    fn add(
        &mut self,
        workload: ReadWorkload,
        round: usize,
        mut run: IoRun,
        baseline_work_per_s_milli: u64,
    ) {
        let throughput = throughput_mib_s_milli(workload.total_bytes() as u64, run.elapsed_ns);
        let work_rate = run.compute.map_or(0, |compute| compute.work_per_s_milli);
        let retention_permille = if baseline_work_per_s_milli == 0 {
            0
        } else {
            ((work_rate as u128 * 1_000) / baseline_work_per_s_milli as u128) as u64
        };

        run.measurements.request_latencies_ns.sort_unstable();
        run.measurements.irq_entry_latencies_ns.sort_unstable();
        run.measurements.irq_notify_latencies_ns.sort_unstable();
        let request_stats = latency_stats(&run.measurements.request_latencies_ns);
        let entry_stats = latency_stats(&run.measurements.irq_entry_latencies_ns);
        let notify_stats = latency_stats(&run.measurements.irq_notify_latencies_ns);
        let compute = run.compute.unwrap_or(ComputeResult {
            elapsed_ns: 0,
            units: 0,
            digest: 0,
            work_per_s_milli: 0,
            window_completed: false,
        });

        warn!(
            "SDMMC_CONCURRENCY_RESULT workload={} round={} case={} io={} compute={} elapsed_ns={} \
             bytes={} requests={} latency_sample_stride={} request_samples={} \
             throughput_mib_s_milli={} request_iops_milli={} \
             request_avg_ns={} request_p50_ns={} request_p95_ns={} request_p99_ns={} \
             request_max_ns={} work_units={} work_elapsed_ns={} work_per_s_milli={} \
             baseline_retained_permille={} compute_digest=0x{:016x} \
             compute_window_completed={} irq_completions={} \
             fast_path_completions={} deadline_fallbacks={} irq_entry_avg_ns={} \
             irq_entry_p95_ns={} irq_entry_p99_ns={} irq_entry_max_ns={} \
             irq_notify_avg_ns={} irq_notify_p95_ns={} irq_notify_p99_ns={} \
             irq_notify_max_ns={} checksum=0x{:016x} data_ok=true terminal_ok=true",
            workload.name,
            round,
            self.name,
            self.io_mode.name(),
            self.compute_mode.map_or("none", ComputeMode::name),
            run.elapsed_ns,
            workload.total_bytes(),
            workload.request_count(),
            MATRIX_LATENCY_SAMPLE_STRIDE,
            run.measurements.request_latencies_ns.len(),
            throughput,
            rate_per_second_milli(workload.request_count() as u64, run.elapsed_ns),
            request_stats.average,
            request_stats.p50,
            request_stats.p95,
            request_stats.p99,
            request_stats.max,
            compute.units,
            compute.elapsed_ns,
            work_rate,
            retention_permille,
            compute.digest,
            compute.window_completed,
            run.measurements.irq_completions,
            run.measurements.fast_path_completions,
            run.measurements.deadline_fallbacks,
            entry_stats.average,
            entry_stats.p95,
            entry_stats.p99,
            entry_stats.max,
            notify_stats.average,
            notify_stats.p95,
            notify_stats.p99,
            notify_stats.max,
            run.measurements.checksum,
        );

        self.throughput_mib_s_milli.push(throughput);
        self.work_per_s_milli.push(work_rate);
        self.request_latencies_ns
            .append(&mut run.measurements.request_latencies_ns);
        self.irq_entry_latencies_ns
            .append(&mut run.measurements.irq_entry_latencies_ns);
        self.irq_notify_latencies_ns
            .append(&mut run.measurements.irq_notify_latencies_ns);
        self.irq_completions += run.measurements.irq_completions;
        self.fast_path_completions += run.measurements.fast_path_completions;
        self.deadline_fallbacks += run.measurements.deadline_fallbacks;
        self.compute_window_completions += usize::from(compute.window_completed);
    }

    fn report_summary(&mut self, workload: ReadWorkload, baseline_work_per_s_milli: u64) {
        self.request_latencies_ns.sort_unstable();
        self.irq_entry_latencies_ns.sort_unstable();
        self.irq_notify_latencies_ns.sort_unstable();
        let throughput = scalar_stats(&self.throughput_mib_s_milli);
        let work = scalar_stats(&self.work_per_s_milli);
        let request = latency_stats(&self.request_latencies_ns);
        let entry = latency_stats(&self.irq_entry_latencies_ns);
        let notify = latency_stats(&self.irq_notify_latencies_ns);
        let retained_permille = if baseline_work_per_s_milli == 0 {
            0
        } else {
            ((work.mean as u128 * 1_000) / baseline_work_per_s_milli as u128) as u64
        };

        warn!(
            "SDMMC_CONCURRENCY_SUMMARY workload={} request_blocks={} total_bytes_per_round={} \
             rounds={} case={} io={} compute={} throughput_mean_mib_s_milli={} \
             throughput_min_mib_s_milli={} throughput_max_mib_s_milli={} \
             throughput_stddev_mib_s_milli={} work_mean_per_s_milli={} \
             work_min_per_s_milli={} work_max_per_s_milli={} work_stddev_per_s_milli={} \
             baseline_retained_permille={} request_samples={} request_avg_ns={} \
             request_p50_ns={} request_p95_ns={} request_p99_ns={} request_max_ns={} \
             irq_samples={} irq_entry_avg_ns={} irq_entry_p50_ns={} irq_entry_p95_ns={} \
             irq_entry_p99_ns={} irq_entry_max_ns={} irq_notify_avg_ns={} \
             irq_notify_p50_ns={} irq_notify_p95_ns={} irq_notify_p99_ns={} \
             irq_notify_max_ns={} compute_window_completions={} irq_completions={} \
             fast_path_completions={} deadline_fallbacks={} data_ok=true terminal_ok=true",
            workload.name,
            workload.request_blocks,
            workload.total_bytes(),
            TEST_ROUNDS,
            self.name,
            self.io_mode.name(),
            self.compute_mode.map_or("none", ComputeMode::name),
            throughput.mean,
            throughput.min,
            throughput.max,
            throughput.stddev,
            work.mean,
            work.min,
            work.max,
            work.stddev,
            retained_permille,
            self.request_latencies_ns.len(),
            request.average,
            request.p50,
            request.p95,
            request.p99,
            request.max,
            self.irq_entry_latencies_ns.len(),
            entry.average,
            entry.p50,
            entry.p95,
            entry.p99,
            entry.max,
            notify.average,
            notify.p50,
            notify.p95,
            notify.p99,
            notify.max,
            self.compute_window_completions,
            self.irq_completions,
            self.fast_path_completions,
            self.deadline_fallbacks,
        );
    }
}

#[derive(Clone, Copy)]
struct LatencyStats {
    average: u64,
    p50: u64,
    p95: u64,
    p99: u64,
    max: u64,
}

#[derive(Clone, Copy)]
struct ScalarStats {
    mean: u64,
    min: u64,
    max: u64,
    stddev: u64,
}

fn latency_stats(sorted: &[u64]) -> LatencyStats {
    if sorted.is_empty() {
        return LatencyStats {
            average: 0,
            p50: 0,
            p95: 0,
            p99: 0,
            max: 0,
        };
    }

    LatencyStats {
        average: (sorted.iter().map(|&value| value as u128).sum::<u128>()
            / sorted.len() as u128) as u64,
        p50: percentile(sorted, 50),
        p95: percentile(sorted, 95),
        p99: percentile(sorted, 99),
        max: sorted[sorted.len() - 1],
    }
}

fn percentile(sorted: &[u64], percentile: usize) -> u64 {
    let rank = (sorted.len() * percentile).div_ceil(100);
    sorted[rank.saturating_sub(1).min(sorted.len() - 1)]
}

fn scalar_stats(values: &[u64]) -> ScalarStats {
    if values.is_empty() {
        return ScalarStats {
            mean: 0,
            min: 0,
            max: 0,
            stddev: 0,
        };
    }

    let mean = (values.iter().map(|&value| value as u128).sum::<u128>()
        / values.len() as u128) as u64;
    let variance = values
        .iter()
        .map(|&value| value.abs_diff(mean) as u128)
        .map(|difference| difference * difference)
        .sum::<u128>()
        / values.len() as u128;
    ScalarStats {
        mean,
        min: *values.iter().min().unwrap(),
        max: *values.iter().max().unwrap(),
        stddev: integer_sqrt(variance) as u64,
    }
}

fn integer_sqrt(value: u128) -> u128 {
    if value < 2 {
        return value;
    }
    let mut low = 1u128;
    let mut high = value.min(u64::MAX as u128) + 1;
    while low + 1 < high {
        let mid = low + (high - low) / 2;
        if mid <= value / mid {
            low = mid;
        } else {
            high = mid;
        }
    }
    low
}

fn rate_per_second_milli(units: u64, elapsed_ns: u64) -> u64 {
    ((units as u128 * 1_000_000_000_000u128) / elapsed_ns.max(1) as u128) as u64
}

fn throughput_mib_s_milli(bytes: u64, elapsed_ns: u64) -> u64 {
    ((bytes as u128 * 1_000_000_000u128 * 1_000u128)
        / (elapsed_ns.max(1) as u128 * 1024u128 * 1024u128)) as u64
}

fn ratio_permille(numerator: u64, denominator: u64) -> u64 {
    ((numerator as u128 * 1_000) / denominator.max(1) as u128) as u64
}

#[derive(Clone, Copy, Debug)]
struct StageDurations {
    api_total_ns: u64,
    api_to_prepare_ns: u64,
    prepare_ns: u64,
    prepare_to_cmd_ns: u64,
    submit_ns: u64,
    command_wait_ns: u64,
    data_wait_ns: u64,
    validation_ns: u64,
    validation_to_cleanup_ns: u64,
    cleanup_ns: u64,
    copy_to_user_ns: u64,
    command_to_terminal_ns: u64,
    software_ns: u64,
    irq_to_terminal_ns: u64,
    data_irq_to_acd_ns: u64,
}

impl StageDurations {
    fn from_observation(
        observation: DriverStageObservation,
        request_blocks: usize,
    ) -> Result<Self, &'static str> {
        if observation.generation == 0 {
            return Err("missing_generation");
        }
        let ordered = [
            observation.api_start_ns,
            observation.prepare_start_ns,
            observation.prepare_end_ns,
            observation.cmd_write_ns,
            observation.submit_end_ns,
            observation.command_seen_ns,
            observation.terminal_seen_ns,
            observation.validation_end_ns,
            observation.cleanup_start_ns,
            observation.cleanup_end_ns,
            observation.api_end_ns,
        ];
        if ordered.contains(&0) {
            return Err("missing_stage_timestamp");
        }
        if !ordered.windows(2).all(|pair| pair[0] <= pair[1]) {
            return Err("stage_timestamp_order");
        }

        let api_to_prepare_ns = observation
            .prepare_start_ns
            .saturating_sub(observation.api_start_ns);
        let prepare_ns = observation
            .prepare_end_ns
            .saturating_sub(observation.prepare_start_ns);
        let prepare_to_cmd_ns = observation
            .cmd_write_ns
            .saturating_sub(observation.prepare_end_ns);
        let submit_ns = observation
            .submit_end_ns
            .saturating_sub(observation.cmd_write_ns);
        let command_wait_ns = observation
            .command_seen_ns
            .saturating_sub(observation.submit_end_ns);
        let data_wait_ns = observation
            .terminal_seen_ns
            .saturating_sub(observation.command_seen_ns);
        let validation_ns = observation
            .validation_end_ns
            .saturating_sub(observation.terminal_seen_ns);
        let validation_to_cleanup_ns = observation
            .cleanup_start_ns
            .saturating_sub(observation.validation_end_ns);
        let cleanup_ns = observation
            .cleanup_end_ns
            .saturating_sub(observation.cleanup_start_ns);
        let copy_to_user_ns = observation
            .api_end_ns
            .saturating_sub(observation.cleanup_end_ns);
        let command_to_terminal_ns = observation
            .terminal_seen_ns
            .saturating_sub(observation.cmd_write_ns);
        let software_ns = api_to_prepare_ns
            .saturating_add(prepare_ns)
            .saturating_add(prepare_to_cmd_ns)
            .saturating_add(validation_ns)
            .saturating_add(validation_to_cleanup_ns)
            .saturating_add(cleanup_ns)
            .saturating_add(copy_to_user_ns);
        let terminal_irq_ns = if request_blocks == 1 {
            observation.irq_data_done_ns
        } else if observation.irq_data_done_ns != 0 && observation.irq_acd_ns != 0 {
            observation.irq_data_done_ns.max(observation.irq_acd_ns)
        } else {
            0
        };
        let irq_to_terminal_ns = if terminal_irq_ns != 0
            && terminal_irq_ns <= observation.terminal_seen_ns
        {
            observation.terminal_seen_ns - terminal_irq_ns
        } else {
            0
        };
        let data_irq_to_acd_ns = if observation.irq_data_done_ns != 0
            && observation.irq_acd_ns >= observation.irq_data_done_ns
        {
            observation.irq_acd_ns - observation.irq_data_done_ns
        } else {
            0
        };

        Ok(Self {
            api_total_ns: observation
                .api_end_ns
                .saturating_sub(observation.api_start_ns),
            api_to_prepare_ns,
            prepare_ns,
            prepare_to_cmd_ns,
            submit_ns,
            command_wait_ns,
            data_wait_ns,
            validation_ns,
            validation_to_cleanup_ns,
            cleanup_ns,
            copy_to_user_ns,
            command_to_terminal_ns,
            software_ns,
            irq_to_terminal_ns,
            data_irq_to_acd_ns,
        })
    }
}

#[derive(Default)]
struct StageAggregate {
    requests: usize,
    api_total_ns: u128,
    api_to_prepare_ns: u128,
    prepare_ns: u128,
    prepare_to_cmd_ns: u128,
    submit_ns: u128,
    command_wait_ns: u128,
    data_wait_ns: u128,
    validation_ns: u128,
    validation_to_cleanup_ns: u128,
    cleanup_ns: u128,
    copy_to_user_ns: u128,
    command_to_terminal_ns: u128,
    software_ns: u128,
    irq_to_terminal_ns: u128,
    irq_to_terminal_samples: usize,
    data_irq_to_acd_ns: u128,
    data_irq_to_acd_samples: usize,
    irq_rintsts_bits: u32,
    irq_idsts_bits: u32,
    slowest_api_ns: u64,
    slowest_request: usize,
    slowest_lba: u32,
    slowest: Option<StageDurations>,
}

impl StageAggregate {
    fn add(
        &mut self,
        request: usize,
        lba: u32,
        durations: StageDurations,
        observation: DriverStageObservation,
    ) {
        self.requests += 1;
        self.api_total_ns += durations.api_total_ns as u128;
        self.api_to_prepare_ns += durations.api_to_prepare_ns as u128;
        self.prepare_ns += durations.prepare_ns as u128;
        self.prepare_to_cmd_ns += durations.prepare_to_cmd_ns as u128;
        self.submit_ns += durations.submit_ns as u128;
        self.command_wait_ns += durations.command_wait_ns as u128;
        self.data_wait_ns += durations.data_wait_ns as u128;
        self.validation_ns += durations.validation_ns as u128;
        self.validation_to_cleanup_ns += durations.validation_to_cleanup_ns as u128;
        self.cleanup_ns += durations.cleanup_ns as u128;
        self.copy_to_user_ns += durations.copy_to_user_ns as u128;
        self.command_to_terminal_ns += durations.command_to_terminal_ns as u128;
        self.software_ns += durations.software_ns as u128;
        if durations.irq_to_terminal_ns != 0 {
            self.irq_to_terminal_ns += durations.irq_to_terminal_ns as u128;
            self.irq_to_terminal_samples += 1;
        }
        if durations.data_irq_to_acd_ns != 0 {
            self.data_irq_to_acd_ns += durations.data_irq_to_acd_ns as u128;
            self.data_irq_to_acd_samples += 1;
        }
        self.irq_rintsts_bits |= observation.irq_rintsts_bits;
        self.irq_idsts_bits |= observation.irq_idsts_bits;
        if durations.api_total_ns > self.slowest_api_ns {
            self.slowest_api_ns = durations.api_total_ns;
            self.slowest_request = request;
            self.slowest_lba = lba;
            self.slowest = Some(durations);
        }
    }

    fn merge(&mut self, other: &Self) {
        self.requests += other.requests;
        self.api_total_ns += other.api_total_ns;
        self.api_to_prepare_ns += other.api_to_prepare_ns;
        self.prepare_ns += other.prepare_ns;
        self.prepare_to_cmd_ns += other.prepare_to_cmd_ns;
        self.submit_ns += other.submit_ns;
        self.command_wait_ns += other.command_wait_ns;
        self.data_wait_ns += other.data_wait_ns;
        self.validation_ns += other.validation_ns;
        self.validation_to_cleanup_ns += other.validation_to_cleanup_ns;
        self.cleanup_ns += other.cleanup_ns;
        self.copy_to_user_ns += other.copy_to_user_ns;
        self.command_to_terminal_ns += other.command_to_terminal_ns;
        self.software_ns += other.software_ns;
        self.irq_to_terminal_ns += other.irq_to_terminal_ns;
        self.irq_to_terminal_samples += other.irq_to_terminal_samples;
        self.data_irq_to_acd_ns += other.data_irq_to_acd_ns;
        self.data_irq_to_acd_samples += other.data_irq_to_acd_samples;
        self.irq_rintsts_bits |= other.irq_rintsts_bits;
        self.irq_idsts_bits |= other.irq_idsts_bits;
        if other.slowest_api_ns > self.slowest_api_ns {
            self.slowest_api_ns = other.slowest_api_ns;
            self.slowest_request = other.slowest_request;
            self.slowest_lba = other.slowest_lba;
            self.slowest = other.slowest;
        }
    }

    fn average(&self, total: u128) -> u64 {
        (total / self.requests.max(1) as u128) as u64
    }

    fn irq_to_terminal_average(&self) -> u64 {
        (self.irq_to_terminal_ns / self.irq_to_terminal_samples.max(1) as u128) as u64
    }

    fn data_irq_to_acd_average(&self) -> u64 {
        (self.data_irq_to_acd_ns / self.data_irq_to_acd_samples.max(1) as u128) as u64
    }
}

struct RangeStageRound {
    elapsed_ns: u64,
    checksum: u64,
    stages: StageAggregate,
}

struct RangeStageSummary {
    scope: RangeScope,
    request_blocks: usize,
    io_mode: IoMode,
    throughput_mib_s_milli: Vec<u64>,
    elapsed_total_ns: u64,
    stages: StageAggregate,
}

#[derive(Clone, Copy)]
struct RangeStageSnapshot {
    scope: RangeScope,
    request_blocks: usize,
    io_mode: IoMode,
    throughput_mib_s_milli: u64,
    api_average_ns: u64,
    command_to_terminal_average_ns: u64,
    software_average_ns: u64,
    irq_to_terminal_average_ns: u64,
}

impl RangeStageSummary {
    fn new(scope: RangeScope, request_blocks: usize, io_mode: IoMode) -> Self {
        Self {
            scope,
            request_blocks,
            io_mode,
            throughput_mib_s_milli: Vec::with_capacity(TEST_ROUNDS),
            elapsed_total_ns: 0,
            stages: StageAggregate::default(),
        }
    }

    fn add_round(&mut self, round: &RangeStageRound) {
        let bytes = self.scope.total_blocks() * SdMmc::BLOCK_SIZE;
        self.throughput_mib_s_milli
            .push(throughput_mib_s_milli(bytes as u64, round.elapsed_ns));
        self.elapsed_total_ns = self.elapsed_total_ns.saturating_add(round.elapsed_ns);
        self.stages.merge(&round.stages);
    }

    fn report(&self) -> RangeStageSnapshot {
        let throughput = scalar_stats(&self.throughput_mib_s_milli);
        let aggregate_throughput = throughput_mib_s_milli(
            (self.scope.total_blocks() * SdMmc::BLOCK_SIZE * TEST_ROUNDS) as u64,
            self.elapsed_total_ns,
        );
        warn!(
            "SDMMC_RANGE_STAGE_SUMMARY scope={} start_lba={} end_lba={} request_blocks={} \
             io={} rounds={} requests={} aggregate_mib_s_milli={} mean_mib_s_milli={} \
             min_mib_s_milli={} max_mib_s_milli={} stddev_mib_s_milli={} \
             api_avg_ns={} api_to_prepare_avg_ns={} prepare_avg_ns={} \
             prepare_to_cmd_avg_ns={} submit_avg_ns={} command_wait_avg_ns={} \
             data_wait_avg_ns={} validation_avg_ns={} validation_to_cleanup_avg_ns={} \
             cleanup_avg_ns={} copy_to_user_avg_ns={} command_to_terminal_avg_ns={} \
             software_avg_ns={} irq_to_terminal_samples={} irq_to_terminal_avg_ns={} \
             data_irq_to_acd_samples={} data_irq_to_acd_avg_ns={} \
             RINTSTS_OR=0x{:08x} IDSTS_OR=0x{:08x} data_ok=true terminal_ok=true",
            self.scope.name(),
            TEST_START_LBA,
            TEST_START_LBA + self.scope.total_blocks() as u32 - 1,
            self.request_blocks,
            self.io_mode.name(),
            TEST_ROUNDS,
            self.stages.requests,
            aggregate_throughput,
            throughput.mean,
            throughput.min,
            throughput.max,
            throughput.stddev,
            self.stages.average(self.stages.api_total_ns),
            self.stages.average(self.stages.api_to_prepare_ns),
            self.stages.average(self.stages.prepare_ns),
            self.stages.average(self.stages.prepare_to_cmd_ns),
            self.stages.average(self.stages.submit_ns),
            self.stages.average(self.stages.command_wait_ns),
            self.stages.average(self.stages.data_wait_ns),
            self.stages.average(self.stages.validation_ns),
            self.stages.average(self.stages.validation_to_cleanup_ns),
            self.stages.average(self.stages.cleanup_ns),
            self.stages.average(self.stages.copy_to_user_ns),
            self.stages.average(self.stages.command_to_terminal_ns),
            self.stages.average(self.stages.software_ns),
            self.stages.irq_to_terminal_samples,
            self.stages.irq_to_terminal_average(),
            self.stages.data_irq_to_acd_samples,
            self.stages.data_irq_to_acd_average(),
            self.stages.irq_rintsts_bits,
            self.stages.irq_idsts_bits,
        );

        RangeStageSnapshot {
            scope: self.scope,
            request_blocks: self.request_blocks,
            io_mode: self.io_mode,
            throughput_mib_s_milli: aggregate_throughput,
            api_average_ns: self.stages.average(self.stages.api_total_ns),
            command_to_terminal_average_ns: self
                .stages
                .average(self.stages.command_to_terminal_ns),
            software_average_ns: self.stages.average(self.stages.software_ns),
            irq_to_terminal_average_ns: self.stages.irq_to_terminal_average(),
        }
    }
}

fn update_checksum(mut checksum: u64, bytes: &[u8]) -> u64 {
    for &byte in bytes {
        checksum ^= byte as u64;
        checksum = checksum.wrapping_mul(0x0000_0100_0000_01b3);
    }
    checksum
}

fn should_sample_latency(request: usize, stride: usize) -> bool {
    let stride = stride.max(1);
    let group = request / stride;
    let mut mixed = group.wrapping_mul(0x9e37_79b9usize);
    mixed ^= mixed >> (usize::BITS / 2);
    mixed ^= mixed >> (usize::BITS / 4);
    request % stride == mixed % stride
}

impl SdMmc {
    fn sync_read_into(
        &mut self,
        workload: ReadWorkload,
        buffer: &mut [u8],
        measurements: &mut IoMeasurements,
        latency_stride: Option<usize>,
    ) -> Result<(), TestFailure> {
        debug_assert_eq!(buffer.len(), workload.total_bytes());
        for request in 0..workload.request_count() {
            let lba = TEST_START_LBA + (request * workload.request_blocks) as u32;
            let offset = request * workload.request_bytes();
            let request_buffer = &mut buffer[offset..offset + workload.request_bytes()];
            let sample_latency =
                latency_stride.is_some_and(|stride| should_sample_latency(request, stride));
            let started_ns = sample_latency.then(axhal::time::monotonic_time_nanos);
            self.read_blocks(lba, request_buffer)
                .map_err(|error| TestFailure::Io {
                    mode: IoMode::Sync,
                    request,
                    error,
                })?;
            if let Some(started_ns) = started_ns {
                measurements.request_latencies_ns.push(
                    axhal::time::monotonic_time_nanos().saturating_sub(started_ns),
                );
            }
        }
        Ok(())
    }

    async fn async_read_into(
        &mut self,
        workload: ReadWorkload,
        buffer: &mut [u8],
        measurements: &mut IoMeasurements,
        latency_stride: Option<usize>,
    ) -> Result<(), TestFailure> {
        debug_assert_eq!(buffer.len(), workload.total_bytes());
        for request in 0..workload.request_count() {
            let lba = TEST_START_LBA + (request * workload.request_blocks) as u32;
            let offset = request * workload.request_bytes();
            let request_buffer = &mut buffer[offset..offset + workload.request_bytes()];
            let sample_latency =
                latency_stride.is_some_and(|stride| should_sample_latency(request, stride));
            let started_ns = sample_latency.then(axhal::time::monotonic_time_nanos);
            set_observation_enabled(true);
            let read_result = self.read_blocks_async(lba, request_buffer).await;
            set_observation_enabled(false);
            read_result.map_err(|error| TestFailure::Io {
                    mode: IoMode::Async,
                    request,
                    error,
                })?;
            let elapsed_ns = started_ns.map(|started_ns| {
                axhal::time::monotonic_time_nanos().saturating_sub(started_ns)
            });
            let observation = async_observation();
            self.validate_async_observation(workload, request, observation)?;

            if let Some(elapsed_ns) = elapsed_ns {
                measurements.request_latencies_ns.push(elapsed_ns);
                if observation.entry_to_resume_ns != 0 {
                    measurements
                        .irq_entry_latencies_ns
                        .push(observation.entry_to_resume_ns);
                }
                if observation.notify_to_resume_ns != 0 {
                    measurements
                        .irq_notify_latencies_ns
                        .push(observation.notify_to_resume_ns);
                }
            }
            if observation.deadline_fallback {
                measurements.deadline_fallbacks += 1;
            } else if observation.irq_events == 0 {
                measurements.fast_path_completions += 1;
            } else {
                measurements.irq_completions += 1;
            }
        }
        Ok(())
    }

    fn sync_read_measurements(
        &mut self,
        workload: ReadWorkload,
        latency_stride: Option<usize>,
    ) -> Result<IoMeasurements, TestFailure> {
        let mut buffer = vec![0u8; workload.total_bytes()];
        let mut measurements =
            IoMeasurements::new(workload.request_count(), false, latency_stride);
        self.sync_read_into(workload, &mut buffer, &mut measurements, latency_stride)?;
        measurements.checksum = update_checksum(CHECKSUM_SEED, &buffer);
        Ok(measurements)
    }

    async fn async_read_measurements(
        &mut self,
        workload: ReadWorkload,
        latency_stride: Option<usize>,
    ) -> Result<IoMeasurements, TestFailure> {
        let mut buffer = vec![0u8; workload.total_bytes()];
        let mut measurements =
            IoMeasurements::new(workload.request_count(), true, latency_stride);
        self.async_read_into(workload, &mut buffer, &mut measurements, latency_stride)
            .await?;
        measurements.checksum = update_checksum(CHECKSUM_SEED, &buffer);
        Ok(measurements)
    }

    fn validate_async_observation(
        &self,
        workload: ReadWorkload,
        request: usize,
        observation: AsyncObservation,
    ) -> Result<(), TestFailure> {
        let rintsts = crate::regs::RIntSts::from_bits(observation.rintsts_bits);
        let idsts = crate::regs::IdSts::from_bits(observation.idsts_bits);
        let terminal_ok = observation.ready
            && observation.generation != 0
            && rintsts.command_done()
            && rintsts.data_transfer_over()
            && idsts.ri()
            && !rintsts.error()
            && !idsts.ais()
            && !idsts.ces()
            && !idsts.du()
            && !idsts.fbe()
            && (workload.request_blocks == 1 || rintsts.auto_command_done());
        if !terminal_ok {
            return Err(TestFailure::AsyncTerminal {
                request,
                reason: "terminal_status",
                observation,
            });
        }
        if observation.deadline_fallback {
            return Err(TestFailure::AsyncTerminal {
                request,
                reason: "deadline_fallback",
                observation,
            });
        }
        if observation.irq_events != 0
            && (observation.entry_to_resume_ns == 0 || observation.notify_to_resume_ns == 0)
        {
            return Err(TestFailure::AsyncTerminal {
                request,
                reason: "missing_irq_latency",
                observation,
            });
        }
        Ok(())
    }

    fn run_timed_read_round(
        &mut self,
        workload: ReadWorkload,
        io_mode: IoMode,
        latency_stride: Option<usize>,
        buffer: &mut [u8],
    ) -> Result<(u64, IoMeasurements), TestFailure> {
        buffer.fill(0);
        let mut measurements =
            IoMeasurements::new(workload.request_count(), io_mode == IoMode::Async, latency_stride);
        let started_ns = axhal::time::monotonic_time_nanos();
        match io_mode {
            IoMode::Sync => {
                self.sync_read_into(workload, buffer, &mut measurements, latency_stride)?
            }
            IoMode::Async => axtask::future::block_on(self.async_read_into(
                workload,
                buffer,
                &mut measurements,
                latency_stride,
            ))?,
        }
        let elapsed_ns = axhal::time::monotonic_time_nanos().saturating_sub(started_ns);
        measurements.checksum = update_checksum(CHECKSUM_SEED, buffer);
        Ok((elapsed_ns, measurements))
    }

    fn run_stage_read_request(
        &mut self,
        workload: ReadWorkload,
        io_mode: IoMode,
        request: usize,
        lba: u32,
        buffer: &mut [u8],
    ) -> Result<(StageDurations, DriverStageObservation), TestFailure> {
        debug_assert_eq!(buffer.len(), workload.request_bytes());
        debug_assert!(RANGE_DIAGNOSTIC_REQUEST_BLOCKS.contains(&workload.request_blocks));

        begin_driver_stage_observation();
        let read_result = match io_mode {
            IoMode::Sync => self.read_blocks(lba, buffer),
            IoMode::Async => {
                set_observation_enabled(true);
                let result = axtask::future::block_on(self.read_blocks_async(lba, buffer));
                set_observation_enabled(false);
                result
            }
        };
        let observation = finish_driver_stage_observation();
        read_result.map_err(|error| TestFailure::Io {
            mode: io_mode,
            request,
            error,
        })?;

        if io_mode == IoMode::Async {
            self.validate_async_observation(workload, request, async_observation())?;
        }
        let durations = StageDurations::from_observation(observation, workload.request_blocks)
            .map_err(|reason| TestFailure::StageObservation {
                workload: workload.name,
                mode: io_mode,
                request,
                reason,
                observation,
            })?;
        Ok((durations, observation))
    }

    fn run_range_stage_round(
        &mut self,
        scope: RangeScope,
        request_blocks: usize,
        io_mode: IoMode,
        expected_checksum: u64,
        buffer: &mut [u8],
    ) -> Result<RangeStageRound, TestFailure> {
        let workload = ReadWorkload {
            name: match request_blocks {
                1 => "range_cmd17",
                8 => "range_cmd18_8block",
                32 => "range_cmd18_32block",
                _ => unreachable!(),
            },
            request_blocks,
            total_blocks: scope.total_blocks(),
        };
        debug_assert_eq!(buffer.len(), workload.total_bytes());
        buffer.fill(0);
        let mut stages = StageAggregate::default();
        let started_ns = axhal::time::monotonic_time_nanos();
        for request in 0..workload.request_count() {
            let lba = TEST_START_LBA + (request * request_blocks) as u32;
            let offset = request * workload.request_bytes();
            let request_buffer = &mut buffer[offset..offset + workload.request_bytes()];
            let (durations, observation) = self.run_stage_read_request(
                workload,
                io_mode,
                request,
                lba,
                request_buffer,
            )?;
            stages.add(request, lba, durations, observation);
        }
        let elapsed_ns = axhal::time::monotonic_time_nanos().saturating_sub(started_ns);
        let checksum = update_checksum(CHECKSUM_SEED, buffer);
        if checksum != expected_checksum {
            return Err(TestFailure::DataMismatch {
                workload: workload.name,
                expected: expected_checksum,
                actual: checksum,
            });
        }
        Ok(RangeStageRound {
            elapsed_ns,
            checksum,
            stages,
        })
    }

    fn log_range_stage_round(
        scope: RangeScope,
        request_blocks: usize,
        round: usize,
        io_mode: IoMode,
        result: &RangeStageRound,
    ) {
        let stages = &result.stages;
        warn!(
            "SDMMC_RANGE_STAGE_RESULT scope={} start_lba={} end_lba={} request_blocks={} \
             round={} io={} elapsed_ns={} bytes={} requests={} throughput_mib_s_milli={} \
             api_avg_ns={} api_to_prepare_avg_ns={} prepare_avg_ns={} \
             prepare_to_cmd_avg_ns={} submit_avg_ns={} command_wait_avg_ns={} \
             data_wait_avg_ns={} validation_avg_ns={} validation_to_cleanup_avg_ns={} \
             cleanup_avg_ns={} copy_to_user_avg_ns={} command_to_terminal_avg_ns={} \
             software_avg_ns={} irq_to_terminal_samples={} irq_to_terminal_avg_ns={} \
             data_irq_to_acd_samples={} data_irq_to_acd_avg_ns={} checksum=0x{:016x} \
             data_ok=true terminal_ok=true",
            scope.name(),
            TEST_START_LBA,
            TEST_START_LBA + scope.total_blocks() as u32 - 1,
            request_blocks,
            round,
            io_mode.name(),
            result.elapsed_ns,
            scope.total_blocks() * SdMmc::BLOCK_SIZE,
            stages.requests,
            throughput_mib_s_milli(
                (scope.total_blocks() * SdMmc::BLOCK_SIZE) as u64,
                result.elapsed_ns,
            ),
            stages.average(stages.api_total_ns),
            stages.average(stages.api_to_prepare_ns),
            stages.average(stages.prepare_ns),
            stages.average(stages.prepare_to_cmd_ns),
            stages.average(stages.submit_ns),
            stages.average(stages.command_wait_ns),
            stages.average(stages.data_wait_ns),
            stages.average(stages.validation_ns),
            stages.average(stages.validation_to_cleanup_ns),
            stages.average(stages.cleanup_ns),
            stages.average(stages.copy_to_user_ns),
            stages.average(stages.command_to_terminal_ns),
            stages.average(stages.software_ns),
            stages.irq_to_terminal_samples,
            stages.irq_to_terminal_average(),
            stages.data_irq_to_acd_samples,
            stages.data_irq_to_acd_average(),
            result.checksum,
        );
        if let Some(slowest) = stages.slowest {
            warn!(
                "SDMMC_RANGE_STAGE_SLOWEST scope={} request_blocks={} round={} io={} \
                 request={} lba={} api_total_ns={} api_to_prepare_ns={} prepare_ns={} \
                 prepare_to_cmd_ns={} submit_ns={} command_wait_ns={} data_wait_ns={} \
                 validation_ns={} validation_to_cleanup_ns={} cleanup_ns={} \
                 copy_to_user_ns={} command_to_terminal_ns={} software_ns={} \
                 irq_to_terminal_ns={} data_irq_to_acd_ns={}",
                scope.name(),
                request_blocks,
                round,
                io_mode.name(),
                stages.slowest_request,
                stages.slowest_lba,
                slowest.api_total_ns,
                slowest.api_to_prepare_ns,
                slowest.prepare_ns,
                slowest.prepare_to_cmd_ns,
                slowest.submit_ns,
                slowest.command_wait_ns,
                slowest.data_wait_ns,
                slowest.validation_ns,
                slowest.validation_to_cleanup_ns,
                slowest.cleanup_ns,
                slowest.copy_to_user_ns,
                slowest.command_to_terminal_ns,
                slowest.software_ns,
                slowest.irq_to_terminal_ns,
                slowest.data_irq_to_acd_ns,
            );
        }
    }

    fn find_range_snapshot(
        snapshots: &[RangeStageSnapshot],
        scope: RangeScope,
        request_blocks: usize,
        io_mode: IoMode,
    ) -> RangeStageSnapshot {
        *snapshots
            .iter()
            .find(|snapshot| {
                snapshot.scope == scope
                    && snapshot.request_blocks == request_blocks
                    && snapshot.io_mode == io_mode
            })
            .expect("complete range-stage result matrix")
    }

    fn report_range_stage_comparisons(snapshots: &[RangeStageSnapshot]) {
        for scope in [RangeScope::Short, RangeScope::Wide] {
            for io_mode in [IoMode::Sync, IoMode::Async] {
                let one = Self::find_range_snapshot(snapshots, scope, 1, io_mode);
                let eight = Self::find_range_snapshot(snapshots, scope, 8, io_mode);
                let thirty_two = Self::find_range_snapshot(snapshots, scope, 32, io_mode);
                warn!(
                    "SDMMC_RANGE_REQUEST_COMPARISON scope={} start_lba={} end_lba={} io={} \
                     rounds_each={} block1_mib_s_milli={} block8_mib_s_milli={} \
                     block32_mib_s_milli={} block8_vs_block1_permille={} \
                     block32_vs_block1_permille={} block32_vs_block8_permille={} \
                     strict_same_lba_range=true",
                    scope.name(),
                    TEST_START_LBA,
                    TEST_START_LBA + scope.total_blocks() as u32 - 1,
                    io_mode.name(),
                    TEST_ROUNDS,
                    one.throughput_mib_s_milli,
                    eight.throughput_mib_s_milli,
                    thirty_two.throughput_mib_s_milli,
                    ratio_permille(
                        eight.throughput_mib_s_milli,
                        one.throughput_mib_s_milli,
                    ),
                    ratio_permille(
                        thirty_two.throughput_mib_s_milli,
                        one.throughput_mib_s_milli,
                    ),
                    ratio_permille(
                        thirty_two.throughput_mib_s_milli,
                        eight.throughput_mib_s_milli,
                    ),
                );
            }
        }

        for scope in [RangeScope::Short, RangeScope::Wide] {
            for request_blocks in RANGE_DIAGNOSTIC_REQUEST_BLOCKS {
                let sync =
                    Self::find_range_snapshot(snapshots, scope, request_blocks, IoMode::Sync);
                let asynchronous =
                    Self::find_range_snapshot(snapshots, scope, request_blocks, IoMode::Async);
                warn!(
                    "SDMMC_RANGE_ASYNC_COMPARISON scope={} request_blocks={} \
                     sync_mib_s_milli={} async_mib_s_milli={} \
                     async_vs_sync_permille={} sync_api_avg_ns={} async_api_avg_ns={} \
                     async_extra_api_ns={} async_irq_to_terminal_avg_ns={} \
                     data_ok=true terminal_ok=true",
                    scope.name(),
                    request_blocks,
                    sync.throughput_mib_s_milli,
                    asynchronous.throughput_mib_s_milli,
                    ratio_permille(
                        asynchronous.throughput_mib_s_milli,
                        sync.throughput_mib_s_milli,
                    ),
                    sync.api_average_ns,
                    asynchronous.api_average_ns,
                    asynchronous
                        .api_average_ns
                        .saturating_sub(sync.api_average_ns),
                    asynchronous.irq_to_terminal_average_ns,
                );
            }
        }

        for request_blocks in RANGE_DIAGNOSTIC_REQUEST_BLOCKS {
            let short = Self::find_range_snapshot(
                snapshots,
                RangeScope::Short,
                request_blocks,
                IoMode::Sync,
            );
            let wide = Self::find_range_snapshot(
                snapshots,
                RangeScope::Wide,
                request_blocks,
                IoMode::Sync,
            );
            let range_ratio = ratio_permille(
                wide.throughput_mib_s_milli,
                short.throughput_mib_s_milli,
            );
            let extra_api_ns = wide.api_average_ns.saturating_sub(short.api_average_ns);
            let extra_command_to_terminal_ns = wide
                .command_to_terminal_average_ns
                .saturating_sub(short.command_to_terminal_average_ns);
            let extra_software_ns = wide
                .software_average_ns
                .saturating_sub(short.software_average_ns);
            let command_share_permille = ratio_permille(extra_command_to_terminal_ns, extra_api_ns);
            let software_share_permille = ratio_permille(extra_software_ns, extra_api_ns);
            let classification = if range_ratio >= 950 {
                "no_material_range_slowdown"
            } else if command_share_permille >= 700 {
                "card_controller_or_bus_wait_dominant"
            } else if software_share_permille >= 700 {
                "driver_software_dominant"
            } else {
                "mixed_or_inconclusive"
            };
            warn!(
                "SDMMC_RANGE_ROOT_CAUSE request_blocks={} basis=sync_path \
                 short_mib_s_milli={} wide_mib_s_milli={} wide_vs_short_permille={} \
                 short_api_avg_ns={} wide_api_avg_ns={} extra_api_ns={} \
                 short_command_to_terminal_avg_ns={} wide_command_to_terminal_avg_ns={} \
                 extra_command_to_terminal_ns={} command_extra_share_permille={} \
                 short_software_avg_ns={} wide_software_avg_ns={} extra_software_ns={} \
                 software_extra_share_permille={} classification={} \
                 sd_card_internal_proven=false controller_or_bus_excluded=false \
                 next_discriminator=second_card_or_sd_bus_trace",
                request_blocks,
                short.throughput_mib_s_milli,
                wide.throughput_mib_s_milli,
                range_ratio,
                short.api_average_ns,
                wide.api_average_ns,
                extra_api_ns,
                short.command_to_terminal_average_ns,
                wide.command_to_terminal_average_ns,
                extra_command_to_terminal_ns,
                command_share_permille,
                short.software_average_ns,
                wide.software_average_ns,
                extra_software_ns,
                software_share_permille,
                classification,
            );
        }
    }

    fn run_range_stage_diagnostic(&mut self) -> Result<(), TestFailure> {
        warn!(
            "SDMMC_RANGE_STAGE begin start_lba={} scopes=short_128k:256blocks,wide_2m:4096blocks \
             request_blocks=1,8,32 rounds_each_mode={} \
             order=odd_round_sync_async,even_round_async_sync \
             strict_same_start_lba=true one_dma_command_per_request=true \
             stages=api,prepare,submit,command_wait,data_wait,validation,cleanup,copy,irq_resume",
            TEST_START_LBA,
            TEST_ROUNDS,
        );
        let mut snapshots = Vec::with_capacity(12);
        for scope in [RangeScope::Short, RangeScope::Wide] {
            let reference_workload = ReadWorkload {
                name: "range_reference_cmd18_32block",
                request_blocks: 32,
                total_blocks: scope.total_blocks(),
            };
            let mut buffer = vec![0u8; reference_workload.total_bytes()];
            let (_, reference) = self.run_timed_read_round(
                reference_workload,
                IoMode::Sync,
                None,
                &mut buffer,
            )?;
            let expected_checksum = reference.checksum;
            warn!(
                "SDMMC_RANGE_STAGE_REFERENCE scope={} start_lba={} end_lba={} \
                 blocks={} checksum=0x{:016x} data_ok=true",
                scope.name(),
                TEST_START_LBA,
                TEST_START_LBA + scope.total_blocks() as u32 - 1,
                scope.total_blocks(),
                expected_checksum,
            );

            for request_blocks in RANGE_DIAGNOSTIC_REQUEST_BLOCKS {
                let mut sync = RangeStageSummary::new(scope, request_blocks, IoMode::Sync);
                let mut asynchronous =
                    RangeStageSummary::new(scope, request_blocks, IoMode::Async);
                for round_index in 0..TEST_ROUNDS {
                    let round = round_index + 1;
                    let modes = if round_index.is_multiple_of(2) {
                        [IoMode::Sync, IoMode::Async]
                    } else {
                        [IoMode::Async, IoMode::Sync]
                    };
                    for io_mode in modes {
                        warn!(
                            "SDMMC_RANGE_STAGE_CASE_BEGIN scope={} request_blocks={} round={} \
                             io={} start_lba={} end_lba={}",
                            scope.name(),
                            request_blocks,
                            round,
                            io_mode.name(),
                            TEST_START_LBA,
                            TEST_START_LBA + scope.total_blocks() as u32 - 1,
                        );
                        let result = self.run_range_stage_round(
                            scope,
                            request_blocks,
                            io_mode,
                            expected_checksum,
                            &mut buffer,
                        )?;
                        Self::log_range_stage_round(
                            scope,
                            request_blocks,
                            round,
                            io_mode,
                            &result,
                        );
                        match io_mode {
                            IoMode::Sync => sync.add_round(&result),
                            IoMode::Async => asynchronous.add_round(&result),
                        }
                    }
                }
                snapshots.push(sync.report());
                snapshots.push(asynchronous.report());
            }
        }
        Self::report_range_stage_comparisons(&snapshots);
        warn!(
            "SDMMC_RANGE_STAGE PASS scopes=2 request_sizes=3 sync_rounds_each=5 \
             async_rounds_each=5 strict_same_lba_range=true data_ok=true terminal_ok=true"
        );
        Ok(())
    }

    fn run_legacy_short_region_control(&mut self) -> Result<(), TestFailure> {
        let reference_workload = ReadWorkload {
            name: "legacy_cmd17_128k",
            request_blocks: 32,
            total_blocks: LEGACY_CONTROL_BLOCKS,
        };
        let workload = ReadWorkload {
            request_blocks: 1,
            ..reference_workload
        };
        let mut buffer = vec![0u8; workload.total_bytes()];
        let (_, reference) =
            self.run_timed_read_round(reference_workload, IoMode::Sync, None, &mut buffer)?;
        let expected_checksum = reference.checksum;
        let (_, verify) =
            self.run_timed_read_round(reference_workload, IoMode::Sync, None, &mut buffer)?;
        if verify.checksum != expected_checksum {
            return Err(TestFailure::DataMismatch {
                workload: workload.name,
                expected: expected_checksum,
                actual: verify.checksum,
            });
        }

        warn!(
            "SDMMC_LEGACY_CONTROL begin start_lba={} end_lba={} region_blocks={} rounds={} \
             request_blocks=1 timing=whole_round per_request_timestamps=false",
            TEST_START_LBA,
            TEST_START_LBA + LEGACY_CONTROL_BLOCKS as u32 - 1,
            LEGACY_CONTROL_BLOCKS,
            TEST_ROUNDS,
        );
        let mut sync_rates = Vec::with_capacity(TEST_ROUNDS);
        let mut async_rates = Vec::with_capacity(TEST_ROUNDS);
        let mut sync_elapsed_total_ns = 0u64;
        let mut async_elapsed_total_ns = 0u64;
        for io_mode in [IoMode::Sync, IoMode::Async] {
            for round in 1..=TEST_ROUNDS {
                let (elapsed_ns, measurements) =
                    self.run_timed_read_round(workload, io_mode, None, &mut buffer)?;
                if measurements.checksum != expected_checksum {
                    return Err(TestFailure::DataMismatch {
                        workload: workload.name,
                        expected: expected_checksum,
                        actual: measurements.checksum,
                    });
                }
                let throughput = throughput_mib_s_milli(workload.total_bytes() as u64, elapsed_ns);
                warn!(
                    "SDMMC_LEGACY_CONTROL_RESULT round={} io={} elapsed_ns={} \
                     throughput_mib_s_milli={} checksum=0x{:016x} data_ok=true \
                     terminal_ok=true deadline_fallbacks={}",
                    round,
                    io_mode.name(),
                    elapsed_ns,
                    throughput,
                    measurements.checksum,
                    measurements.deadline_fallbacks,
                );
                match io_mode {
                    IoMode::Sync => {
                        sync_rates.push(throughput);
                        sync_elapsed_total_ns = sync_elapsed_total_ns.saturating_add(elapsed_ns);
                    }
                    IoMode::Async => {
                        async_rates.push(throughput);
                        async_elapsed_total_ns = async_elapsed_total_ns.saturating_add(elapsed_ns);
                    }
                }
            }
        }

        let sync = scalar_stats(&sync_rates);
        let asynchronous = scalar_stats(&async_rates);
        warn!(
            "SDMMC_LEGACY_CONTROL_SUMMARY rounds={} bytes_per_round={} \
             sync_aggregate_mib_s_milli={} async_aggregate_mib_s_milli={} \
             sync_mean_mib_s_milli={} sync_min_mib_s_milli={} sync_max_mib_s_milli={} \
             async_mean_mib_s_milli={} async_min_mib_s_milli={} async_max_mib_s_milli={} \
             expected_historical_sync_mib_s_milli=1369..1429 data_ok=true terminal_ok=true",
            TEST_ROUNDS,
            workload.total_bytes(),
            throughput_mib_s_milli(
                (workload.total_bytes() * TEST_ROUNDS) as u64,
                sync_elapsed_total_ns,
            ),
            throughput_mib_s_milli(
                (workload.total_bytes() * TEST_ROUNDS) as u64,
                async_elapsed_total_ns,
            ),
            sync.mean,
            sync.min,
            sync.max,
            asynchronous.mean,
            asynchronous.min,
            asynchronous.max,
        );
        Ok(())
    }

    fn run_sustained_timing_diagnostic(&mut self) -> Result<(), TestFailure> {
        let workload = ReadWorkload {
            name: "cmd17_sustained_8m",
            request_blocks: 1,
            total_blocks: SUSTAINED_DIAGNOSTIC_BLOCKS,
        };
        let mut buffer = vec![0u8; workload.total_bytes()];
        let mut whole_runs = 0usize;
        let mut instrumented_runs = 0usize;
        let mut whole_elapsed_total_ns = 0u64;
        let mut instrumented_elapsed_total_ns = 0u64;
        let mut all_latencies = Vec::with_capacity(workload.request_count() * 2);
        let mut expected_checksum = None;
        let modes = [None, Some(1), Some(1), None];

        warn!(
            "SDMMC_SUSTAINED_DIAGNOSTIC begin start_lba={} end_lba={} total_bytes={} \
             requests={} order=whole,per_request,per_request,whole",
            TEST_START_LBA,
            TEST_START_LBA + workload.total_blocks as u32 - 1,
            workload.total_bytes(),
            workload.request_count(),
        );
        for (index, latency_stride) in modes.into_iter().enumerate() {
            let (elapsed_ns, measurements) = self.run_timed_read_round(
                workload,
                IoMode::Sync,
                latency_stride,
                &mut buffer,
            )?;
            let expected = *expected_checksum.get_or_insert(measurements.checksum);
            if measurements.checksum != expected {
                return Err(TestFailure::DataMismatch {
                    workload: workload.name,
                    expected,
                    actual: measurements.checksum,
                });
            }

            let throughput = throughput_mib_s_milli(workload.total_bytes() as u64, elapsed_ns);
            let mode_name = if latency_stride.is_some() {
                "per_request"
            } else {
                "whole"
            };
            warn!(
                "SDMMC_SUSTAINED_DIAGNOSTIC_RESULT run={} timing={} elapsed_ns={} \
                 throughput_mib_s_milli={} latency_samples={} checksum=0x{:016x} data_ok=true",
                index + 1,
                mode_name,
                elapsed_ns,
                throughput,
                measurements.request_latencies_ns.len(),
                measurements.checksum,
            );
            if latency_stride.is_some() {
                for (request, &latency_ns) in measurements
                    .request_latencies_ns
                    .iter()
                    .enumerate()
                    .filter(|(_, latency_ns)| **latency_ns >= 1_000_000)
                    .take(8)
                {
                    warn!(
                        "SDMMC_SUSTAINED_SLOW_SAMPLE run={} request={} lba={} latency_ns={}",
                        index + 1,
                        request,
                        TEST_START_LBA + request as u32,
                        latency_ns,
                    );
                }
                instrumented_runs += 1;
                instrumented_elapsed_total_ns =
                    instrumented_elapsed_total_ns.saturating_add(elapsed_ns);
                all_latencies.extend_from_slice(&measurements.request_latencies_ns);
            } else {
                whole_runs += 1;
                whole_elapsed_total_ns = whole_elapsed_total_ns.saturating_add(elapsed_ns);
            }
        }

        all_latencies.sort_unstable();
        let latency = latency_stats(&all_latencies);
        let below_500us = all_latencies.partition_point(|&value| value < 500_000);
        let below_1ms = all_latencies.partition_point(|&value| value < 1_000_000);
        let below_3ms = all_latencies.partition_point(|&value| value < 3_000_000);
        debug_assert_eq!(whole_runs, instrumented_runs);
        let diagnostic_total_bytes = (workload.total_bytes() * whole_runs) as u64;
        let whole_throughput =
            throughput_mib_s_milli(diagnostic_total_bytes, whole_elapsed_total_ns);
        let instrumented_throughput =
            throughput_mib_s_milli(diagnostic_total_bytes, instrumented_elapsed_total_ns);
        warn!(
            "SDMMC_SUSTAINED_DIAGNOSTIC_SUMMARY bytes_per_run={} runs_per_mode=2 \
             whole_aggregate_mib_s_milli={} instrumented_aggregate_mib_s_milli={} \
             instrumentation_ratio_permille={} latency_samples={} request_avg_ns={} \
             request_p50_ns={} request_p95_ns={} request_p99_ns={} request_max_ns={} \
             bucket_lt_500us={} bucket_500us_to_1ms={} bucket_1ms_to_3ms={} \
             bucket_ge_3ms={} data_ok=true",
            workload.total_bytes(),
            whole_throughput,
            instrumented_throughput,
            if whole_throughput == 0 {
                0
            } else {
                (instrumented_throughput as u128 * 1_000 / whole_throughput as u128) as u64
            },
            all_latencies.len(),
            latency.average,
            latency.p50,
            latency.p95,
            latency.p99,
            latency.max,
            below_500us,
            below_1ms - below_500us,
            below_3ms - below_1ms,
            all_latencies.len() - below_3ms,
        );
        Ok(())
    }

    fn run_compute_baseline(&self, mode: ComputeMode) -> ComputeResult {
        let worker = ComputeWorker::spawn(mode);
        let started_ns = axhal::time::monotonic_time_nanos();
        worker.start(started_ns, COMPUTE_BASELINE_DURATION);
        worker.join()
    }

    fn run_io_case(
        &mut self,
        workload: ReadWorkload,
        io_mode: IoMode,
        compute_mode: Option<ComputeMode>,
        expected_checksum: u64,
    ) -> Result<IoRun, TestFailure> {
        let mut buffer = vec![0u8; workload.total_bytes()];
        let latency_stride = Some(MATRIX_LATENCY_SAMPLE_STRIDE);
        let mut measurements = IoMeasurements::new(
            workload.request_count(),
            io_mode == IoMode::Async,
            latency_stride,
        );
        let worker = compute_mode.map(ComputeWorker::spawn);
        let started_ns = axhal::time::monotonic_time_nanos();
        if let Some(worker) = &worker {
            worker.start(started_ns, COMPUTE_CASE_DURATION);
        }

        let io_result = match io_mode {
            IoMode::Sync => self.sync_read_into(
                workload,
                &mut buffer,
                &mut measurements,
                latency_stride,
            ),
            IoMode::Async => axtask::future::block_on(self.async_read_into(
                workload,
                &mut buffer,
                &mut measurements,
                latency_stride,
            )),
        };
        let elapsed_ns = axhal::time::monotonic_time_nanos().saturating_sub(started_ns);
        let compute = match worker {
            Some(worker) => Some(worker.stop_and_join()),
            None => None,
        };
        io_result?;
        measurements.checksum = update_checksum(CHECKSUM_SEED, &buffer);
        if measurements.checksum != expected_checksum {
            return Err(TestFailure::DataMismatch {
                workload: workload.name,
                expected: expected_checksum,
                actual: measurements.checksum,
            });
        }

        Ok(IoRun {
            elapsed_ns,
            measurements,
            compute,
        })
    }

    fn run_case_and_record(
        &mut self,
        workload: ReadWorkload,
        round: usize,
        aggregate: &mut CaseAggregate,
        expected_checksum: u64,
        baseline_work_per_s_milli: u64,
    ) -> Result<(), TestFailure> {
        warn!(
            "SDMMC_CONCURRENCY_CASE_BEGIN workload={} round={} case={} io={} compute={}",
            workload.name,
            round,
            aggregate.name,
            aggregate.io_mode.name(),
            aggregate.compute_mode.map_or("none", ComputeMode::name),
        );
        let run = self.run_io_case(
            workload,
            aggregate.io_mode,
            aggregate.compute_mode,
            expected_checksum,
        )?;
        aggregate.add(workload, round, run, baseline_work_per_s_milli);
        Ok(())
    }

    fn run_workload_matrix(
        &mut self,
        workload: ReadWorkload,
        baseline_no_yield: u64,
        baseline_yield: u64,
    ) -> Result<(), TestFailure> {
        warn!(
            "SDMMC_CONCURRENCY_REFERENCE_BEGIN workload={} start_lba={} end_lba={} \
             request_blocks={} total_blocks={} total_bytes={}",
            workload.name,
            TEST_START_LBA,
            TEST_START_LBA + workload.total_blocks as u32 - 1,
            workload.request_blocks,
            workload.total_blocks,
            workload.total_bytes(),
        );
        let reference = self.sync_read_measurements(workload, None)?;
        let expected_checksum = reference.checksum;

        let warmup = workload.with_total_blocks(workload.request_blocks * WARMUP_REQUESTS);
        let sync_warmup = self.sync_read_measurements(warmup, None)?;
        let async_warmup = axtask::future::block_on(self.async_read_measurements(warmup, None))?;
        if sync_warmup.checksum != async_warmup.checksum {
            return Err(TestFailure::DataMismatch {
                workload: workload.name,
                expected: sync_warmup.checksum,
                actual: async_warmup.checksum,
            });
        }
        warn!(
            "SDMMC_CONCURRENCY_REFERENCE_READY workload={} checksum=0x{:016x} \
             warmup_requests={} warmup_data_ok=true warmup_terminal_ok=true",
            workload.name,
            expected_checksum,
            WARMUP_REQUESTS,
        );

        let mut sync_only = CaseAggregate::new("sync_only", IoMode::Sync, None);
        let mut async_only = CaseAggregate::new("async_only", IoMode::Async, None);
        let mut sync_no_yield = CaseAggregate::new(
            "sync_compute_no_yield",
            IoMode::Sync,
            Some(ComputeMode::NoYield),
        );
        let mut async_no_yield = CaseAggregate::new(
            "async_compute_no_yield",
            IoMode::Async,
            Some(ComputeMode::NoYield),
        );
        let mut sync_yield = CaseAggregate::new(
            "sync_compute_yield",
            IoMode::Sync,
            Some(ComputeMode::YieldPerUnit),
        );
        let mut async_yield = CaseAggregate::new(
            "async_compute_yield",
            IoMode::Async,
            Some(ComputeMode::YieldPerUnit),
        );

        for round_index in 0..TEST_ROUNDS {
            let round = round_index + 1;
            if round_index.is_multiple_of(2) {
                self.run_case_and_record(
                    workload,
                    round,
                    &mut sync_only,
                    expected_checksum,
                    0,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut async_only,
                    expected_checksum,
                    0,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut sync_no_yield,
                    expected_checksum,
                    baseline_no_yield,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut async_no_yield,
                    expected_checksum,
                    baseline_no_yield,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut sync_yield,
                    expected_checksum,
                    baseline_yield,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut async_yield,
                    expected_checksum,
                    baseline_yield,
                )?;
            } else {
                self.run_case_and_record(
                    workload,
                    round,
                    &mut async_only,
                    expected_checksum,
                    0,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut sync_only,
                    expected_checksum,
                    0,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut async_no_yield,
                    expected_checksum,
                    baseline_no_yield,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut sync_no_yield,
                    expected_checksum,
                    baseline_no_yield,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut async_yield,
                    expected_checksum,
                    baseline_yield,
                )?;
                self.run_case_and_record(
                    workload,
                    round,
                    &mut sync_yield,
                    expected_checksum,
                    baseline_yield,
                )?;
            }
        }

        sync_only.report_summary(workload, 0);
        async_only.report_summary(workload, 0);
        sync_no_yield.report_summary(workload, baseline_no_yield);
        async_no_yield.report_summary(workload, baseline_no_yield);
        sync_yield.report_summary(workload, baseline_yield);
        async_yield.report_summary(workload, baseline_yield);
        Ok(())
    }

    fn run_concurrency_test_inner(&mut self) -> Result<(), TestFailure> {
        let largest_workload = WORKLOADS[WORKLOADS.len() - 1];
        let required_end_lba = TEST_START_LBA as u64 + largest_workload.total_blocks as u64;
        let dma_ready = self
            .dma_buffer
            .as_ref()
            .is_some_and(|buffer| buffer.size >= DMA_BUFFER_SIZE);
        let cpu_count = axhal::cpu_num();
        if cpu_count != 1 || required_end_lba > self.num_blocks || !dma_ready {
            return Err(TestFailure::InvalidEnvironment {
                cpu_count,
                card_blocks: self.num_blocks,
                required_end_lba,
                dma_ready,
            });
        }

        warn!(
            "SDMMC_CONCURRENCY_TEST begin destructive=false read_only=true start_lba={} \
             max_end_lba={} rounds={} smp={} scheduler_expected=RR \
             request_plan=CMD17_1block_2MiB,CMD18_8blocks_16MiB,CMD18_32blocks_16MiB \
             legacy_control=CMD17_1block_128KiB_x5 \
             strict_range_control=1,8,32blocks_x_sync,async_x5_short128KiB,wide2MiB \
             sustained_diagnostic=CMD17_1block_8MiB_ABBA one_request_in_flight=true \
             latency_sample_stride={} compute_iterations_per_unit={} \
             compute_window_ms={} compute_modes=no_yield,yield_per_unit \
             throughput_threshold_enforced=false filesystem_started=false",
            TEST_START_LBA,
            required_end_lba - 1,
            TEST_ROUNDS,
            cpu_count,
            MATRIX_LATENCY_SAMPLE_STRIDE,
            COMPUTE_ITERATIONS_PER_UNIT,
            COMPUTE_CASE_DURATION.as_millis(),
        );

        self.run_legacy_short_region_control()?;
        self.run_range_stage_diagnostic()?;
        self.run_sustained_timing_diagnostic()?;

        let mut no_yield_rates = Vec::with_capacity(TEST_ROUNDS);
        let mut yield_rates = Vec::with_capacity(TEST_ROUNDS);
        for round_index in 0..TEST_ROUNDS {
            let modes = if round_index.is_multiple_of(2) {
                [ComputeMode::NoYield, ComputeMode::YieldPerUnit]
            } else {
                [ComputeMode::YieldPerUnit, ComputeMode::NoYield]
            };
            for mode in modes {
                let result = self.run_compute_baseline(mode);
                warn!(
                    "SDMMC_COMPUTE_BASELINE round={} mode={} elapsed_ns={} work_units={} \
                     work_per_s_milli={} digest=0x{:016x} window_completed={}",
                    round_index + 1,
                    mode.name(),
                    result.elapsed_ns,
                    result.units,
                    result.work_per_s_milli,
                    result.digest,
                    result.window_completed,
                );
                match mode {
                    ComputeMode::NoYield => no_yield_rates.push(result.work_per_s_milli),
                    ComputeMode::YieldPerUnit => yield_rates.push(result.work_per_s_milli),
                }
            }
        }

        let no_yield_stats = scalar_stats(&no_yield_rates);
        let yield_stats = scalar_stats(&yield_rates);
        warn!(
            "SDMMC_COMPUTE_SUMMARY rounds={} no_yield_mean_per_s_milli={} \
             no_yield_min_per_s_milli={} no_yield_max_per_s_milli={} \
             no_yield_stddev_per_s_milli={} yield_mean_per_s_milli={} \
             yield_min_per_s_milli={} yield_max_per_s_milli={} \
             yield_stddev_per_s_milli={}",
            TEST_ROUNDS,
            no_yield_stats.mean,
            no_yield_stats.min,
            no_yield_stats.max,
            no_yield_stats.stddev,
            yield_stats.mean,
            yield_stats.min,
            yield_stats.max,
            yield_stats.stddev,
        );

        for workload in WORKLOADS {
            self.run_workload_matrix(workload, no_yield_stats.mean, yield_stats.mean)?;
        }
        Ok(())
    }

    /// Runs the feature-gated read/compute concurrency benchmark and then halts.
    pub fn run_concurrency_test(&mut self) -> ! {
        match self.run_concurrency_test_inner() {
            Ok(()) => {
                warn!(
                    "SDMMC_CONCURRENCY_TEST PASS destructive=false data_ok=true terminal_ok=true \
                     deadline_fallbacks=0 matrix_complete=true \
                     throughput_threshold_enforced=false filesystem_started=false"
                );
                halt_after_test("PASS")
            }
            Err(error) => {
                error.log();
                warn!(
                    "SDMMC_CONCURRENCY_TEST FAILED error={error:?} performance_results_valid=false \
                     destructive=false filesystem_started=false"
                );
                halt_after_test("FAILED")
            }
        }
    }
}

fn halt_after_test(status: &str) -> ! {
    warn!(
        "SDMMC_CONCURRENCY_TEST_HALT status={} reset_board_after_log_capture=true",
        status
    );
    loop {
        axhal::asm::halt();
    }
}
