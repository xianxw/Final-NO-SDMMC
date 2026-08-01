// Feature-gated destructive write/compute benchmark for VisionFive 2.

use alloc::{sync::Arc, vec, vec::Vec};
use core::{
    hint::black_box,
    sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering},
    time::Duration,
};

use log::warn;

use super::*;

const TEST_START_LBA: u32 = 2_099_200;
const TEST_REGION_BLOCKS: usize = 64;
const TEST_ROUNDS: usize = 5;
const COMPUTE_ITERATIONS_PER_UNIT: usize = 4_096;
const COMPUTE_DURATION: Duration = Duration::from_secs(2);
const PATTERN_SEED: u64 = 0x7f4a_7c15_6a09_e667;

const WORKLOADS: [WriteWorkload; 3] = [
    WriteWorkload {
        name: "cmd24_1block",
        request_blocks: 1,
        fixed_requests_per_round: 512,
    },
    WriteWorkload {
        name: "cmd25_8block",
        request_blocks: 8,
        fixed_requests_per_round: 512,
    },
    WriteWorkload {
        name: "cmd25_32block",
        request_blocks: 32,
        fixed_requests_per_round: 512,
    },
];

static OBSERVATION_ENABLED: AtomicBool = AtomicBool::new(false);
static OBSERVATION_CLOSED: AtomicBool = AtomicBool::new(true);
static DMA_OBSERVATION_CLOSED: AtomicBool = AtomicBool::new(true);
static API_START_NS: AtomicU64 = AtomicU64::new(0);
static TRANSFER_GENERATION: AtomicUsize = AtomicUsize::new(0);
static TRANSFER_START_NS: AtomicU64 = AtomicU64::new(0);
static IRQ_ENTRY_NS: AtomicU64 = AtomicU64::new(0);
static IRQ_EVENTS: AtomicUsize = AtomicUsize::new(0);
static IRQ_RINTSTS_OR: AtomicU32 = AtomicU32::new(0);
static IRQ_IDSTS_OR: AtomicU32 = AtomicU32::new(0);
static DMA_RESUME_NS: AtomicU64 = AtomicU64::new(0);
static DMA_TERMINAL_NS: AtomicU64 = AtomicU64::new(0);
static DMA_TIMED_OUT: AtomicBool = AtomicBool::new(false);
static DMA_TERMINAL_RINTSTS: AtomicU32 = AtomicU32::new(0);
static DMA_TERMINAL_IDSTS: AtomicU32 = AtomicU32::new(0);
static BUSY_READY: AtomicBool = AtomicBool::new(false);
static BUSY_ASYNC: AtomicBool = AtomicBool::new(false);
static BUSY_START_NS: AtomicU64 = AtomicU64::new(0);
static BUSY_END_NS: AtomicU64 = AtomicU64::new(0);
static BUSY_ATTEMPTS: AtomicUsize = AtomicUsize::new(0);
static BUSY_R1: AtomicU32 = AtomicU32::new(0);
static BUSY_SUCCESS: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy, Debug)]
struct RequestObservation {
    api_start_ns: u64,
    api_end_ns: u64,
    generation: usize,
    transfer_start_ns: u64,
    irq_entry_ns: u64,
    irq_events: usize,
    irq_rintsts_bits: u32,
    irq_idsts_bits: u32,
    dma_resume_ns: u64,
    dma_terminal_ns: u64,
    dma_timed_out: bool,
    dma_terminal_rintsts_bits: u32,
    dma_terminal_idsts_bits: u32,
    busy_ready: bool,
    busy_async: bool,
    busy_start_ns: u64,
    busy_end_ns: u64,
    busy_attempts: usize,
    busy_r1: u32,
    busy_success: bool,
}

fn begin_request_observation() {
    OBSERVATION_ENABLED.store(false, Ordering::Release);
    OBSERVATION_CLOSED.store(true, Ordering::Release);
    DMA_OBSERVATION_CLOSED.store(true, Ordering::Release);
    TRANSFER_GENERATION.store(0, Ordering::Relaxed);
    TRANSFER_START_NS.store(0, Ordering::Relaxed);
    IRQ_ENTRY_NS.store(0, Ordering::Relaxed);
    IRQ_EVENTS.store(0, Ordering::Relaxed);
    IRQ_RINTSTS_OR.store(0, Ordering::Relaxed);
    IRQ_IDSTS_OR.store(0, Ordering::Relaxed);
    DMA_RESUME_NS.store(0, Ordering::Relaxed);
    DMA_TERMINAL_NS.store(0, Ordering::Relaxed);
    DMA_TIMED_OUT.store(false, Ordering::Relaxed);
    DMA_TERMINAL_RINTSTS.store(0, Ordering::Relaxed);
    DMA_TERMINAL_IDSTS.store(0, Ordering::Relaxed);
    BUSY_READY.store(false, Ordering::Relaxed);
    BUSY_ASYNC.store(false, Ordering::Relaxed);
    BUSY_START_NS.store(0, Ordering::Relaxed);
    BUSY_END_NS.store(0, Ordering::Relaxed);
    BUSY_ATTEMPTS.store(0, Ordering::Relaxed);
    BUSY_R1.store(0, Ordering::Relaxed);
    BUSY_SUCCESS.store(false, Ordering::Relaxed);
    API_START_NS.store(axhal::time::monotonic_time_nanos(), Ordering::Relaxed);
    DMA_OBSERVATION_CLOSED.store(false, Ordering::Release);
    OBSERVATION_CLOSED.store(false, Ordering::Release);
    OBSERVATION_ENABLED.store(true, Ordering::Release);
}

fn finish_request_observation() -> RequestObservation {
    let api_end_ns = axhal::time::monotonic_time_nanos();
    OBSERVATION_CLOSED.store(true, Ordering::Release);
    DMA_OBSERVATION_CLOSED.store(true, Ordering::Release);
    OBSERVATION_ENABLED.store(false, Ordering::Release);

    RequestObservation {
        api_start_ns: API_START_NS.load(Ordering::Acquire),
        api_end_ns,
        generation: TRANSFER_GENERATION.load(Ordering::Acquire),
        transfer_start_ns: TRANSFER_START_NS.load(Ordering::Acquire),
        irq_entry_ns: IRQ_ENTRY_NS.load(Ordering::Acquire),
        irq_events: IRQ_EVENTS.load(Ordering::Acquire),
        irq_rintsts_bits: IRQ_RINTSTS_OR.load(Ordering::Acquire),
        irq_idsts_bits: IRQ_IDSTS_OR.load(Ordering::Acquire),
        dma_resume_ns: DMA_RESUME_NS.load(Ordering::Acquire),
        dma_terminal_ns: DMA_TERMINAL_NS.load(Ordering::Acquire),
        dma_timed_out: DMA_TIMED_OUT.load(Ordering::Acquire),
        dma_terminal_rintsts_bits: DMA_TERMINAL_RINTSTS.load(Ordering::Acquire),
        dma_terminal_idsts_bits: DMA_TERMINAL_IDSTS.load(Ordering::Acquire),
        busy_ready: BUSY_READY.load(Ordering::Acquire),
        busy_async: BUSY_ASYNC.load(Ordering::Acquire),
        busy_start_ns: BUSY_START_NS.load(Ordering::Acquire),
        busy_end_ns: BUSY_END_NS.load(Ordering::Acquire),
        busy_attempts: BUSY_ATTEMPTS.load(Ordering::Acquire),
        busy_r1: BUSY_R1.load(Ordering::Acquire),
        busy_success: BUSY_SUCCESS.load(Ordering::Acquire),
    }
}

pub(super) fn observation_enabled() -> bool {
    OBSERVATION_ENABLED.load(Ordering::Acquire)
        && !OBSERVATION_CLOSED.load(Ordering::Acquire)
}

pub(super) fn record_transfer_started(generation: usize) {
    if observation_enabled() {
        TRANSFER_GENERATION.store(generation, Ordering::Release);
        TRANSFER_START_NS.store(axhal::time::monotonic_time_nanos(), Ordering::Release);
    }
}

pub(super) fn record_sync_terminal(generation: usize, rintsts_bits: u32, idsts_bits: u32) {
    if !observation_enabled()
        || TRANSFER_GENERATION.load(Ordering::Acquire) != generation
    {
        return;
    }

    DMA_TERMINAL_NS.store(axhal::time::monotonic_time_nanos(), Ordering::Relaxed);
    DMA_TERMINAL_RINTSTS.store(rintsts_bits, Ordering::Relaxed);
    DMA_TERMINAL_IDSTS.store(idsts_bits, Ordering::Relaxed);
    DMA_OBSERVATION_CLOSED.store(true, Ordering::Release);
}

pub(super) fn record_irq(
    generation: usize,
    entry_ns: u64,
    rintsts_bits: u32,
    idsts_bits: u32,
) {
    if !observation_enabled()
        || DMA_OBSERVATION_CLOSED.load(Ordering::Acquire)
        || TRANSFER_GENERATION.load(Ordering::Acquire) != generation
    {
        return;
    }

    IRQ_ENTRY_NS.store(entry_ns, Ordering::Release);
    IRQ_EVENTS.fetch_add(1, Ordering::AcqRel);
    IRQ_RINTSTS_OR.fetch_or(rintsts_bits, Ordering::AcqRel);
    IRQ_IDSTS_OR.fetch_or(idsts_bits, Ordering::AcqRel);
}

pub(super) fn record_async_resume(
    generation: usize,
    timed_out: bool,
    rintsts_bits: u32,
    idsts_bits: u32,
) {
    if !observation_enabled()
        || TRANSFER_GENERATION.load(Ordering::Acquire) != generation
    {
        return;
    }

    let resumed_ns = axhal::time::monotonic_time_nanos();
    DMA_RESUME_NS.store(resumed_ns, Ordering::Relaxed);
    DMA_TERMINAL_NS.store(resumed_ns, Ordering::Relaxed);
    DMA_TIMED_OUT.store(timed_out, Ordering::Relaxed);
    DMA_TERMINAL_RINTSTS.store(rintsts_bits, Ordering::Relaxed);
    DMA_TERMINAL_IDSTS.store(idsts_bits, Ordering::Relaxed);
    DMA_OBSERVATION_CLOSED.store(true, Ordering::Release);
}

pub(super) fn record_write_busy_end(
    asynchronous: bool,
    started_ns: u64,
    attempts: usize,
    r1: u32,
    success: bool,
) {
    if !observation_enabled() {
        return;
    }

    let ended_ns = axhal::time::monotonic_time_nanos();
    BUSY_ASYNC.store(asynchronous, Ordering::Relaxed);
    BUSY_START_NS.store(started_ns, Ordering::Relaxed);
    BUSY_ATTEMPTS.store(attempts, Ordering::Relaxed);
    BUSY_R1.store(r1, Ordering::Relaxed);
    BUSY_SUCCESS.store(success, Ordering::Relaxed);
    BUSY_END_NS.store(ended_ns, Ordering::Relaxed);
    BUSY_READY.store(true, Ordering::Release);
}

#[derive(Clone, Copy)]
struct WriteWorkload {
    name: &'static str,
    request_blocks: usize,
    fixed_requests_per_round: usize,
}

impl WriteWorkload {
    const fn request_bytes(self) -> usize {
        self.request_blocks * SdMmc::BLOCK_SIZE
    }

    const fn fixed_total_bytes(self) -> usize {
        self.request_bytes() * self.fixed_requests_per_round
    }

    const fn requests_per_region(self) -> usize {
        TEST_REGION_BLOCKS / self.request_blocks
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum IoMode {
    Sync,
    Async,
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
        region_aligned: bool,
        driver_faulted: bool,
    },
    BackupIo(SdMmcError),
    BackupMismatch,
    Io {
        workload: &'static str,
        mode: IoMode,
        request: usize,
        error: SdMmcError,
    },
    Observation {
        workload: &'static str,
        mode: IoMode,
        request: usize,
        reason: &'static str,
        observation: RequestObservation,
    },
    DataMismatch {
        workload: &'static str,
        expected: u64,
        actual: u64,
    },
    RestoreIo {
        stage: &'static str,
        error: SdMmcError,
    },
    RestoreMismatch {
        stage: &'static str,
        expected: u64,
        actual: u64,
    },
    ComputeWindow {
        workload: &'static str,
        case: &'static str,
        elapsed_ns: u64,
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
                region_aligned,
                driver_faulted,
            } => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=invalid_environment cpu_count={} \
                 card_blocks={} required_end_lba={} dma_ready={} region_aligned={} \
                 driver_faulted={}",
                cpu_count,
                card_blocks,
                required_end_lba,
                dma_ready,
                region_aligned,
                driver_faulted,
            ),
            Self::BackupIo(error) => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=backup_io error={error:?} destructive_started=false"
            ),
            Self::BackupMismatch => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=backup_mismatch destructive_started=false"
            ),
            Self::Io {
                workload,
                mode,
                request,
                error,
            } => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=io workload={} mode={} request={} error={:?}",
                workload,
                mode.name(),
                request,
                error,
            ),
            Self::Observation {
                workload,
                mode,
                request,
                reason,
                observation,
            } => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=observation workload={} mode={} request={} \
                 reason={} generation={} transfer_start_ns={} irq_entry_ns={} irq_events={} \
                 dma_terminal_ns={} dma_resume_ns={} \
                 dma_timed_out={} busy_ready={} busy_async={} busy_start_ns={} \
                 busy_end_ns={} busy_attempts={} busy_r1=0x{:08x} busy_success={} \
                 RINTSTS=0x{:08x} IDSTS=0x{:08x}",
                workload,
                mode.name(),
                request,
                reason,
                observation.generation,
                observation.transfer_start_ns,
                observation.irq_entry_ns,
                observation.irq_events,
                observation.dma_terminal_ns,
                observation.dma_resume_ns,
                observation.dma_timed_out,
                observation.busy_ready,
                observation.busy_async,
                observation.busy_start_ns,
                observation.busy_end_ns,
                observation.busy_attempts,
                observation.busy_r1,
                observation.busy_success,
                observation.dma_terminal_rintsts_bits,
                observation.dma_terminal_idsts_bits,
            ),
            Self::DataMismatch {
                workload,
                expected,
                actual,
            } => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=data_mismatch workload={} expected=0x{:016x} \
                 actual=0x{:016x}",
                workload,
                expected,
                actual,
            ),
            Self::RestoreIo { stage, error } => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=restore_io stage={} error={:?} \
                 region_restored=false",
                stage,
                error,
            ),
            Self::RestoreMismatch {
                stage,
                expected,
                actual,
            } => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=restore_mismatch stage={} expected=0x{:016x} \
                 actual=0x{:016x} region_restored=false",
                stage,
                expected,
                actual,
            ),
            Self::ComputeWindow {
                workload,
                case,
                elapsed_ns,
            } => warn!(
                "SDMMC_WRITE_PERF_FAILURE kind=compute_window workload={} case={} \
                 elapsed_ns={} required_ns={}",
                workload,
                case,
                elapsed_ns,
                COMPUTE_DURATION.as_nanos(),
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
            alloc::format!("sdmmc-write-compute-{}", mode.name()),
        );

        let worker = Self { control, task };
        while !worker.control.ready.load(Ordering::Acquire) {
            axtask::yield_now();
        }
        worker
    }

    fn start(&self, started_ns: u64) {
        self.control.started_ns.store(started_ns, Ordering::Release);
        self.control.deadline_ns.store(
            started_ns + COMPUTE_DURATION.as_nanos() as u64,
            Ordering::Release,
        );
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

#[derive(Default)]
struct IoMeasurements {
    request_latencies_ns: Vec<u64>,
    api_to_transfer_ns: Vec<u64>,
    dma_transfer_ns: Vec<u64>,
    wait_end_to_busy_ns: Vec<u64>,
    busy_ns: Vec<u64>,
    irq_to_resume_ns: Vec<u64>,
    busy_attempts_total: u64,
    busy_attempts_max: usize,
    irq_completions: usize,
    irq_events: usize,
    deadline_fallbacks: usize,
    rintsts_or: u32,
    idsts_or: u32,
    busy_r1_or: u32,
}

impl IoMeasurements {
    fn with_capacity(requests: usize, asynchronous: bool) -> Self {
        Self {
            request_latencies_ns: Vec::with_capacity(requests),
            api_to_transfer_ns: Vec::with_capacity(requests),
            dma_transfer_ns: Vec::with_capacity(requests),
            wait_end_to_busy_ns: Vec::with_capacity(requests),
            busy_ns: Vec::with_capacity(requests),
            irq_to_resume_ns: Vec::with_capacity(if asynchronous { requests } else { 0 }),
            ..Self::default()
        }
    }

    fn add(
        &mut self,
        workload: WriteWorkload,
        mode: IoMode,
        request: usize,
        observation: RequestObservation,
    ) -> Result<(), TestFailure> {
        let invalid_common = observation.generation == 0
            || observation.transfer_start_ns < observation.api_start_ns
            || !observation.busy_ready
            || !observation.busy_success
            || observation.busy_async != (mode == IoMode::Async)
            || observation.api_start_ns == 0
            || observation.api_end_ns < observation.api_start_ns
            || observation.busy_end_ns < observation.busy_start_ns
            || observation.api_end_ns < observation.busy_end_ns
            || observation.busy_attempts == 0;
        if invalid_common {
            return Err(TestFailure::Observation {
                workload: workload.name,
                mode,
                request,
                reason: "invalid_busy_or_api_timeline",
                observation,
            });
        }

        if mode == IoMode::Async {
            let invalid_async = observation.dma_timed_out
                || observation.dma_resume_ns == 0
                || observation.dma_terminal_ns != observation.dma_resume_ns
                || observation.irq_entry_ns == 0
                || observation.irq_events == 0
                || observation.irq_entry_ns < observation.transfer_start_ns
                || observation.dma_resume_ns < observation.irq_entry_ns
                || observation.busy_start_ns < observation.dma_resume_ns;
            if invalid_async {
                return Err(TestFailure::Observation {
                    workload: workload.name,
                    mode,
                    request,
                    reason: "async_dma_irq_or_watchdog",
                    observation,
                });
            }
            self.irq_completions += 1;
            self.irq_events += observation.irq_events;
            self.irq_to_resume_ns
                .push(observation.dma_resume_ns - observation.irq_entry_ns);
            if observation.dma_timed_out {
                self.deadline_fallbacks += 1;
            }
        } else {
            if observation.dma_terminal_ns < observation.transfer_start_ns
                || observation.busy_start_ns < observation.dma_terminal_ns
            {
                return Err(TestFailure::Observation {
                    workload: workload.name,
                    mode,
                    request,
                    reason: "sync_dma_terminal_timeline",
                    observation,
                });
            }
            self.rintsts_or |= observation.irq_rintsts_bits;
            self.idsts_or |= observation.irq_idsts_bits;
        }

        self.request_latencies_ns
            .push(observation.api_end_ns - observation.api_start_ns);
        self.api_to_transfer_ns
            .push(observation.transfer_start_ns - observation.api_start_ns);
        let hardware_terminal_ns = if mode == IoMode::Async {
            observation.irq_entry_ns
        } else {
            observation.dma_terminal_ns
        };
        let wait_end_ns = if mode == IoMode::Async {
            observation.dma_resume_ns
        } else {
            observation.dma_terminal_ns
        };
        self.dma_transfer_ns
            .push(hardware_terminal_ns - observation.transfer_start_ns);
        self.wait_end_to_busy_ns
            .push(observation.busy_start_ns - wait_end_ns);
        self.busy_ns
            .push(observation.busy_end_ns - observation.busy_start_ns);
        self.busy_attempts_total = self
            .busy_attempts_total
            .saturating_add(observation.busy_attempts as u64);
        self.busy_attempts_max = self.busy_attempts_max.max(observation.busy_attempts);
        self.rintsts_or |= observation.irq_rintsts_bits;
        self.idsts_or |= observation.irq_idsts_bits;
        self.rintsts_or |= observation.dma_terminal_rintsts_bits;
        self.idsts_or |= observation.dma_terminal_idsts_bits;
        self.busy_r1_or |= observation.busy_r1;
        Ok(())
    }

    fn append(&mut self, other: &Self) {
        self.request_latencies_ns
            .extend_from_slice(&other.request_latencies_ns);
        self.api_to_transfer_ns
            .extend_from_slice(&other.api_to_transfer_ns);
        self.dma_transfer_ns
            .extend_from_slice(&other.dma_transfer_ns);
        self.wait_end_to_busy_ns
            .extend_from_slice(&other.wait_end_to_busy_ns);
        self.busy_ns.extend_from_slice(&other.busy_ns);
        self.irq_to_resume_ns
            .extend_from_slice(&other.irq_to_resume_ns);
        self.busy_attempts_total = self
            .busy_attempts_total
            .saturating_add(other.busy_attempts_total);
        self.busy_attempts_max = self.busy_attempts_max.max(other.busy_attempts_max);
        self.irq_completions += other.irq_completions;
        self.irq_events += other.irq_events;
        self.deadline_fallbacks += other.deadline_fallbacks;
        self.rintsts_or |= other.rintsts_or;
        self.idsts_or |= other.idsts_or;
        self.busy_r1_or |= other.busy_r1_or;
    }
}

struct IoRun {
    elapsed_ns: u64,
    requests: usize,
    bytes: u64,
    measurements: IoMeasurements,
    compute: Option<ComputeResult>,
}

struct CaseAggregate {
    name: &'static str,
    io_mode: IoMode,
    compute_mode: Option<ComputeMode>,
    elapsed_total_ns: u64,
    requests_total: usize,
    bytes_total: u64,
    throughput_per_round: Vec<u64>,
    compute_rates: Vec<u64>,
    measurements: IoMeasurements,
}

impl CaseAggregate {
    fn new(name: &'static str, io_mode: IoMode, compute_mode: Option<ComputeMode>) -> Self {
        Self {
            name,
            io_mode,
            compute_mode,
            elapsed_total_ns: 0,
            requests_total: 0,
            bytes_total: 0,
            throughput_per_round: Vec::with_capacity(TEST_ROUNDS),
            compute_rates: Vec::with_capacity(TEST_ROUNDS),
            measurements: IoMeasurements::default(),
        }
    }

    fn add(
        &mut self,
        workload: WriteWorkload,
        round: usize,
        run: IoRun,
        baseline_work_per_s_milli: u64,
        data_checksum: u64,
    ) {
        let throughput = throughput_mib_s_milli(run.bytes, run.elapsed_ns);
        let request_stats = stats(&run.measurements.request_latencies_ns);
        let api_to_transfer_stats = stats(&run.measurements.api_to_transfer_ns);
        let dma_stats = stats(&run.measurements.dma_transfer_ns);
        let wait_end_to_busy_stats = stats(&run.measurements.wait_end_to_busy_ns);
        let busy_stats = stats(&run.measurements.busy_ns);
        let irq_stats = stats(&run.measurements.irq_to_resume_ns);
        let compute_rate = run.compute.map_or(0, |result| result.work_per_s_milli);
        let compute_retention = ratio_permille(compute_rate, baseline_work_per_s_milli);
        let busy_share = ratio_permille(
            sum_u64(&run.measurements.busy_ns),
            sum_u64(&run.measurements.request_latencies_ns),
        );

        warn!(
            "SDMMC_WRITE_PERF_RESULT workload={} round={} case={} io={} compute={} \
             request_blocks={} requests={} bytes={} elapsed_ns={} throughput_mib_s_milli={} \
             request_avg_ns={} request_p95_ns={} request_p99_ns={} request_max_ns={} \
             api_to_transfer_avg_ns={} dma_hardware_avg_ns={} wait_end_to_busy_avg_ns={} \
             busy_policy={} busy_avg_ns={} busy_p95_ns={} busy_max_ns={} \
             busy_share_permille={} busy_attempts_avg_milli={} busy_attempts_max={} \
             irq_completions={} irq_events={} irq_to_resume_avg_ns={} irq_to_resume_p99_ns={} \
             irq_to_resume_max_ns={} deadline_fallbacks={} RINTSTS_OR=0x{:08x} \
             IDSTS_OR=0x{:08x} busy_R1_OR=0x{:08x} compute_elapsed_ns={} \
             compute_work_units={} compute_work_per_s_milli={} compute_retention_permille={} \
             compute_window_completed={} data_checksum=0x{:016x} data_ok=true \
             region_restored=true terminal_ok=true",
            workload.name,
            round,
            self.name,
            self.io_mode.name(),
            self.compute_mode.map_or("none", ComputeMode::name),
            workload.request_blocks,
            run.requests,
            run.bytes,
            run.elapsed_ns,
            throughput,
            request_stats.mean,
            request_stats.p95,
            request_stats.p99,
            request_stats.max,
            api_to_transfer_stats.mean,
            dma_stats.mean,
            wait_end_to_busy_stats.mean,
            if self.io_mode == IoMode::Async { "CMD13" } else { "DAT_STATUS" },
            busy_stats.mean,
            busy_stats.p95,
            busy_stats.max,
            busy_share,
            average_milli(
                run.measurements.busy_attempts_total,
                run.measurements.request_latencies_ns.len(),
            ),
            run.measurements.busy_attempts_max,
            run.measurements.irq_completions,
            run.measurements.irq_events,
            irq_stats.mean,
            irq_stats.p99,
            irq_stats.max,
            run.measurements.deadline_fallbacks,
            run.measurements.rintsts_or,
            run.measurements.idsts_or,
            run.measurements.busy_r1_or,
            run.compute.map_or(0, |result| result.elapsed_ns),
            run.compute.map_or(0, |result| result.units),
            compute_rate,
            compute_retention,
            run.compute.is_none_or(|result| result.window_completed),
            data_checksum,
        );

        self.elapsed_total_ns = self.elapsed_total_ns.saturating_add(run.elapsed_ns);
        self.requests_total = self.requests_total.saturating_add(run.requests);
        self.bytes_total = self.bytes_total.saturating_add(run.bytes);
        self.throughput_per_round.push(throughput);
        if let Some(compute) = run.compute {
            self.compute_rates.push(compute.work_per_s_milli);
        }
        self.measurements.append(&run.measurements);
    }

    fn report_summary(&self, workload: WriteWorkload, baseline_work_per_s_milli: u64) {
        let throughput = stats(&self.throughput_per_round);
        let request = stats(&self.measurements.request_latencies_ns);
        let api_to_transfer = stats(&self.measurements.api_to_transfer_ns);
        let dma = stats(&self.measurements.dma_transfer_ns);
        let wait_end_to_busy = stats(&self.measurements.wait_end_to_busy_ns);
        let busy = stats(&self.measurements.busy_ns);
        let irq = stats(&self.measurements.irq_to_resume_ns);
        let compute = stats(&self.compute_rates);
        warn!(
            "SDMMC_WRITE_PERF_SUMMARY workload={} case={} io={} compute={} rounds={} \
             request_blocks={} requests_total={} bytes_total={} \
             aggregate_mib_s_milli={} mean_mib_s_milli={} min_mib_s_milli={} \
             max_mib_s_milli={} request_avg_ns={} request_p95_ns={} request_p99_ns={} \
             request_max_ns={} api_to_transfer_avg_ns={} dma_hardware_avg_ns={} \
             wait_end_to_busy_avg_ns={} busy_policy={} busy_avg_ns={} \
             busy_p95_ns={} busy_p99_ns={} busy_max_ns={} busy_share_permille={} \
             busy_attempts_avg_milli={} busy_attempts_max={} irq_completions={} \
             irq_events={} irq_to_resume_avg_ns={} irq_to_resume_p99_ns={} \
             irq_to_resume_max_ns={} deadline_fallbacks={} RINTSTS_OR=0x{:08x} \
             IDSTS_OR=0x{:08x} busy_R1_OR=0x{:08x} compute_mean_per_s_milli={} \
             compute_min_per_s_milli={} compute_max_per_s_milli={} \
             compute_retention_permille={} data_ok=true region_restored=true terminal_ok=true",
            workload.name,
            self.name,
            self.io_mode.name(),
            self.compute_mode.map_or("none", ComputeMode::name),
            TEST_ROUNDS,
            workload.request_blocks,
            self.requests_total,
            self.bytes_total,
            throughput_mib_s_milli(self.bytes_total, self.elapsed_total_ns),
            throughput.mean,
            throughput.min,
            throughput.max,
            request.mean,
            request.p95,
            request.p99,
            request.max,
            api_to_transfer.mean,
            dma.mean,
            wait_end_to_busy.mean,
            if self.io_mode == IoMode::Async { "CMD13" } else { "DAT_STATUS" },
            busy.mean,
            busy.p95,
            busy.p99,
            busy.max,
            ratio_permille(
                sum_u64(&self.measurements.busy_ns),
                sum_u64(&self.measurements.request_latencies_ns),
            ),
            average_milli(
                self.measurements.busy_attempts_total,
                self.measurements.request_latencies_ns.len(),
            ),
            self.measurements.busy_attempts_max,
            self.measurements.irq_completions,
            self.measurements.irq_events,
            irq.mean,
            irq.p99,
            irq.max,
            self.measurements.deadline_fallbacks,
            self.measurements.rintsts_or,
            self.measurements.idsts_or,
            self.measurements.busy_r1_or,
            compute.mean,
            compute.min,
            compute.max,
            ratio_permille(compute.mean, baseline_work_per_s_milli),
        );
    }
}

#[derive(Clone, Copy, Default)]
struct Stats {
    mean: u64,
    min: u64,
    max: u64,
    p95: u64,
    p99: u64,
}

fn stats(values: &[u64]) -> Stats {
    if values.is_empty() {
        return Stats::default();
    }
    let mut sorted = values.to_vec();
    sorted.sort_unstable();
    Stats {
        mean: (sorted.iter().map(|&value| value as u128).sum::<u128>() / sorted.len() as u128)
            as u64,
        min: sorted[0],
        max: sorted[sorted.len() - 1],
        p95: percentile(&sorted, 95),
        p99: percentile(&sorted, 99),
    }
}

fn percentile(sorted: &[u64], percentile: usize) -> u64 {
    let index = (sorted.len() * percentile).div_ceil(100).saturating_sub(1);
    sorted[index.min(sorted.len() - 1)]
}

fn sum_u64(values: &[u64]) -> u64 {
    values
        .iter()
        .fold(0u64, |sum, &value| sum.saturating_add(value))
}

fn average_milli(total: u64, count: usize) -> u64 {
    if count == 0 {
        0
    } else {
        (total as u128 * 1_000 / count as u128) as u64
    }
}

fn ratio_permille(value: u64, baseline: u64) -> u64 {
    if baseline == 0 {
        0
    } else {
        (value as u128 * 1_000 / baseline as u128) as u64
    }
}

fn throughput_mib_s_milli(bytes: u64, elapsed_ns: u64) -> u64 {
    if elapsed_ns == 0 {
        0
    } else {
        (bytes as u128 * 1_000 * 1_000_000_000
            / (elapsed_ns as u128 * 1024 * 1024)) as u64
    }
}

fn rate_per_second_milli(units: u64, elapsed_ns: u64) -> u64 {
    if elapsed_ns == 0 {
        0
    } else {
        (units as u128 * 1_000 * 1_000_000_000 / elapsed_ns as u128) as u64
    }
}

fn checksum(data: &[u8]) -> u64 {
    data.iter().fold(0xcbf2_9ce4_8422_2325, |hash, &byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x100_0000_01b3)
    })
}

fn fill_pattern(data: &mut [u8], seed: u64) {
    for (block_index, block) in data.chunks_exact_mut(SdMmc::BLOCK_SIZE).enumerate() {
        let lba = u64::from(TEST_START_LBA) + block_index as u64;
        let mut state = seed ^ lba.wrapping_mul(0x9e37_79b9_7f4a_7c15);
        for (word_index, bytes) in block.chunks_exact_mut(8).enumerate() {
            state ^= (word_index as u64).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            state ^= state >> 30;
            state = state.wrapping_mul(0xbf58_476d_1ce4_e5b9);
            state ^= state >> 27;
            state = state.wrapping_mul(0x94d0_49bb_1331_11eb);
            state ^= state >> 31;
            bytes.copy_from_slice(&state.to_le_bytes());
        }
    }
}

impl SdMmc {
    fn sync_write_requests(
        &mut self,
        workload: WriteWorkload,
        pattern: &[u8],
        started_ns: u64,
        deadline_ns: Option<u64>,
    ) -> Result<(u64, usize, IoMeasurements), TestFailure> {
        let mut measurements =
            IoMeasurements::with_capacity(workload.fixed_requests_per_round, false);
        let mut request = 0usize;
        while request == 0
            || deadline_ns.map_or(
                request < workload.fixed_requests_per_round,
                |deadline_ns| axhal::time::monotonic_time_nanos() < deadline_ns,
            )
        {
            let region_request = request % workload.requests_per_region();
            let block_offset = region_request * workload.request_blocks;
            let byte_offset = block_offset * SdMmc::BLOCK_SIZE;
            let data = &pattern[byte_offset..byte_offset + workload.request_bytes()];
            begin_request_observation();
            let result = self.write_blocks(TEST_START_LBA + block_offset as u32, data);
            let observation = finish_request_observation();
            if let Err(error) = result {
                return Err(TestFailure::Io {
                    workload: workload.name,
                    mode: IoMode::Sync,
                    request,
                    error,
                });
            }
            measurements.add(workload, IoMode::Sync, request, observation)?;
            request += 1;
        }
        Ok((
            axhal::time::monotonic_time_nanos().saturating_sub(started_ns),
            request,
            measurements,
        ))
    }

    async fn async_write_requests(
        &mut self,
        workload: WriteWorkload,
        pattern: &[u8],
        started_ns: u64,
        deadline_ns: Option<u64>,
    ) -> Result<(u64, usize, IoMeasurements), TestFailure> {
        let mut measurements =
            IoMeasurements::with_capacity(workload.fixed_requests_per_round, true);
        let mut request = 0usize;
        while request == 0
            || deadline_ns.map_or(
                request < workload.fixed_requests_per_round,
                |deadline_ns| axhal::time::monotonic_time_nanos() < deadline_ns,
            )
        {
            let region_request = request % workload.requests_per_region();
            let block_offset = region_request * workload.request_blocks;
            let byte_offset = block_offset * SdMmc::BLOCK_SIZE;
            let data = &pattern[byte_offset..byte_offset + workload.request_bytes()];
            begin_request_observation();
            let result = self
                .write_blocks_async(TEST_START_LBA + block_offset as u32, data)
                .await;
            let observation = finish_request_observation();
            if let Err(error) = result {
                return Err(TestFailure::Io {
                    workload: workload.name,
                    mode: IoMode::Async,
                    request,
                    error,
                });
            }
            measurements.add(workload, IoMode::Async, request, observation)?;
            request += 1;
        }
        Ok((
            axhal::time::monotonic_time_nanos().saturating_sub(started_ns),
            request,
            measurements,
        ))
    }

    fn expected_region_after_requests(
        workload: WriteWorkload,
        backup: &[u8],
        pattern: &[u8],
        requests: usize,
    ) -> Vec<u8> {
        let mut expected = backup.to_vec();
        for region_request in 0..requests.min(workload.requests_per_region()) {
            let byte_offset = region_request * workload.request_bytes();
            expected[byte_offset..byte_offset + workload.request_bytes()]
                .copy_from_slice(&pattern[byte_offset..byte_offset + workload.request_bytes()]);
        }
        expected
    }

    fn read_region(&mut self) -> Result<Vec<u8>, SdMmcError> {
        let mut data = vec![0u8; TEST_REGION_BLOCKS * SdMmc::BLOCK_SIZE];
        self.read_blocks(TEST_START_LBA, &mut data)?;
        Ok(data)
    }

    fn restore_region(&mut self, backup: &[u8], stage: &'static str) -> Result<(), TestFailure> {
        OBSERVATION_ENABLED.store(false, Ordering::Release);
        self.write_blocks(TEST_START_LBA, backup)
            .map_err(|error| TestFailure::RestoreIo { stage, error })?;
        let restored = self
            .read_region()
            .map_err(|error| TestFailure::RestoreIo { stage, error })?;
        if restored != backup {
            return Err(TestFailure::RestoreMismatch {
                stage,
                expected: checksum(backup),
                actual: checksum(&restored),
            });
        }
        Ok(())
    }

    fn run_compute_baseline(&self, mode: ComputeMode) -> ComputeResult {
        let worker = ComputeWorker::spawn(mode);
        let started_ns = axhal::time::monotonic_time_nanos();
        worker.start(started_ns);
        worker.join()
    }

    fn run_write_case(
        &mut self,
        workload: WriteWorkload,
        round: usize,
        case_name: &'static str,
        io_mode: IoMode,
        compute_mode: Option<ComputeMode>,
        backup: &[u8],
    ) -> Result<(IoRun, u64), TestFailure> {
        self.restore_region(backup, "case_prepare")?;
        let mut pattern = vec![0u8; TEST_REGION_BLOCKS * SdMmc::BLOCK_SIZE];
        fill_pattern(
            &mut pattern,
            PATTERN_SEED
                ^ (round as u64) << 32
                ^ (workload.request_blocks as u64) << 16,
        );

        let worker = compute_mode.map(ComputeWorker::spawn);
        let case_started_ns = axhal::time::monotonic_time_nanos();
        let case_deadline_ns = compute_mode
            .map(|_| case_started_ns.saturating_add(COMPUTE_DURATION.as_nanos() as u64));
        if let Some(worker) = &worker {
            worker.start(case_started_ns);
        }
        let io_result = match io_mode {
            IoMode::Sync => {
                self.sync_write_requests(workload, &pattern, case_started_ns, case_deadline_ns)
            }
            IoMode::Async => {
                axtask::future::block_on(self.async_write_requests(
                    workload,
                    &pattern,
                    case_started_ns,
                    case_deadline_ns,
                ))
            }
        };
        let compute = worker.map(|worker| {
            if io_result.is_ok() {
                worker.join()
            } else {
                worker.stop_and_join()
            }
        });

        let expected = io_result.as_ref().ok().map(|(_, requests, _)| {
            Self::expected_region_after_requests(workload, backup, &pattern, *requests)
        });
        let data_result = if let Some(expected) = &expected {
            self.read_region()
                .map(|actual| (checksum(&actual), actual.as_slice() == expected.as_slice()))
        } else {
            Ok((0, false))
        };
        let restore_result = self.restore_region(backup, "case_complete");
        if let Err(error) = restore_result {
            return Err(error);
        }
        let (elapsed_ns, requests, measurements) = io_result?;
        let bytes = (requests as u64).saturating_mul(workload.request_bytes() as u64);
        let (actual_checksum, data_ok) = data_result.map_err(|error| TestFailure::Io {
            workload: workload.name,
            mode: io_mode,
            request: requests,
            error,
        })?;
        let expected_checksum = expected.as_deref().map_or(0, checksum);
        if !data_ok {
            return Err(TestFailure::DataMismatch {
                workload: workload.name,
                expected: expected_checksum,
                actual: actual_checksum,
            });
        }
        if let Some(result) = compute
            && !result.window_completed
        {
            return Err(TestFailure::ComputeWindow {
                workload: workload.name,
                case: case_name,
                elapsed_ns: result.elapsed_ns,
            });
        }

        Ok((
            IoRun {
                elapsed_ns,
                requests,
                bytes,
                measurements,
                compute,
            },
            expected_checksum,
        ))
    }

    fn run_workload_matrix(
        &mut self,
        workload: WriteWorkload,
        backup: &[u8],
        baseline_no_yield: u64,
        baseline_yield: u64,
    ) -> Result<(), TestFailure> {
        let mut cases = vec![
            CaseAggregate::new("sync_only", IoMode::Sync, None),
            CaseAggregate::new("async_only", IoMode::Async, None),
            CaseAggregate::new(
                "sync_compute_no_yield",
                IoMode::Sync,
                Some(ComputeMode::NoYield),
            ),
            CaseAggregate::new(
                "async_compute_no_yield",
                IoMode::Async,
                Some(ComputeMode::NoYield),
            ),
            CaseAggregate::new(
                "sync_compute_yield",
                IoMode::Sync,
                Some(ComputeMode::YieldPerUnit),
            ),
            CaseAggregate::new(
                "async_compute_yield",
                IoMode::Async,
                Some(ComputeMode::YieldPerUnit),
            ),
        ];

        warn!(
            "SDMMC_WRITE_PERF_WORKLOAD_BEGIN workload={} command={} request_blocks={} \
             pure_io_requests_per_round={} pure_io_bytes_per_round={} \
             pure_io_region_cycles_per_round={} compute_case_window_ms={} rounds={}",
            workload.name,
            if workload.request_blocks == 1 { "CMD24" } else { "CMD25" },
            workload.request_blocks,
            workload.fixed_requests_per_round,
            workload.fixed_total_bytes(),
            workload.fixed_requests_per_round / workload.requests_per_region(),
            COMPUTE_DURATION.as_millis(),
            TEST_ROUNDS,
        );

        for round_index in 0..TEST_ROUNDS {
            let order = if round_index.is_multiple_of(2) {
                [0usize, 1, 2, 3, 4, 5]
            } else {
                [1usize, 0, 3, 2, 5, 4]
            };
            for case_index in order {
                let case = &cases[case_index];
                warn!(
                    "SDMMC_WRITE_PERF_CASE_BEGIN workload={} round={} case={} io={} compute={} \
                     stop_policy={} target_requests={} target_duration_ms={} \
                     destructive=true region_restored_before_case=true",
                    workload.name,
                    round_index + 1,
                    case.name,
                    case.io_mode.name(),
                    case.compute_mode.map_or("none", ComputeMode::name),
                    if case.compute_mode.is_some() {
                        "fixed_duration_at_request_boundary"
                    } else {
                        "fixed_requests"
                    },
                    if case.compute_mode.is_some() {
                        0
                    } else {
                        workload.fixed_requests_per_round
                    },
                    if case.compute_mode.is_some() {
                        COMPUTE_DURATION.as_millis()
                    } else {
                        0
                    },
                );
                let baseline = match case.compute_mode {
                    None => 0,
                    Some(ComputeMode::NoYield) => baseline_no_yield,
                    Some(ComputeMode::YieldPerUnit) => baseline_yield,
                };
                let (run, data_checksum) = self.run_write_case(
                    workload,
                    round_index + 1,
                    case.name,
                    case.io_mode,
                    case.compute_mode,
                    backup,
                )?;
                cases[case_index].add(
                    workload,
                    round_index + 1,
                    run,
                    baseline,
                    data_checksum,
                );
            }
        }

        for case in &cases {
            let baseline = match case.compute_mode {
                None => 0,
                Some(ComputeMode::NoYield) => baseline_no_yield,
                Some(ComputeMode::YieldPerUnit) => baseline_yield,
            };
            case.report_summary(workload, baseline);
        }
        Ok(())
    }

    fn validate_write_test_environment(&self) -> Result<(), TestFailure> {
        let required_end_lba = u64::from(TEST_START_LBA) + TEST_REGION_BLOCKS as u64;
        let cpu_count = axhal::cpu_num();
        let dma_ready = self
            .dma_buffer
            .as_ref()
            .is_some_and(|buffer| buffer.size >= DMA_BUFFER_SIZE);
        let region_aligned = WORKLOADS.iter().all(|workload| {
            TEST_REGION_BLOCKS.is_multiple_of(workload.request_blocks)
                && workload
                    .fixed_requests_per_round
                    .is_multiple_of(workload.requests_per_region())
        });
        if cpu_count != 1
            || required_end_lba > self.num_blocks
            || !dma_ready
            || !region_aligned
            || self.idmac_faulted
        {
            return Err(TestFailure::InvalidEnvironment {
                cpu_count,
                card_blocks: self.num_blocks,
                required_end_lba,
                dma_ready,
                region_aligned,
                driver_faulted: self.idmac_faulted,
            });
        }
        Ok(())
    }

    /// Runs the feature-gated destructive write/compute benchmark and then halts.
    pub fn run_write_performance_test(&mut self) -> ! {
        let result = self.validate_write_test_environment().and_then(|()| {
            warn!(
                "SDMMC_WRITE_PERF_TEST begin destructive=true read_only=false start_lba={} \
                 end_lba={} region_blocks={} region_bytes={} rounds={} smp={} \
                 workloads=CMD24_1block,CMD25_8blocks,CMD25_32blocks \
                 cases=sync_only,async_only,sync_compute_no_yield,async_compute_no_yield,\
sync_compute_yield,async_compute_yield \
                 sync_busy_policy=DAT_STATUS async_busy_policy=CMD13 acmd23=false \
                 one_request_in_flight=true strict_same_lba_and_data=true compute_window_ms={} \
                 power_loss_can_corrupt_region=true requires_spare_card_or_reserved_region=true \
                 filesystem_started=false",
                TEST_START_LBA,
                TEST_START_LBA + TEST_REGION_BLOCKS as u32 - 1,
                TEST_REGION_BLOCKS,
                TEST_REGION_BLOCKS * SdMmc::BLOCK_SIZE,
                TEST_ROUNDS,
                axhal::cpu_num(),
                COMPUTE_DURATION.as_millis(),
            );

            let backup = self.read_region().map_err(TestFailure::BackupIo)?;
            let backup_verify = self.read_region().map_err(TestFailure::BackupIo)?;
            if backup != backup_verify {
                return Err(TestFailure::BackupMismatch);
            }
            warn!(
                "SDMMC_WRITE_PERF_TEST backup_verified start_lba={} blocks={} \
                 checksum=0x{:016x}",
                TEST_START_LBA,
                TEST_REGION_BLOCKS,
                checksum(&backup),
            );

            let matrix_result = (|| {
                let mut no_yield_rates = Vec::with_capacity(TEST_ROUNDS);
                let mut yield_rates = Vec::with_capacity(TEST_ROUNDS);
                for round_index in 0..TEST_ROUNDS {
                    let modes = if round_index.is_multiple_of(2) {
                        [ComputeMode::NoYield, ComputeMode::YieldPerUnit]
                    } else {
                        [ComputeMode::YieldPerUnit, ComputeMode::NoYield]
                    };
                    for mode in modes {
                        let compute = self.run_compute_baseline(mode);
                        warn!(
                            "SDMMC_WRITE_COMPUTE_BASELINE round={} mode={} elapsed_ns={} \
                             work_units={} work_per_s_milli={} digest=0x{:016x} \
                             window_completed={}",
                            round_index + 1,
                            mode.name(),
                            compute.elapsed_ns,
                            compute.units,
                            compute.work_per_s_milli,
                            compute.digest,
                            compute.window_completed,
                        );
                        if !compute.window_completed {
                            return Err(TestFailure::ComputeWindow {
                                workload: "compute_baseline",
                                case: mode.name(),
                                elapsed_ns: compute.elapsed_ns,
                            });
                        }
                        match mode {
                            ComputeMode::NoYield => {
                                no_yield_rates.push(compute.work_per_s_milli)
                            }
                            ComputeMode::YieldPerUnit => {
                                yield_rates.push(compute.work_per_s_milli)
                            }
                        }
                    }
                }
                let no_yield = stats(&no_yield_rates);
                let yielding = stats(&yield_rates);
                warn!(
                    "SDMMC_WRITE_COMPUTE_SUMMARY rounds={} no_yield_mean_per_s_milli={} \
                     no_yield_min_per_s_milli={} no_yield_max_per_s_milli={} \
                     yield_mean_per_s_milli={} yield_min_per_s_milli={} \
                     yield_max_per_s_milli={}",
                    TEST_ROUNDS,
                    no_yield.mean,
                    no_yield.min,
                    no_yield.max,
                    yielding.mean,
                    yielding.min,
                    yielding.max,
                );

                for workload in WORKLOADS {
                    self.run_workload_matrix(
                        workload,
                        &backup,
                        no_yield.mean,
                        yielding.mean,
                    )?;
                }
                Ok(())
            })();

            let final_restore = self.restore_region(&backup, "final");
            match (matrix_result, final_restore) {
                (_, Err(restore_error)) => Err(restore_error),
                (Err(matrix_error), Ok(())) => Err(matrix_error),
                (Ok(()), Ok(())) => Ok(()),
            }
        });

        match result {
            Ok(()) => {
                warn!(
                    "SDMMC_WRITE_PERF_TEST PASS cases=90 destructive=true data_ok=true \
                     region_restored=true terminal_ok=true deadline_fallbacks=0 \
                     matrix_complete=true acmd23=false filesystem_started=false"
                );
                halt_after_write_test("PASS")
            }
            Err(error) => {
                error.log();
                warn!(
                    "SDMMC_WRITE_PERF_TEST FAILED performance_results_valid=false \
                     destructive=true filesystem_started=false"
                );
                halt_after_write_test("FAILED")
            }
        }
    }
}

fn halt_after_write_test(status: &str) -> ! {
    warn!(
        "SDMMC_WRITE_PERF_TEST_HALT status={} reset_board_after_log_capture=true",
        status
    );
    loop {
        axhal::asm::halt();
    }
}
