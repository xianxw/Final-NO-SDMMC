// BEGIN SDMMC TEST ONLY
// Remove this entire file together with the test feature blocks and Makefile targets.

extern crate alloc;

#[cfg(feature = "sdmmc-async-read-test")]
use alloc::{vec, vec::Vec};
#[cfg(feature = "sdmmc-async-read-test")]
use core::{
    future::Future,
    pin::pin,
    task::{Context, Poll, Waker},
};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
#[cfg(feature = "sdmmc-error-irq-test")]
use core::sync::atomic::AtomicUsize;

use log::warn;

use super::*;

#[cfg(all(
    feature = "sdmmc-async-read-test",
    feature = "sdmmc-error-irq-test"
))]
compile_error!("select exactly one SDMMC diagnostic test feature");

const TEST_START_LBA: u32 = 2_099_200;
const TEST_REGION_BLOCKS: usize = 256;
#[cfg(feature = "sdmmc-async-read-test")]
const TEST_REGION_BYTES: usize = TEST_REGION_BLOCKS * SdMmc::BLOCK_SIZE;
#[cfg(feature = "sdmmc-async-read-test")]
const TEST_ROUNDS: usize = 5;
#[cfg(feature = "sdmmc-async-read-test")]
const READ_REQUEST_BLOCKS: [usize; 3] = [1, 4, 32];

static LAST_IRQ_ENTRY_NS: AtomicU64 = AtomicU64::new(0);
static LAST_IRQ_NOTIFY_NS: AtomicU64 = AtomicU64::new(0);
static LAST_IRQ_ENTRY_TO_RESUME_NS: AtomicU64 = AtomicU64::new(0);
static LAST_IRQ_NOTIFY_TO_RESUME_NS: AtomicU64 = AtomicU64::new(0);
static LAST_TERMINAL_RINTSTS: AtomicU32 = AtomicU32::new(0);
static LAST_TERMINAL_IDSTS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_RESUME_MINTSTS: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_RESUME_INTMASK: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_RESUME_IDINTEN: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_RESUME_CTRL: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_CONTEXT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_PRIORITY_75: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_THRESHOLD: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_PENDING: [AtomicU32; 5] = [const { AtomicU32::new(0) }; 5];
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_ENABLE: [AtomicU32; 5] = [const { AtomicU32::new(0) }; 5];
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_CLAIM_PROBE_ATTEMPTED: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_CLAIM_COUNT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_CLAIM_SOURCES: [AtomicU32; 8] = [const { AtomicU32::new(0) }; 8];
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_CLAIM_PRIORITIES: [AtomicU32; 8] = [const { AtomicU32::new(0) }; 8];
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_PLIC_PENDING_AFTER_COMPLETE: AtomicU32 = AtomicU32::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_SIP_AFTER_COMPLETE: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_SSTATUS: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_SIE: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_SIP: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static LAST_SCAUSE: AtomicUsize = AtomicUsize::new(0);
static LAST_OBSERVATION_READY: AtomicBool = AtomicBool::new(false);

#[cfg(feature = "sdmmc-error-irq-test")]
const VF2_PLIC_PADDR: usize = 0x0c00_0000;
#[cfg(feature = "sdmmc-error-irq-test")]
const VF2_PLIC_SDIO1_SOURCE: usize = 75;
#[cfg(feature = "sdmmc-error-irq-test")]
const PLIC_PENDING_OFFSET: usize = 0x001000;
#[cfg(feature = "sdmmc-error-irq-test")]
const PLIC_ENABLE_OFFSET: usize = 0x002000;
#[cfg(feature = "sdmmc-error-irq-test")]
const PLIC_ENABLE_CONTEXT_STRIDE: usize = 0x80;
#[cfg(feature = "sdmmc-error-irq-test")]
const PLIC_CONTEXT_OFFSET: usize = 0x200000;
#[cfg(feature = "sdmmc-error-irq-test")]
const PLIC_CONTEXT_STRIDE: usize = 0x1000;

#[derive(Clone, Copy)]
struct AsyncObservation {
    ready: bool,
    irq_entry_to_resume_ns: u64,
    irq_notify_to_resume_ns: u64,
    rintsts_bits: u32,
    idsts_bits: u32,
    #[cfg(feature = "sdmmc-error-irq-test")]
    mintsts_bits: u32,
    #[cfg(feature = "sdmmc-error-irq-test")]
    intmask_bits: u32,
    #[cfg(feature = "sdmmc-error-irq-test")]
    idinten_bits: u32,
    #[cfg(feature = "sdmmc-error-irq-test")]
    ctrl_bits: u32,
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_context: usize,
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_priority_75: u32,
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_threshold: u32,
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_pending: [u32; 5],
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_enable: [u32; 5],
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_claim_probe_attempted: bool,
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_claim_count: usize,
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_claim_sources: [u32; 8],
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_claim_priorities: [u32; 8],
    #[cfg(feature = "sdmmc-error-irq-test")]
    plic_pending_after_complete: u32,
    #[cfg(feature = "sdmmc-error-irq-test")]
    sip_after_complete: usize,
    #[cfg(feature = "sdmmc-error-irq-test")]
    sstatus: usize,
    #[cfg(feature = "sdmmc-error-irq-test")]
    sie: usize,
    #[cfg(feature = "sdmmc-error-irq-test")]
    sip: usize,
    #[cfg(feature = "sdmmc-error-irq-test")]
    scause: usize,
}

#[cfg(feature = "sdmmc-error-irq-test")]
const ERROR_IRQ_TRACE_CAPACITY: usize = 8;

#[cfg(feature = "sdmmc-error-irq-test")]
struct ErrorIrqTraceSlot {
    snapshot_ready: AtomicBool,
    generation: AtomicUsize,
    rintsts_bits: AtomicU32,
    idsts_bits: AtomicU32,
    mintsts_bits: AtomicU32,
    should_notify: AtomicBool,
    notify_attempted: AtomicBool,
    waiter_woken: AtomicBool,
}

#[cfg(feature = "sdmmc-error-irq-test")]
impl ErrorIrqTraceSlot {
    const fn new() -> Self {
        Self {
            snapshot_ready: AtomicBool::new(false),
            generation: AtomicUsize::new(0),
            rintsts_bits: AtomicU32::new(0),
            idsts_bits: AtomicU32::new(0),
            mintsts_bits: AtomicU32::new(0),
            should_notify: AtomicBool::new(false),
            notify_attempted: AtomicBool::new(false),
            waiter_woken: AtomicBool::new(false),
        }
    }

    fn reset(&self) {
        self.snapshot_ready.store(false, Ordering::Relaxed);
        self.generation.store(0, Ordering::Relaxed);
        self.rintsts_bits.store(0, Ordering::Relaxed);
        self.idsts_bits.store(0, Ordering::Relaxed);
        self.mintsts_bits.store(0, Ordering::Relaxed);
        self.should_notify.store(false, Ordering::Relaxed);
        self.notify_attempted.store(false, Ordering::Relaxed);
        self.waiter_woken.store(false, Ordering::Relaxed);
    }
}

#[cfg(feature = "sdmmc-error-irq-test")]
static ERROR_IRQ_HANDLER_ENTRY_COUNT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static ERROR_IRQ_SHOULD_NOTIFY_COUNT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static ERROR_IRQ_NOTIFY_COUNT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static ERROR_IRQ_WAITER_WOKEN_COUNT: AtomicUsize = AtomicUsize::new(0);
#[cfg(feature = "sdmmc-error-irq-test")]
static ERROR_IRQ_TRACE: [ErrorIrqTraceSlot; ERROR_IRQ_TRACE_CAPACITY] =
    [const { ErrorIrqTraceSlot::new() }; ERROR_IRQ_TRACE_CAPACITY];

#[cfg(feature = "sdmmc-error-irq-test")]
#[derive(Clone, Copy)]
struct ErrorIrqTraceSnapshot {
    snapshot_ready: bool,
    generation: usize,
    rintsts_bits: u32,
    idsts_bits: u32,
    mintsts_bits: u32,
    should_notify: bool,
    notify_attempted: bool,
    waiter_woken: bool,
}

pub(super) fn reset_async_observation() {
    LAST_IRQ_ENTRY_NS.store(0, Ordering::Relaxed);
    LAST_IRQ_NOTIFY_NS.store(0, Ordering::Relaxed);
    LAST_IRQ_ENTRY_TO_RESUME_NS.store(0, Ordering::Relaxed);
    LAST_IRQ_NOTIFY_TO_RESUME_NS.store(0, Ordering::Relaxed);
    LAST_TERMINAL_RINTSTS.store(0, Ordering::Relaxed);
    LAST_TERMINAL_IDSTS.store(0, Ordering::Relaxed);
    #[cfg(feature = "sdmmc-error-irq-test")]
    {
        LAST_RESUME_MINTSTS.store(0, Ordering::Relaxed);
        LAST_RESUME_INTMASK.store(0, Ordering::Relaxed);
        LAST_RESUME_IDINTEN.store(0, Ordering::Relaxed);
        LAST_RESUME_CTRL.store(0, Ordering::Relaxed);
        LAST_PLIC_CONTEXT.store(0, Ordering::Relaxed);
        LAST_PLIC_PRIORITY_75.store(0, Ordering::Relaxed);
        LAST_PLIC_THRESHOLD.store(0, Ordering::Relaxed);
        for value in &LAST_PLIC_PENDING {
            value.store(0, Ordering::Relaxed);
        }
        for value in &LAST_PLIC_ENABLE {
            value.store(0, Ordering::Relaxed);
        }
        LAST_PLIC_CLAIM_PROBE_ATTEMPTED.store(false, Ordering::Relaxed);
        LAST_PLIC_CLAIM_COUNT.store(0, Ordering::Relaxed);
        for value in &LAST_PLIC_CLAIM_SOURCES {
            value.store(0, Ordering::Relaxed);
        }
        for value in &LAST_PLIC_CLAIM_PRIORITIES {
            value.store(0, Ordering::Relaxed);
        }
        LAST_PLIC_PENDING_AFTER_COMPLETE.store(0, Ordering::Relaxed);
        LAST_SIP_AFTER_COMPLETE.store(0, Ordering::Relaxed);
        LAST_SSTATUS.store(0, Ordering::Relaxed);
        LAST_SIE.store(0, Ordering::Relaxed);
        LAST_SIP.store(0, Ordering::Relaxed);
        LAST_SCAUSE.store(0, Ordering::Relaxed);
        ERROR_IRQ_HANDLER_ENTRY_COUNT.store(0, Ordering::Relaxed);
        ERROR_IRQ_SHOULD_NOTIFY_COUNT.store(0, Ordering::Relaxed);
        ERROR_IRQ_NOTIFY_COUNT.store(0, Ordering::Relaxed);
        ERROR_IRQ_WAITER_WOKEN_COUNT.store(0, Ordering::Relaxed);
        for slot in &ERROR_IRQ_TRACE {
            slot.reset();
        }
    }
    LAST_OBSERVATION_READY.store(false, Ordering::Release);
}

pub(super) fn record_irq_wake(entry_ns: u64, notify_ns: u64) {
    LAST_IRQ_ENTRY_NS.store(entry_ns, Ordering::Release);
    LAST_IRQ_NOTIFY_NS.store(notify_ns, Ordering::Release);
}

pub(super) fn record_async_resume(rintsts: crate::regs::RIntSts, idsts: crate::regs::IdSts) {
    let resumed_ns = axhal::time::monotonic_time_nanos();
    let entry_ns = LAST_IRQ_ENTRY_NS.load(Ordering::Acquire);
    let notify_ns = LAST_IRQ_NOTIFY_NS.load(Ordering::Acquire);

    LAST_IRQ_ENTRY_TO_RESUME_NS.store(
        (entry_ns != 0)
            .then(|| resumed_ns.saturating_sub(entry_ns))
            .unwrap_or(0),
        Ordering::Relaxed,
    );
    LAST_IRQ_NOTIFY_TO_RESUME_NS.store(
        (notify_ns != 0)
            .then(|| resumed_ns.saturating_sub(notify_ns))
            .unwrap_or(0),
        Ordering::Relaxed,
    );
    LAST_TERMINAL_RINTSTS.store(rintsts.into_bits(), Ordering::Relaxed);
    LAST_TERMINAL_IDSTS.store(idsts.into_bits(), Ordering::Relaxed);
    #[cfg(not(feature = "sdmmc-error-irq-test"))]
    LAST_OBSERVATION_READY.store(true, Ordering::Release);
}

#[cfg(feature = "sdmmc-error-irq-test")]
pub(super) fn record_async_resume_registers(
    mintsts_bits: u32,
    intmask_bits: u32,
    idinten_bits: u32,
    ctrl_bits: u32,
) {
    LAST_RESUME_MINTSTS.store(mintsts_bits, Ordering::Relaxed);
    LAST_RESUME_INTMASK.store(intmask_bits, Ordering::Relaxed);
    LAST_RESUME_IDINTEN.store(idinten_bits, Ordering::Relaxed);
    LAST_RESUME_CTRL.store(ctrl_bits, Ordering::Relaxed);

    // This diagnostic is intentionally read-only. In particular, do not read
    // the PLIC claim/complete register because a claim changes gateway state.
    unsafe {
        core::arch::asm!("fence iorw, iorw", options(nostack, preserves_flags));
    }
    let context = (axhal::percpu::this_cpu_id() + 1) * 2;
    LAST_PLIC_CONTEXT.store(context, Ordering::Relaxed);
    LAST_PLIC_PRIORITY_75.store(
        read_plic_register(VF2_PLIC_SDIO1_SOURCE * core::mem::size_of::<u32>()),
        Ordering::Relaxed,
    );
    LAST_PLIC_THRESHOLD.store(
        read_plic_register(PLIC_CONTEXT_OFFSET + context * PLIC_CONTEXT_STRIDE),
        Ordering::Relaxed,
    );
    for (index, value) in LAST_PLIC_PENDING.iter().enumerate() {
        value.store(
            read_plic_register(
                PLIC_PENDING_OFFSET + index * core::mem::size_of::<u32>(),
            ),
            Ordering::Relaxed,
        );
    }
    for (index, value) in LAST_PLIC_ENABLE.iter().enumerate() {
        value.store(
            read_plic_register(
                PLIC_ENABLE_OFFSET
                    + context * PLIC_ENABLE_CONTEXT_STRIDE
                    + index * core::mem::size_of::<u32>(),
            ),
            Ordering::Relaxed,
        );
    }

    let (sstatus, sie, sip, scause): (usize, usize, usize, usize);
    unsafe {
        core::arch::asm!("csrr {}, sstatus", out(reg) sstatus, options(nomem, nostack));
        core::arch::asm!("csrr {}, sie", out(reg) sie, options(nomem, nostack));
        core::arch::asm!("csrr {}, sip", out(reg) sip, options(nomem, nostack));
        core::arch::asm!("csrr {}, scause", out(reg) scause, options(nomem, nostack));
    }
    LAST_SSTATUS.store(sstatus, Ordering::Relaxed);
    LAST_SIE.store(sie, Ordering::Relaxed);
    LAST_SIP.store(sip, Ordering::Relaxed);
    LAST_SCAUSE.store(scause, Ordering::Relaxed);

    if ERROR_IRQ_HANDLER_ENTRY_COUNT.load(Ordering::Acquire) == 0 {
        let irqs_were_enabled = axhal::asm::irqs_enabled();
        axhal::asm::disable_irqs();
        LAST_PLIC_CLAIM_PROBE_ATTEMPTED.store(true, Ordering::Relaxed);

        let claim_complete_offset =
            PLIC_CONTEXT_OFFSET + context * PLIC_CONTEXT_STRIDE + core::mem::size_of::<u32>();
        let mut claim_count = 0;
        for index in 0..LAST_PLIC_CLAIM_SOURCES.len() {
            let source = read_plic_register(claim_complete_offset);
            if source == 0 {
                break;
            }
            LAST_PLIC_CLAIM_SOURCES[index].store(source, Ordering::Relaxed);
            LAST_PLIC_CLAIM_PRIORITIES[index].store(
                read_plic_register(source as usize * core::mem::size_of::<u32>()),
                Ordering::Relaxed,
            );
            claim_count += 1;
        }
        LAST_PLIC_CLAIM_COUNT.store(claim_count, Ordering::Relaxed);

        for index in 0..claim_count {
            write_plic_register(
                claim_complete_offset,
                LAST_PLIC_CLAIM_SOURCES[index].load(Ordering::Relaxed),
            );
        }
        unsafe {
            core::arch::asm!("fence iorw, iorw", options(nostack, preserves_flags));
        }
        LAST_PLIC_PENDING_AFTER_COMPLETE.store(
            read_plic_register(
                PLIC_PENDING_OFFSET + plic_source_group() * core::mem::size_of::<u32>(),
            ),
            Ordering::Relaxed,
        );
        let sip_after_complete: usize;
        unsafe {
            core::arch::asm!(
                "csrr {}, sip",
                out(reg) sip_after_complete,
                options(nomem, nostack)
            );
        }
        LAST_SIP_AFTER_COMPLETE.store(sip_after_complete, Ordering::Relaxed);

        if irqs_were_enabled {
            axhal::asm::enable_irqs();
        }
    }
    LAST_OBSERVATION_READY.store(true, Ordering::Release);
}

#[cfg(feature = "sdmmc-error-irq-test")]
fn read_plic_register(offset: usize) -> u32 {
    let address = axhal::mem::phys_to_virt(axhal::mem::PhysAddr::from_usize(
        VF2_PLIC_PADDR + offset,
    ));
    unsafe { core::ptr::read_volatile(address.as_ptr_of::<u32>()) }
}

#[cfg(feature = "sdmmc-error-irq-test")]
fn write_plic_register(offset: usize, value: u32) {
    let address = axhal::mem::phys_to_virt(axhal::mem::PhysAddr::from_usize(
        VF2_PLIC_PADDR + offset,
    ));
    unsafe { core::ptr::write_volatile(address.as_mut_ptr_of::<u32>(), value) }
}

#[cfg(feature = "sdmmc-error-irq-test")]
const fn plic_source_group() -> usize {
    VF2_PLIC_SDIO1_SOURCE / u32::BITS as usize
}

#[cfg(feature = "sdmmc-error-irq-test")]
pub(super) fn begin_error_irq_handler_entry() -> usize {
    let ordinal = ERROR_IRQ_HANDLER_ENTRY_COUNT.fetch_add(1, Ordering::AcqRel);
    if let Some(slot) = ERROR_IRQ_TRACE.get(ordinal) {
        slot.reset();
    }
    ordinal
}

#[cfg(feature = "sdmmc-error-irq-test")]
pub(super) fn record_error_irq_initial_status(
    ordinal: usize,
    generation: usize,
    rintsts_bits: u32,
    idsts_bits: u32,
    mintsts_bits: u32,
    should_notify: bool,
) {
    if should_notify {
        ERROR_IRQ_SHOULD_NOTIFY_COUNT.fetch_add(1, Ordering::AcqRel);
    }
    let Some(slot) = ERROR_IRQ_TRACE.get(ordinal) else {
        return;
    };
    slot.generation.store(generation, Ordering::Relaxed);
    slot.rintsts_bits.store(rintsts_bits, Ordering::Relaxed);
    slot.idsts_bits.store(idsts_bits, Ordering::Relaxed);
    slot.mintsts_bits.store(mintsts_bits, Ordering::Relaxed);
    slot.should_notify
        .store(should_notify, Ordering::Relaxed);
    slot.snapshot_ready.store(true, Ordering::Release);
}

#[cfg(feature = "sdmmc-error-irq-test")]
pub(super) fn record_error_irq_notify(ordinal: usize, waiter_woken: bool) {
    ERROR_IRQ_NOTIFY_COUNT.fetch_add(1, Ordering::AcqRel);
    if waiter_woken {
        ERROR_IRQ_WAITER_WOKEN_COUNT.fetch_add(1, Ordering::AcqRel);
    }
    if let Some(slot) = ERROR_IRQ_TRACE.get(ordinal) {
        slot.waiter_woken.store(waiter_woken, Ordering::Relaxed);
        slot.notify_attempted.store(true, Ordering::Release);
    }
}

#[cfg(feature = "sdmmc-error-irq-test")]
fn error_irq_trace_counts() -> (usize, usize, usize, usize) {
    (
        ERROR_IRQ_HANDLER_ENTRY_COUNT.load(Ordering::Acquire),
        ERROR_IRQ_SHOULD_NOTIFY_COUNT.load(Ordering::Acquire),
        ERROR_IRQ_NOTIFY_COUNT.load(Ordering::Acquire),
        ERROR_IRQ_WAITER_WOKEN_COUNT.load(Ordering::Acquire),
    )
}

#[cfg(feature = "sdmmc-error-irq-test")]
fn error_irq_trace_snapshot(ordinal: usize) -> Option<ErrorIrqTraceSnapshot> {
    let slot = ERROR_IRQ_TRACE.get(ordinal)?;
    let snapshot_ready = slot.snapshot_ready.load(Ordering::Acquire);
    Some(ErrorIrqTraceSnapshot {
        snapshot_ready,
        generation: slot.generation.load(Ordering::Relaxed),
        rintsts_bits: slot.rintsts_bits.load(Ordering::Relaxed),
        idsts_bits: slot.idsts_bits.load(Ordering::Relaxed),
        mintsts_bits: slot.mintsts_bits.load(Ordering::Relaxed),
        should_notify: slot.should_notify.load(Ordering::Relaxed),
        notify_attempted: slot.notify_attempted.load(Ordering::Acquire),
        waiter_woken: slot.waiter_woken.load(Ordering::Relaxed),
    })
}

fn async_observation() -> AsyncObservation {
    let ready = LAST_OBSERVATION_READY.load(Ordering::Acquire);
    AsyncObservation {
        ready,
        irq_entry_to_resume_ns: LAST_IRQ_ENTRY_TO_RESUME_NS.load(Ordering::Relaxed),
        irq_notify_to_resume_ns: LAST_IRQ_NOTIFY_TO_RESUME_NS.load(Ordering::Relaxed),
        rintsts_bits: LAST_TERMINAL_RINTSTS.load(Ordering::Relaxed),
        idsts_bits: LAST_TERMINAL_IDSTS.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        mintsts_bits: LAST_RESUME_MINTSTS.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        intmask_bits: LAST_RESUME_INTMASK.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        idinten_bits: LAST_RESUME_IDINTEN.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        ctrl_bits: LAST_RESUME_CTRL.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_context: LAST_PLIC_CONTEXT.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_priority_75: LAST_PLIC_PRIORITY_75.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_threshold: LAST_PLIC_THRESHOLD.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_pending: core::array::from_fn(|index| {
            LAST_PLIC_PENDING[index].load(Ordering::Relaxed)
        }),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_enable: core::array::from_fn(|index| {
            LAST_PLIC_ENABLE[index].load(Ordering::Relaxed)
        }),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_claim_probe_attempted: LAST_PLIC_CLAIM_PROBE_ATTEMPTED.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_claim_count: LAST_PLIC_CLAIM_COUNT.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_claim_sources: core::array::from_fn(|index| {
            LAST_PLIC_CLAIM_SOURCES[index].load(Ordering::Relaxed)
        }),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_claim_priorities: core::array::from_fn(|index| {
            LAST_PLIC_CLAIM_PRIORITIES[index].load(Ordering::Relaxed)
        }),
        #[cfg(feature = "sdmmc-error-irq-test")]
        plic_pending_after_complete: LAST_PLIC_PENDING_AFTER_COMPLETE.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        sip_after_complete: LAST_SIP_AFTER_COMPLETE.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        sstatus: LAST_SSTATUS.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        sie: LAST_SIE.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        sip: LAST_SIP.load(Ordering::Relaxed),
        #[cfg(feature = "sdmmc-error-irq-test")]
        scause: LAST_SCAUSE.load(Ordering::Relaxed),
    }
}

#[cfg(feature = "sdmmc-async-read-test")]
fn first_mismatch(actual: &[u8], expected: &[u8]) -> Option<(usize, u8, u8)> {
    actual
        .iter()
        .copied()
        .zip(expected.iter().copied())
        .enumerate()
        .find_map(|(offset, (actual, expected))| {
            (actual != expected).then_some((offset, expected, actual))
        })
}

#[cfg(feature = "sdmmc-async-read-test")]
fn assert_data_equal(stage: &str, request_blocks: usize, actual: &[u8], expected: &[u8]) {
    if let Some((offset, expected, actual)) = first_mismatch(actual, expected) {
        panic!(
            "{stage} mismatch request_blocks={request_blocks} offset={offset} lba={} \
             byte_in_block={} expected=0x{expected:02x} actual=0x{actual:02x}",
            TEST_START_LBA + (offset / SdMmc::BLOCK_SIZE) as u32,
            offset % SdMmc::BLOCK_SIZE,
        );
    }
}

#[cfg(feature = "sdmmc-async-read-test")]
fn throughput_mib_s_milli(total_bytes: u64, elapsed_ns: u64) -> u64 {
    ((total_bytes as u128 * 1_000_000_000 * 1_000)
        / (elapsed_ns.max(1) as u128 * 1_048_576)) as u64
}

#[cfg(feature = "sdmmc-async-read-test")]
fn percentile(sorted: &[u64], percent: usize) -> u64 {
    let rank = (sorted.len() * percent).div_ceil(100);
    sorted[rank.saturating_sub(1)]
}

#[cfg(feature = "sdmmc-async-read-test")]
fn print_latency_stats(request_blocks: usize, kind: &str, mut samples: Vec<u64>) {
    samples.sort_unstable();
    let total = samples.iter().copied().sum::<u64>();
    warn!(
        "SDMMC_ASYNC_WAKE request_blocks={} latency={} samples={} average_ns={} min_ns={} \
         p50_ns={} p95_ns={} p99_ns={} max_ns={}",
        request_blocks,
        kind,
        samples.len(),
        total / samples.len() as u64,
        samples[0],
        percentile(&samples, 50),
        percentile(&samples, 95),
        percentile(&samples, 99),
        samples[samples.len() - 1],
    );
}

fn halt_after_test(name: &str, status: &str) -> ! {
    warn!(
        "SDMMC_TEST_HALT test={} status={} filesystem_started=false reset_board_for_next_image=true",
        name, status
    );
    loop {
        axhal::asm::halt();
    }
}

impl SdMmc {
    fn assert_test_region(&self) {
        let end_lba = TEST_START_LBA as u64 + TEST_REGION_BLOCKS as u64;
        assert!(
            end_lba <= self.num_blocks,
            "SDMMC test region exceeds card capacity"
        );
        assert!(
            self.dma_buffer
                .as_ref()
                .is_some_and(|buffer| buffer.size >= DMA_BUFFER_SIZE),
            "SDMMC tests require the IDMAC bounce buffer"
        );
    }

    #[cfg(feature = "sdmmc-async-read-test")]
    fn test_sync_read_round(&mut self, request_blocks: usize, buffer: &mut [u8]) -> u64 {
        buffer.fill(0);
        let request_bytes = request_blocks * Self::BLOCK_SIZE;
        let started_ns = axhal::time::monotonic_time_nanos();
        for request in 0..(TEST_REGION_BLOCKS / request_blocks) {
            let block_offset = request * request_blocks;
            let byte_offset = block_offset * Self::BLOCK_SIZE;
            self.read_blocks(
                TEST_START_LBA + block_offset as u32,
                &mut buffer[byte_offset..byte_offset + request_bytes],
            )
            .unwrap_or_else(|error| {
                panic!(
                    "sync read failed request_blocks={request_blocks} request={request}: {error:?}"
                )
            });
        }
        axhal::time::monotonic_time_nanos().saturating_sub(started_ns)
    }

    #[cfg(feature = "sdmmc-async-read-test")]
    fn test_async_read_round(
        &mut self,
        request_blocks: usize,
        buffer: &mut [u8],
        entry_latencies: &mut Vec<u64>,
        notify_latencies: &mut Vec<u64>,
    ) -> u64 {
        buffer.fill(0);
        let request_bytes = request_blocks * Self::BLOCK_SIZE;
        let started_ns = axhal::time::monotonic_time_nanos();
        axtask::future::block_on(async {
            for request in 0..(TEST_REGION_BLOCKS / request_blocks) {
                let block_offset = request * request_blocks;
                let byte_offset = block_offset * Self::BLOCK_SIZE;
                self.read_blocks_async(
                    TEST_START_LBA + block_offset as u32,
                    &mut buffer[byte_offset..byte_offset + request_bytes],
                )
                .await
                .unwrap_or_else(|error| {
                    panic!(
                        "async read failed request_blocks={request_blocks} request={request}: \
                         {error:?}"
                    )
                });

                let observation = async_observation();
                let rintsts = crate::regs::RIntSts::from_bits(observation.rintsts_bits);
                let idsts = crate::regs::IdSts::from_bits(observation.idsts_bits);
                let terminal_irq_ok = observation.ready
                    && observation.irq_entry_to_resume_ns != 0
                    && observation.irq_notify_to_resume_ns != 0
                    && rintsts.command_done()
                    && rintsts.data_transfer_over()
                    && idsts.ri()
                    && !rintsts.error()
                    && !idsts.ais()
                    && !idsts.ces()
                    && !idsts.du()
                    && !idsts.fbe()
                    && (request_blocks == 1 || rintsts.auto_command_done());
                assert!(
                    terminal_irq_ok,
                    "async terminal IRQ mismatch request_blocks={} request={} ready={} \
                     entry_to_resume_ns={} notify_to_resume_ns={} RINTSTS={:?} IDSTS={:?}",
                    request_blocks,
                    request,
                    observation.ready,
                    observation.irq_entry_to_resume_ns,
                    observation.irq_notify_to_resume_ns,
                    rintsts,
                    idsts,
                );
                entry_latencies.push(observation.irq_entry_to_resume_ns);
                notify_latencies.push(observation.irq_notify_to_resume_ns);
            }
        });
        axhal::time::monotonic_time_nanos().saturating_sub(started_ns)
    }

    #[cfg(feature = "sdmmc-async-read-test")]
    pub(super) fn run_async_read_and_cancel_test(&mut self) -> ! {
        self.assert_test_region();
        warn!(
            "SDMMC_ASYNC_READ_TEST begin start_lba={} end_lba={} region_blocks={} rounds={} \
             request_blocks=1,4,32 destructive=false throughput_tolerance_percent=15",
            TEST_START_LBA,
            TEST_START_LBA + TEST_REGION_BLOCKS as u32 - 1,
            TEST_REGION_BLOCKS,
            TEST_ROUNDS,
        );

        let mut reference = vec![0u8; TEST_REGION_BYTES];
        let mut verify = vec![0u8; TEST_REGION_BYTES];
        self.test_sync_read_round(32, &mut reference);
        self.test_sync_read_round(32, &mut verify);
        assert_data_equal("reference", 32, &verify, &reference);
        warn!("SDMMC_ASYNC_READ_TEST stage=reference status=ok");

        let mut throughput_consistent = true;
        let total_bytes = (TEST_REGION_BYTES * TEST_ROUNDS) as u64;
        for request_blocks in READ_REQUEST_BLOCKS {
            let mut sync_elapsed_ns = 0u64;
            for _ in 0..TEST_ROUNDS {
                sync_elapsed_ns = sync_elapsed_ns
                    .saturating_add(self.test_sync_read_round(request_blocks, &mut verify));
                assert_data_equal("sync", request_blocks, &verify, &reference);
            }

            let requests_per_round = TEST_REGION_BLOCKS / request_blocks;
            let sample_capacity = requests_per_round * TEST_ROUNDS;
            let mut entry_latencies = Vec::with_capacity(sample_capacity);
            let mut notify_latencies = Vec::with_capacity(sample_capacity);
            let mut async_elapsed_ns = 0u64;
            for _ in 0..TEST_ROUNDS {
                async_elapsed_ns = async_elapsed_ns.saturating_add(self.test_async_read_round(
                    request_blocks,
                    &mut verify,
                    &mut entry_latencies,
                    &mut notify_latencies,
                ));
                assert_data_equal("async", request_blocks, &verify, &reference);
            }

            let sync_throughput = throughput_mib_s_milli(total_bytes, sync_elapsed_ns);
            let async_throughput = throughput_mib_s_milli(total_bytes, async_elapsed_ns);
            let async_vs_sync_milli =
                (sync_elapsed_ns as u128 * 1_000 / async_elapsed_ns.max(1) as u128) as u64;
            let consistent = (850..=1_150).contains(&async_vs_sync_milli);
            throughput_consistent &= consistent;
            warn!(
                "SDMMC_ASYNC_READ_RESULT request_blocks={} sync_elapsed_ns={} \
                 async_elapsed_ns={} sync_throughput_mib_s={}.{:03} \
                 async_throughput_mib_s={}.{:03} async_vs_sync_percent={}.{} \
                 data_consistency=ok irq_terminal=ok throughput_consistency={}",
                request_blocks,
                sync_elapsed_ns,
                async_elapsed_ns,
                sync_throughput / 1_000,
                sync_throughput % 1_000,
                async_throughput / 1_000,
                async_throughput % 1_000,
                async_vs_sync_milli / 10,
                async_vs_sync_milli % 10,
                if consistent { "ok" } else { "failed" },
            );
            print_latency_stats(request_blocks, "irq_entry_to_future_resume", entry_latencies);
            print_latency_stats(request_blocks, "irq_notify_to_future_resume", notify_latencies);
        }

        warn!(
            "SDMMC_ASYNC_READ_TEST stage=read_compare status={} data_consistency=ok \
             irq_terminal=ok",
            if throughput_consistent {
                "ok"
            } else {
                "failed"
            }
        );

        let mut cancel_buffer = vec![0u8; MAX_DMA_BLOCKS * Self::BLOCK_SIZE];
        warn!(
            "SDMMC_CANCEL_TEST begin request_blocks={} policy=reset_then_permanent_fault",
            MAX_DMA_BLOCKS
        );
        let cancellation_poll = {
            let mut future = pin!(self.read_blocks_async(TEST_START_LBA, &mut cancel_buffer));
            let mut context = Context::from_waker(Waker::noop());
            Future::poll(future.as_mut(), &mut context)
        };
        assert!(
            matches!(cancellation_poll, Poll::Pending),
            "cancellation test future completed before it could be dropped: {cancellation_poll:?}"
        );

        let bmod = self.regs.bmod().read();
        let ctrl = self.regs.ctrl().read();
        let intmask = self.regs.intmask().read();
        let idinten = self.regs.idinten().read();
        let mut fault_probe = [0u8; Self::BLOCK_SIZE];
        let fault_result = self.read_block(TEST_START_LBA, &mut fault_probe);
        let cancellation_ok = self.idmac_faulted
            && !self.idmac_reset_failed
            && !bmod.de()
            && !bmod.swr()
            && !ctrl.use_internal_dmac()
            && !ctrl.dma_reset()
            && !ctrl.int_enable()
            && intmask.into_bits() == 0
            && idinten.into_bits() == 0
            && matches!(fault_result, Err(SdMmcError::DriverFaulted));
        warn!(
            "SDMMC_CANCEL_RESULT status={} future_first_poll=pending driver_faulted={} \
             reset_failed={} BMOD={:?} CTRL={:?} INTMASK=0x{:08x} IDINTEN=0x{:08x} \
             next_read={:?}",
            if cancellation_ok { "ok" } else { "failed" },
            self.idmac_faulted,
            self.idmac_reset_failed,
            bmod,
            ctrl,
            intmask.into_bits(),
            idinten.into_bits(),
            fault_result,
        );

        let overall_ok = throughput_consistent && cancellation_ok;
        warn!(
            "SDMMC_ASYNC_READ_TEST complete status={} cancellation={} destructive=false",
            if overall_ok { "ok" } else { "failed" },
            if cancellation_ok { "ok" } else { "failed" },
        );
        halt_after_test(
            "async_read_and_cancel",
            if overall_ok { "ok" } else { "failed" },
        )
    }

    #[cfg(feature = "sdmmc-error-irq-test")]
    pub(super) fn run_error_irq_test(&mut self) -> ! {
        self.assert_test_region();
        let invalid_lba = u32::try_from(self.num_blocks)
            .expect("error IRQ test requires capacity below the 32-bit LBA limit");
        let mut buffer = [0u8; Self::BLOCK_SIZE];
        warn!(
            "SDMMC_ERROR_IRQ_TEST begin invalid_read_lba={} num_blocks={} command=CMD17 \
             destructive=false expected_error=DRTO_or_card_error software_watchdog_ms={}",
            invalid_lba,
            self.num_blocks,
            IDMAC_DATA_WATCHDOG_TIMEOUT.as_millis(),
        );

        let started_ns = axhal::time::monotonic_time_nanos();
        let result = axtask::future::block_on(self.read_dma_chunk_async(invalid_lba, &mut buffer));
        let elapsed_ns = axhal::time::monotonic_time_nanos().saturating_sub(started_ns);
        let observation = async_observation();
        let rintsts = crate::regs::RIntSts::from_bits(observation.rintsts_bits);
        let idsts = crate::regs::IdSts::from_bits(observation.idsts_bits);
        let mintsts = crate::regs::MIntSts::from_bits(observation.mintsts_bits);
        let intmask = crate::regs::IntMask::from_bits(observation.intmask_bits);
        let idinten = crate::regs::IdIntEn::from_bits(observation.idinten_bits);
        let resume_ctrl = crate::regs::Ctrl::from_bits(observation.ctrl_bits);
        let plic_source_group = VF2_PLIC_SDIO1_SOURCE / u32::BITS as usize;
        let plic_source_mask = 1u32 << (VF2_PLIC_SDIO1_SOURCE % u32::BITS as usize);
        let plic_pending_75 = observation.plic_pending[plic_source_group] & plic_source_mask != 0;
        let plic_enabled_75 = observation.plic_enable[plic_source_group] & plic_source_mask != 0;
        let sstatus_sie = observation.sstatus & (1 << 1) != 0;
        let sie_seie = observation.sie & (1 << 9) != 0;
        let sip_seip = observation.sip & (1 << 9) != 0;
        warn!(
            "SDMMC_ERROR_IRQ_PLATFORM_SNAPSHOT context={} source={} priority={} threshold={} \
             pending={} enabled={} sstatus_sie={} sie_seie={} sip_seip={} \
             PENDING=[0x{:08x},0x{:08x},0x{:08x},0x{:08x},0x{:08x}] \
             ENABLE=[0x{:08x},0x{:08x},0x{:08x},0x{:08x},0x{:08x}] \
             SSTATUS=0x{:016x} SIE=0x{:016x} SIP=0x{:016x} SCAUSE=0x{:016x}",
            observation.plic_context,
            VF2_PLIC_SDIO1_SOURCE,
            observation.plic_priority_75,
            observation.plic_threshold,
            plic_pending_75,
            plic_enabled_75,
            sstatus_sie,
            sie_seie,
            sip_seip,
            observation.plic_pending[0],
            observation.plic_pending[1],
            observation.plic_pending[2],
            observation.plic_pending[3],
            observation.plic_pending[4],
            observation.plic_enable[0],
            observation.plic_enable[1],
            observation.plic_enable[2],
            observation.plic_enable[3],
            observation.plic_enable[4],
            observation.sstatus,
            observation.sie,
            observation.sip,
            observation.scause,
        );
        let plic_pending_75_after_complete =
            observation.plic_pending_after_complete & plic_source_mask != 0;
        let sip_seip_after_complete = observation.sip_after_complete & (1 << 9) != 0;
        warn!(
            "SDMMC_ERROR_IRQ_PLIC_CLAIM_PROBE attempted={} claim_count={} \
             CLAIMS=[{}:{},{}:{},{}:{},{}:{},{}:{},{}:{},{}:{},{}:{}] \
             pending75_after_complete={} sip_seip_after_complete={} \
             PENDING_GROUP_AFTER_COMPLETE=0x{:08x} SIP_AFTER_COMPLETE=0x{:016x}",
            observation.plic_claim_probe_attempted,
            observation.plic_claim_count,
            observation.plic_claim_sources[0],
            observation.plic_claim_priorities[0],
            observation.plic_claim_sources[1],
            observation.plic_claim_priorities[1],
            observation.plic_claim_sources[2],
            observation.plic_claim_priorities[2],
            observation.plic_claim_sources[3],
            observation.plic_claim_priorities[3],
            observation.plic_claim_sources[4],
            observation.plic_claim_priorities[4],
            observation.plic_claim_sources[5],
            observation.plic_claim_priorities[5],
            observation.plic_claim_sources[6],
            observation.plic_claim_priorities[6],
            observation.plic_claim_sources[7],
            observation.plic_claim_priorities[7],
            plic_pending_75_after_complete,
            sip_seip_after_complete,
            observation.plic_pending_after_complete,
            observation.sip_after_complete,
        );
        let (irq_entry_count, should_notify_count, notify_count, waiter_woken_count) =
            error_irq_trace_counts();
        let captured_irq_entries = irq_entry_count.min(ERROR_IRQ_TRACE_CAPACITY);
        let dropped_irq_entries = irq_entry_count.saturating_sub(ERROR_IRQ_TRACE_CAPACITY);
        warn!(
            "SDMMC_ERROR_IRQ_TRACE_SUMMARY handler_entries={} should_notify_count={} \
             notify_count={} waiter_woken_count={} captured_entries={} dropped_entries={}",
            irq_entry_count,
            should_notify_count,
            notify_count,
            waiter_woken_count,
            captured_irq_entries,
            dropped_irq_entries,
        );
        for ordinal in 0..captured_irq_entries {
            let trace = error_irq_trace_snapshot(ordinal).unwrap();
            warn!(
                "SDMMC_ERROR_IRQ_TRACE_ENTRY ordinal={} snapshot_ready={} generation={} \
                 RINTSTS=0x{:08x} IDSTS=0x{:08x} MINTSTS=0x{:08x} \
                 should_notify={} notify_attempted={} waiter_woken={}",
                ordinal,
                trace.snapshot_ready,
                trace.generation,
                trace.rintsts_bits,
                trace.idsts_bits,
                trace.mintsts_bits,
                trace.should_notify,
                trace.notify_attempted,
                trace.waiter_woken,
            );
        }
        let irq_error_observed = rintsts.error()
            || idsts.ais()
            || idsts.ces()
            || idsts.du()
            || idsts.fbe();
        let irq_config_ok = intmask.drto()
            && intmask.dto()
            && intmask.re()
            && idinten.ai()
            && idinten.ces()
            && idinten.ri()
            && resume_ctrl.int_enable()
            && resume_ctrl.use_internal_dmac();
        let irq_trace_ok = irq_entry_count != 0
            && should_notify_count != 0
            && notify_count != 0
            && waiter_woken_count != 0
            && dropped_irq_entries == 0;
        let mut fault_probe = [0u8; Self::BLOCK_SIZE];
        let fault_result = self.read_block(TEST_START_LBA, &mut fault_probe);
        let bmod = self.regs.bmod().read();
        let ctrl = self.regs.ctrl().read();
        let error_test_ok = matches!(result, Err(SdMmcError::Hardware))
            && observation.ready
            && observation.irq_entry_to_resume_ns != 0
            && observation.irq_notify_to_resume_ns != 0
            && irq_error_observed
            && irq_config_ok
            && irq_trace_ok
            && self.idmac_faulted
            && !self.idmac_reset_failed
            && !bmod.de()
            && !ctrl.use_internal_dmac()
            && matches!(fault_result, Err(SdMmcError::DriverFaulted));

        warn!(
            "SDMMC_ERROR_IRQ_RESULT status={} result={:?} elapsed_ns={} \
             irq_entry_to_resume_ns={} irq_notify_to_resume_ns={} RINTSTS={:?} IDSTS={:?} \
             MINTSTS_AT_RESUME={:?} INTMASK_AT_RESUME={:?} IDINTEN_AT_RESUME={:?} \
             CTRL_AT_RESUME={:?} irq_config_ok={} irq_trace_ok={} handler_entries={} \
             should_notify_count={} notify_count={} waiter_woken_count={} \
             driver_faulted={} reset_failed={} BMOD={:?} CTRL={:?} next_read={:?}",
            if error_test_ok { "ok" } else { "failed" },
            result,
            elapsed_ns,
            observation.irq_entry_to_resume_ns,
            observation.irq_notify_to_resume_ns,
            rintsts,
            idsts,
            mintsts,
            intmask,
            idinten,
            resume_ctrl,
            irq_config_ok,
            irq_trace_ok,
            irq_entry_count,
            should_notify_count,
            notify_count,
            waiter_woken_count,
            self.idmac_faulted,
            self.idmac_reset_failed,
            bmod,
            ctrl,
            fault_result,
        );
        warn!(
            "SDMMC_ERROR_IRQ_TEST complete status={} destructive=false",
            if error_test_ok { "ok" } else { "failed" }
        );
        halt_after_test(
            "error_irq",
            if error_test_ok { "ok" } else { "failed" },
        )
    }
}

// END SDMMC TEST ONLY
