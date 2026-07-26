use core::{
    alloc::Layout,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering},
    task::Poll,
    time::Duration,
};

use axtask::WaitQueue;
use log::{debug, info, trace, warn};
use volatile::VolatilePtr;

use crate::{
    cmd::{Command, DataXfer},
    dma::{DMABuffer, DMAInfo, IdmacDescriptor, alloc_coherent, dealloc_coherent},
    regs::{ClkDiv, ClkEna, RegisterBlock, RegisterBlockVolatileFieldAccess},
    utils::{Cid, CsdV2, R1CardStatus, R1CurrentState},
};

#[cfg(feature = "sdmmc-concurrency-test")]
#[path = "sdmmc_concurrency_test.rs"]
mod concurrency_test;

// VisionFive 2 firmware configures SDIO1 CIU as PLL2 / 3 / 8 = 49.5 MHz.
// For CLKDIV=n, DW-MMC outputs CIU / (2*n) to the card.
const VISIONFIVE2_SDIO_CIU_CLOCK_HZ: u32 = 49_500_000;
const IDENTIFICATION_CLOCK_DIVIDER: u8 = 100;
const DEFAULT_SPEED_CLOCK_DIVIDER: u8 = 1;
const MAX_DMA_BLOCKS: usize = 32;
const DMA_BUFFER_SIZE: usize = MAX_DMA_BLOCKS * 512;
const IDMAC_DESCRIPTOR_BUFFER_SIZE: usize = 8 * 512;
const MIN_MULTI_BLOCK_READ_BLOCKS: usize = 4;
const PRE_SUBMIT_IDLE_SPIN_TIMEOUT: Duration = Duration::from_micros(100);
const START_CMD_SPIN_TIMEOUT: Duration = Duration::from_millis(1);
const IDMAC_COMMAND_TIMEOUT: Duration = Duration::from_secs(2);
// Fallback only: controller TMOUT errors normally wake the waiter through IRQ.
const IDMAC_DATA_WATCHDOG_TIMEOUT: Duration = Duration::from_secs(5);
const ASYNC_WRITE_BUSY_TIMEOUT: Duration = Duration::from_secs(5);

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
static ASYNC_WRITE_BUSY_SEQUENCE: AtomicUsize = AtomicUsize::new(0);

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

    #[cfg(feature = "sdmmc-concurrency-test")]
    fn current_generation(&self) -> usize {
        self.generation.load(Ordering::Acquire)
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

async fn cooperative_yield_once() {
    let mut yielded = false;
    core::future::poll_fn(move |cx| {
        if yielded {
            Poll::Ready(())
        } else {
            yielded = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    })
    .await
}

struct IdmacTransferContext {
    cmd: crate::regs::Cmd,
    arg: u32,
    expects_r1: bool,
    generation: usize,
    dma_desc_info: DMAInfo,
    layout: Layout,
    desc_ptr: *mut IdmacDescriptor,
    descriptor_count: usize,
}

/// Errors returned by SD/MMC initialization and block transfers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdMmcError {
    /// Card initialization did not reach a usable transfer state.
    InitializationFailed,
    /// The card is outside the supported SDHC/SDXC block-addressed subset.
    UnsupportedCard,
    /// The card reported one or more R1 error bits.
    CardStatus(R1CardStatus),
    /// CMD55 completed without the APP_CMD acceptance bit.
    AppCommandRejected(R1CardStatus),
    /// The transfer buffer is empty or is not block aligned.
    InvalidParameter,
    /// The requested block range exceeds the detected card capacity.
    OutOfRange,
    /// IDMAC was not initialized, so block transfers are unavailable.
    DmaUnavailable,
    /// A coherent DMA descriptor allocation failed.
    DmaAllocation,
    /// A DMA buffer or descriptor address cannot be represented by this IDMAC.
    DmaAddress,
    /// A previous submitted transfer failed or was cancelled.
    DriverFaulted,
    /// The command state machine did not become idle before submission.
    CommandBusy,
    /// The data state machine did not become idle before submission.
    DataBusy,
    /// The controller did not retain the published descriptor configuration.
    DescriptorPublication,
    /// The controller did not accept START_CMD within the short polling deadline.
    CommandStartTimeout,
    /// The command response phase timed out.
    CommandTimeout,
    /// The data transfer phase timed out.
    DataTimeout,
    /// The controller, card, or IDMAC reported a transfer error.
    Hardware,
    /// Completion registers or descriptors did not reach the required terminal state.
    TerminalValidation,
    /// IDMAC could not be stopped safely after a submitted transfer failed.
    RecoveryFailed,
    /// The card did not release DAT busy after a write.
    CardBusyTimeout,
}

/// Result type returned by SD/MMC block transfers.
pub type SdMmcResult<T = ()> = Result<T, SdMmcError>;

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

    /// Relative card address assigned by CMD3.
    rca: u16,

    /// OCR Card Capacity Status captured from the completed ACMD41 response.
    ocr_ccs: bool,

    /// Indicates whether the Internal DMA (IDMAC) is enabled for data transfer.
    ahb_data_width: AHBDataWidth,

    /// Coherent buffer used for DMA transfers.
    dma_buffer: Option<DMABuffer>,

    /// Once set, no further benchmark transfer may be submitted.
    idmac_faulted: bool,

    /// The DMA buffer must be retained if hardware could not be stopped.
    idmac_reset_failed: bool,
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

    fn wait_sync(&self) -> SdMmcResult {
        self.sdmmc.wait_transfer_sync(self.context())
    }

    async fn wait_async(&self) -> SdMmcResult {
        self.sdmmc.wait_transfer_async(self.context()).await
    }

    fn status(&self) -> (crate::regs::RIntSts, crate::regs::IdSts) {
        self.sdmmc
            .idmac_completion_status(self.context().generation)
    }

    fn validate(&self) -> bool {
        self.sdmmc.validate_idmac_terminal(self.context())
    }

    fn response(&self) -> [u32; 4] {
        self.sdmmc.regs.resp().read()
    }

    fn validated_response(&self) -> SdMmcResult<[u32; 4]> {
        let response = self.response();
        if self.context().expects_r1 {
            SdMmc::validate_r1_response(response[0], false)?;
        }
        Ok(response)
    }

    fn fault(&mut self) {
        self.sdmmc.idmac_faulted = true;
    }

    fn finish(mut self, abort: bool) -> SdMmcResult {
        let context = self.context.take().unwrap();
        self.sdmmc.finish_idmac_transfer(context, abort)
    }
}

impl Drop for ActiveIdmacTransfer<'_> {
    fn drop(&mut self) {
        let Some(context) = self.context.take() else {
            return;
        };

        // Cancellation is fail-stop: stop IDMAC before freeing its descriptor and
        // never reuse controller/card state whose terminal state was not observed.
        warn!("active IDMAC future dropped; stopping IDMAC and faulting the driver");
        self.sdmmc.idmac_faulted = true;
        if let Err(error) = self.sdmmc.finish_idmac_transfer(context, true) {
            warn!("failed to stop cancelled IDMAC transfer safely: {error:?}");
        }
    }
}

struct AsyncWriteBusyGuard<'a> {
    sdmmc: &'a mut SdMmc,
    resolved: bool,
}

impl<'a> AsyncWriteBusyGuard<'a> {
    fn new(sdmmc: &'a mut SdMmc) -> Self {
        Self {
            sdmmc,
            resolved: false,
        }
    }

    fn resolve(&mut self) {
        self.resolved = true;
    }

    fn fault(&mut self) {
        self.sdmmc.idmac_faulted = true;
        self.resolved = true;
    }
}

impl Drop for AsyncWriteBusyGuard<'_> {
    fn drop(&mut self) {
        if !self.resolved {
            warn!(
                "asynchronous write future dropped while card busy status was unresolved; \
                 faulting the driver"
            );
            self.sdmmc.idmac_faulted = true;
        }
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
    pub unsafe fn new(base: usize, register_irq: impl FnOnce() -> bool) -> SdMmcResult<Self> {
        let regs = unsafe { VolatilePtr::new(NonNull::new_unchecked(base as *mut _)) };

        let mut this = Self {
            regs,
            num_blocks: 0,
            rca: 0,
            ocr_ccs: false,
            ahb_data_width: AHBDataWidth::Bits32,
            dma_buffer: None,
            idmac_faulted: false,
            idmac_reset_failed: false,
        };
        this.log_register_snapshot();
        this.init()?;
        SDMMC_REGS_BASE.store(base, Ordering::Release);
        this.try_enable_idmac(DMA_BUFFER_SIZE, AHBDataWidth::Bits32, register_irq);
        Ok(this)
    }

    fn log_register_snapshot(&self) {
        let hcon = self.regs.hcon().read().into_bits();
        let fifoth = self.regs.fifoth().read().into_bits();
        let bmod = self.regs.bmod().read().into_bits();
        let tmout = self.regs.tmout().read().into_bits();
        let clksrc = self.regs.clksrc().read().into_bits();
        let uhs = self.regs.uhs().read().into_bits();
        let ctrl = self.regs.ctrl().read().into_bits();

        warn!(
            "SDMMC_REGISTER_SNAPSHOT stage=pre_init HCON=0x{hcon:08x} FIFOTH=0x{fifoth:08x} \
             BMOD=0x{bmod:08x} TMOUT=0x{tmout:08x} CLKSRC=0x{clksrc:08x} UHS=0x{uhs:08x} \
             CTRL=0x{ctrl:08x}"
        );
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

    fn idmac_controller_interrupt_mask() -> crate::regs::IntMask {
        crate::regs::IntMask::new()
            .with_ebe(true)
            .with_acd(true)
            .with_hle(true)
            .with_frun(true)
            .with_hto(true)
            .with_drto(true)
            .with_rto(true)
            .with_dcrc(true)
            .with_rcrc(true)
            .with_dto(true)
            .with_re(true)
    }

    fn idmac_controller_interrupt_mask_matches(mask: crate::regs::IntMask) -> bool {
        mask.ebe()
            && mask.acd()
            // Bit 13 is SBE/Busy Complete depending on controller configuration.
            // Keep it masked because this driver cannot distinguish the two meanings.
            && !mask.sbe()
            && mask.hle()
            && mask.frun()
            && mask.hto()
            && mask.drto()
            && mask.rto()
            && mask.dcrc()
            && mask.rcrc()
            && mask.dto()
            && mask.re()
            && !mask.cmd()
            && !mask.rxdr()
            && !mask.txdr()
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

    fn release_dma_buffer(&mut self) {
        let Some(dma_buffer) = self.dma_buffer.take() else {
            return;
        };
        let Ok(layout) =
            Layout::from_size_align(dma_buffer.size, self.ahb_data_width.align_value())
        else {
            warn!("retaining DMA buffer because its allocation layout is invalid");
            self.dma_buffer = Some(dma_buffer);
            return;
        };

        unsafe { dealloc_coherent(dma_buffer.addr, layout) };
    }

    fn disable_idmac_after_enable_failure(&mut self) {
        self.regs.intmask().write(crate::regs::IntMask::new());
        self.regs.idinten().write(crate::regs::IdIntEn::new());
        self.regs
            .ctrl()
            .update(|r| r.with_int_enable(false).with_use_internal_dmac(false));
        dma_io_fence();

        if !self.reset_idmac() {
            warn!("IDMAC enable rollback failed; retaining the DMA buffer");
            self.idmac_faulted = true;
            self.idmac_reset_failed = true;
            return;
        }

        self.release_dma_buffer();
    }

    fn finish_idmac_transfer(&mut self, context: IdmacTransferContext, abort: bool) -> SdMmcResult {
        if abort {
            self.regs.intmask().write(crate::regs::IntMask::new());
            self.regs.idinten().write(crate::regs::IdIntEn::new());
            dma_io_fence();
        }

        if abort && !self.reset_idmac() {
            warn!("IDMAC recovery failed; retaining the descriptor to avoid DMA use-after-free");
            self.idmac_faulted = true;
            self.idmac_reset_failed = true;
            let rintsts = self.regs.rintsts().read();
            let idsts = self.regs.idsts().read();
            self.regs.rintsts().write(rintsts);
            self.regs.idsts().write(idsts);
            return Err(SdMmcError::RecoveryFailed);
        }

        let rintsts = self.regs.rintsts().read();
        let idsts = self.regs.idsts().read();
        self.regs.rintsts().write(rintsts);
        self.regs.idsts().write(idsts);
        dma_io_fence();
        unsafe { dealloc_coherent(context.dma_desc_info, context.layout) };

        if abort {
            self.regs
                .ctrl()
                .update(|r| r.with_use_internal_dmac(false).with_int_enable(false));
            dma_io_fence();
        }

        Ok(())
    }

    fn abort_idmac_transfer(
        &mut self,
        context: IdmacTransferContext,
        error: SdMmcError,
    ) -> SdMmcError {
        self.idmac_faulted = true;
        match self.finish_idmac_transfer(context, true) {
            Ok(()) => error,
            Err(recovery_error) => recovery_error,
        }
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
        if self.send_cmd(Command::ResetClock).is_err() {
            warn!("failed to latch disabled card clock before setting CLKDIV={divider}");
            return false;
        }

        self.regs
            .clkdiv()
            .write(ClkDiv::new().with_clk_divider0(divider));
        if self.send_cmd(Command::ResetClock).is_err() {
            warn!("failed to latch CLKDIV={divider} while card clock was disabled");
            return false;
        }

        self.regs.clkena().write(ClkEna::new().with_cclk_enable(1));
        if self.send_cmd(Command::ResetClock).is_err() {
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

    fn validate_r1_response(raw: u32, require_app_cmd: bool) -> SdMmcResult {
        let status = R1CardStatus::from_raw(raw);
        if status.has_error() {
            return Err(SdMmcError::CardStatus(status));
        }
        if require_app_cmd && !status.app_cmd() {
            return Err(SdMmcError::AppCommandRejected(status));
        }
        Ok(())
    }

    fn send_cmd(&self, command: Command<'_>) -> SdMmcResult<[u32; 4]> {
        self.send_cmd_with_deadline(command, None)
    }

    fn send_cmd_with_deadline(
        &self,
        command: Command<'_>,
        overall_deadline: Option<Duration>,
    ) -> SdMmcResult<[u32; 4]> {
        let is_reset_clock = matches!(command, Command::ResetClock);
        let expects_busy = matches!(command, Command::SelectCard(_));
        let expects_r1 = command.has_r1_response();
        let requires_app_cmd = command.requires_app_cmd_accepted();
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
            if cmd_wait_count > cmd_max_wait
                || overall_deadline
                    .is_some_and(|deadline| axhal::time::monotonic_time() >= deadline)
            {
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
            return Err(SdMmcError::CommandBusy);
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
            if start_cmd_wait_count > cmd_max_wait
                || overall_deadline
                    .is_some_and(|deadline| axhal::time::monotonic_time() >= deadline)
            {
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
            return Err(SdMmcError::CommandStartTimeout);
        }
        trace!("cmd {} sent", cmd.cmd_index());

        let mut command_timed_out = false;
        if !is_reset_clock {
            // Every real command, including response-less CMD0, completes by
            // setting command_done or an error bit. Clock-update commands are
            // the only exception and complete when start_cmd clears.
            let mut completion_wait_count = 0u64;
            let command_deadline = axhal::time::monotonic_time() + Duration::from_secs(1);
            let completion_deadline = overall_deadline
                .map(|deadline| deadline.min(command_deadline))
                .unwrap_or(command_deadline);
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
            return Err(SdMmcError::CommandTimeout);
        }

        let command_status = self.regs.rintsts().read();
        if command_status.error() {
            let resp = self.regs.resp().read();
            self.regs.rintsts().write(command_status);
            warn!(
                "cmd {} failed before data/busy phase: rintsts={command_status:?}, resp={resp:?}",
                cmd.cmd_index(),
            );
            return Err(SdMmcError::Hardware);
        }

        let mut busy_timed_out = false;
        if expects_busy {
            let command_deadline = axhal::time::monotonic_time() + Duration::from_secs(1);
            let busy_deadline = overall_deadline
                .map(|deadline| deadline.min(command_deadline))
                .unwrap_or(command_deadline);
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
            return Err(SdMmcError::CardBusyTimeout);
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
            return if rintsts.data_read_timeout() {
                Err(SdMmcError::DataTimeout)
            } else {
                Err(SdMmcError::Hardware)
            };
        }
        if expects_r1 {
            Self::validate_r1_response(resp[0], requires_app_cmd).inspect_err(|error| {
                warn!(
                    "cmd {} returned unsuccessful R1 status: {error:?}",
                    cmd.cmd_index()
                );
            })?;
        }
        Ok(resp)
    }

    fn init(&mut self) -> SdMmcResult {
        info!("Initializing SD/MMC driver at {:?}", self.regs);

        // U-Boot leaves the controller configured, but the driver needs a clean status baseline.
        let rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(rintsts);

        // Reconfigure the card clock while it is disabled.
        self.regs.clkena().write(ClkEna::new());
        self.send_cmd(Command::ResetClock)?;
        self.regs
            .clkdiv()
            .write(ClkDiv::new().with_clk_divider0(IDENTIFICATION_CLOCK_DIVIDER));
        self.regs.clkena().write(ClkEna::new().with_cclk_enable(1));
        self.send_cmd(Command::ResetClock)?;

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

        self.send_cmd(Command::GoIdleState)?;

        let if_cond = match self.send_cmd(Command::SendIfCond(0x1aa)) {
            Ok(resp) => resp,
            Err(error) => {
                warn!("CMD8 failed; legacy SD v1/SDSC cards are not supported: {error:?}");
                return Err(SdMmcError::UnsupportedCard);
            }
        };
        if if_cond[0] & 0xfff != 0x1aa {
            warn!("CMD8 returned an unsupported voltage/check pattern: response={if_cond:?}");
            return Err(SdMmcError::UnsupportedCard);
        }

        let mut attempt = 0;
        let mut ready_ocr = None;
        let acmd41_deadline = axhal::time::monotonic_time() + Duration::from_secs(2);
        while axhal::time::monotonic_time() < acmd41_deadline {
            attempt += 1;
            if self.send_cmd(Command::AppCmd(0)).is_ok() {
                match self.send_cmd(Command::SdSendOpCond(0x40FF_8000)) {
                    Ok(resp) => {
                        let ocr = resp[0];
                        if ocr & 0x8000_0000 != 0 {
                            info!(
                                "SD card is ready after {} attempts, OCR={ocr:#010x}",
                                attempt
                            );
                            ready_ocr = Some(ocr);
                            break;
                        }
                    }
                    Err(error) => {
                        warn!("SdSendOpCond failed on attempt {}: {error:?}", attempt)
                    }
                }
            } else {
                warn!("AppCmd failed on attempt {}", attempt);
            }

            axhal::time::busy_wait(Duration::from_millis(10));
        }
        let Some(ocr) = ready_ocr else {
            warn!("ACMD41 timed out after {} attempts", attempt);
            return Err(SdMmcError::InitializationFailed);
        };
        self.ocr_ccs = ocr & 0x4000_0000 != 0;
        if !self.ocr_ccs {
            warn!(
                "ACMD41 reported CCS=0; SDSC byte addressing is not implemented, OCR={ocr:#010x}"
            );
            return Err(SdMmcError::UnsupportedCard);
        }
        debug!("SD card uses high-capacity block addressing");

        let cid_response = self.send_cmd(Command::AllSendCid)?;
        let cid = unsafe { core::mem::transmute::<[u32; 4], Cid>(cid_response) };
        info!("cid: {cid:?}");

        let rca_response = self.send_cmd(Command::SendRelativeAddr)?;
        let r6_status = rca_response[0] & 0xffff;
        if r6_status & 0xe000 != 0 {
            warn!(
                "CMD3 returned an unsuccessful R6 status: {:#06x}",
                r6_status
            );
            return Err(SdMmcError::InitializationFailed);
        }
        self.rca = (rca_response[0] >> 16) as u16;
        if self.rca == 0 {
            warn!("CMD3 assigned invalid RCA 0");
            return Err(SdMmcError::InitializationFailed);
        }
        debug!("rca: {:#x}", self.rca);
        let rca_arg = u32::from(self.rca) << 16;

        let csd_response = self.send_cmd(Command::SendCsd(rca_arg))?;
        let csd = unsafe { core::mem::transmute::<[u32; 4], CsdV2>(csd_response) };
        debug!("csd: {csd:?}");
        if csd.csd_structure() != 1 {
            warn!(
                "CSD structure {} is unsupported; only SDHC/SDXC CSD v2 is implemented",
                csd.csd_structure()
            );
            return Err(SdMmcError::UnsupportedCard);
        }
        self.num_blocks = csd.num_blocks();
        info!("SD card capacity: {:#x} blocks", self.num_blocks);

        self.send_cmd(Command::SelectCard(rca_arg))?;

        self.send_cmd(Command::AppCmd(rca_arg))?;

        // A block-sized buffer keeps the short SCR transfer DMA-aligned.
        self.set_transaction_size(8, 8);
        let mut buf = [0u8; 512];
        match self.send_cmd(Command::SendScr(&mut buf)) {
            Ok(_) => {
                let scr = u64::from_be_bytes(buf[..8].try_into().unwrap());
                debug!("Bus width supported: {:#x?}", (scr >> 48) & 0xf);
            }
            Err(error) => warn!("SendScr failed: {error:?}"),
        }

        let rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(rintsts);

        if !self.switch_card_clock_divider(DEFAULT_SPEED_CLOCK_DIVIDER) {
            warn!("failed to enter SD Default Speed");
            return Err(SdMmcError::InitializationFailed);
        }
        let card_clock_hz =
            VISIONFIVE2_SDIO_CIU_CLOCK_HZ / (2 * DEFAULT_SPEED_CLOCK_DIVIDER as u32);
        warn!(
            "SD/MMC card clock switched to Default Speed: ciu_clock_hz={}, CLKDIV={}, \
             card_clock_hz={}",
            VISIONFIVE2_SDIO_CIU_CLOCK_HZ, DEFAULT_SPEED_CLOCK_DIVIDER, card_clock_hz,
        );

        info!("SD/MMC driver initialized");
        Ok(())
    }

    fn validate_block_buffer(&self, block: u32, len: usize) -> SdMmcResult<usize> {
        if len == 0 || !len.is_multiple_of(Self::BLOCK_SIZE) {
            return Err(SdMmcError::InvalidParameter);
        }
        let blocks = len / Self::BLOCK_SIZE;
        let end = (block as u64)
            .checked_add(blocks as u64)
            .ok_or(SdMmcError::OutOfRange)?;
        if end > self.num_blocks || end > u32::MAX as u64 + 1 {
            return Err(SdMmcError::OutOfRange);
        }
        Ok(blocks)
    }

    fn read_dma_chunk(&mut self, block: u32, buf: &mut [u8]) -> SdMmcResult {
        debug_assert!(buf.len() <= DMA_BUFFER_SIZE);
        debug_assert!(buf.len().is_multiple_of(Self::BLOCK_SIZE));
        debug_assert!(
            buf.len() == Self::BLOCK_SIZE
                || buf.len() >= MIN_MULTI_BLOCK_READ_BLOCKS * Self::BLOCK_SIZE
        );
        self.set_transaction_size(Self::BLOCK_SIZE as u16, buf.len() as u32);

        let dma_buf_info = self.dma_buffer.as_ref().ok_or(SdMmcError::DmaUnavailable)?;
        let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
        let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
            .map_err(|_| SdMmcError::DmaAddress)?;
        let dma_buf = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
        let command = if buf.len() == Self::BLOCK_SIZE {
            Command::ReadSingleBlock(block, dma_buf)
        } else {
            Command::ReadMultipleBlocks(block, dma_buf)
        };
        self.send_cmd_idmac(command, dma_bus_addr)?;

        let dma_usr_slice = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
        buf.copy_from_slice(dma_usr_slice);
        Ok(())
    }

    async fn read_dma_chunk_async(&mut self, block: u32, buf: &mut [u8]) -> SdMmcResult {
        debug_assert!(buf.len() <= DMA_BUFFER_SIZE);
        debug_assert!(buf.len().is_multiple_of(Self::BLOCK_SIZE));
        debug_assert!(
            buf.len() == Self::BLOCK_SIZE
                || buf.len() >= MIN_MULTI_BLOCK_READ_BLOCKS * Self::BLOCK_SIZE
        );
        self.set_transaction_size(Self::BLOCK_SIZE as u16, buf.len() as u32);

        let dma_buf_info = self.dma_buffer.as_ref().ok_or(SdMmcError::DmaUnavailable)?;
        let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
        let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
            .map_err(|_| SdMmcError::DmaAddress)?;
        let dma_buf = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
        let command = if buf.len() == Self::BLOCK_SIZE {
            Command::ReadSingleBlock(block, dma_buf)
        } else {
            Command::ReadMultipleBlocks(block, dma_buf)
        };
        self.send_cmd_idmac_async(command, dma_bus_addr).await?;

        let dma_usr_slice = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
        buf.copy_from_slice(dma_usr_slice);
        Ok(())
    }

    fn wait_card_ready_after_write(&mut self) -> SdMmcResult {
        let deadline = axhal::time::monotonic_time() + Duration::from_secs(5);
        while !self.can_send_data() {
            if axhal::time::monotonic_time() >= deadline {
                self.idmac_faulted = true;
                return Err(SdMmcError::CardBusyTimeout);
            }
            core::hint::spin_loop();
        }
        Ok(())
    }

    async fn wait_card_ready_after_write_async(&mut self) -> SdMmcResult {
        let sequence = ASYNC_WRITE_BUSY_SEQUENCE
            .fetch_add(1, Ordering::AcqRel)
            .wrapping_add(1);
        let started_nanos = axhal::time::monotonic_time_nanos();
        let deadline = axhal::time::monotonic_time() + ASYNC_WRITE_BUSY_TIMEOUT;
        let rca_arg = u32::from(self.rca) << 16;
        let mut attempts = 0usize;
        let mut last_card_status: Option<R1CardStatus> = None;
        let mut guard = AsyncWriteBusyGuard::new(self);

        loop {
            if let Some(card_status) = last_card_status
                && axhal::time::monotonic_time() >= deadline
            {
                let wait_us =
                    axhal::time::monotonic_time_nanos().saturating_sub(started_nanos) / 1_000;
                let controller_status = guard.sdmmc.regs.status().read();
                warn!(
                    "SDMMC_ASYNC_WRITE_BUSY sample={} result=TIMEOUT attempts={} wait_us={} \
                     r1=0x{:08x} error_bits=0x{:08x} state={:?} ready_for_data={} data_busy={} \
                     data_state_busy={}",
                    sequence,
                    attempts,
                    wait_us,
                    card_status.raw(),
                    card_status.error_bits(),
                    card_status.current_state(),
                    card_status.ready_for_data(),
                    controller_status.data_busy(),
                    controller_status.data_state_mc_busy(),
                );
                guard.fault();
                return Err(SdMmcError::CardBusyTimeout);
            }

            attempts = attempts.saturating_add(1);
            let response = match guard
                .sdmmc
                .send_cmd_with_deadline(Command::SendStatus(rca_arg), Some(deadline))
            {
                Ok(response) => response,
                Err(error) => {
                    let wait_us =
                        axhal::time::monotonic_time_nanos().saturating_sub(started_nanos) / 1_000;
                    let controller_status = guard.sdmmc.regs.status().read();
                    let response = guard.sdmmc.regs.resp().read()[0];
                    warn!(
                        "SDMMC_ASYNC_WRITE_BUSY sample={} result=CMD13_ERROR attempts={} \
                         wait_us={} error={:?} response=0x{:08x} data_busy={} data_state_busy={}",
                        sequence,
                        attempts,
                        wait_us,
                        error,
                        response,
                        controller_status.data_busy(),
                        controller_status.data_state_mc_busy(),
                    );
                    guard.fault();
                    return Err(error);
                }
            };

            let card_status = R1CardStatus::from_raw(response[0]);
            let controller_status = guard.sdmmc.regs.status().read();
            let ready = !card_status.has_error()
                && card_status.ready_for_data()
                && card_status.current_state() == R1CurrentState::Transfer
                && !controller_status.data_busy()
                && !controller_status.data_state_mc_busy();
            let wait_us = axhal::time::monotonic_time_nanos().saturating_sub(started_nanos) / 1_000;

            if ready {
                if sequence <= 8 || sequence.is_power_of_two() {
                    warn!(
                        "SDMMC_ASYNC_WRITE_BUSY sample={} result=READY attempts={} wait_us={} \
                         r1=0x{:08x} state={:?} ready_for_data={} data_busy={} data_state_busy={}",
                        sequence,
                        attempts,
                        wait_us,
                        card_status.raw(),
                        card_status.current_state(),
                        card_status.ready_for_data(),
                        controller_status.data_busy(),
                        controller_status.data_state_mc_busy(),
                    );
                }
                guard.resolve();
                return Ok(());
            }

            // CMD13 is complete before this suspension point. A dropped future
            // therefore leaves no command or descriptor in flight; the guard
            // still faults the driver because the card's programming state is
            // unresolved.
            last_card_status = Some(card_status);
            if axhal::time::monotonic_time() < deadline {
                cooperative_yield_once().await;
            }
        }
    }

    fn write_dma_chunk(&mut self, block: u32, buf: &[u8]) -> SdMmcResult {
        debug_assert!(buf.len() <= DMA_BUFFER_SIZE);
        debug_assert!(buf.len().is_multiple_of(Self::BLOCK_SIZE));
        self.set_transaction_size(Self::BLOCK_SIZE as u16, buf.len() as u32);

        let dma_buf_info = self.dma_buffer.as_ref().ok_or(SdMmcError::DmaUnavailable)?;
        let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
        let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
            .map_err(|_| SdMmcError::DmaAddress)?;
        let dma_usr_slice = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
        dma_usr_slice.copy_from_slice(buf);

        let dma_buf = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
        let command = if buf.len() == Self::BLOCK_SIZE {
            Command::WriteSingleBlock(block, dma_buf)
        } else {
            Command::WriteMultipleBlocks(block, dma_buf)
        };
        self.send_cmd_idmac(command, dma_bus_addr)?;
        self.wait_card_ready_after_write()
    }

    async fn write_dma_chunk_async(&mut self, block: u32, buf: &[u8]) -> SdMmcResult {
        debug_assert!(buf.len() <= DMA_BUFFER_SIZE);
        debug_assert!(buf.len().is_multiple_of(Self::BLOCK_SIZE));
        self.set_transaction_size(Self::BLOCK_SIZE as u16, buf.len() as u32);

        let dma_buf_info = self.dma_buffer.as_ref().ok_or(SdMmcError::DmaUnavailable)?;
        let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
        let dma_usr_slice = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
        dma_usr_slice.copy_from_slice(buf);

        let dma_bus_addr = u32::try_from(dma_buf_info.addr.bus_addr.as_u64())
            .map_err(|_| SdMmcError::DmaAddress)?;
        let dma_buf = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
        let command = if buf.len() == Self::BLOCK_SIZE {
            Command::WriteSingleBlock(block, dma_buf)
        } else {
            Command::WriteMultipleBlocks(block, dma_buf)
        };
        self.send_cmd_idmac_async(command, dma_bus_addr).await?;
        self.wait_card_ready_after_write_async().await
    }

    /// Reads one or more contiguous blocks from the SD/MMC card.
    pub fn read_blocks(&mut self, mut block: u32, mut buf: &mut [u8]) -> SdMmcResult {
        self.validate_block_buffer(block, buf.len())?;
        while !buf.is_empty() {
            let remaining_blocks = buf.len() / Self::BLOCK_SIZE;
            let chunk_blocks = if remaining_blocks >= MIN_MULTI_BLOCK_READ_BLOCKS {
                remaining_blocks.min(MAX_DMA_BLOCKS)
            } else {
                1
            };
            let (chunk, remaining) = buf.split_at_mut(chunk_blocks * Self::BLOCK_SIZE);
            self.read_dma_chunk(block, chunk)?;
            buf = remaining;
            if !buf.is_empty() {
                block = block
                    .checked_add(chunk_blocks as u32)
                    .ok_or(SdMmcError::OutOfRange)?;
            }
        }
        Ok(())
    }

    /// Reads one or more contiguous blocks and asynchronously waits for each DMA chunk.
    ///
    /// # Cancellation
    /// Dropping this future after command submission stops IDMAC and permanently faults the
    /// driver. This fail-stop policy prevents DMA from accessing a freed descriptor while the
    /// controller and card terminal states are unknown.
    pub async fn read_blocks_async(&mut self, mut block: u32, mut buf: &mut [u8]) -> SdMmcResult {
        self.validate_block_buffer(block, buf.len())?;
        while !buf.is_empty() {
            let remaining_blocks = buf.len() / Self::BLOCK_SIZE;
            let chunk_blocks = if remaining_blocks >= MIN_MULTI_BLOCK_READ_BLOCKS {
                remaining_blocks.min(MAX_DMA_BLOCKS)
            } else {
                1
            };
            let (chunk, remaining) = buf.split_at_mut(chunk_blocks * Self::BLOCK_SIZE);
            self.read_dma_chunk_async(block, chunk).await?;
            buf = remaining;
            if !buf.is_empty() {
                block = block
                    .checked_add(chunk_blocks as u32)
                    .ok_or(SdMmcError::OutOfRange)?;
            }
        }
        Ok(())
    }

    /// Writes one or more contiguous blocks to the SD/MMC card.
    pub fn write_blocks(&mut self, mut block: u32, mut buf: &[u8]) -> SdMmcResult {
        self.validate_block_buffer(block, buf.len())?;
        while !buf.is_empty() {
            let chunk_blocks = (buf.len() / Self::BLOCK_SIZE).min(MAX_DMA_BLOCKS);
            let (chunk, remaining) = buf.split_at(chunk_blocks * Self::BLOCK_SIZE);
            self.write_dma_chunk(block, chunk)?;
            buf = remaining;
            if !buf.is_empty() {
                block = block
                    .checked_add(chunk_blocks as u32)
                    .ok_or(SdMmcError::OutOfRange)?;
            }
        }
        Ok(())
    }

    /// Writes one or more contiguous blocks and asynchronously waits for each DMA chunk.
    ///
    /// # Cancellation
    /// Dropping this future while IDMAC is active stops IDMAC before freeing its descriptor.
    /// Dropping it after DMA completion but before CMD13 reports the card ready leaves no command
    /// in flight, but still permanently faults the driver because card programming is unresolved.
    pub async fn write_blocks_async(&mut self, mut block: u32, mut buf: &[u8]) -> SdMmcResult {
        self.validate_block_buffer(block, buf.len())?;
        while !buf.is_empty() {
            let chunk_blocks = (buf.len() / Self::BLOCK_SIZE).min(MAX_DMA_BLOCKS);
            let (chunk, remaining) = buf.split_at(chunk_blocks * Self::BLOCK_SIZE);
            self.write_dma_chunk_async(block, chunk).await?;
            buf = remaining;
            if !buf.is_empty() {
                block = block
                    .checked_add(chunk_blocks as u32)
                    .ok_or(SdMmcError::OutOfRange)?;
            }
        }
        Ok(())
    }

    /// Reads a single block from the SD/MMC card.
    pub fn read_block(&mut self, block: u32, buf: &mut [u8; 512]) -> SdMmcResult {
        self.read_blocks(block, buf)
    }

    /// Reads a single block using IDMAC and asynchronously waits for completion.
    ///
    /// See [`Self::read_blocks_async`] for the cancellation policy.
    pub async fn read_block_async(&mut self, block: u32, buf: &mut [u8; 512]) -> SdMmcResult {
        self.read_blocks_async(block, buf).await
    }

    /// Writes a single block to the SD/MMC card.
    pub fn write_block(&mut self, block: u32, buf: &[u8; 512]) -> SdMmcResult {
        self.write_blocks(block, buf)
    }

    /// Writes a single block using IDMAC and asynchronously waits for completion.
    ///
    /// See [`Self::write_blocks_async`] for the cancellation policy.
    pub async fn write_block_async(&mut self, block: u32, buf: &[u8; 512]) -> SdMmcResult {
        self.write_blocks_async(block, buf).await
    }

    /// Returns the number of blocks.
    pub fn num_blocks(&self) -> u64 {
        self.num_blocks
    }

    /// Returns the relative card address assigned by CMD3.
    pub fn relative_card_address(&self) -> u16 {
        self.rca
    }

    /// Returns the OCR Card Capacity Status captured from ACMD41.
    ///
    /// This driver only completes initialization when this is `true`; SDSC
    /// byte-addressed cards are intentionally unsupported.
    pub fn is_high_capacity(&self) -> bool {
        self.ocr_ccs
    }

    /// Enables the Internal DMA (IDMAC) for DMA transfers.
    pub fn try_enable_idmac(
        &mut self,
        buf_size: usize,
        ahb_data_width: AHBDataWidth,
        register_irq: impl FnOnce() -> bool,
    ) {
        // Do not inherit interrupt enables from firmware. The handler is
        // registered below before this driver enables any interrupt source.
        self.regs.intmask().write(crate::regs::IntMask::new());
        self.regs.idinten().write(crate::regs::IdIntEn::new());
        self.regs.ctrl().update(|r| r.with_int_enable(false));
        dma_io_fence();

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
            self.idmac_faulted = true;
            return;
        }

        let Ok(layout) = Layout::from_size_align(buf_size, hardware_data_width.align_value())
        else {
            warn!("Invalid DMA buffer layout; IDMAC block transfers are unavailable");
            return;
        };
        match unsafe { alloc_coherent(layout) } {
            Ok(dma_info) => {
                self.dma_buffer = Some(DMABuffer {
                    addr: dma_info,
                    size: buf_size,
                });
            }
            Err(e) => {
                warn!(
                    "Failed to allocate DMA buffer: {:?}; IDMAC block transfers are unavailable",
                    e
                );
                return;
            }
        }

        if !register_irq() {
            warn!("Failed to register IRQ; IDMAC block transfers are unavailable");
            self.disable_idmac_after_enable_failure();
            return;
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
                "Failed to set BMOD register for IDMAC; block transfers are unavailable; actual: \
                 de={}, dsl={}, fb={}, pbl={}",
                bmod_after.de(),
                bmod_after.dsl(),
                bmod_after.fb(),
                bmod_after.pbl(),
            );
            self.disable_idmac_after_enable_failure();
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
                "Failed to set CTRL register for IDMAC and interrupt output; block transfers are \
                 unavailable; expected use_internal_dmac=true, int_enable=true. actual: \
                 use_internal_dmac={}, int_enable={}. IDSTS={:?}",
                ctrl_after.use_internal_dmac(),
                ctrl_after.int_enable(),
                idsts_after_ctrl
            );
            self.disable_idmac_after_enable_failure();
            return;
        }
        if idsts_after_ctrl.du() || idsts_after_ctrl.fbe() || idsts_after_ctrl.ais() {
            warn!(
                "try_enable_idmac: abnormal IDSTS after CTRL enable; disabling IDMAC path: {:?}",
                idsts_after_ctrl
            );
            self.disable_idmac_after_enable_failure();
            return;
        }

        // Successful command completion is observed together with the terminal
        // DMA event. Errors are unmasked so both sync and async paths see them
        // immediately without changing the polling semantics of non-DMA commands.
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
            .write(Self::idmac_controller_interrupt_mask());

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
        if !Self::idmac_controller_interrupt_mask_matches(intmask_after) {
            warn!(
                "try_enable_idmac: INTMASK mismatch after write; expected=0x{:08x}, actual={:?}",
                Self::idmac_controller_interrupt_mask().into_bits(),
                intmask_after,
            );
        }
        warn!(
            "SDMMC_ASYNC_WRITE_BUSY configured: policy=CMD13 rca=0x{:04x} \
             wait_prvdata_complete=false timeout_ms={} yield=cooperative",
            self.rca,
            ASYNC_WRITE_BUSY_TIMEOUT.as_millis(),
        );
        if idsts_after_enable.du() || idsts_after_enable.fbe() || idsts_after_enable.ais() {
            warn!(
                "try_enable_idmac: abnormal post-enable IDSTS detected: {:?}",
                idsts_after_enable
            );
        }
    }

    fn wait_for_transfer_idle_short(&self) -> SdMmcResult {
        let deadline = axhal::time::monotonic_time() + PRE_SUBMIT_IDLE_SPIN_TIMEOUT;
        loop {
            let command_idle = self.can_send_cmd();
            let data_idle = self.can_send_data();
            if command_idle && data_idle {
                return Ok(());
            }
            if axhal::time::monotonic_time() >= deadline {
                warn!(
                    "IDMAC pre-submit state stayed busy: command_idle={}, data_idle={}, CMD={:?}, \
                     STATUS={:?}, RINTSTS={:?}, IDSTS={:?}",
                    command_idle,
                    data_idle,
                    self.regs.cmd().read(),
                    self.regs.status().read(),
                    self.regs.rintsts().read(),
                    self.regs.idsts().read(),
                );
                return if !command_idle {
                    Err(SdMmcError::CommandBusy)
                } else {
                    Err(SdMmcError::DataBusy)
                };
            }
            core::hint::spin_loop();
        }
    }

    fn prepare_idmac_transfer(
        &mut self,
        command: Command<'_>,
        dma_bus_addr: u32,
    ) -> SdMmcResult<IdmacTransferContext> {
        if self.idmac_faulted {
            warn!("refusing IDMAC transfer because the driver is faulted");
            return Err(SdMmcError::DriverFaulted);
        }

        let expects_r1 = command.has_r1_response();
        let (cmd, arg, xfer) = command.build();
        if !cmd.data_expected() {
            return Err(SdMmcError::InvalidParameter);
        }
        let xfer = xfer.ok_or(SdMmcError::InvalidParameter)?;

        self.wait_for_transfer_idle_short()?;

        // Establish a clean W1C status baseline for the new transaction.
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

        IDMAC_DONE_FLAG.store(false, Ordering::Release);
        IDMAC_ERROR_FLAG.store(false, Ordering::Release);

        let buf_len = match xfer {
            DataXfer::Read(buf) => buf.len(),
            DataXfer::Write(buf) => buf.len(),
        };

        if buf_len == 0 || buf_len > DMA_BUFFER_SIZE {
            return Err(SdMmcError::InvalidParameter);
        }
        if dma_bus_addr.checked_add((buf_len - 1) as u32).is_none() {
            return Err(SdMmcError::DmaAddress);
        }

        let descriptor_count = buf_len.div_ceil(IDMAC_DESCRIPTOR_BUFFER_SIZE);
        let layout = Layout::array::<IdmacDescriptor>(descriptor_count)
            .map_err(|_| SdMmcError::DmaAllocation)?;
        let dma_desc_info =
            unsafe { alloc_coherent(layout) }.map_err(|_| SdMmcError::DmaAllocation)?;
        let desc_ptr = dma_desc_info.cpu_addr.as_ptr() as *mut IdmacDescriptor;
        let desc_phy_addr = match u32::try_from(dma_desc_info.bus_addr.as_u64()) {
            Ok(address) => address,
            Err(_) => {
                unsafe { dealloc_coherent(dma_desc_info, layout) };
                return Err(SdMmcError::DmaAddress);
            }
        };
        let last_descriptor_offset =
            ((descriptor_count - 1) * core::mem::size_of::<IdmacDescriptor>()) as u32;
        if desc_phy_addr.checked_add(last_descriptor_offset).is_none() {
            unsafe { dealloc_coherent(dma_desc_info, layout) };
            return Err(SdMmcError::DmaAddress);
        }

        for index in 0..descriptor_count {
            let offset = index * IDMAC_DESCRIPTOR_BUFFER_SIZE;
            let segment_len = (buf_len - offset).min(IDMAC_DESCRIPTOR_BUFFER_SIZE);
            let last = index + 1 == descriptor_count;
            let buffer_addr = dma_bus_addr + offset as u32;
            let next_descriptor_addr = if last {
                0
            } else {
                desc_phy_addr + ((index + 1) * core::mem::size_of::<IdmacDescriptor>()) as u32
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

        let context = IdmacTransferContext {
            cmd,
            arg,
            expects_r1,
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
            return Err(self.abort_idmac_transfer(context, SdMmcError::DescriptorPublication));
        }

        Ok(context)
    }

    fn start_idmac_transfer(
        &mut self,
        mut context: IdmacTransferContext,
    ) -> SdMmcResult<IdmacTransferContext> {
        let cmd = context.cmd;
        context.generation = IDMAC_COMPLETION.begin_transfer();
        #[cfg(feature = "sdmmc-concurrency-test")]
        if concurrency_test::observation_enabled() {
            concurrency_test::begin_transfer_observation(context.generation);
        }

        self.regs.cmdarg().write(context.arg);
        dma_io_fence();
        self.regs.cmd().write(cmd);
        dma_io_fence();

        let mut start_cmd_wait_count = 0u64;
        let start_cmd_deadline = axhal::time::monotonic_time() + START_CMD_SPIN_TIMEOUT;
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
                return Err(self.abort_idmac_transfer(context, SdMmcError::CommandStartTimeout));
            }
        }

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
            return Err(self.abort_idmac_transfer(context, SdMmcError::Hardware));
        }
        if idsts_after_pldmnd.ais() || idsts_after_pldmnd.fbe() {
            warn!("send_cmd_idmac: IDMAC abnormal status after CMD+PLDMND; disabling IDMAC path");
            return Err(self.abort_idmac_transfer(context, SdMmcError::Hardware));
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

        Ok(context)
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

    fn wait_transfer_sync(&self, context: &IdmacTransferContext) -> SdMmcResult {
        if context.cmd.response_expect() {
            let deadline = axhal::time::monotonic_time() + IDMAC_COMMAND_TIMEOUT;
            while !self.idmac_command_done_or_error(context) {
                if axhal::time::monotonic_time() >= deadline {
                    return Err(SdMmcError::CommandTimeout);
                }
                core::hint::spin_loop();
            }
        }

        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            return Err(SdMmcError::Hardware);
        }

        let deadline = axhal::time::monotonic_time() + IDMAC_DATA_WATCHDOG_TIMEOUT;
        while !self.idmac_terminal_events_or_error(context) {
            if axhal::time::monotonic_time() >= deadline {
                return Err(SdMmcError::DataTimeout);
            }
            core::hint::spin_loop();
        }

        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            Err(SdMmcError::Hardware)
        } else {
            Ok(())
        }
    }

    async fn wait_transfer_async(&self, context: &IdmacTransferContext) -> SdMmcResult {
        if context.cmd.response_expect() {
            #[cfg(feature = "sdmmc-concurrency-test")]
            let command_wait_started_ns = axhal::time::monotonic_time_nanos();
            let command_timed_out = IDMAC_WAIT_QUEUE
                .wait_timeout_until_async(IDMAC_COMMAND_TIMEOUT, || {
                    self.idmac_command_done_or_error(context)
                })
                .await;
            #[cfg(feature = "sdmmc-concurrency-test")]
            concurrency_test::record_async_command_wait(
                context.generation,
                command_timed_out,
                axhal::time::monotonic_time_nanos().saturating_sub(command_wait_started_ns),
            );
            if command_timed_out {
                return Err(SdMmcError::CommandTimeout);
            }
        }

        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        if Self::idmac_status_has_error(&rintsts, &idsts) {
            return Err(SdMmcError::Hardware);
        }

        #[cfg(feature = "sdmmc-concurrency-test")]
        let data_wait_started_ns = axhal::time::monotonic_time_nanos();
        let data_timed_out = IDMAC_WAIT_QUEUE
            .wait_timeout_until_async(IDMAC_DATA_WATCHDOG_TIMEOUT, || {
                self.idmac_terminal_events_or_error(context)
            })
            .await;
        let (rintsts, idsts) = self.idmac_completion_status(context.generation);
        #[cfg(feature = "sdmmc-concurrency-test")]
        concurrency_test::record_async_resume(
            context.generation,
            data_timed_out,
            axhal::time::monotonic_time_nanos().saturating_sub(data_wait_started_ns),
            rintsts,
            idsts,
        );
        if data_timed_out {
            return Err(SdMmcError::DataTimeout);
        }

        if Self::idmac_status_has_error(&rintsts, &idsts) {
            Err(SdMmcError::Hardware)
        } else {
            Ok(())
        }
    }

    async fn send_cmd_idmac_async(
        &mut self,
        command: Command<'_>,
        dma_bus_addr: u32,
    ) -> SdMmcResult<[u32; 4]> {
        let context = self.prepare_idmac_transfer(command, dma_bus_addr)?;
        let context = self.start_idmac_transfer(context)?;
        let mut transfer = ActiveIdmacTransfer::new(self, context);

        trace!(
            "send_cmd_idmac_async: Async DMA transfer started for command index {}",
            transfer.context().cmd.cmd_index()
        );

        if let Err(error) = transfer.wait_async().await {
            let (rintsts, idsts) = transfer.status();
            log::error!(
                "SDMMC DMA error outside IRQ: async=true cmd={} error={:?} RINTSTS=0x{:08x} \
                 IDSTS=0x{:08x}",
                transfer.context().cmd.cmd_index(),
                error,
                rintsts.into_bits(),
                idsts.into_bits(),
            );
            transfer.fault();
            transfer.finish(true)?;
            return Err(error);
        }

        if !transfer.validate() {
            transfer.fault();
            transfer.finish(true)?;
            return Err(SdMmcError::TerminalValidation);
        }

        let resp = match transfer.validated_response() {
            Ok(response) => response,
            Err(error) => {
                warn!(
                    "async IDMAC cmd {} returned unsuccessful R1 status: {error:?}",
                    transfer.context().cmd.cmd_index()
                );
                transfer.fault();
                transfer.finish(true)?;
                return Err(error);
            }
        };
        transfer.finish(false)?;
        Ok(resp)
    }

    fn send_cmd_idmac(&mut self, command: Command<'_>, dma_bus_addr: u32) -> SdMmcResult<[u32; 4]> {
        let context = self.prepare_idmac_transfer(command, dma_bus_addr)?;
        let context = self.start_idmac_transfer(context)?;
        let mut transfer = ActiveIdmacTransfer::new(self, context);

        if let Err(error) = transfer.wait_sync() {
            let (rintsts, idsts) = transfer.status();
            log::error!(
                "SDMMC DMA error outside IRQ: async=false cmd={} error={:?} RINTSTS=0x{:08x} \
                 IDSTS=0x{:08x}",
                transfer.context().cmd.cmd_index(),
                error,
                rintsts.into_bits(),
                idsts.into_bits(),
            );
            transfer.fault();
            transfer.finish(true)?;
            return Err(error);
        }

        if !transfer.validate() {
            transfer.fault();
            transfer.finish(true)?;
            return Err(SdMmcError::TerminalValidation);
        }

        let resp = match transfer.validated_response() {
            Ok(response) => response,
            Err(error) => {
                warn!(
                    "synchronous IDMAC cmd {} returned unsuccessful R1 status: {error:?}",
                    transfer.context().cmd.cmd_index()
                );
                transfer.fault();
                transfer.finish(true)?;
                return Err(error);
            }
        };
        transfer.finish(false)?;
        Ok(resp)
    }

    /// The interrupt handler for the IDMAC DMA transfer completion.
    pub fn dma_irq_handler() {
        let regs_base = SDMMC_REGS_BASE.load(Ordering::Acquire);
        let mut should_notify = false;
        #[cfg(feature = "sdmmc-concurrency-test")]
        let observe_irq = concurrency_test::observation_enabled();
        #[cfg(feature = "sdmmc-concurrency-test")]
        let irq_entry_ns = observe_irq.then(axhal::time::monotonic_time_nanos);
        if regs_base != 0 {
            let regs = unsafe { VolatilePtr::new(NonNull::new_unchecked(regs_base as *mut _)) };
            let rintsts = regs.rintsts().read();
            let idsts = regs.idsts().read();
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
            should_notify = transfer_event || idmac_error;

            if idmac_error {
                IDMAC_ERROR_FLAG.store(true, Ordering::Release);
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
            dma_io_fence();

            IDMAC_COMPLETION.record_irq(rintsts, idsts);
        }

        if should_notify {
            #[cfg(feature = "sdmmc-concurrency-test")]
            if let Some(irq_entry_ns) = irq_entry_ns {
                concurrency_test::record_irq_wake(
                    IDMAC_COMPLETION.current_generation(),
                    irq_entry_ns,
                    axhal::time::monotonic_time_nanos(),
                );
            }
            IDMAC_DONE_FLAG.store(true, Ordering::Release);
            let _waiter_woken = IDMAC_WAIT_QUEUE.notify_one(false);
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
        }
        self.release_dma_buffer();
    }
}

// SAFETY: all externally reachable methods that mutate controller state require
// exclusive `&mut self`; the IRQ handler only accesses MMIO and atomic snapshots.
unsafe impl Send for SdMmc {}
unsafe impl Sync for SdMmc {}
