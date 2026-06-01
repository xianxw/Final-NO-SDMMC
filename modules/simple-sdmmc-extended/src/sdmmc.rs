use core::{
    ptr::NonNull,
    sync::atomic::{
        fence, Ordering,
        AtomicBool, AtomicUsize,
    },
    alloc::Layout,
    time::Duration,
};

use log::{debug, info, trace, warn};
use volatile::VolatilePtr;
use axtask::WaitQueue;

use crate::{
    cmd::{Command, DataXfer},
    regs::{ClkDiv, ClkEna, RegisterBlock, RegisterBlockVolatileFieldAccess},
    utils::{Cid, CsdV2},
    dma::{IdmacDescriptor, DMABuffer, alloc_coherent, dealloc_coherent},
};

fn wait_until<F>(mut f: F)
where
    F: FnMut() -> bool,
{
    // TODO: yield?
    while !f() {
        core::hint::spin_loop();
    }
}

static IDMAC_WAIT_QUEUE: WaitQueue = WaitQueue::new();
static IDMAC_DONE_FLAG: AtomicBool = AtomicBool::new(false);
static SDMMC_REGS_BASE: AtomicUsize = AtomicUsize::new(0);

/// Data width for SD/MMC data transfer, used to configure the CTYPE register of the controller.
/// Will decide alignment requirements for DMA buffer and data in FIFO.
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
pub struct SdMmc { /// Register block for the SD/MMC controller, accessed through volatile reads/writes.
    regs: VolatilePtr<'static, RegisterBlock>,

    /// Number of blocks on the SD/MMC card, determined during initialization from the CSD register.
    num_blocks: u64,

    /// Indicates whether the Internal DMA (IDMAC) is enabled for data transfer.
    ahb_data_width: AHBDataWidth,

    // Address of the DMA buffer allocated for data transfer.
    dma_buffer: Option<DMABuffer>,

    // Information about the DMA descriptor allocated for data transfer.
    //dma_descriptor: Option<IdmacDescriptor>,

    // Indicates whether the SD/MMC controller supports 64-bit DMA addresses.
    // On VisionFive2, the DWC_MSHC is configured to operate in 32-bit addressing mode, so this is false.
    // support_dma_64bit_address: bool,

    // The size of the descriptor ring buffer, which is the number of descriptors in the ring.
    //dma_descriptor_ring_size: usize,

    // The virtual address of scatter-gather descriptors for DMA transfer.
    // This is accessed by CPU, and must be a valid pointer to memory where the descriptors are allocated.
    // sg_cpu: *mut IdmacDescriptor,

    // The physical address of scatter-gather descriptors for DMA transfer.
    // This is accessed by the SD/MMC controller's IDMAC, and must be bus-addressable.
    //sg_dma: *mut IdmacDescriptor,
}

impl SdMmc {
    /// The offset of the FIFO register from the base address of the SD/MMC controller's register block.
    const FIFO: usize = 0x200;

    // The size of the descriptor ring buffer, which is the number of descriptors in the ring.
    // Equal to page size (4096 bytes) divided by the size of each descriptor (16 bytes), resulting in 256 descriptors.
    // const DESC_RING_BUF_SZ: usize = 4096;

    /// The offset between the kernel's physical address space and virtual address space.
    /// This is used to convert between physical addresses (used for DMA) and virtual addresses (used by the CPU).
    // const KERNEL_VIRT_PHYS_OFFSET: usize = 0xffff_ffc0_0000_0000;

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
            // support_dma_64bit_address: false,
            // dma_descriptor_ring_size: Self::DESC_RING_BUF_SZ / core::mem::size_of::<IdmacDescriptor>(),
            // sg_cpu: core::ptr::null_mut(),
            // sg_dma: core::ptr::null_mut(),
            dma_buffer: None,
            // dma_descriptor: None,
        };
        this.init();
        // Try to enable IDMAC for DMA transfer.
        this.try_enable_idmac(512, AHBDataWidth::Bits32, register_irq);
        
        this
    }

    fn can_send_cmd(&self) -> bool {
        !self.regs.cmd().read().start_cmd()
    }

    fn can_send_data(&self) -> bool {
        !self.regs.status().read().data_busy()
    }

    fn has_response(&self) -> bool {
        self.regs.rintsts().read().command_done()
    }

    fn clear_idsts(&self) {
        let idsts = self.regs.idsts().read();
        if idsts.ais() || idsts.nis() || idsts.ces() || idsts.du() || idsts.fbe() || idsts.ri() || idsts.ti() {
            debug!("Clearing IDSTS: {:?}", idsts);
            self.regs.idsts().write(idsts);
        }
    }

    fn fifo_cnt(&self) -> usize {
        self.regs.status().read().fifo_count() as usize
    }

    fn set_transaction_size(&self, blk_size: u16, byte_cnt: u32) {
        self.regs.blksiz().update(|r| r.with_block_size(blk_size));
        self.regs.bytcnt().write(byte_cnt);

        let blksiz_after = self.regs.blksiz().read();
        let bytcnt_after = self.regs.bytcnt().read();
        info!("set_transaction_size: wrote byte_cnt={} then blk_size={} -> BlkSiz={:?}, ByteCnt=0x{:08x}",
            byte_cnt,
            blk_size,
            blksiz_after,
            bytcnt_after,
        );
    }

    fn send_cmd(&self, command: Command<'_>) -> Option<[u32; 4]> {
        let is_reset_clock = matches!(command, Command::ResetClock);
        let is_go_idle = matches!(command, Command::GoIdleState);
        
        if is_reset_clock {
            info!(">>> Sending ResetClock command");
        }
        if is_go_idle {
            info!(">>> Sending GoIdleState command");
        }
        trace!("send_cmd {command:#x?}");

        let (cmd, arg, xfer) = command.build();
        assert_eq!(cmd.data_expected(), xfer.is_some());

        if is_reset_clock {
            info!("    ResetClock: update_clock_registers_only, response_expect={}", cmd.response_expect());
        }
        if is_go_idle {
            info!("    cmd: {:?}", cmd);
            info!("    response_expect: {}", cmd.response_expect());
            info!("    send_initialization: {}", cmd.send_initialization());
        }
        trace!("send_cmd {cmd:?} {arg:#x?}");

        if is_reset_clock {
            info!("    waiting for can_send_cmd...");
        }

        // Wait for command to be sendable (with timeout counter)
        let mut cmd_wait_count = 0u64;
        let cmd_max_wait = 1_000_000u64;  // ~1M iterations = few seconds on modern CPU
        while !self.can_send_cmd() {
            core::hint::spin_loop();
            cmd_wait_count += 1;
            if cmd_wait_count > cmd_max_wait {
                if is_go_idle {
                    warn!("    can_send_cmd timeout after {} iterations", cmd_wait_count);
                }
                break;
            }
        }
        if is_reset_clock {
            info!("    can_send_cmd: true (waited {} iterations)", cmd_wait_count);
        }
        if is_go_idle {
            info!("    can_send_cmd: true (waited {} iterations)", cmd_wait_count);
        }
        
        if cmd.data_expected() {
            let mut data_wait_count = 0u64;
            while !self.can_send_data() {
                core::hint::spin_loop();
                data_wait_count += 1;
            }
            if data_wait_count > 1000 && is_reset_clock {
                info!("    can_send_data: true (waited {} iterations)", data_wait_count);
            }
        }

        if is_go_idle {
            info!("    can_send_cmd: true");
        }
        self.regs.cmdarg().write(arg);
        self.regs.cmd().write(cmd);

        if is_reset_clock {
            info!("    wrote cmd register, cmd={:?}", cmd);
        }
        if is_go_idle {
            info!("    wrote cmd register");
            info!("    cmd register after write: {:?}", self.regs.cmd().read());
        }

        if is_reset_clock {
            info!("    waiting for start_cmd to clear...");
        }
        
        // Wait for command to complete (with timeout counter)
        let mut start_cmd_wait_count = 0u64;
        while !self.can_send_cmd() {
            core::hint::spin_loop();
            start_cmd_wait_count += 1;
            if start_cmd_wait_count > cmd_max_wait {
                if is_go_idle {
                    warn!("    start_cmd clear timeout after {} iterations", start_cmd_wait_count);
                }
                break;
            }
        }
        trace!("cmd {} sent", cmd.cmd_index());
        if is_reset_clock // Information about the DMA buffer allocated for data transfer.
{
            info!("    start_cmd cleared (waited {} iterations)", start_cmd_wait_count);
        }

        if cmd.response_expect() {
            if is_go_idle {
                info!("    waiting for response...");
                let status_before = self.regs.status().read();
                let rintsts_before = self.regs.rintsts().read();
                info!("    Status before wait: {:?}", status_before);
                info!("    RINTSTS before wait: {:?}", rintsts_before);
            }
            
            // Wait for response (with timeout counter)
            let mut resp_wait_count = 0u64;
            while !self.has_response() {
                core::hint::spin_loop();
                resp_wait_count += 1;
                if resp_wait_count > cmd_max_wait {
                    if is_go_idle {
                        warn!("    response timeout after {} iterations", resp_wait_count);
                        let status_timeout = self.regs.status().read();
                        let rintsts_timeout = self.regs.rintsts().read();
                        warn!("    Status at timeout: {:?}", status_timeout);
                        warn!("    RINTSTS at timeout: {:?}", rintsts_timeout);
                    }
                    break;
                }
            }
            
            trace!("cmd {} received response", cmd.cmd_index());
            if is_go_idle {
                info!("    received response (waited {} iterations)", resp_wait_count);
            }
        } else {
            if is_reset_clock {
                info!("    no response expected for this command");
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
                            while self.fifo_cnt() >= 2 {
                                let data = unsafe { fifo_base.byte_add(offset).read_volatile() };
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
                            // Hard coded FIFO depth
                            while self.fifo_cnt() < 120 && offset < buf.len() {
                                let data =
                                    u64::from_le_bytes(buf[offset..offset + 8].try_into().unwrap());
                                unsafe { fifo_base.byte_add(offset).write_volatile(data) };
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
        // clear interrupt status
        self.regs.rintsts().write(rintsts);

        if rintsts.error() {
            warn!("cmd {} error - rintsts: {rintsts:?}, resp: {resp:?}", cmd.cmd_index());
            warn!("  response_timeout: {}, data_read_timeout: {}, start_bit_error: {}, end_bit_error: {}",
                  rintsts.response_timeout(), rintsts.data_read_timeout(), rintsts.start_bit_error(), rintsts.end_bit_error());
            warn!("  data_crc_error: {}, response_crc_error: {}, response_error: {}, hardware_locked_write: {}",
                  rintsts.data_crc_error(), rintsts.response_crc_error(), rintsts.response_error(), rintsts.hardware_locked_write());
            return None;
        }
        Some(resp)
    } 

    fn init(&mut self) {
        info!("Initializing SD/MMC driver at {:?}", self.regs);

        // On VisionFive2, some registers have been initialized by the bootloader(U-Boot).
        // But some default values are not suitable for our driver, so we need to reset and reconfigure them.
        trace!("ctrl: {:?}", self.regs.ctrl().read());
        trace!("pwren: {:?}", self.regs.pwren().read());
        trace!("clkdiv: {:?}", self.regs.clkdiv().read());
        trace!("clksrc: {:?}", self.regs.clksrc().read());
        trace!("clkena: {:?}", self.regs.clkena().read());
        trace!("tmout: {:?}", self.regs.tmout().read());
        trace!("ctype: {:?}", self.regs.ctype().read());
        trace!("cdetect: {:?}", self.regs.cdetect().read());
        trace!("wrtprt: {:?}", self.regs.wrtprt().read());
        trace!("usrid: {:?}", self.regs.usrid().read());
        trace!("verid: {:?}", self.regs.verid().read());
        trace!("hcon: {:?}", self.regs.hcon().read());
        trace!("uhs: {:?}", self.regs.uhs().read());
        trace!("bmod: {:?}", self.regs.bmod().read());
        trace!("dbaddr: {:?}", self.regs.dbaddr().read());

        // Clear any stale interrupt status flags left by bootloader
        // Writing 1 to these bits clears them
        let rintsts = self.regs.rintsts().read();
        trace!("initial rintsts: {rintsts:?}");
        self.regs.rintsts().write(rintsts);
        trace!("cleared interrupt status");

        // Clock is already initialized by U-Boot, but we need to reconfigure it
        // First, check current state
        warn!("=== SD/MMC Clock Initialization ===");
        warn!("Initial clkena: {:?}", self.regs.clkena().read());
        warn!("Initial clkdiv: {:?}", self.regs.clkdiv().read());
        warn!("Initial ctrl: {:?}", self.regs.ctrl().read());

        // Disable clock for configuration
        warn!("Step 1: Disabling clock...");
        self.regs.clkena().write(ClkEna::new());
        warn!("  clkena after disable: {:?}", self.regs.clkena().read());
        
        // Send ResetClock command to update clock in disabled state
        warn!("Step 2: Sending ResetClock in disabled state...");
        match self.send_cmd(Command::ResetClock) {
            Some(_) => warn!("  ResetClock succeeded"),
            None => warn!("  ResetClock FAILED - continuing anyway"),
        }

        // Set clock divider to lower frequency (slower for compatibility)
        warn!("Step 3: Setting clock divider to 100 (lower frequency)...");
        self.regs.clkdiv().write(ClkDiv::new().with_clk_divider0(100));
        warn!("  clkdiv after set: {:?}", self.regs.clkdiv().read());

        // Now enable clock with new divider
        warn!("Step 4: Enabling clock...");
        self.regs.clkena().write(ClkEna::new().with_cclk_enable(1));
        warn!("  clkena after enable: {:?}", self.regs.clkena().read());
        
        // Send ResetClock to activate new clock settings
        warn!("Step 5: Sending ResetClock to activate new clock...");
        match self.send_cmd(Command::ResetClock) {
            Some(_) => warn!("  ResetClock succeeded"),
            None => warn!("  ResetClock FAILED - continuing anyway"),
        }
        
        // Long delay to let everything stabilize
        warn!("Step 6: Waiting for clock stabilization...");
        for _ in 0..10000 {
            core::hint::spin_loop();
        }
        
        warn!("Clock initialization complete:");
        warn!("  Final clkena: {:?}", self.regs.clkena().read());
        warn!("  Final clkdiv: {:?}", self.regs.clkdiv().read());
        warn!("  Final status: {:?}", self.regs.status().read());
        warn!("=== End Clock Initialization ===");

        // Check card presence and status
        warn!("=== Pre-Command Diagnostics ===");
        warn!("CTYPE (card type): {:?}", self.regs.ctype().read());
        warn!("STATUS register: {:?}", self.regs.status().read());
        let status = self.regs.status().read();
        warn!("  card_data_3_status: {}", status.data_3_status());
        warn!("  fifo_count: {}", status.fifo_count());
        warn!("  command_fsm_states: {}", status.command_fsm_states());
        warn!("RINTSTS (interrupt status): {:?}", self.regs.rintsts().read());
        warn!("MINTSTS (masked interrupt): {:?}", self.regs.mintsts().read());
        
        // Enable card power if available in PWREN register
        warn!("Setting card power enable (PWREN)...");
        self.regs.pwren().write(1u32.into());  // Card power enable
        warn!("  PWREN: {:?}", self.regs.pwren().read());
        
        // Increased stabilization delay
        warn!("Extended clock stabilization delay (100k cycles)...");
        for _ in 0..100000 {
            core::hint::spin_loop();
        }
        
        // set data width -> 1bit
        self.regs.ctype().write(0.into());

        // reset dma
        self.regs.bmod().update(|r| r.with_de(false).with_swr(true));
        self.regs
            .ctrl()
            .update(|r| r.with_dma_reset(true).with_use_internal_dmac(false));

        trace!("dma reset");

        trace!("ctrl: {:?}", self.regs.ctrl().read());

        warn!("=== Sending GoIdleState command ===");
        warn!("  Before GoIdleState - STATUS: {:?}", self.regs.status().read());
        warn!("  Before GoIdleState - RINTSTS: {:?}", self.regs.rintsts().read());
        // Note: GoIdleState may timeout during initial card detection phase.
        // This is not fatal - the card responds to SendIfCond and continues initialization normally.
        // The timeout likely occurs because the card is still stabilizing at the new clock frequency.
        match self.send_cmd(Command::GoIdleState) {
            Some(_) => warn!("GoIdleState succeeded"),
            None => warn!("GoIdleState timeout (expected during initialization) - continuing..."),
        }
        trace!("idle state set");

        warn!("Sending SendIfCond command...");
        let has_valid_resp = match self.send_cmd(Command::SendIfCond(0x1aa)) {
            Some(resp) => {
                warn!("SendIfCond succeeded: {:?}", resp);
                if resp[0] & 0xff != 0xaa {
                    warn!("Warning: unexpected response for SendIfCond");
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

        warn!("Starting ACMD41 loop to detect SD card...");
        let mut attempt = 0;
        let mut card_initialized = false;
        loop {
            attempt += 1;
            if attempt > 100 {
                warn!("ACMD41 loop exceeded 100 attempts - giving up");
                break;
            }
            
            match self.send_cmd(Command::AppCmd(0)) {
                Some(_) => trace!("AppCmd succeeded"),
                None => {
                    warn!("AppCmd failed on attempt {}", attempt);
                    continue;
                }
            }
            
            match self.send_cmd(Command::SdSendOpCond(0x41FF_8000)) {
                Some(resp) => {
                    let ocr = resp[0];
                    if ocr & 0x8000_0000 != 0 {
                        warn!("SD card is ready after {} attempts", attempt);
                        card_initialized = true;
                        if ocr & 0x4000_0000 != 0 {
                            debug!("SD card supports high capacity");
                        } else {
                            debug!("SD card is standard capacity");
                        }
                        break;
                    } else {
                        trace!("SD card not ready yet, attempt {}, ocr: {ocr:x}", attempt);
                    }
                }
                None => {
                    warn!("SdSendOpCond failed on attempt {}", attempt);
                }
            }
            
            core::hint::spin_loop();
        }
        
        if !card_initialized {
            warn!("Card initialization failed - continuing anyway");
            return;  // Cannot continue without card
        }

        warn!("Sending AllSendCid command...");
        match self.send_cmd(Command::AllSendCid) {
            Some(resp) => {
                let cid = unsafe { core::mem::transmute::<[u32; 4], Cid>(resp) };
                warn!("cid: {cid:?}");
            }
            None => {
                warn!("AllSendCid failed - cannot determine card ID");
                return;
            }
        }

        warn!("Sending SendRelativeAddr command...");
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

        warn!("Sending SendCsd command...");
        match self.send_cmd(Command::SendCsd(rca << 16)) {
            Some(resp) => {
                let csd = unsafe { core::mem::transmute::<[u32; 4], CsdV2>(resp) };
                debug!("csd: {csd:?}");
                self.num_blocks = csd.num_blocks();
                warn!("SD card capacity: {:#x} blocks", self.num_blocks);
            }
            None => {
                warn!("SendCsd failed - cannot determine card capacity");
                self.num_blocks = 0;
            }
        }

        warn!("Sending SelectCard command...");
        match self.send_cmd(Command::SelectCard(rca << 16)) {
            Some(_) => warn!("SelectCard succeeded"),
            None => warn!("SelectCard failed"),
        }

        warn!("Sending AppCmd command...");
        match self.send_cmd(Command::AppCmd(rca << 16)) {
            Some(_) => warn!("AppCmd succeeded"),
            None => warn!("AppCmd failed"),
        }

        // Read SCR register of SD card to determine supported bus widths.
        // This is needed before we can set the bus width.
        self.set_transaction_size(8, 8);
        // Although the SCR register is only 8 bytes, we allocate a 512-byte buffer here.
        // This is because many controllers and drivers require the buffer to be block-aligned (e.g., 512 bytes),
        // and only the first 8 bytes will be filled with valid data. The rest is ignored.
        // This ensures compatibility with hardware and avoids DMA or alignment issues.
        let mut buf = [0u8; 512];
        warn!("Sending SendScr command...");
        match self.send_cmd(Command::SendScr(&mut buf)) {
            Some(_) => warn!("SendScr succeeded"),
            None => warn!("SendScr failed")
        }

        trace!("fifo count: {}", self.fifo_cnt());
        let resp = unsafe {
            self.regs
                .as_raw_ptr()
                .byte_add(Self::FIFO)
                .cast::<u64>()
                .read_volatile()
        };
        debug!("Bus width supported: {:#x?}", (resp >> 8) & 0xf);
        trace!("fifo count: {}", self.fifo_cnt());
        
        trace!("ctrl: {:?}", self.regs.ctrl().read());
        let rintsts = self.regs.rintsts().read();
        trace!("rintsts: {rintsts:?}");
        self.regs.rintsts().write(rintsts); // clear interrupt status

        info!("SD/MMC driver initialized");
    }

    /// Reads a single block from the SD/MMC card.
    pub fn read_block(&mut self, block: u32, buf: &mut [u8; 512]) {
        self.set_transaction_size(512, 512);
        
        info!("read block: block={}", block);

        if let Some(dma_buf_info) = &self.dma_buffer {
            trace!("Using DMA buffer for read: virt=0x{:08x}, phys=0x{:08x}, size={}", 
                dma_buf_info.addr.cpu_addr.as_ptr() as usize, dma_buf_info.addr.bus_addr.as_u64() as usize, dma_buf_info.size);
            
            let dma_buf_phy_ptr = dma_buf_info.addr.bus_addr.as_u64() as *mut u8;
            let dma_buf = unsafe { core::slice::from_raw_parts_mut(dma_buf_phy_ptr, buf.len()) };

            info!("read_block: before send_cmd_idmac - BlkSiz={:?}, ByteCnt=0x{:08x}, CType={:?}, FIFOTH={:?}",
                self.regs.blksiz().read(),
                self.regs.bytcnt().read(),
                self.regs.ctype().read(),
                self.regs.fifoth().read(),
            );
            self.send_cmd_idmac(Command::ReadSingleBlock(block, dma_buf)).unwrap();
            info!("read_block: after send_cmd_idmac - BlkSiz={:?}, ByteCnt=0x{:08x}, CType={:?}, FIFOTH={:?}",
                self.regs.blksiz().read(),
                self.regs.bytcnt().read(),
                self.regs.ctype().read(),
                self.regs.fifoth().read(),
            );

            let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
            let dma_usr_slice = unsafe { core::slice::from_raw_parts(dma_buf_virt_ptr, buf.len()) };
            buf.copy_from_slice(dma_usr_slice);
        } else {
            warn!("No DMA buffer available - read may fail or be very slow");
            self.send_cmd(Command::ReadSingleBlock(block, buf)).unwrap();
        }

        trace!("fifo count: {}", self.fifo_cnt());
    }

    /// Writes a single block to the SD/MMC card.
    pub fn write_block(&mut self, block: u32, buf: &[u8; 512]) {
        self.set_transaction_size(512, 512);
        
        info!("write block: block={}", block);

        // try_enable_idmac ensures that only when IDMAC is activated, there's an allocated DMA buffer.
        if let Some(dma_buf_info) = &self.dma_buffer {
            trace!("Using DMA buffer for write: virt=0x{:08x}, phys=0x{:08x}, size={}", 
                dma_buf_info.addr.cpu_addr.as_ptr() as usize, dma_buf_info.addr.bus_addr.as_u64() as usize, dma_buf_info.size);

            let dma_buf_virt_ptr = dma_buf_info.addr.cpu_addr.as_ptr();
            let dma_usr_slice = unsafe { core::slice::from_raw_parts_mut(dma_buf_virt_ptr, buf.len()) };
            dma_usr_slice.copy_from_slice(buf);

            let dma_buf_phy_ptr = dma_buf_info.addr.bus_addr.as_u64() as *mut u8;
            let dma_buf = unsafe { core::slice::from_raw_parts(dma_buf_phy_ptr, buf.len()) };
            info!("write_block: before send_cmd_idmac - BlkSiz={:?}, ByteCnt=0x{:08x}, CType={:?}, FIFOTH={:?}",
                self.regs.blksiz().read(),
                self.regs.bytcnt().read(),
                self.regs.ctype().read(),
                self.regs.fifoth().read(),
            );
            self.send_cmd_idmac(Command::WriteSingleBlock(block, dma_buf)).unwrap();
            info!("write_block: after send_cmd_idmac - BlkSiz={:?}, ByteCnt=0x{:08x}, CType={:?}, FIFOTH={:?}",
                self.regs.blksiz().read(),
                self.regs.bytcnt().read(),
                self.regs.ctype().read(),
                self.regs.fifoth().read(),
            );
        } else {
            warn!("No DMA buffer available - write may fail or be very slow");
            self.send_cmd(Command::WriteSingleBlock(block, buf)).unwrap();
        }
        
        trace!("fifo count: {}", self.fifo_cnt());
    }

    /// Returns the number of blocks.
    pub fn num_blocks(&self) -> u64 {
        self.num_blocks
    }

    /// Enables the Internal DMA (IDMAC) for DMA transfers.
    pub fn try_enable_idmac(&mut self, buf_size: usize, ahb_data_width: AHBDataWidth, register_irq: impl FnOnce() -> bool) {
        info!("Trying to enable IDMAC for DMA transfer");

        // Step 1: Allocate a DMA buffer for the data transfer.
        // According to DW_MSHC specification, data in the buffer must be 4 bytes aligned in 32 modes
        let layout = Layout::from_size_align(buf_size, ahb_data_width.align_value()).expect("Invalid layout for DMA buffer");
        match unsafe { alloc_coherent(layout) } {
            Ok(dma_info) => {
                info!("DMA buffer allocated: virt=0x{:08x}, phys=0x{:08x}, size={}", 
                    dma_info.cpu_addr.as_ptr() as usize, dma_info.bus_addr.as_u64(), buf_size);
                self.dma_buffer = Some(DMABuffer { addr: dma_info, size: buf_size });
            }
            Err(e) => {
                warn!("Failed to allocate DMA buffer: {:?}, use PIO mode instead", e);
                return;
            }
        }
        
        // Step 2: Set up the IDMAC descriptor ring and point the DBADDR register to it.

        // Step 3: Configure the BMOD and CTRL registers to enable IDMAC.
        // If failed, deallocate the DMA buffer and return without enabling IDMAC.

        let rintsts_before_enable = self.regs.rintsts().read();
        let idsts_before_enable = self.regs.idsts().read();
        let idinten_before_enable = self.regs.idinten().read();
        info!("try_enable_idmac: pre-enable RINTSTS={:?}, IDSTS={:?}, IDINTEN={:?}, DBADDR=0x{:08x}",
            rintsts_before_enable,
            idsts_before_enable,
            idinten_before_enable,
            self.regs.dbaddr().read(),
        );
        if rintsts_before_enable.error() || rintsts_before_enable.data_transfer_over() || rintsts_before_enable.receive_fifo_data_request() || rintsts_before_enable.transmit_fifo_data_request() {
            info!("try_enable_idmac: clearing stale RINTSTS before IDMAC enable: {:?}", rintsts_before_enable);
            self.regs.rintsts().write(rintsts_before_enable);
        }
        if idsts_before_enable.ais() || idsts_before_enable.nis() || idsts_before_enable.ces() || idsts_before_enable.du() || idsts_before_enable.fbe() || idsts_before_enable.ri() || idsts_before_enable.ti() {
            debug!("try_enable_idmac: stale IDSTS before IDMAC enable: {:?}", idsts_before_enable);
            self.clear_idsts();
        }

        // Set the BMOD register to enable the internal DMA controller (IDMAC).
        // BMOD's PBL value is read-only value and is the mirror of MSIZE of FIFOTH register.
        // And the DSL value is applicable only for dual buffer structure.
        self.regs.bmod().update(|r| r.with_de(true).with_dsl(0).with_fb(true));
        // Immediately reading back BMOD register after writing is necessary to ensure that the write has taken effect before proceeding.
        let bmod_after = self.regs.bmod().read();
        info!("BMOD after enabling IDMAC: {:?}", bmod_after);
        info!("BMOD expected: de=true, dsl=0, fb=true; actual: de={}, dsl={}, fb={}, pbl={}",
            bmod_after.de(),
            bmod_after.dsl(),
            bmod_after.fb(),
            bmod_after.pbl(),
        );
        let idsts_after_bmod = self.regs.idsts().read();
        if idsts_after_bmod.du() || idsts_after_bmod.fbe() || idsts_after_bmod.ais() {
            warn!("try_enable_idmac: abnormal IDSTS after BMOD enable: {:?}", idsts_after_bmod);
        } else {
            info!("try_enable_idmac: IDSTS after BMOD enable: {:?}", idsts_after_bmod);
        }
        if !bmod_after.de() || bmod_after.dsl() != 0 || !bmod_after.fb() {
            warn!("Failed to set BMOD register for IDMAC, use PIO mode instead; actual: de={}, dsl={}, fb={}, pbl={}",
                bmod_after.de(),
                bmod_after.dsl(),
                bmod_after.fb(),
                bmod_after.pbl(),
            );
            unsafe { dealloc_coherent(self.dma_buffer.as_ref().unwrap().addr, layout);}
            self.dma_buffer = None;
            return;
        }

        // Set the CTRL register to enable the use of the internal DMA controller (IDMAC)
        // and enable the SD/MMC controller interrupt output.
        self.regs.ctrl().update(|r| r.with_use_internal_dmac(true).with_int_enable(true));
        // Immediately reading back CTRL register after writing is necessary to ensure that the write has taken effect before proceeding.
        let ctrl_after = self.regs.ctrl().read();
        let idsts_after_ctrl = self.regs.idsts().read();
        info!("CTRL after enabling IDMAC: {:?}", ctrl_after);
        info!("CTRL expected: use_internal_dmac=true, int_enable=true; actual: use_internal_dmac={}, int_enable={}",
            ctrl_after.use_internal_dmac(),
            ctrl_after.int_enable(),
        );
        info!("try_enable_idmac: IDSTS after CTRL enable: {:?}", idsts_after_ctrl);
        if !ctrl_after.use_internal_dmac() || !ctrl_after.int_enable() {
            warn!("Failed to set CTRL register for IDMAC and interrupt output, use PIO mode instead; expected use_internal_dmac=true, int_enable=true. actual: use_internal_dmac={}, int_enable={}. IDSTS={:?}",
                ctrl_after.use_internal_dmac(),
                ctrl_after.int_enable(),
                idsts_after_ctrl
            );
            unsafe { dealloc_coherent(self.dma_buffer.as_ref().unwrap().addr, layout);}
            self.dma_buffer = None;
            return;
        }
        if idsts_after_ctrl.du() || idsts_after_ctrl.fbe() || idsts_after_ctrl.ais() {
            warn!("try_enable_idmac: abnormal IDSTS after CTRL enable; disabling IDMAC path: {:?}", idsts_after_ctrl);
            unsafe { dealloc_coherent(self.dma_buffer.as_ref().unwrap().addr, layout);}
            self.dma_buffer = None;
            return;
        }

        // Step 4: Enable IDMAC interrupts inside the SD/MMC controller.
        // Without these, the controller will not raise an external IRQ for DMA completion.
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
        self.regs.intmask().write(
            crate::regs::IntMask::new()
                .with_dto(true),
        );

        // Immediately read back the interrupt settings for verification.
        let idinten_after = self.regs.idinten().read();
        let intmask_after = self.regs.intmask().read();
        let idsts_after_enable = self.regs.idsts().read();
        let rintsts_after_enable = self.regs.rintsts().read();
        info!("IDINTEN after enable: {:?}", idinten_after);
        info!("INTMASK after enable: {:?}", intmask_after);
        info!("try_enable_idmac: post-enable RINTSTS={:?}, IDSTS={:?}, IDINTEN={:?}, DBADDR=0x{:08x}",
            rintsts_after_enable,
            idsts_after_enable,
            idinten_after,
            self.regs.dbaddr().read(),
        );
        info!("IDINTEN expected: ai=true, ni=true, ces=true, du=true, fbe=true, ri=true, ti=true; actual: ai={}, ni={}, ces={}, du={}, fbe={}, ri={}, ti={}",
            idinten_after.ai(),
            idinten_after.ni(),
            idinten_after.ces(),
            idinten_after.du(),
            idinten_after.fbe(),
            idinten_after.ri(),
            idinten_after.ti(),
        );
        if !idinten_after.ai() || !idinten_after.ni() || !idinten_after.ces() || !idinten_after.du() || !idinten_after.fbe() || !idinten_after.ri() || !idinten_after.ti() {
            warn!("try_enable_idmac: IDINTEN mismatch after write; verify hardware support and register access");
        }
        if !intmask_after.dto() || intmask_after.cmd() || intmask_after.rxdr() || intmask_after.txdr() {
            warn!("try_enable_idmac: INTMASK mismatch after write; dto={}, cmd={}, rxdr={}, txdr={}",
                intmask_after.dto(),
                intmask_after.cmd(),
                intmask_after.rxdr(),
                intmask_after.txdr(),
            );
        }
        if idsts_after_enable.du() || idsts_after_enable.fbe() || idsts_after_enable.ais() {
            warn!("try_enable_idmac: abnormal post-enable IDSTS detected: {:?}", idsts_after_enable);
        }
        info!("SDMMC interrupt mask config: dto={}, cmd={}, rxdr={}, txdr={}",
            intmask_after.dto(),
            intmask_after.cmd(),
            intmask_after.rxdr(),
            intmask_after.txdr()
        );

        // Step 5: Enable a kernel IRQ handler for the SD/MMC device.
        // This is done after setting up the DMA buffer and registers to avoid handling interrupts before ready,
        // which could lead to errors or undefined behavior, and also to avoid introducing the unregister function
        // and related complexity in this module.
        let irq_registered = register_irq();
        info!("try_enable_idmac: IRQ registration returned {}", irq_registered);
        if !irq_registered {
            let idsts_on_irq_fail = self.regs.idsts().read();
            let idinten_on_irq_fail = self.regs.idinten().read();
            let rintsts_on_irq_fail = self.regs.rintsts().read();
            warn!("Failed to register IRQ for IDMAC, use PIO mode instead; RINTSTS={:?}, IDSTS={:?}, IDINTEN={:?}, DBADDR=0x{:08x}",
                rintsts_on_irq_fail,
                idsts_on_irq_fail,
                idinten_on_irq_fail,
                self.regs.dbaddr().read(),
            );
            // DMA buffer should have been allocated successfully if we reached this point,
            // but we won't be able to use it without IRQ, so deallocate it and return without enabling IDMAC.
            unsafe { dealloc_coherent(self.dma_buffer.as_ref().unwrap().addr, layout);}
            self.dma_buffer = None;

            // Reset registers to disable IDMAC and return to a clean state 
            self.regs.bmod().update(|r| r.with_de(false).with_swr(true));
            self.regs.ctrl().update(|r| r.with_dma_reset(true).with_use_internal_dmac(false));
            return;
        }

        info!("IDMAC enabled for DMA transfer");
    }

    // TODO: make it only for data read/write to avoid code duplication with send_cmd,
    // and also to avoid the complexity of handling response and command completion in this function.
    /// Sends a command using the Internal DMA (IDMAC) for data transfer if required.
    pub fn send_cmd_idmac(
        &self,
        command: Command<'_>,
    ) -> Option<[u32; 4]> {
        trace!("send_cmd_idmac {command:#x?}");

        let (cmd, arg, xfer) = command.build();
        assert!(cmd.data_expected(), "send_cmd_idmac should only be used for commands that require data transfer");
        assert!(xfer.is_some(), "send_cmd_idmac requires a data buffer for transfer");
        // assert_eq!(cmd.data_expected(), xfer.is_some());

        // let has_data = xfer.is_some();  // temp code
        info!("send_cmd_idmac {cmd:?} {arg:#x?}");

        let ctype_before = self.regs.ctype().read();
        let fifoth_before = self.regs.fifoth().read();
        let blksiz_before = self.regs.blksiz().read();
        let bytcnt_before = self.regs.bytcnt().read();
        let cmd_before = self.regs.cmd().read();
        let idinten_before = self.regs.idinten().read();
        let dbaddr_before = self.regs.dbaddr().read();
        let idsts_before = self.regs.idsts().read();
        info!("send_cmd_idmac status before transfer: CType={:?}, FIFOTH={:?}, BlkSiz={:?}, ByteCnt=0x{:08x}, CMD={:?}",
            ctype_before,
            fifoth_before,
            blksiz_before,
            bytcnt_before,
            cmd_before,
        );
        info!("send_cmd_idmac pre-transfer IDINTEN={:?} DBADDR=0x{:08x} IDSTS={:?}",
            idinten_before,
            dbaddr_before,
            idsts_before,
        );
        debug!("send_cmd_idmac: waiting for can_send_cmd");
        wait_until(|| self.can_send_cmd());
        debug!("send_cmd_idmac: can_send_cmd ready");
        debug!("send_cmd_idmac: waiting for can_send_data");
        wait_until(|| self.can_send_data());
        debug!("send_cmd_idmac: can_send_data ready");

        // Clear stale status before a new command/DMA transaction.
        // rintsts is a write-1-to-clear register.
        let stale_rintsts = self.regs.rintsts().read();
        self.regs.rintsts().write(stale_rintsts);

        // idsts is a write-1-to-clear register.
        let stale_idsts = self.regs.idsts().read();
        if stale_idsts.ais() || stale_idsts.nis() || stale_idsts.ces() || stale_idsts.du() || stale_idsts.fbe() || stale_idsts.ri() || stale_idsts.ti() {
            debug!("send_cmd_idmac: clearing stale IDSTS before transfer: {:?}", stale_idsts);
            self.clear_idsts();
        }

        // Read the IDMAC status before starting the transfer, so that we can detect any new errors that occur during the transfer by comparing with this baseline.
        let idsts_before = self.regs.idsts().read();

        let xfer = xfer.unwrap();
        
        info!("Data required, using IDMAC for transfer");

            // Reset the IDMAC done flag before starting a new transfer.
            IDMAC_DONE_FLAG.store(false, Ordering::Release);

            let (buf_len, buf_ptr) = match xfer {
                DataXfer::Read(buf) => {
                    let len = buf.len();
                    (len, buf.as_ptr() as usize)
                }
                DataXfer::Write(buf) => {
                    let len = buf.len();
                    (len, buf.as_ptr() as usize)
                }
            };

            assert!(
                buf_len <= 0x1fff,
                "IDMAC single descriptor buffer too large: {buf_len}"
            );

            info!("send_cmd_idmac: Buffer physical address: 0x{:08x}", buf_ptr);

            // TODO: Deallocate this descriptor after the transfer is done.
            // Set up the IDMAC descriptor for the DMA transfer.
            // Use one descriptor for one contiguous buffer.
            let layout = Layout::new::<IdmacDescriptor>();
            let dma_desc_info = unsafe { alloc_coherent(layout) }.expect("Failed to allocate DMA descriptor");
            let desc_ptr = dma_desc_info.cpu_addr.as_ptr() as *mut IdmacDescriptor;

            unsafe { core::ptr::write_volatile(desc_ptr, IdmacDescriptor::new()) };
            
            let desc = unsafe { &mut *desc_ptr };
            // Set the control bits for the DMA transfer in des0.
            // OWN must be set so IDMAC can fetch and process the descriptor.
            desc.set_desc0_control_descriptor(true, false, false, false, true, true, false);
            desc.set_des1_buffer1_size(buf_len as u16);
            desc.set_des2_buffer1_address(buf_ptr as u32);
            desc.set_des3_next_descriptor_address(0);

            info!("IDMAC descriptor control: own={}, ces={}, er={}, ch={}, fs={}, ld={}, dic={}",
                desc.des0.own(),
                desc.des0.ces(),
                desc.des0.er(),
                desc.des0.ch(),
                desc.des0.fs(),
                desc.des0.ld(),
                desc.des0.dic(),
            );
            info!("IDMAC descriptor sizes: bs1={}, desc2=0x{:08x}, desc3=0x{:08x}",
                desc.des1.bs1(),
                desc.des2,
                desc.des3,
            );

            // Write the physical address of the descriptor to the DBADDR register to set up the DMA transfer.
            let desc_phy_addr = (dma_desc_info.bus_addr.as_u64()) as u32;
            // Ensure descriptor writes are visible before giving its address to IDMAC.
            fence(Ordering::Release);
            let bytcnt_before_dbaddr = self.regs.bytcnt().read();
            info!("send_cmd_idmac: ByteCnt before DBADDR = 0x{:08x}", bytcnt_before_dbaddr);
            self.regs.bytcnt().write(buf_len as u32);
            let bytcnt_after_dbaddr_write = self.regs.bytcnt().read();
            info!("send_cmd_idmac: ByteCnt rewritten before DBADDR = 0x{:08x}", bytcnt_after_dbaddr_write);
            self.regs.dbaddr().write(desc_phy_addr);
            let dbaddr_after = self.regs.dbaddr().read();
            let idsts_after_dbaddr = self.regs.idsts().read();
            let bytcnt_after_dbaddr = self.regs.bytcnt().read();
            debug!("send_cmd_idmac stage=DBADDR; desc_phy=0x{:08x}; DBADDR=0x{:08x}; ByteCnt=0x{:08x}; IDSTS={:?}",
                desc_phy_addr,
                dbaddr_after,
                bytcnt_after_dbaddr,
                idsts_after_dbaddr,
            );

            self.regs.cmdarg().write(arg);
            let cmdarg_after = self.regs.cmdarg().read();
            let bytcnt_after_cmdarg = self.regs.bytcnt().read();
            let dbaddr_after_cmdarg = self.regs.dbaddr().read();
            debug!("send_cmd_idmac stage=CMDARG; CMDARG=0x{:08x}; ByteCnt=0x{:08x}; DBADDR=0x{:08x}",
                cmdarg_after,
                bytcnt_after_cmdarg,
                dbaddr_after_cmdarg,
            );

            self.regs.cmd().write(cmd);
            let cmd_after = self.regs.cmd().read();
            let idsts_after_cmd = self.regs.idsts().read();
            let dbaddr_after_cmd = self.regs.dbaddr().read();
            let bytcnt_after_cmd = self.regs.bytcnt().read();
            debug!("send_cmd_idmac stage=CMD; CMD={:?}; ByteCnt=0x{:08x}; DBADDR=0x{:08x}; IDSTS={:?}; start_cmd={}; response_expect={}",
                cmd_after,
                bytcnt_after_cmd,
                dbaddr_after_cmd,
                idsts_after_cmd,
                cmd_after.start_cmd(),
                cmd_after.response_expect(),
            );
            info!("cmd {} sent", cmd.cmd_index());

            let mut start_cmd_wait_count = 0u64;
            let start_cmd_max_wait = 1_000_000u64;
            while self.regs.cmd().read().start_cmd() {
                core::hint::spin_loop();
                start_cmd_wait_count += 1;
                if start_cmd_wait_count >= start_cmd_max_wait {
                    warn!("send_cmd_idmac: start_cmd did not clear after {} iterations", start_cmd_wait_count);
                    break;
                }
            }
            info!("send_cmd_idmac: start_cmd cleared after {} iterations; current CMD={:?}",
                start_cmd_wait_count,
                self.regs.cmd().read(),
            );

            if idsts_after_cmd.du() {
                warn!("send_cmd_idmac: IDSTS indicates Descriptor Unavailable after CMD; issuing a second PLDMND");
                self.regs.pldmnd().write(1);
            }

            // After the command is issued, use PLDMND to wake IDMAC if it is suspended.
            self.regs.pldmnd().write(1);
            let ctype_after_pldmnd = self.regs.ctype().read();
            let fifoth_after_pldmnd = self.regs.fifoth().read();
            let blksiz_after_pldmnd = self.regs.blksiz().read();
            let bytcnt_after_pldmnd = self.regs.bytcnt().read();
            let cmd_after_pldmnd = self.regs.cmd().read();
            let idsts_after_pldmnd = self.regs.idsts().read();
            let rintsts_after_pldmnd = self.regs.rintsts().read();
            debug!("send_cmd_idmac stage=PLDMND; CType={:?}; FIFOTH={:?}; BlkSiz={:?}; ByteCnt=0x{:08x}; CMD={:?}; IDSTS={:?}; RINTSTS={:?}",
                ctype_after_pldmnd,
                fifoth_after_pldmnd,
                blksiz_after_pldmnd,
                bytcnt_after_pldmnd,
                cmd_after_pldmnd,
                idsts_after_pldmnd,
                rintsts_after_pldmnd,
            );
            debug!("send_cmd_idmac stage=PLDMND bits; command_done={}, data_transfer_over={}, response_error={}, receive_fifo_data_request={}, transmit_fifo_data_request={}",
                rintsts_after_pldmnd.command_done(),
                rintsts_after_pldmnd.data_transfer_over(),
                rintsts_after_pldmnd.response_error(),
                rintsts_after_pldmnd.receive_fifo_data_request(),
                rintsts_after_pldmnd.transmit_fifo_data_request(),
            );
            debug!("send_cmd_idmac stage=PLDMND flags; ais={}, nis={}, ces={}, du={}, fbe={}, ri={}, ti={}",
                idsts_after_pldmnd.ais(),
                idsts_after_pldmnd.nis(),
                idsts_after_pldmnd.ces(),
                idsts_after_pldmnd.du(),
                idsts_after_pldmnd.fbe(),
                idsts_after_pldmnd.ri(),
                idsts_after_pldmnd.ti(),
            );
            if idsts_after_pldmnd.du() {
                warn!("send_cmd_idmac: IDSTS still indicates Descriptor Unavailable after CMD+PLDMND; disabling IDMAC path");
                unsafe { dealloc_coherent(dma_desc_info, layout); }
                return None;
            }
            if idsts_after_pldmnd.ais() || idsts_after_pldmnd.fbe() {
                warn!("send_cmd_idmac: IDMAC abnormal status after CMD+PLDMND; disabling IDMAC path");
                unsafe { dealloc_coherent(dma_desc_info, layout); }
                return None;
            }
            for i in 1..4 {
                core::hint::spin_loop();
                let idsts_loop = self.regs.idsts().read();
                let rintsts_loop = self.regs.rintsts().read();
                let bytcnt_loop = self.regs.bytcnt().read();
                debug!("send_cmd_idmac stage=PLDMND[{}]; ByteCnt=0x{:08x}; IDSTS={:?}; RINTSTS={:?}",
                    i,
                    bytcnt_loop,
                    idsts_loop,
                    rintsts_loop,
                );
            }

            info!("IDMAC descriptor set up at physical address: 0x{:08x}", desc_phy_addr);

        // Wait for the DMA interrupt handler to confirm the transfer completion.
        let deadline = axhal::time::wall_time() + Duration::from_secs(1);
        let mut dma_irq_timed_out = false;
        while !IDMAC_DONE_FLAG.load(Ordering::Acquire) {
            if axhal::time::wall_time() >= deadline {
                dma_irq_timed_out = true;
                break;
            }
            axtask::yield_now();
        }

        let rintsts_during_irq = self.regs.rintsts().read();
        let idsts_during_irq = self.regs.idsts().read();
        if dma_irq_timed_out {
            warn!("send_cmd_idmac: DMA IRQ did not arrive within 1 second");
            warn!("send_cmd_idmac: timeout rintsts={rintsts_during_irq:?} idsts={idsts_during_irq:?}");
            warn!("send_cmd_idmac: DMA transfer appears stalled, check IDMAC/SDMMC interrupt enable and descriptor status");
        } else {
            info!("send_cmd_idmac: DMA IRQ received; rintsts={rintsts_during_irq:?}, idsts={idsts_during_irq:?}");
        }

        // Wait for the command to be sent and the response to be received, checking for errors.
        if cmd.response_expect() {
            debug!("send_cmd_idmac: waiting for command response");
            let response_deadline = axhal::time::wall_time() + Duration::from_secs(2);
            let mut response_wait_count = 0u64;
            let response_wait_log_interval = 1_000_000u64;
            let mut response_timeout = false;
            
            while !self.has_response() {
                core::hint::spin_loop();
                response_wait_count += 1;
                
                if response_wait_count % response_wait_log_interval == 0 {
                    let rintsts = self.regs.rintsts().read();
                    warn!("send_cmd_idmac: waiting for response after {} iterations; rintsts={:?}", response_wait_count, rintsts);
                }
                
                if axhal::time::wall_time() >= response_deadline {
                    response_timeout = true;
                    warn!("send_cmd_idmac: response timeout after {} iterations, cmd {}", response_wait_count, cmd.cmd_index());
                    break;
                }
            }
            
            if response_timeout {
                let rintsts = self.regs.rintsts().read();
                warn!("send_cmd_idmac: command response timeout for cmd {}; rintsts={:?}", cmd.cmd_index(), rintsts);
                return None;  // Return early with error
            }
            
            debug!("send_cmd_idmac: command response received after {} iterations", response_wait_count);
        }

        let mut last_status = None;
        // If data transfer is expected, wait for the transfer to complete, checking for errors.
        // Add timeout to prevent infinite loops
        let data_deadline = axhal::time::wall_time() + Duration::from_secs(5);
        let mut data_wait_count = 0u64;
        let mut data_timeout = false;
        
        // if cmd.data_expected() {
            while data_wait_count < u64::MAX {
                if axhal::time::wall_time() >= data_deadline {
                    data_timeout = true;
                    warn!("send_cmd_idmac: data_transfer_over timeout after {} iterations for cmd {}", data_wait_count, cmd.cmd_index());
                    break;
                }
                
                let rintsts = self.regs.rintsts().read();
                let idsts = self.regs.idsts().read();
                let idmac_new_error =
                    (!idsts_before.fbe() && idsts.fbe())
                        || (!idsts_before.du() && idsts.du())
                        || (!idsts_before.ces() && idsts.ces());

                last_status = Some((rintsts, idsts, idmac_new_error));

                if IDMAC_DONE_FLAG.load(Ordering::Acquire) {
                    debug!("send_cmd_idmac: DMA completion flag set while waiting for data; exiting data wait loop");
                }
                if rintsts.data_transfer_over() {
                    debug!("send_cmd_idmac: RINTSTS indicates data_transfer_over; IDSTS={:?}", idsts);
                }
                if idsts.ri() || idsts.nis() {
                    debug!("send_cmd_idmac: IDSTS interrupt bits set while waiting: ri={}, nis={}", idsts.ri(), idsts.nis());
                }

                // Quit loop when:
                // 1. Data transfer is over, which is the normal completion condition we want to wait for.
                // 2. An error occurs.
                // 3. A new IDMAC error is detected.
                if IDMAC_DONE_FLAG.load(Ordering::Acquire) || rintsts.data_transfer_over() || rintsts.error() || idmac_new_error {
                    break;
                }
                
                data_wait_count += 1;
                core::hint::spin_loop();
            }
        // }

        // Read response and check for errors after sending the command and setting up DMA.
        let resp = self.regs.resp().read();

        let (rintsts, idsts, idmac_new_error) = last_status.unwrap_or_else(|| {
            let rintsts = self.regs.rintsts().read();
            let idsts = self.regs.idsts().read();
            (rintsts, idsts, false)
        });
        
        debug!("send_cmd_idmac final wait result: rintsts={:?}, idsts={:?}, idmac_new_error={}",
            rintsts,
            idsts,
            idmac_new_error,
        );
        
        // Check for timeout condition
        if data_timeout {
            warn!("send_cmd_idmac: data transfer timeout for cmd {}", cmd.cmd_index());
            // Don't clear RINTSTS yet for diagnostic purposes
            warn!("send_cmd_idmac: final state - rintsts={:?}, idsts={:?}", rintsts, idsts);
            return None;
        }
        
        // clear interrupt status
        self.regs.rintsts().write(rintsts);

        // Deallocate the DMA descriptor we allocated for this transfer.
        unsafe { dealloc_coherent(dma_desc_info, layout); }

        if rintsts.error() || idmac_new_error {
            trace!(
                "cmd {} error: rintsts={rintsts:?} idsts={idsts:?} resp={resp:?}",
                cmd.cmd_index()
            );
            warn!("send_cmd_idmac: transfer failed for cmd {}", cmd.cmd_index());
            return None;
        }

        info!("send_cmd_idmac: transfer complete for cmd {}; resp={:?}", cmd.cmd_index(), resp);
        Some(resp)
    }

    /// The interrupt handler for the IDMAC DMA transfer completion.
    pub fn dma_irq_handler() {
        let previous_flag = IDMAC_DONE_FLAG.load(Ordering::Acquire);
        debug!("SdMmc::dma_irq_handler entered; previous IDMAC_DONE_FLAG={}", previous_flag);

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

            if has_idsts {
                debug!("SdMmc::dma_irq_handler: clearing IDSTS in interrupt handler: {:?}", idsts);
                regs.idsts().write(idsts);
            }

            if rintsts.data_transfer_over() || rintsts.receive_fifo_data_request() || rintsts.transmit_fifo_data_request() {
                let mut clear_rintsts = crate::regs::RIntSts::new();
                clear_rintsts = clear_rintsts
                    .with_data_transfer_over(rintsts.data_transfer_over())
                    .with_receive_fifo_data_request(rintsts.receive_fifo_data_request())
                    .with_transmit_fifo_data_request(rintsts.transmit_fifo_data_request());
                debug!("SdMmc::dma_irq_handler: clearing DTO/RXDR/TXDR bits in RINTSTS: {:?}", clear_rintsts);
                regs.rintsts().write(clear_rintsts);
            }

            if has_idsts || has_rintsts {
                should_notify = true;
            }

            if !has_rintsts && !has_idsts {
                warn!("SdMmc::dma_irq_handler: IRQ entered with no RINTSTS/IDSTS bits set");
                warn!("SdMmc::dma_irq_handler: stray IRQ? RINTSTS={:?} IDSTS={:?}", rintsts, idsts);
            }
        } else {
            warn!("SdMmc::dma_irq_handler: no SDMMC register base available to clear IDSTS");
        }

        if should_notify {
            IDMAC_DONE_FLAG.store(true, Ordering::Release);
            let after_flag = IDMAC_DONE_FLAG.load(Ordering::Acquire);
            debug!("SdMmc::dma_irq_handler: IDMAC_DONE_FLAG updated to {}", after_flag);
            IDMAC_WAIT_QUEUE.notify_one(true);
            debug!("SdMmc::dma_irq_handler: notified wait queue");
        }
    }

    /// The size of a block in bytes.
    pub const BLOCK_SIZE: usize = 512;

    // TODO: DMA buffer verification.
}

impl Drop for SdMmc {
    fn drop(&mut self) {
        if let Some(dma_buf) = &self.dma_buffer {
            info!("Deallocating DMA buffer: virt=0x{:08x}, phys=0x{:08x}, size={}", 
                dma_buf.addr.cpu_addr.as_ptr() as u64, dma_buf.addr.bus_addr.as_u64(), dma_buf.size);
            let layout = Layout::from_size_align(
                dma_buf.size,
                self.ahb_data_width.align_value()
            ).expect("Invalid layout for DMA buffer"); 
            unsafe { dealloc_coherent(dma_buf.addr, layout); }
        }
    }
}

unsafe impl Send for SdMmc {}
unsafe impl Sync for SdMmc {}
