// Define interfaces for DMA operations using IDMAC（Internal DMA Controller）

// According to Synopsys's DesignWare Cores Mobile Storage Host Controller Databook, 
// the IDMAC of DWC_MSHC supports both Double-Buffer and Chained Descriptor modes.
// Here we use the Chained Descriptor mode, which allows for more flexible
// and efficient DMA transfers by linking multiple descriptors together.
//
// The IDMAC of DWC_MSHC supports both 32-bit and 64-bit addressing modes, but in
// VisionFive2, the DWC_MSHC is configured to operate in 32-bit addressing mode,
// so the descriptors and buffer addresses are 32-bit values.

use bitfield_struct::bitfield;
use log::trace;

pub use axdma::{alloc_coherent, dealloc_coherent, DMAInfo};

// DMA buffer information, including both the CPU virtual address and the physical address for DMA.
pub struct DMABuffer {
    pub addr: DMAInfo,
    pub size: usize,
}

/// IDMAC Descriptor of DWC_MSHC of 32-bit address mode.
//  The descriptors must be 4-byte aligned and usually contains 4 of 32-bit words.
#[repr(C, align(4))]
pub struct IdmacDescriptor {
    /// Control Descriptors
    /// Contains control information for the DMA transfer, such as ownership and segment flags.
    pub des0: IdmacDes0,

    /// Buffer Size
    /// Specifies the size of the data buffer for the DMA transfer.
    pub des1: IdmacDes1,

    /// Buffer 1 Physical Address
    /// These bits indicate the physical address of the first data buffer. The IDMAC
    /// ignores DES2 [2/1/0:0], corresponding to the bus width of 64/32/16, internally.
    pub des2: u32,

    /// Next Descriptor Physical Address / Buffer 2 Physical Address
    /// These bits indicate the physical address of the second buffer when the
    /// dual-buffer structure is used. If the Second Address Chained (DES0[4])
    /// bit is set, then this address contains the pointer to the physical memory
    /// where the Next Descriptor is present.
    /// 
    /// If this is not the last descriptor, then the Next Descriptor address pointer
    /// must be bus-width aligned (DES3[2/1/0:0] = 0 corresponding to buswidth of 64/32/16,
    /// internally the LSBs are ignored).
    pub des3: u32,
}

/// IDMAC Descriptor Flags
#[bitfield(u32, order = Msb)]
pub struct IdmacDes0 {
    /// OWN bit (bit 31)
    /// When set, this bit indicates that the descriptor is owned by the IDMAC.
    /// When this bit is reset, it indicates that the descriptor is owned by the Host.
    /// The IDMAC clears this bit when it completes the data transfer.
    pub own: bool,
    
    /// Card Error Summary (CES) bit (bit 30)
    /// These error bits indicate the status of the transaction to or from the card.
    /// These bits are also present in RINTSTS
    /// Indicates the logical OR of the following bits:
    ///     • EBE: End Bit Error
    ///     • RTO: Response Time out
    ///     • RCRC: Response CRC
    ///     • SBE: Start Bit Error
    ///     • DRTO: Data Read Timeout
    ///     • DCRC: Data CRC for Receive
    ///     • RE: Response Error
    pub ces: bool,
    
    /// Reserved bits (bits 29-6)
    #[bits(24)]
    pub _reserved1: u32,
    
    /// End of Ring (ER) bit (bit 5)
    /// When set, this bit indicates that the descriptor list reached its final
    /// descriptor. The IDMAC returns to the base address of the list, creating a
    /// Descriptor Ring. This is meaningful for only a dual-buffer descriptor
    /// structure.
    pub er: bool,

    /// Second Address Chained(CH) bit (bit 4)
    /// When set, this bit indicates that the second address in the descriptor is the
    /// Next Descriptor address rather than the second buffer address. When this
    /// bit is set, BS2 (DES1[25:13]) should be all zeros.
    pub ch: bool,

    /// First Descriptor (FS) bit (bit 3)
    /// When set, this bit indicates that this descriptor contains the first buffer of
    /// the data. If the size of the first buffer is 0, next Descriptor contains the
    /// beginning of the data.
    pub fs: bool,

    /// Last Descriptor (LD) bit (bit 2)
    /// When set, this bit indicates that the buffers pointed to by this descriptor are
    /// the last buffers of the data.
    pub ld: bool,

    /// Disable Interrupt on Completion (DIC) bit (bit 1)
    /// When set, this bit will prevent the setting of the TI/RI bit of the IDMAC
    /// Status Register (IDSTS) for the data that ends in the buffer pointed to by
    /// this descriptor.
    pub dic: bool,

    // Reserved bit (bit 0)
    #[bits(1)]
    pub _reserved0: u8,
}

#[bitfield(u32, order = Msb)]
pub struct IdmacDes1 {
    /// Reserved bits (bits 31-13)
    #[bits(19)]
    pub _reserved: u32,

    /// Not defined in chained descriptor mode. Included in reserved bits. 
    /// Buffer 2 Size (BS2) (bits 25-13)
    /// This field is not valid if DES0[4] is set, and all its bits should be set to 0.
    /// These bits indicate the second data buffer byte size. The buffer size must
    /// be a multiple of 2, 4, or 8, depending upon the bus widths—16, 32, and 64,
    /// respectively. In the case where the buffer size is not a multiple of 2, 4, or 8,
    /// the resulting behavior is undefined.

    /// Buffer 1 Size (bits 12-0)
    /// Indicates the data buffer byte size, which must be a multiple of 2, 4, or 8
    /// bytes, depending upon the bus widths—16, 32, and 64, respectively. In the
    /// case where the buffer size is not a multiple of 2, 4, or 8, the resulting
    /// behavior is undefined. If this field is 0, the DMA ignores this buffer and
    /// proceeds to the next descriptor in case of a chain structure, or to the next
    /// buffer in case of a dual-buffer structure.
    /// Note: If there is only one descriptor and only one buffer to be programmed,
    /// you need to use only the Buffer 1 and not Buffer 2.
    #[bits(13)]
    pub bs1: u16,
}

impl IdmacDescriptor {
    pub fn new() -> Self {
        trace!("Creating a new IDMAC Descriptor with default values");
        Self {
            des0: IdmacDes0::default(),
            des1: IdmacDes1::default(),
            des2: 0,
            des3: 0,
        }
    }

    /// Sets the control bits for the DMA transfer in des0.
    pub fn set_desc0_control_descriptor(&mut self, own: bool, ces: bool, er: bool, ch: bool, fs: bool, ld: bool, dic: bool) {
        trace!(
            "Setting control descriptor:\nown={}, ces={}, er={}, ch={}, fs={}, ld={}, dic={}",
            own, ces, er, ch, fs, ld, dic
        );

        self.des0 = IdmacDes0::new().with_own(own).with_ces(ces).with_er(er).with_ch(ch).with_fs(fs).with_ld(ld).with_dic(dic);
    }

    /// Sets the size of the data buffer for the DMA transfer in des1.
    pub fn set_des1_buffer1_size(&mut self, size: u16) {
        trace!("Setting buffer1 size: {}", size);

        // size.le()
        self.des1.set_bs1(size);
    }

    /// Sets the address of the first data buffer for the DMA transfer in des2.
    pub fn set_des2_buffer1_address(&mut self, addr: u32) {
        trace!("Setting buffer1 address: 0x{:08x}", addr);

        self.des2 = addr;
    }

    pub fn set_des3_next_descriptor_address(&mut self, addr: u32) {
        trace!("Setting next descriptor address: 0x{:08x}", addr);

        self.des3 = addr;
    }
}

// TODO: support descriptor ring to allow multi-block transfers without CPU intervention.