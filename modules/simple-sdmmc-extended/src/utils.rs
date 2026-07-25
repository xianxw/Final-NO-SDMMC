use core::fmt;

use bitfield_struct::bitfield;

/// The card state encoded in an SD R1 response.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum R1CurrentState {
    /// Card is in the idle state.
    Idle,
    /// Card is ready after initialization.
    Ready,
    /// Card is in the identification state.
    Identification,
    /// Card is in the standby state.
    Standby,
    /// Card is ready for data transfer.
    Transfer,
    /// Card is sending data.
    SendingData,
    /// Card is receiving data.
    ReceiveData,
    /// Card is programming received data.
    Programming,
    /// Card is disconnected.
    Disconnect,
    /// Reserved state value reported by the card.
    Reserved(u8),
}

/// Decoded SD card status returned by commands with an R1 or R1b response.
#[derive(Clone, Copy, Eq, PartialEq)]
pub struct R1CardStatus(u32);

impl R1CardStatus {
    /// All R1 bits that make a command unsuccessful for this driver.
    pub const ERROR_MASK: u32 = 0xfff9_8088;

    /// Creates a card status from the raw 32-bit response.
    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    /// Returns the raw 32-bit card status.
    pub const fn raw(self) -> u32 {
        self.0
    }

    /// Returns all asserted error bits.
    pub const fn error_bits(self) -> u32 {
        self.0 & Self::ERROR_MASK
    }

    /// Returns whether any R1 error bit is asserted.
    pub const fn has_error(self) -> bool {
        self.error_bits() != 0
    }

    /// Address or block range is outside the card capacity.
    pub const fn out_of_range(self) -> bool {
        self.bit(31)
    }

    /// A command address is not correctly aligned.
    pub const fn address_error(self) -> bool {
        self.bit(30)
    }

    /// The requested block length is invalid.
    pub const fn block_len_error(self) -> bool {
        self.bit(29)
    }

    /// An erase command occurred in an invalid sequence.
    pub const fn erase_seq_error(self) -> bool {
        self.bit(28)
    }

    /// An erase parameter is invalid.
    pub const fn erase_param(self) -> bool {
        self.bit(27)
    }

    /// The command attempted to write protected data.
    pub const fn wp_violation(self) -> bool {
        self.bit(26)
    }

    /// The card is locked.
    pub const fn card_is_locked(self) -> bool {
        self.bit(25)
    }

    /// A lock or unlock operation failed.
    pub const fn lock_unlock_failed(self) -> bool {
        self.bit(24)
    }

    /// The card detected a command CRC error.
    pub const fn com_crc_error(self) -> bool {
        self.bit(23)
    }

    /// The command is illegal for the card state or card type.
    pub const fn illegal_command(self) -> bool {
        self.bit(22)
    }

    /// The card's internal ECC failed.
    pub const fn card_ecc_failed(self) -> bool {
        self.bit(21)
    }

    /// An internal card controller error occurred.
    pub const fn cc_error(self) -> bool {
        self.bit(20)
    }

    /// A general card error occurred.
    pub const fn error(self) -> bool {
        self.bit(19)
    }

    /// CID or CSD overwrite protection was violated.
    pub const fn cid_csd_overwrite(self) -> bool {
        self.bit(16)
    }

    /// A write-protected erase group was skipped.
    pub const fn wp_erase_skip(self) -> bool {
        self.bit(15)
    }

    /// Card internal ECC is disabled.
    pub const fn card_ecc_disabled(self) -> bool {
        self.bit(14)
    }

    /// The card reset its erase sequence before execution.
    pub const fn erase_reset(self) -> bool {
        self.bit(13)
    }

    /// Returns the current card state.
    pub const fn current_state(self) -> R1CurrentState {
        match ((self.0 >> 9) & 0xf) as u8 {
            0 => R1CurrentState::Idle,
            1 => R1CurrentState::Ready,
            2 => R1CurrentState::Identification,
            3 => R1CurrentState::Standby,
            4 => R1CurrentState::Transfer,
            5 => R1CurrentState::SendingData,
            6 => R1CurrentState::ReceiveData,
            7 => R1CurrentState::Programming,
            8 => R1CurrentState::Disconnect,
            state => R1CurrentState::Reserved(state),
        }
    }

    /// The card can accept the next data command.
    pub const fn ready_for_data(self) -> bool {
        self.bit(8)
    }

    /// A switch command failed.
    pub const fn switch_error(self) -> bool {
        self.bit(7)
    }

    /// The card is reporting an exception event.
    pub const fn exception_event(self) -> bool {
        self.bit(6)
    }

    /// The preceding CMD55 was accepted and the next command is application-specific.
    pub const fn app_cmd(self) -> bool {
        self.bit(5)
    }

    /// An authentication sequence error occurred.
    pub const fn ake_seq_error(self) -> bool {
        self.bit(3)
    }

    const fn bit(self, bit: u32) -> bool {
        self.0 & (1 << bit) != 0
    }
}

impl fmt::Debug for R1CardStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("R1CardStatus")
            .field("raw", &format_args!("{:#010x}", self.raw()))
            .field("error_bits", &format_args!("{:#010x}", self.error_bits()))
            .field("errors", &R1ErrorNames(*self))
            .field("current_state", &self.current_state())
            .field("ready_for_data", &self.ready_for_data())
            .field("app_cmd", &self.app_cmd())
            .field("exception_event", &self.exception_event())
            .finish()
    }
}

struct R1ErrorNames(R1CardStatus);

impl fmt::Debug for R1ErrorNames {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let status = self.0;
        let mut errors = f.debug_list();
        if status.out_of_range() {
            errors.entry(&"OUT_OF_RANGE");
        }
        if status.address_error() {
            errors.entry(&"ADDRESS_ERROR");
        }
        if status.block_len_error() {
            errors.entry(&"BLOCK_LEN_ERROR");
        }
        if status.erase_seq_error() {
            errors.entry(&"ERASE_SEQ_ERROR");
        }
        if status.erase_param() {
            errors.entry(&"ERASE_PARAM");
        }
        if status.wp_violation() {
            errors.entry(&"WP_VIOLATION");
        }
        if status.card_is_locked() {
            errors.entry(&"CARD_IS_LOCKED");
        }
        if status.lock_unlock_failed() {
            errors.entry(&"LOCK_UNLOCK_FAILED");
        }
        if status.com_crc_error() {
            errors.entry(&"COM_CRC_ERROR");
        }
        if status.illegal_command() {
            errors.entry(&"ILLEGAL_COMMAND");
        }
        if status.card_ecc_failed() {
            errors.entry(&"CARD_ECC_FAILED");
        }
        if status.cc_error() {
            errors.entry(&"CC_ERROR");
        }
        if status.error() {
            errors.entry(&"ERROR");
        }
        if status.cid_csd_overwrite() {
            errors.entry(&"CID_CSD_OVERWRITE");
        }
        if status.wp_erase_skip() {
            errors.entry(&"WP_ERASE_SKIP");
        }
        if status.switch_error() {
            errors.entry(&"SWITCH_ERROR");
        }
        if status.ake_seq_error() {
            errors.entry(&"AKE_SEQ_ERROR");
        }
        errors.finish()
    }
}

/// Card Identification
///
/// Reference: https://www.cameramemoryspeed.com/sd-memory-card-faq/reading-sd-card-cid-serial-psn-internal-numbers/
#[bitfield(u128, order = Msb, debug = false)]
pub struct Cid {
    /// Manufacturer ID
    pub mid: u8,
    /// OEM/Application ID
    pub oid: u16,
    /// Product name
    #[bits(40)]
    pub pnm: u64,
    /// Product Revision
    #[bits(8)]
    pub prv: ProductRevision,
    /// Product Serial Number
    pub psn: u32,
    /// Manufacturing Date
    #[bits(16)]
    pub mdt: ManufacturingDate,
    /// CRC7 checksum
    #[bits(7)]
    pub crc: u8,
    __: bool,
}

#[bitfield(u8, order = Msb, debug = false)]
pub struct ProductRevision {
    #[bits(4)]
    pub hwrev: u8,
    #[bits(4)]
    pub fwrev: u8,
}

impl fmt::Debug for ProductRevision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.hwrev(), self.fwrev())
    }
}

#[bitfield(u16, order = Msb, debug = false)]
pub struct ManufacturingDate {
    #[bits(4)]
    __: u8,
    /// Manufacture Date Code - Year
    pub year: u8,
    /// Manufacture Date Code - Month
    #[bits(4)]
    pub month: u8,
}

impl fmt::Debug for ManufacturingDate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:02}/{}", self.month(), self.year() as u32 + 2000)
    }
}

impl fmt::Debug for Cid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Cid")
            .field("mid", &self.mid())
            .field(
                "oid",
                &str::from_utf8(&self.oid().to_be_bytes()).unwrap_or("Invalid OEM ID"),
            )
            .field(
                "pnm",
                &str::from_utf8(&self.pnm().to_be_bytes()[3..8]).unwrap_or("Invalid Product Name"),
            )
            .field("prv", &self.prv())
            .field("psn", &self.psn())
            .field("mdt", &self.mdt())
            .field("crc", &self.crc())
            .finish()
    }
}

/// Card Specific Data, version 2
#[bitfield(u128, order = Msb)]
pub struct CsdV2 {
    /// CSD structure
    #[bits(2)]
    pub csd_structure: u8,
    #[bits(6)]
    __: u8,
    /// Data read access time 1
    pub taac: u8,
    /// Data write access time 1
    pub nsac: u8,
    /// Max data transfer rate
    pub tran_speed: u8,
    /// Card command class
    #[bits(12)]
    pub ccc: u16,
    /// Max read block length
    #[bits(4)]
    pub read_bl_len: u8,
    /// Partial blocks for read allowed
    pub read_blk_partial: bool,
    /// Write block misalignment
    pub write_blk_misaligned: bool,
    /// Read block misalignment
    pub read_blk_misaligned: bool,
    /// DSR implemented
    pub dsr_imp: bool,
    #[bits(6)]
    __: u8,
    /// Device size
    #[bits(22)]
    pub c_size: u32,
    __: bool,
    /// Erase single block enabled
    pub erase_blk_en: bool,
    /// Erase sector size
    #[bits(7)]
    pub sector_size: u8,
    /// Write protect group size
    #[bits(7)]
    pub wp_grp_size: u8,
    /// Write protect group enable
    pub wp_grp_enable: bool,
    #[bits(2)]
    __: u8,
    /// Write speed factor
    #[bits(3)]
    pub r2w_factor: u8,
    /// Max write block length
    #[bits(4)]
    pub write_bl_len: u8,
    /// Partial blocks for write allowed
    pub write_blk_partial: bool,
    #[bits(5)]
    __: u8,
    /// File format group
    pub file_format_grp: bool,
    /// Copy flag
    pub copy: bool,
    /// Permanent write protection
    pub perm_write_protect: bool,
    /// Temporary write protection
    pub tmp_write_protect: bool,
    /// File format
    #[bits(2)]
    pub file_format: u8,
    #[bits(2)]
    __: u8,
    /// CRC checksum
    #[bits(7)]
    pub crc: u8,
    __: bool,
}

impl CsdV2 {
    /// Returns the number of blocks.
    pub fn num_blocks(&self) -> u64 {
        (self.c_size() as u64 + 1) * 1024
    }
}

#[cfg(test)]
mod tests {
    use super::{R1CardStatus, R1CurrentState};

    #[test]
    fn r1_decodes_state_and_non_error_status() {
        let raw = (4 << 9) | (1 << 8) | (1 << 6) | (1 << 5);
        let status = R1CardStatus::from_raw(raw);

        assert_eq!(status.current_state(), R1CurrentState::Transfer);
        assert!(status.ready_for_data());
        assert!(status.exception_event());
        assert!(status.app_cmd());
        assert!(!status.has_error());
    }

    #[test]
    fn r1_error_mask_covers_every_sd_error_bit() {
        let error_bits = [
            31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 16, 15, 7, 3,
        ];

        for bit in error_bits {
            let status = R1CardStatus::from_raw(1 << bit);
            assert!(status.has_error(), "R1 bit {bit} was not treated as an error");
            assert_eq!(status.error_bits(), 1 << bit);
        }
    }

    #[test]
    fn r1_status_bits_are_not_errors() {
        let status = R1CardStatus::from_raw((1 << 14) | (1 << 13) | (7 << 9) | (1 << 8));

        assert!(status.card_ecc_disabled());
        assert!(status.erase_reset());
        assert_eq!(status.current_state(), R1CurrentState::Programming);
        assert!(status.ready_for_data());
        assert!(!status.has_error());
    }
}
