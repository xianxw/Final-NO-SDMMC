use core::fmt;

use crate::regs::Cmd;

pub enum Command<'a> {
    GoIdleState,                // CMD0
    AllSendCid,                 // CMD2
    SendRelativeAddr,           // CMD3
    SelectCard(u32),            // CMD7
    SendIfCond(u32),            // CMD8
    SendCsd(u32),               // CMD9
    ReadSingleBlock(u32, &'a mut [u8]),   // CMD17
    ReadMultipleBlocks(u32, &'a mut [u8]), // CMD18
    WriteSingleBlock(u32, &'a [u8]),      // CMD24
    WriteMultipleBlocks(u32, &'a [u8]),   // CMD25
    SdSendOpCond(u32),          // ACMD41
    SendScr(&'a mut [u8]),      // ACMD51
    AppCmd(u32),                // CMD55
    /// Psuedo-command to reset the clock
    ResetClock,                 // Not a real command
}

impl fmt::Debug for Command<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Command::GoIdleState => write!(f, "GoIdleState"),
            Command::AllSendCid => write!(f, "AllSendCid"),
            Command::SendRelativeAddr => write!(f, "SendRelativeAddr"),
            Command::SelectCard(arg) => write!(f, "SelectCard({arg})"),
            Command::SendIfCond(arg) => write!(f, "SendIfCond({arg})"),
            Command::SendCsd(rca) => write!(f, "SendCsd({rca})"),
            Command::ReadSingleBlock(block, _) => write!(f, "ReadSingleBlock({block})"),
            Command::ReadMultipleBlocks(block, _) => {
                write!(f, "ReadMultipleBlocks({block})")
            }
            Command::WriteSingleBlock(block, _) => write!(f, "WriteSingleBlock({block})"),
            Command::WriteMultipleBlocks(block, _) => {
                write!(f, "WriteMultipleBlocks({block})")
            }
            Command::SdSendOpCond(arg) => write!(f, "SdSendOpCond({arg})"),
            Command::SendScr(_) => write!(f, "SendScr"),
            Command::AppCmd(arg) => write!(f, "AppCmd({arg})"),
            Command::ResetClock => write!(f, "ResetClock"),
        }
    }
}

pub enum DataXfer<'a> {
    Read(&'a mut [u8]),
    Write(&'a [u8]),
}

impl<'a> Command<'a> {
    fn cmd_index(&self) -> u8 {
        match self {
            Command::GoIdleState => 0,
            Command::AllSendCid => 2,
            Command::SendRelativeAddr => 3,
            Command::SelectCard(_) => 7,
            Command::SendIfCond(_) => 8,
            Command::SendCsd(_) => 9,
            Command::ReadSingleBlock(..) => 17,
            Command::ReadMultipleBlocks(..) => 18,
            Command::WriteSingleBlock(..) => 24,
            Command::WriteMultipleBlocks(..) => 25,
            Command::SdSendOpCond(_) => 41,
            Command::SendScr(_) => 51,
            Command::AppCmd(_) => 55,

            Command::ResetClock => 0, // Special case, not a real command
        }
    }

    pub(crate) fn build(self) -> (Cmd, u32, Option<DataXfer<'a>>) {
        let cmd = Cmd::default()
            .with_use_hold_reg(true)
            .with_cmd_index(self.cmd_index());
        let cmd_resp = cmd.with_response_expect(true);
        let cmd_crc = cmd_resp.with_check_response_crc(true);

        match self {
            Command::GoIdleState => (cmd.with_send_initialization(true), 0, None),
            Command::SendRelativeAddr => (cmd_crc, 0, None),
            Command::SelectCard(arg) => (cmd_crc, arg, None),
            Command::SendIfCond(arg) | Command::AppCmd(arg) => (cmd_crc, arg, None),

            Command::AllSendCid => (cmd_crc.with_response_length(true), 0, None),
            Command::SendCsd(arg) => (cmd_crc.with_response_length(true), arg, None),
            Command::SdSendOpCond(arg) => (cmd_resp, arg, None),

            Command::ReadSingleBlock(block, buf) => (
                cmd_crc.with_data_expected(true),
                block,
                Some(DataXfer::Read(buf)),
            ),
            Command::ReadMultipleBlocks(block, buf) => (
                cmd_crc.with_data_expected(true).with_send_auto_stop(true),
                block,
                Some(DataXfer::Read(buf)),
            ),
            Command::SendScr(buf) => (
                cmd_crc.with_data_expected(true),
                0,
                Some(DataXfer::Read(buf)),
            ),
            Command::WriteSingleBlock(block, buf) => (
                cmd_crc.with_data_expected(true).with_read_write(true),
                block,
                Some(DataXfer::Write(buf)),
            ),
            Command::WriteMultipleBlocks(block, buf) => (
                cmd_crc
                    .with_data_expected(true)
                    .with_read_write(true)
                    .with_send_auto_stop(true),
                block,
                Some(DataXfer::Write(buf)),
            ),

            Command::ResetClock => (
                Cmd::default()
                    .with_update_clock_registers_only(true)
                    .with_response_expect(false),  // Critical: no response expected for clock update only
                0,
                None,
            ),
        }
    }
}
