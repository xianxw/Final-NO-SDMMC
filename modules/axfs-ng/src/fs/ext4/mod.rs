mod fs;
mod inode;
mod util;

#[allow(unused_imports)]
use axdriver::{AxBlockDevice, prelude::BlockDriverOps};
pub use fs::*;
pub use inode::*;
use lwext4_rust::{
    BlockDevice, Ext4Error, Ext4Result,
    ffi::{EIO, EROFS},
};

pub(crate) struct Ext4Disk {
    dev: AxBlockDevice,
    read_only: bool,
}

impl Ext4Disk {
    fn new(dev: AxBlockDevice, read_only: bool) -> Self {
        Self { dev, read_only }
    }
}

impl BlockDevice for Ext4Disk {
    fn read_blocks(&mut self, block_id: u64, buf: &mut [u8]) -> Ext4Result<usize> {
        self.dev
            .read_block(block_id, buf)
            .map_err(|_| Ext4Error::new(EIO as _, None))?;
        Ok(buf.len())
    }

    fn write_blocks(&mut self, block_id: u64, buf: &[u8]) -> Ext4Result<usize> {
        if self.read_only {
            warn!("Blocked ext4 write to block {block_id} on read-only root filesystem");
            return Err(Ext4Error::new(EROFS as _, None));
        }

        self.dev
            .write_block(block_id, buf)
            .map_err(|_| Ext4Error::new(EIO as _, None))?;
        Ok(buf.len())
    }

    fn num_blocks(&self) -> Ext4Result<u64> {
        Ok(self.dev.num_blocks())
    }
}
