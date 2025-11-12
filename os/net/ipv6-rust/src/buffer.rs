//! Packet buffer management

use crate::types::*;

/// Maximum packet buffer size (aligned with UIP_LINK_MTU)
pub const MAX_BUF_SIZE: usize = 1280;

/// Packet buffer for IPv6 packets
pub struct UipBuffer<'a> {
    data: &'a mut [u8],
    len: usize,
}

impl<'a> UipBuffer<'a> {
    /// Create a new buffer from a mutable slice
    pub fn new(data: &'a mut [u8]) -> Self {
        let len = data.len();
        Self { data, len }
    }

    /// Get the current packet length
    pub fn len(&self) -> usize {
        self.len
    }

    /// Set the packet length
    pub fn set_len(&mut self, len: usize) -> Result<()> {
        if len > self.data.len() {
            return Err(Error::BufferTooSmall);
        }
        self.len = len;
        Ok(())
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Get the buffer as a slice
    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len]
    }

    /// Get the buffer as a mutable slice
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.data[..self.len]
    }

    /// Get the IPv6 header
    pub fn ipv6_header(&self) -> Result<&Ipv6Header> {
        if self.len < Ipv6Header::SIZE {
            return Err(Error::BufferTooSmall);
        }

        unsafe {
            let ptr = self.data.as_ptr() as *const Ipv6Header;
            Ok(&*ptr)
        }
    }

    /// Get the IPv6 header mutably
    pub fn ipv6_header_mut(&mut self) -> Result<&mut Ipv6Header> {
        if self.len < Ipv6Header::SIZE {
            return Err(Error::BufferTooSmall);
        }

        unsafe {
            let ptr = self.data.as_mut_ptr() as *mut Ipv6Header;
            Ok(&mut *ptr)
        }
    }

    /// Get payload after IPv6 header
    pub fn payload(&self, offset: usize) -> Result<&[u8]> {
        if offset >= self.len {
            return Err(Error::BufferTooSmall);
        }
        Ok(&self.data[offset..self.len])
    }

    /// Get mutable payload after IPv6 header
    pub fn payload_mut(&mut self, offset: usize) -> Result<&mut [u8]> {
        if offset >= self.len {
            return Err(Error::BufferTooSmall);
        }
        Ok(&mut self.data[offset..self.len])
    }
}
