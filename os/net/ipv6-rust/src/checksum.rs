//! Checksum calculation for IPv6 packets

use crate::types::*;

/// Calculate IPv6 pseudo-header checksum for upper-layer protocols
/// This follows RFC 2460 section 8.1
pub fn ipv6_pseudo_header_checksum(
    src: &Ipv6Addr,
    dst: &Ipv6Addr,
    payload_len: u32,
    next_header: u8,
) -> u16 {
    let mut sum: u32 = 0;

    // Source address (16 bytes = 8 x 16-bit words)
    for i in (0..16).step_by(2) {
        sum += u16::from_be_bytes([src.u8[i], src.u8[i + 1]]) as u32;
    }

    // Destination address (16 bytes = 8 x 16-bit words)
    for i in (0..16).step_by(2) {
        sum += u16::from_be_bytes([dst.u8[i], dst.u8[i + 1]]) as u32;
    }

    // Upper-layer packet length (32 bits = 2 x 16-bit words)
    sum += (payload_len >> 16) as u32;
    sum += (payload_len & 0xFFFF) as u32;

    // Next header (8 bits, padded to 16 bits)
    sum += next_header as u32;

    // Fold 32-bit sum to 16 bits
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    !sum as u16
}

/// Calculate checksum for a data buffer
pub fn data_checksum(data: &[u8]) -> u16 {
    let mut sum: u32 = 0;

    // Process 16-bit words
    let mut i = 0;
    while i < data.len() - 1 {
        sum += u16::from_be_bytes([data[i], data[i + 1]]) as u32;
        i += 2;
    }

    // Handle odd byte
    if i < data.len() {
        sum += (data[i] as u32) << 8;
    }

    // Fold to 16 bits
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    !sum as u16
}

/// Calculate combined checksum (pseudo-header + data)
pub fn calculate_checksum(
    src: &Ipv6Addr,
    dst: &Ipv6Addr,
    payload_len: u32,
    next_header: u8,
    data: &[u8],
) -> u16 {
    let mut sum: u32 = 0;

    // Add pseudo-header checksum
    let pseudo_sum = ipv6_pseudo_header_checksum(src, dst, payload_len, next_header);
    sum += (!pseudo_sum) as u32;  // Invert back to add to sum

    // Add data checksum
    let data_sum = data_checksum(data);
    sum += (!data_sum) as u32;  // Invert back to add to sum

    // Fold to 16 bits
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    !sum as u16
}

// Logging (C functions)
extern "C" {
    fn rust_debug_log(msg: *const u8);
    fn rust_debug_log_int(msg: *const u8, val: i32);
}

/// Verify checksum by calculating over entire packet (including checksum field)
/// Result should be 0xFFFF if checksum is correct
pub fn verify_checksum(
    src: &Ipv6Addr,
    dst: &Ipv6Addr,
    payload_len: u32,
    next_header: u8,
    data: &[u8],
    _expected: u16,
) -> bool {
    unsafe {
        rust_debug_log(b"[CHECKSUM] verify_checksum() called\n\0".as_ptr());
        rust_debug_log_int(b"[CHECKSUM] payload_len:\0".as_ptr(), payload_len as i32);
        rust_debug_log_int(b"[CHECKSUM] next_header:\0".as_ptr(), next_header as i32);
        rust_debug_log_int(b"[CHECKSUM] data.len():\0".as_ptr(), data.len() as i32);
    }

    // Calculate checksum over pseudo-header + data (including checksum field)
    // For a valid checksum, the result should be 0xFFFF
    let mut sum: u32 = 0;

    // Add pseudo-header: source address
    for i in (0..16).step_by(2) {
        sum += u16::from_be_bytes([src.u8[i], src.u8[i + 1]]) as u32;
    }

    // Add pseudo-header: destination address
    for i in (0..16).step_by(2) {
        sum += u16::from_be_bytes([dst.u8[i], dst.u8[i + 1]]) as u32;
    }

    // Add pseudo-header: upper-layer packet length
    sum += (payload_len >> 16) as u32;
    sum += (payload_len & 0xFFFF) as u32;

    // Add pseudo-header: next header
    sum += next_header as u32;

    // Add data (including checksum field)
    let mut i = 0;
    while i < data.len() - 1 {
        sum += u16::from_be_bytes([data[i], data[i + 1]]) as u32;
        i += 2;
    }

    // Handle odd byte
    if i < data.len() {
        sum += (data[i] as u32) << 8;
    }

    // Fold 32-bit sum to 16 bits
    while sum >> 16 != 0 {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    let result = !sum as u16;

    unsafe {
        rust_debug_log_int(b"[CHECKSUM] calculated result:\0".as_ptr(), result as i32);
        rust_debug_log_int(b"[CHECKSUM] valid (should be 0xFFFF):\0".as_ptr(), if result == 0xFFFF { 1 } else { 0 });
    }

    result == 0xFFFF
}
