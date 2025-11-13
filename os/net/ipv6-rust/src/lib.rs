#![no_std]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

//! Rust implementation of the uIP6 IPv6 stack for Contiki-NG
//!
//! This is a memory-safe Rust port of the uip6 IPv6 stack, providing:
//! - IPv6 packet processing
//! - ICMPv6 handling
//! - Neighbor Discovery (ND6)
//! - Address management
//! - Routing table management
//! - 6LoWPAN compression/decompression

// Core modules
pub mod types;
pub mod buffer;
pub mod ipv6;
pub mod icmpv6;
pub mod nd6;
pub mod ds6;
pub mod checksum;
pub mod ffi;
pub mod tcpip;
pub mod udp;
pub mod uipbuf;

// Re-export common types
pub use types::*;
pub use buffer::UipBuffer;

/// Initialize the Rust uIP6 stack
#[no_mangle]
pub extern "C" fn uip6_rust_init() {
    // Initialize all subsystems
    ds6::init();
    nd6::init();
    icmpv6::init();
}

/// Process an incoming IPv6 packet
/// Returns 0 on success, negative on error
#[no_mangle]
pub extern "C" fn uip6_rust_input(buf: *mut u8, len: u16) -> i32 {
    if buf.is_null() || len == 0 {
        return -1;
    }

    // Safety: We're working with raw pointers from C
    // The caller guarantees the buffer is valid
    unsafe {
        let slice = core::slice::from_raw_parts_mut(buf, len as usize);
        match ipv6::process_input(slice) {
            Ok(_) => 0,
            Err(_) => -1,
        }
    }
}

/// Panic handler for no_std environment
#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
