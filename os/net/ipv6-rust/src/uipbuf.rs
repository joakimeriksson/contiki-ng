//! Buffer management for uIP
//!
//! This module provides the global packet buffer and length variables
//! that are used throughout the uIP stack.

/// Buffer size - IPv6 minimum MTU
/// This should match UIP_BUFSIZE from C configuration
pub const UIP_BUFSIZE: usize = 1280;

/// Buffer structure (aligned to 32-bit boundary)
#[repr(C, align(4))]
pub struct UipBufT {
    pub u8: [u8; UIP_BUFSIZE],
}

impl UipBufT {
    pub const fn new() -> Self {
        Self {
            u8: [0; UIP_BUFSIZE],
        }
    }
}

/// Global aligned packet buffer
/// This is the main buffer used for incoming and outgoing packets
static mut UIP_ALIGNED_BUF: UipBufT = UipBufT::new();

/// Global packet length variable
/// Indicates the current length of data in uip_buf
static mut UIP_LEN: u16 = 0;

/// Initialize buffer management
pub fn init() {
    unsafe {
        UIP_ALIGNED_BUF = UipBufT::new();
        UIP_LEN = 0;
    }
}

/// Get mutable reference to the packet buffer
pub fn get_buffer() -> &'static mut [u8] {
    unsafe { &mut UIP_ALIGNED_BUF.u8 }
}

/// Get the current packet length
pub fn get_len() -> u16 {
    unsafe { UIP_LEN }
}

/// Set the packet length
pub fn set_len(len: u16) {
    unsafe {
        UIP_LEN = len;
    }
}

/// Clear the buffer (set length to 0)
pub fn clear() {
    unsafe {
        UIP_LEN = 0;
    }
}

// FFI exports for C code

/// Get pointer to uip_buf (for C code)
#[no_mangle]
pub extern "C" fn uip_buf_ptr() -> *mut u8 {
    unsafe { UIP_ALIGNED_BUF.u8.as_mut_ptr() }
}

/// Get pointer to uip_len (for C code)
#[no_mangle]
pub extern "C" fn uip_len_ptr() -> *mut u16 {
    unsafe { &mut UIP_LEN as *mut u16 }
}

/// Access uip_aligned_buf from C
#[no_mangle]
pub extern "C" fn uip_aligned_buf_ptr() -> *mut UipBufT {
    unsafe { &mut UIP_ALIGNED_BUF as *mut UipBufT }
}

/// Set uip_len from C
#[no_mangle]
pub extern "C" fn uip_set_len(len: u16) {
    set_len(len);
}

/// Get uip_len from C
#[no_mangle]
pub extern "C" fn uip_get_len() -> u16 {
    get_len()
}
