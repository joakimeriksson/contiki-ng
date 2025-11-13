//! UDP (User Datagram Protocol) implementation for IPv6
//!
//! This module provides UDP connection management and packet processing.

use crate::types::*;
use crate::checksum;

/// Maximum number of UDP connections
const UIP_UDP_CONNS: usize = 8;

/// UDP header length (8 bytes)
const UIP_UDPH_LEN: u16 = 8;

/// IPv6 + UDP header length
const UIP_IPUDPH_LEN: usize = 48; // 40 (IPv6) + 8 (UDP)

/// Last allocated local port
static mut LASTPORT: u16 = 4096;

/// UDP connection structure
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct UdpConn {
    pub ripaddr: Ipv6Addr,      // Remote IP address
    pub lport: u16,             // Local port (network byte order)
    pub rport: u16,             // Remote port (network byte order)
    pub ttl: u8,                // Time-to-live
    pub appstate_p: *mut u8,    // Application state pointer (for C compatibility)
    pub appstate: *mut u8,      // Application state data
}

impl UdpConn {
    pub const fn new() -> Self {
        Self {
            ripaddr: Ipv6Addr::unspecified(),
            lport: 0,
            rport: 0,
            ttl: 64,
            appstate_p: core::ptr::null_mut(),
            appstate: core::ptr::null_mut(),
        }
    }

    pub fn is_used(&self) -> bool {
        self.lport != 0
    }

    pub fn matches(&self, dest_port: u16, src_port: u16, src_addr: &Ipv6Addr) -> bool {
        if self.lport == 0 {
            return false;
        }

        // Check destination port
        if dest_port != self.lport {
            return false;
        }

        // Check source port (if bound)
        if self.rport != 0 && src_port != self.rport {
            return false;
        }

        // Check source address (if bound)
        if !self.ripaddr.is_unspecified() && self.ripaddr != *src_addr {
            return false;
        }

        true
    }
}

/// UDP header structure
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct UdpHeader {
    pub srcport: u16,
    pub destport: u16,
    pub udplen: u16,
    pub udpchksum: u16,
}

/// Global UDP connections array
static mut UDP_CONNS: [UdpConn; UIP_UDP_CONNS] = [UdpConn::new(); UIP_UDP_CONNS];

/// Current UDP connection (for application callback)
static mut CURRENT_UDP_CONN: *mut UdpConn = core::ptr::null_mut();

/// Initialize UDP
pub fn init() {
    unsafe {
        UDP_CONNS = [UdpConn::new(); UIP_UDP_CONNS];
        LASTPORT = 4096;
        CURRENT_UDP_CONN = core::ptr::null_mut();
    }
}

/// Find an unused local port
fn find_unused_port() -> u16 {
    unsafe {
        loop {
            LASTPORT += 1;
            if LASTPORT >= 32000 {
                LASTPORT = 4096;
            }

            let mut port_in_use = false;
            for conn in &UDP_CONNS {
                if conn.lport == LASTPORT.to_be() {
                    port_in_use = true;
                    break;
                }
            }

            if !port_in_use {
                return LASTPORT;
            }
        }
    }
}

/// Create a new UDP connection
/// Returns a pointer to the connection or null on failure
pub fn udp_new(ripaddr: Option<&Ipv6Addr>, rport: u16) -> *mut UdpConn {
    unsafe {
        // Find a free connection slot
        let mut free_conn: Option<&mut UdpConn> = None;
        for conn in &mut UDP_CONNS {
            if !conn.is_used() {
                free_conn = Some(conn);
                break;
            }
        }

        match free_conn {
            Some(conn) => {
                // Allocate local port
                let lport = find_unused_port();
                conn.lport = lport.to_be();
                conn.rport = rport;

                // Set remote address
                if let Some(addr) = ripaddr {
                    conn.ripaddr = *addr;
                } else {
                    conn.ripaddr = Ipv6Addr::unspecified();
                }

                // Set default TTL (will be set from ds6_if in C wrapper)
                conn.ttl = 64;

                // Return pointer to connection
                conn as *mut UdpConn
            }
            None => core::ptr::null_mut(),
        }
    }
}

/// Bind a UDP connection to a specific local port
pub fn udp_bind(conn: *mut UdpConn, lport: u16) {
    unsafe {
        if !conn.is_null() {
            (*conn).lport = lport;
        }
    }
}

/// Get current UDP connection (for application callback)
pub fn get_current_conn() -> *mut UdpConn {
    unsafe { CURRENT_UDP_CONN }
}

/// Process incoming UDP packet
/// Buffer contains IPv6 + UDP headers + data
/// Returns 0 on success, -1 on error
pub fn process_udp_input(buffer: &mut [u8]) -> Result<i32> {
    if buffer.len() < UIP_IPUDPH_LEN {
        return Err(Error::PacketTooShort);
    }

    // Parse IPv6 header
    let ipv6_hdr = unsafe { &*(buffer.as_ptr() as *const Ipv6Header) };

    // Parse UDP header (starts after IPv6 header at offset 40)
    let udp_hdr = unsafe { &*(buffer.as_ptr().add(40) as *const UdpHeader) };

    let src_port = u16::from_be(udp_hdr.srcport);
    let dest_port = u16::from_be(udp_hdr.destport);
    let udp_len = u16::from_be(udp_hdr.udplen);

    // Validate UDP length
    if (udp_len as usize) < 8 || (udp_len as usize) > buffer.len() - 40 {
        return Err(Error::InvalidLength);
    }

    // Check for zero destination port
    if dest_port == 0 {
        return Err(Error::InvalidPort);
    }

    // Verify checksum (if non-zero)
    #[cfg(feature = "udp_checksums")]
    {
        if udp_hdr.udpchksum != 0 {
            let calculated_checksum = calculate_udp_checksum(
                &ipv6_hdr.srcipaddr,
                &ipv6_hdr.destipaddr,
                &buffer[40..40 + udp_len as usize],
            );
            if calculated_checksum != 0xffff {
                return Err(Error::ChecksumMismatch);
            }
        }
    }

    // Find matching connection
    unsafe {
        for conn in &mut UDP_CONNS {
            if conn.matches(dest_port.to_be(), src_port.to_be(), &ipv6_hdr.srcipaddr) {
                // Found matching connection
                CURRENT_UDP_CONN = conn as *mut UdpConn;

                // Signal application callback (done in C wrapper)
                // Application will process UDP payload at buffer[48..]
                return Ok(0);
            }
        }
    }

    // No matching connection found
    Err(Error::NoConnection)
}

/// Calculate UDP checksum
fn calculate_udp_checksum(src_addr: &Ipv6Addr, dest_addr: &Ipv6Addr, udp_packet: &[u8]) -> u16 {
    // UDP checksum includes IPv6 pseudo-header
    // Note: The checksum calculation should include the packet data,
    // but our pseudo-header checksum function doesn't handle this yet.
    // For now, just return the pseudo-header checksum.
    checksum::ipv6_pseudo_header_checksum(
        src_addr,
        dest_addr,
        udp_packet.len() as u32,
        17, // UDP protocol number
    )
}

/// Prepare UDP packet for sending
/// Returns the total packet length (IPv6 + UDP + payload)
pub fn prepare_udp_output(
    buffer: &mut [u8],
    conn: *const UdpConn,
    payload_len: usize,
) -> Result<usize> {
    if buffer.len() < UIP_IPUDPH_LEN + payload_len {
        return Err(Error::BufferTooSmall);
    }

    unsafe {
        if conn.is_null() {
            return Err(Error::InvalidConnection);
        }

        let conn_ref = &*conn;

        // Build IPv6 header
        let ipv6_hdr = &mut *(buffer.as_mut_ptr() as *mut Ipv6Header);
        ipv6_hdr.vtc = 0x60;
        ipv6_hdr.tcflow = 0x00;
        ipv6_hdr.flow[0] = 0;
        ipv6_hdr.flow[1] = 0;

        let payload_len_u16 = (payload_len + 8) as u16; // UDP header + payload
        ipv6_hdr.len[0] = (payload_len_u16 >> 8) as u8;
        ipv6_hdr.len[1] = (payload_len_u16 & 0xff) as u8;

        ipv6_hdr.next = 17; // UDP
        ipv6_hdr.hlim = conn_ref.ttl;

        // Source and destination addresses set by caller

        // Build UDP header
        let udp_hdr = &mut *(buffer.as_mut_ptr().add(40) as *mut UdpHeader);
        udp_hdr.srcport = conn_ref.lport;
        udp_hdr.destport = conn_ref.rport;
        udp_hdr.udplen = ((payload_len + 8) as u16).to_be();
        udp_hdr.udpchksum = 0; // Will be calculated by caller if needed

        Ok(40 + 8 + payload_len)
    }
}

// FFI exports for C code

/// Create a new UDP connection (C FFI)
#[no_mangle]
pub extern "C" fn uip_udp_new_rust(ripaddr: *const Ipv6Addr, rport: u16) -> *mut UdpConn {
    let addr = if ripaddr.is_null() {
        None
    } else {
        unsafe { Some(&*ripaddr) }
    };
    udp_new(addr, rport)
}

/// Bind UDP connection to local port (C FFI)
#[no_mangle]
pub extern "C" fn uip_udp_bind_rust(conn: *mut UdpConn, lport: u16) {
    udp_bind(conn, lport);
}

/// Get current UDP connection (C FFI)
#[no_mangle]
pub extern "C" fn uip_udp_conn_rust() -> *mut UdpConn {
    get_current_conn()
}

/// Get UDP connections array (C FFI)
#[no_mangle]
pub extern "C" fn uip_udp_conns_rust() -> *mut UdpConn {
    unsafe { UDP_CONNS.as_mut_ptr() }
}
