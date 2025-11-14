// TCP/IP stack core - Rust implementation
// Handles input/output/routing for the IPv6 stack

use crate::ipv6;
use crate::types::*;
use crate::ds6;
use crate::nd6;
use crate::uipbuf;
use core::ptr;

// External C functions we need to call
extern "C" {
    // Buffer management
    fn uipbuf_clear();
    fn uipbuf_get_attr(attr_type: u8) -> u16;
    fn uipbuf_set_attr(attr_type: u8, value: u16);

    // Network stack
    fn netstack_process_ip_callback(event: u8, data: *const u8) -> u8;

    // Logging
    fn tcpip_rust_log_info(msg: *const u8);
    fn tcpip_rust_log_warn(msg: *const u8);
    fn tcpip_rust_log_err(msg: *const u8);

    // Link layer output
    fn tcpip_rust_network_output(lladdr: *const u8) -> i32;

    // Routing
    fn tcpip_rust_get_nexthop(dest: *const Ipv6Addr, out: *mut Ipv6Addr) -> i32;
    fn tcpip_rust_is_addr_onlink(addr: *const Ipv6Addr) -> i32;
    fn tcpip_rust_is_my_addr(addr: *const Ipv6Addr) -> i32;
}

// Logging macros
macro_rules! log_info {
    ($msg:expr) => {
        unsafe { tcpip_rust_log_info(concat!($msg, "\0").as_ptr()) }
    };
}

macro_rules! log_warn {
    ($msg:expr) => {
        unsafe { tcpip_rust_log_warn(concat!($msg, "\0").as_ptr()) }
    };
}

macro_rules! log_err {
    ($msg:expr) => {
        unsafe { tcpip_rust_log_err(concat!($msg, "\0").as_ptr()) }
    };
}

// Constants
const NETSTACK_IP_INPUT: u8 = 0;
const NETSTACK_IP_OUTPUT: u8 = 1;
const NETSTACK_IP_PROCESS: u8 = 1;

const UIP_LINK_MTU: u16 = 1280;
const UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS: u8 = 1;

// Event types (must match C enums)
const TCP_POLL: u8 = 0;
const UDP_POLL: u8 = 1;
const PACKET_INPUT: u8 = 2;

/// Initialize the TCP/IP stack
#[no_mangle]
pub extern "C" fn tcpip_rust_init() {
    // Initialize IPv6 subsystems
    ds6::init();
    nd6::init();

    unsafe {
        tcpip_rust_log_info(b"TCP/IP: Rust stack initialized\n\0".as_ptr());
    }
}

/// Process an incoming packet
/// Called when PACKET_INPUT event is received
#[no_mangle]
pub extern "C" fn tcpip_rust_packet_input() -> i32 {
    let len = uipbuf::get_len();

    if len == 0 {
        log_info!("[TCPIP] Packet input: empty buffer (len=0)");
        return 0;
    }

    log_info!("[TCPIP] Packet input: processing packet");

    // Get packet buffer
    let buf = &mut uipbuf::get_buffer()[..len as usize];

    // Process with Rust IPv6 stack
    match ipv6::process_input(buf) {
        Ok(_) => {
            log_info!("[TCPIP] Packet processed successfully");
            // If there's output to send, handle it
            if uipbuf::get_len() > 0 {
                log_info!("[TCPIP] Output available, calling ipv6_output");
                tcpip_rust_ipv6_output();
            } else {
                log_info!("[TCPIP] No output to send");
            }
            0
        }
        Err(_) => {
            log_warn!("[TCPIP] Packet processing FAILED");
            uipbuf::clear();
            -1
        }
    }
}

/// Called by lower layers to signal packet arrival
#[no_mangle]
pub extern "C" fn tcpip_rust_input() {
    unsafe {
        // Check if network stack wants to process this
        if netstack_process_ip_callback(NETSTACK_IP_INPUT, ptr::null()) == NETSTACK_IP_PROCESS {
            // In real implementation, would post PACKET_INPUT event to process
            // For now, process directly
            tcpip_rust_packet_input();
        }
    }
    uipbuf::clear();
}

/// Output an IPv6 packet with routing
#[no_mangle]
pub extern "C" fn tcpip_rust_ipv6_output() -> i32 {
    let len = uipbuf::get_len();

    log_info!("[OUTPUT] IPv6 output called");

    if len == 0 {
        log_warn!("[OUTPUT] Empty buffer, nothing to send");
        return 0;
    }

    if len > UIP_LINK_MTU {
        log_err!("[OUTPUT] Packet too big for link MTU");
        uipbuf::clear();
        return -1;
    }

    // Get destination address from packet
    if len < 40 {
        log_err!("[OUTPUT] Packet too small (< 40 bytes)");
        uipbuf::clear();
        return -1;
    }

    let buffer = uipbuf::get_buffer();
    let ipv6_hdr = unsafe { &*(buffer.as_ptr() as *const Ipv6Header) };
    let dest_addr = &ipv6_hdr.destipaddr;

    // Check if destination is unspecified
    if dest_addr.is_unspecified() {
        log_err!("[OUTPUT] Destination address is unspecified");
        uipbuf::clear();
        return -1;
    }

    log_info!("[OUTPUT] Destination address valid");

    // Handle multicast
    if dest_addr.is_multicast() {
        log_info!("[OUTPUT] Multicast destination, sending directly");
        return tcpip_rust_output(ptr::null());
    }

    // Check if sending to ourselves (loopback)
    unsafe {
        if tcpip_rust_is_my_addr(dest_addr) != 0 {
            log_info!("[OUTPUT] Loopback: sending to self");
            return tcpip_rust_packet_input();
        }
    }

    // Get next hop
    log_info!("[OUTPUT] Looking up next hop for destination");
    let mut nexthop = Ipv6Addr::unspecified();
    let nexthop_result = unsafe { tcpip_rust_get_nexthop(dest_addr, &mut nexthop) };

    if nexthop_result < 0 {
        log_warn!("[OUTPUT] No route to destination");
        uipbuf::clear();
        return -1;
    }

    log_info!("[OUTPUT] Next hop found, looking up neighbor");

    // Look up neighbor for link-layer address
    let mut lladdr = [0u8; 8];
    let nbr_result = nd6::lookup_neighbor(&nexthop, &mut lladdr);

    if nbr_result < 0 {
        log_info!("[OUTPUT] Neighbor not in cache - need NS");
        // In real implementation, would trigger NS and queue packet
        // For now, just drop
        uipbuf::clear();
        return -1;
    }

    // Send packet to link layer
    log_info!("[OUTPUT] Neighbor found, sending to link layer");
    let result = tcpip_rust_output(lladdr.as_ptr());

    if result >= 0 {
        log_info!("[OUTPUT] Packet sent successfully");
    } else {
        log_warn!("[OUTPUT] Link layer send failed");
    }

    uipbuf::clear();
    result
}

/// Send packet to network layer (link layer output)
#[no_mangle]
pub extern "C" fn tcpip_rust_output(lladdr: *const u8) -> i32 {
    unsafe {
        if netstack_process_ip_callback(NETSTACK_IP_OUTPUT, lladdr) == NETSTACK_IP_PROCESS {
            tcpip_rust_network_output(lladdr)
        } else {
            // Ignore and drop
            uipbuf::clear();
            0
        }
    }
}

/// Event handler - dispatches events to appropriate handlers
#[no_mangle]
pub extern "C" fn tcpip_rust_eventhandler(ev: u8, _data: *mut u8) {
    match ev {
        PACKET_INPUT => {
            tcpip_rust_packet_input();
        }
        TCP_POLL => {
            unsafe {
                tcpip_rust_log_info(b"TCP/IP: TCP poll event\n\0".as_ptr());
            }
            // TODO: Handle TCP polling when TCP is ported
        }
        UDP_POLL => {
            unsafe {
                tcpip_rust_log_info(b"TCP/IP: UDP poll event\n\0".as_ptr());
            }
            // TODO: Handle UDP polling when UDP is ported
        }
        _ => {
            // Unknown event, ignore
        }
    }
}
