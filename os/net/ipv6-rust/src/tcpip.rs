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

    // Debug logging helpers (defined in tcpip-rust.c)
    fn rust_debug_log(msg: *const u8);
    fn rust_debug_log_int(msg: *const u8, val: i32);

    // Buffer sync (sync Rust UIP_LEN to C uip_len)
    fn sync_rust_len_to_c();

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
    unsafe {
        rust_debug_log(b"[RUST] tcpip_rust_packet_input() entered\n\0".as_ptr());
    }

    let len = uipbuf::get_len();

    unsafe {
        rust_debug_log_int(b"[RUST] uipbuf::get_len() returned\0".as_ptr(), len as i32);
    }

    if len == 0 {
        unsafe {
            rust_debug_log(b"[RUST] Length is 0, returning early\n\0".as_ptr());
        }
        log_info!("[TCPIP] Packet input: empty buffer (len=0)");
        return 0;
    }

    unsafe {
        rust_debug_log(b"[RUST] Length is non-zero, processing packet\n\0".as_ptr());
    }
    log_info!("[TCPIP] Packet input: processing packet");

    // Get packet buffer
    unsafe {
        rust_debug_log(b"[RUST] Getting buffer slice\n\0".as_ptr());
    }
    let buf = &mut uipbuf::get_buffer()[..len as usize];

    // Process with Rust IPv6 stack
    unsafe {
        rust_debug_log(b"[RUST] Calling ipv6::process_input()\n\0".as_ptr());
    }
    match ipv6::process_input(buf) {
        Ok(_) => {
            unsafe {
                rust_debug_log(b"[RUST] ipv6::process_input() returned Ok\n\0".as_ptr());
            }
            log_info!("[TCPIP] Packet processed successfully");

            // The buffer may have been modified in-place (e.g., Echo Reply)
            // Preserve the length for output
            unsafe {
                rust_debug_log_int(b"[RUST] Buffer length after processing:\0".as_ptr(), len as i32);
            }
            uipbuf::set_len(len as u16);

            // If there's output to send, handle it
            if uipbuf::get_len() > 0 {
                unsafe {
                    rust_debug_log(b"[RUST] Output available, calling ipv6_output\n\0".as_ptr());
                }
                log_info!("[TCPIP] Output available, calling ipv6_output");
                tcpip_rust_ipv6_output();
            } else {
                unsafe {
                    rust_debug_log(b"[RUST] No output to send\n\0".as_ptr());
                }
                log_info!("[TCPIP] No output to send");
            }
            0
        }
        Err(e) => {
            unsafe {
                rust_debug_log(b"[RUST] ipv6::process_input() returned Err\n\0".as_ptr());
            }
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
    unsafe {
        rust_debug_log(b"[OUTPUT] tcpip_rust_ipv6_output() entered\n\0".as_ptr());
    }

    let len = uipbuf::get_len();

    unsafe {
        rust_debug_log_int(b"[OUTPUT] uipbuf::get_len() returned:\0".as_ptr(), len as i32);
    }

    log_info!("[OUTPUT] IPv6 output called");

    unsafe {
        rust_debug_log(b"[OUTPUT] Checking len==0\n\0".as_ptr());
    }

    if len == 0 {
        unsafe {
            rust_debug_log(b"[OUTPUT] ERROR: len=0, returning early\n\0".as_ptr());
        }
        log_warn!("[OUTPUT] Empty buffer, nothing to send");
        return 0;
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Checking len > MTU\n\0".as_ptr());
    }

    if len > UIP_LINK_MTU {
        log_err!("[OUTPUT] Packet too big for link MTU");
        uipbuf::clear();
        return -1;
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Checking len < 40\n\0".as_ptr());
    }

    // Get destination address from packet
    if len < 40 {
        log_err!("[OUTPUT] Packet too small (< 40 bytes)");
        uipbuf::clear();
        return -1;
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Getting buffer and parsing IPv6 header\n\0".as_ptr());
    }

    let buffer = uipbuf::get_buffer();
    let ipv6_hdr = unsafe { &*(buffer.as_ptr() as *const Ipv6Header) };
    let dest_addr = &ipv6_hdr.destipaddr;

    unsafe {
        rust_debug_log(b"[OUTPUT] Parsed destination address\n\0".as_ptr());
        rust_debug_log_int(b"[OUTPUT] dest_addr byte 0:\0".as_ptr(), dest_addr.u8[0] as i32);
    }

    // Check if destination is unspecified
    if dest_addr.is_unspecified() {
        unsafe {
            rust_debug_log(b"[OUTPUT] ERROR: dest is unspecified\n\0".as_ptr());
        }
        log_err!("[OUTPUT] Destination address is unspecified");
        uipbuf::clear();
        return -1;
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Destination address is valid\n\0".as_ptr());
    }
    log_info!("[OUTPUT] Destination address valid");

    // Handle multicast
    if dest_addr.is_multicast() {
        unsafe {
            rust_debug_log(b"[OUTPUT] Multicast destination\n\0".as_ptr());
        }
        log_info!("[OUTPUT] Multicast destination, sending directly");
        return tcpip_rust_output(ptr::null());
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Checking if dest is my addr\n\0".as_ptr());
    }

    // Check if sending to ourselves (loopback)
    unsafe {
        if tcpip_rust_is_my_addr(dest_addr) != 0 {
            rust_debug_log(b"[OUTPUT] Loopback detected\n\0".as_ptr());
            log_info!("[OUTPUT] Loopback: sending to self");
            return tcpip_rust_packet_input();
        }
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Not loopback, looking up next hop\n\0".as_ptr());
    }

    // Get next hop
    log_info!("[OUTPUT] Looking up next hop for destination");
    let mut nexthop = Ipv6Addr::unspecified();

    unsafe {
        rust_debug_log(b"[OUTPUT] Calling tcpip_rust_get_nexthop()\n\0".as_ptr());
    }

    let nexthop_result = unsafe { tcpip_rust_get_nexthop(dest_addr, &mut nexthop) };

    unsafe {
        rust_debug_log_int(b"[OUTPUT] tcpip_rust_get_nexthop() returned:\0".as_ptr(), nexthop_result);
    }

    if nexthop_result < 0 {
        unsafe {
            rust_debug_log(b"[OUTPUT] ERROR: No route to destination\n\0".as_ptr());
        }
        log_warn!("[OUTPUT] No route to destination");
        uipbuf::clear();
        return -1;
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Next hop found, looking up neighbor\n\0".as_ptr());
    }
    log_info!("[OUTPUT] Next hop found, looking up neighbor");

    // Look up neighbor for link-layer address
    let mut lladdr = [0u8; 8];

    unsafe {
        rust_debug_log(b"[OUTPUT] Calling nd6::lookup_neighbor()\n\0".as_ptr());
    }

    let nbr_result = nd6::lookup_neighbor(&nexthop, &mut lladdr);

    unsafe {
        rust_debug_log_int(b"[OUTPUT] nd6::lookup_neighbor() returned:\0".as_ptr(), nbr_result);
    }

    if nbr_result < 0 {
        unsafe {
            rust_debug_log(b"[OUTPUT] Neighbor not in cache, using autofill\n\0".as_ptr());
        }
        log_info!("[OUTPUT] Neighbor not in cache - using autofill");

        // Autofill: Derive link-layer address from IPv6 address
        // This is not standard-compliant but convenient for native/tun6
        // For 8-byte lladdr: copy last 8 bytes of IPv6 and flip universal/local bit
        lladdr[0] = nexthop.u8[8] ^ 0x02;
        lladdr[1] = nexthop.u8[9];
        lladdr[2] = nexthop.u8[10];
        lladdr[3] = nexthop.u8[11];
        lladdr[4] = nexthop.u8[12];
        lladdr[5] = nexthop.u8[13];
        lladdr[6] = nexthop.u8[14];
        lladdr[7] = nexthop.u8[15];

        unsafe {
            rust_debug_log(b"[OUTPUT] Autofilled link-layer address from IPv6 IID\n\0".as_ptr());
            rust_debug_log_int(b"[OUTPUT] lladdr[0]:\0".as_ptr(), lladdr[0] as i32);
            rust_debug_log_int(b"[OUTPUT] lladdr[7]:\0".as_ptr(), lladdr[7] as i32);
        }

        // Add to neighbor cache so we don't need to autofill every time
        let link_addr = LinkAddr {
            addr: lladdr,
            len: 8,
        };

        unsafe {
            rust_debug_log(b"[OUTPUT] Adding autofilled neighbor to cache\n\0".as_ptr());
        }

        match nd6::nbr_add(&nexthop, &link_addr, false, nd6::NbrState::Reachable) {
            Ok(_) => {
                unsafe {
                    rust_debug_log(b"[OUTPUT] Neighbor added to cache successfully\n\0".as_ptr());
                }
            }
            Err(_) => {
                unsafe {
                    rust_debug_log(b"[OUTPUT] Warning: Failed to add neighbor to cache (cache full?)\n\0".as_ptr());
                }
            }
        }
    } else {
        unsafe {
            rust_debug_log(b"[OUTPUT] Neighbor found in cache\n\0".as_ptr());
        }
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Proceeding to send packet to link layer\n\0".as_ptr());
    }

    // Send packet to link layer
    log_info!("[OUTPUT] Neighbor found, sending to link layer");

    unsafe {
        rust_debug_log(b"[OUTPUT] Calling tcpip_rust_output()\n\0".as_ptr());
    }

    let result = tcpip_rust_output(lladdr.as_ptr());

    unsafe {
        rust_debug_log_int(b"[OUTPUT] tcpip_rust_output() returned:\0".as_ptr(), result);
    }

    if result >= 0 {
        unsafe {
            rust_debug_log(b"[OUTPUT] Packet sent successfully\n\0".as_ptr());
        }
        log_info!("[OUTPUT] Packet sent successfully");
    } else {
        unsafe {
            rust_debug_log(b"[OUTPUT] Link layer send failed\n\0".as_ptr());
        }
        log_warn!("[OUTPUT] Link layer send failed");
    }

    unsafe {
        rust_debug_log(b"[OUTPUT] Clearing buffer before return\n\0".as_ptr());
    }

    uipbuf::clear();

    unsafe {
        rust_debug_log_int(b"[OUTPUT] Returning result:\0".as_ptr(), result);
    }

    result
}

/// Send packet to network layer (link layer output)
#[no_mangle]
pub extern "C" fn tcpip_rust_output(lladdr: *const u8) -> i32 {
    unsafe {
        rust_debug_log(b"[OUTPUT] tcpip_rust_output() entered\n\0".as_ptr());

        // CRITICAL: Sync Rust UIP_LEN to C uip_len before calling callback!
        // The callback checks C's uip_len, not Rust's UIP_LEN
        rust_debug_log(b"[OUTPUT] Syncing Rust len to C uip_len\n\0".as_ptr());
        sync_rust_len_to_c();

        rust_debug_log(b"[OUTPUT] Calling netstack_process_ip_callback(NETSTACK_IP_OUTPUT)\n\0".as_ptr());

        let callback_result = netstack_process_ip_callback(NETSTACK_IP_OUTPUT, lladdr);

        rust_debug_log_int(b"[OUTPUT] netstack_process_ip_callback returned:\0".as_ptr(), callback_result as i32);
        rust_debug_log_int(b"[OUTPUT] NETSTACK_IP_PROCESS constant:\0".as_ptr(), NETSTACK_IP_PROCESS as i32);

        if callback_result == NETSTACK_IP_PROCESS {
            rust_debug_log(b"[OUTPUT] Callback says PROCESS, calling tcpip_rust_network_output()\n\0".as_ptr());
            tcpip_rust_network_output(lladdr)
        } else {
            rust_debug_log(b"[OUTPUT] Callback says DROP/IGNORE, clearing buffer and returning 0\n\0".as_ptr());
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
