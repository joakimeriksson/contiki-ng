//! Foreign Function Interface (FFI) for C interoperability

use crate::types::*;

/// C-compatible IPv6 address
#[repr(C)]
#[derive(Copy, Clone)]
pub struct uip_ip6addr_t {
    pub u8: [u8; 16],
}

impl From<Ipv6Addr> for uip_ip6addr_t {
    fn from(addr: Ipv6Addr) -> Self {
        Self { u8: addr.u8 }
    }
}

impl From<uip_ip6addr_t> for Ipv6Addr {
    fn from(addr: uip_ip6addr_t) -> Self {
        Self { u8: addr.u8 }
    }
}

/// C-compatible link-layer address
#[repr(C)]
#[derive(Copy, Clone)]
pub struct linkaddr_t {
    pub u8: [u8; 8],
}

/// Add an IPv6 address (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_addr_add(addr: *const uip_ip6addr_t, addr_type: u8) -> i32 {
    if addr.is_null() {
        return -1;
    }

    unsafe {
        let ipv6_addr = Ipv6Addr::from(*addr);
        let addr_type = match addr_type {
            0 => crate::ds6::AddrType::Autoconf,
            1 => crate::ds6::AddrType::Dhcp,
            _ => crate::ds6::AddrType::Manual,
        };

        let netif = crate::ds6::get_netif();
        match netif.addr_add(&ipv6_addr, addr_type) {
            Ok(_) => 0,
            Err(_) => -1,
        }
    }
}

/// Lookup an IPv6 address (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_addr_lookup(addr: *const uip_ip6addr_t) -> i32 {
    if addr.is_null() {
        return 0;
    }

    unsafe {
        let ipv6_addr = Ipv6Addr::from(*addr);
        let netif = crate::ds6::get_netif();

        match netif.addr_lookup(&ipv6_addr) {
            Some(_) => 1,
            None => 0,
        }
    }
}

/// Remove an IPv6 address (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_addr_rm(addr: *const uip_ip6addr_t) -> i32 {
    if addr.is_null() {
        return -1;
    }

    unsafe {
        let ipv6_addr = Ipv6Addr::from(*addr);
        let netif = crate::ds6::get_netif();

        match netif.addr_rm(&ipv6_addr) {
            Ok(_) => 0,
            Err(_) => -1,
        }
    }
}

/// Select source address for destination (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_select_src(
    dest: *const uip_ip6addr_t,
    src_out: *mut uip_ip6addr_t,
) -> i32 {
    if dest.is_null() || src_out.is_null() {
        return -1;
    }

    unsafe {
        let dest_addr = Ipv6Addr::from(*dest);

        match crate::ds6::select_src(&dest_addr) {
            Some(src_addr) => {
                *src_out = uip_ip6addr_t::from(src_addr);
                0
            }
            None => -1,
        }
    }
}

/// Add a neighbor (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_nbr_add(
    ipaddr: *const uip_ip6addr_t,
    lladdr: *const linkaddr_t,
    is_router: u8,
) -> i32 {
    if ipaddr.is_null() || lladdr.is_null() {
        return -1;
    }

    unsafe {
        let ipv6_addr = Ipv6Addr::from(*ipaddr);
        let mut link_addr = LinkAddr::new();
        link_addr.addr = (*lladdr).u8;
        link_addr.len = 8;

        match crate::nd6::nbr_add(
            &ipv6_addr,
            &link_addr,
            is_router != 0,
            crate::nd6::NbrState::Reachable,
        ) {
            Ok(_) => 0,
            Err(_) => -1,
        }
    }
}

/// Lookup a neighbor (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_nbr_lookup(
    ipaddr: *const uip_ip6addr_t,
    lladdr_out: *mut linkaddr_t,
) -> i32 {
    if ipaddr.is_null() {
        return -1;
    }

    unsafe {
        let ipv6_addr = Ipv6Addr::from(*ipaddr);

        match crate::nd6::nbr_lookup(&ipv6_addr) {
            Some(nbr) => {
                if !lladdr_out.is_null() {
                    (*lladdr_out).u8 = nbr.lladdr.addr;
                }
                0
            }
            None => -1,
        }
    }
}

/// Check if address is multicast (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_is_multicast(addr: *const uip_ip6addr_t) -> i32 {
    if addr.is_null() {
        return 0;
    }

    unsafe {
        let ipv6_addr = Ipv6Addr::from(*addr);
        if ipv6_addr.is_multicast() { 1 } else { 0 }
    }
}

/// Check if address is link-local (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_is_link_local(addr: *const uip_ip6addr_t) -> i32 {
    if addr.is_null() {
        return 0;
    }

    unsafe {
        let ipv6_addr = Ipv6Addr::from(*addr);
        if ipv6_addr.is_link_local() { 1 } else { 0 }
    }
}

/// Get the version string (C FFI)
#[no_mangle]
pub extern "C" fn uip6_rust_version() -> *const u8 {
    b"uIP6-Rust v0.1.0\0".as_ptr()
}
