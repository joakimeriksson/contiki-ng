//! IPv6 Data Structures (DS6)
//!
//! Manages IPv6 addresses, prefixes, neighbors, and routes

use crate::types::*;

// Debug logging helpers
extern "C" {
    fn rust_debug_log(msg: *const u8);
    fn rust_debug_log_int(msg: *const u8, val: i32);
}

/// Maximum number of IPv6 addresses per interface
const MAX_ADDRS: usize = 4;

/// Maximum number of prefixes
const MAX_PREFIXES: usize = 2;

/// Maximum number of multicast addresses
const MAX_MADDRS: usize = 4;

/// IPv6 address state
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AddrState {
    Unused,
    Tentative,
    Preferred,
    Deprecated,
}

/// IPv6 address type
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AddrType {
    Autoconf,
    Dhcp,
    Manual,
}

/// IPv6 address entry
#[derive(Copy, Clone)]
pub struct Ds6Addr {
    pub ipaddr: Ipv6Addr,
    pub state: AddrState,
    pub addr_type: AddrType,
    pub is_infinite: bool,
    // Timers would be managed by C code for now
}

impl Ds6Addr {
    pub const fn new() -> Self {
        Self {
            ipaddr: Ipv6Addr::unspecified(),
            state: AddrState::Unused,
            addr_type: AddrType::Manual,
            is_infinite: false,
        }
    }

    pub fn is_used(&self) -> bool {
        !matches!(self.state, AddrState::Unused)
    }
}

/// IPv6 prefix entry
#[derive(Copy, Clone)]
pub struct Ds6Prefix {
    pub ipaddr: Ipv6Addr,
    pub length: u8,
    pub is_used: bool,
}

impl Ds6Prefix {
    pub const fn new() -> Self {
        Self {
            ipaddr: Ipv6Addr::unspecified(),
            length: 0,
            is_used: false,
        }
    }
}

/// IPv6 multicast address entry
#[derive(Copy, Clone)]
pub struct Ds6Maddr {
    pub ipaddr: Ipv6Addr,
    pub is_used: bool,
}

impl Ds6Maddr {
    pub const fn new() -> Self {
        Self {
            ipaddr: Ipv6Addr::unspecified(),
            is_used: false,
        }
    }
}

/// IPv6 interface
pub struct Ds6Netif {
    pub link_mtu: u32,
    pub cur_hop_limit: u8,
    pub base_reachable_time: u32,
    pub reachable_time: u32,
    pub retrans_timer: u32,
    pub addrs: [Ds6Addr; MAX_ADDRS],
    pub prefixes: [Ds6Prefix; MAX_PREFIXES],
    pub maddrs: [Ds6Maddr; MAX_MADDRS],
}

impl Ds6Netif {
    pub const fn new() -> Self {
        Self {
            link_mtu: 1280,
            cur_hop_limit: 64,
            base_reachable_time: 30000,
            reachable_time: 30000,
            retrans_timer: 1000,
            addrs: [Ds6Addr::new(); MAX_ADDRS],
            prefixes: [Ds6Prefix::new(); MAX_PREFIXES],
            maddrs: [Ds6Maddr::new(); MAX_MADDRS],
        }
    }

    /// Add an IPv6 address
    pub fn addr_add(&mut self, addr: &Ipv6Addr, addr_type: AddrType) -> Result<&mut Ds6Addr> {
        // Find free slot
        for entry in &mut self.addrs {
            if !entry.is_used() {
                entry.ipaddr = *addr;
                entry.state = AddrState::Tentative;
                entry.addr_type = addr_type;
                entry.is_infinite = true;
                return Ok(entry);
            }
        }
        Err(Error::BufferTooSmall)
    }

    /// Lookup an IPv6 address
    pub fn addr_lookup(&self, addr: &Ipv6Addr) -> Option<&Ds6Addr> {
        unsafe {
            rust_debug_log(b"[addr_lookup] Looking up address\n\0".as_ptr());
        }

        for (i, entry) in self.addrs.iter().enumerate() {
            if entry.is_used() {
                unsafe {
                    rust_debug_log_int(b"[addr_lookup] Checking addr slot:\0".as_ptr(), i as i32);
                    rust_debug_log_int(b"[addr_lookup]   Configured addr byte 0:\0".as_ptr(), entry.ipaddr.u8[0] as i32);
                    rust_debug_log_int(b"[addr_lookup]   Configured addr byte 1:\0".as_ptr(), entry.ipaddr.u8[1] as i32);
                    rust_debug_log_int(b"[addr_lookup]   Configured addr byte 14:\0".as_ptr(), entry.ipaddr.u8[14] as i32);
                    rust_debug_log_int(b"[addr_lookup]   Configured addr byte 15:\0".as_ptr(), entry.ipaddr.u8[15] as i32);
                }
                if entry.ipaddr == *addr {
                    unsafe {
                        rust_debug_log(b"[addr_lookup] MATCH FOUND!\n\0".as_ptr());
                    }
                    return Some(entry);
                } else {
                    unsafe {
                        rust_debug_log(b"[addr_lookup]   No match, continuing\n\0".as_ptr());
                    }
                }
            }
        }

        unsafe {
            rust_debug_log(b"[addr_lookup] No match found in any slot\n\0".as_ptr());
        }
        None
    }

    /// Remove an IPv6 address
    pub fn addr_rm(&mut self, addr: &Ipv6Addr) -> Result<()> {
        for entry in &mut self.addrs {
            if entry.is_used() && entry.ipaddr == *addr {
                entry.state = AddrState::Unused;
                return Ok(());
            }
        }
        Err(Error::InvalidAddress)
    }

    /// Add a prefix
    pub fn prefix_add(&mut self, prefix: &Ipv6Addr, len: u8) -> Result<&mut Ds6Prefix> {
        for entry in &mut self.prefixes {
            if !entry.is_used {
                entry.ipaddr = *prefix;
                entry.length = len;
                entry.is_used = true;
                return Ok(entry);
            }
        }
        Err(Error::BufferTooSmall)
    }

    /// Lookup a prefix
    pub fn prefix_lookup(&self, prefix: &Ipv6Addr, len: u8) -> Option<&Ds6Prefix> {
        self.prefixes
            .iter()
            .find(|entry| entry.is_used && entry.ipaddr == *prefix && entry.length == len)
    }

    /// Add a multicast address
    pub fn maddr_add(&mut self, addr: &Ipv6Addr) -> Result<&mut Ds6Maddr> {
        if !addr.is_multicast() {
            return Err(Error::InvalidAddress);
        }

        for entry in &mut self.maddrs {
            if !entry.is_used {
                entry.ipaddr = *addr;
                entry.is_used = true;
                return Ok(entry);
            }
        }
        Err(Error::BufferTooSmall)
    }
}

/// Global DS6 interface (static allocation for embedded)
static mut DS6_IF: Ds6Netif = Ds6Netif::new();

/// Initialize DS6
pub fn init() {
    unsafe {
        DS6_IF = Ds6Netif::new();

        // Add link-local all-nodes multicast address (FF02::1)
        let all_nodes = Ipv6Addr::new([
            0xff, 0x02, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0x01
        ]);
        let ds6_if = &mut *core::ptr::addr_of_mut!(DS6_IF);
        let _ = ds6_if.maddr_add(&all_nodes);
    }
}

/// Get the global DS6 interface
pub fn get_netif() -> &'static mut Ds6Netif {
    unsafe { &mut *core::ptr::addr_of_mut!(DS6_IF) }
}

/// Select source address for a given destination
pub fn select_src(dest: &Ipv6Addr) -> Option<Ipv6Addr> {
    let netif = get_netif();

    // Prefer matching scope
    for addr in &netif.addrs {
        if !addr.is_used() || addr.state != AddrState::Preferred {
            continue;
        }

        // Link-local destination -> link-local source
        if dest.is_link_local() && addr.ipaddr.is_link_local() {
            return Some(addr.ipaddr);
        }

        // Global destination -> global source
        if dest.is_global() && addr.ipaddr.is_global() {
            return Some(addr.ipaddr);
        }
    }

    // Fallback: return first preferred address
    netif.addrs
        .iter()
        .find(|addr| addr.is_used() && addr.state == AddrState::Preferred)
        .map(|addr| addr.ipaddr)
}
