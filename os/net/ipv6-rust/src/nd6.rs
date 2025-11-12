//! IPv6 Neighbor Discovery (ND6) - RFC 4861

use crate::types::*;
use crate::buffer::UipBuffer;
use crate::ds6::{self, AddrState};

/// Maximum number of neighbors
const MAX_NEIGHBORS: usize = 8;

/// Neighbor Cache Entry states (RFC 4861)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum NbrState {
    Unused,
    Incomplete,
    Reachable,
    Stale,
    Delay,
    Probe,
}

/// Neighbor cache entry
#[derive(Copy, Clone)]
pub struct Neighbor {
    pub ipaddr: Ipv6Addr,
    pub lladdr: LinkAddr,
    pub state: NbrState,
    pub is_router: bool,
    // Timers managed by C for now
}

impl Neighbor {
    pub const fn new() -> Self {
        Self {
            ipaddr: Ipv6Addr::unspecified(),
            lladdr: LinkAddr::new(),
            state: NbrState::Unused,
            is_router: false,
        }
    }

    pub fn is_used(&self) -> bool {
        !matches!(self.state, NbrState::Unused)
    }
}

/// Neighbor Discovery options
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Nd6Option {
    SourceLinkLayerAddr = 1,
    TargetLinkLayerAddr = 2,
    PrefixInfo = 3,
    RedirectedHeader = 4,
    Mtu = 5,
}

/// ND6 Option header
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct Nd6OptionHeader {
    pub type_: u8,
    pub len: u8,  // Length in units of 8 bytes
}

/// Router Solicitation message
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct RouterSol {
    pub type_: u8,
    pub code: u8,
    pub chksum: [u8; 2],
    pub reserved: [u8; 4],
}

/// Router Advertisement message
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct RouterAdv {
    pub type_: u8,
    pub code: u8,
    pub chksum: [u8; 2],
    pub cur_hop_limit: u8,
    pub flags: u8,
    pub router_lifetime: [u8; 2],
    pub reachable_time: [u8; 4],
    pub retrans_timer: [u8; 4],
}

/// Neighbor Solicitation message
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct NeighborSol {
    pub type_: u8,
    pub code: u8,
    pub chksum: [u8; 2],
    pub reserved: [u8; 4],
    pub target: Ipv6Addr,
}

/// Neighbor Advertisement message
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct NeighborAdv {
    pub type_: u8,
    pub code: u8,
    pub chksum: [u8; 2],
    pub flags: u8,
    pub reserved: [u8; 3],
    pub target: Ipv6Addr,
}

/// Neighbor Advertisement flags
pub const NA_FLAG_ROUTER: u8 = 0x80;
pub const NA_FLAG_SOLICITED: u8 = 0x40;
pub const NA_FLAG_OVERRIDE: u8 = 0x20;

/// Global neighbor cache
static mut NEIGHBOR_CACHE: [Neighbor; MAX_NEIGHBORS] = [Neighbor::new(); MAX_NEIGHBORS];

/// Initialize ND6
pub fn init() {
    unsafe {
        NEIGHBOR_CACHE = [Neighbor::new(); MAX_NEIGHBORS];
    }
}

/// Lookup neighbor by IPv6 address
pub fn nbr_lookup(addr: &Ipv6Addr) -> Option<&'static mut Neighbor> {
    unsafe {
        NEIGHBOR_CACHE
            .iter_mut()
            .find(|nbr| nbr.is_used() && nbr.ipaddr == *addr)
    }
}

/// Add or update neighbor
pub fn nbr_add(
    addr: &Ipv6Addr,
    lladdr: &LinkAddr,
    is_router: bool,
    state: NbrState,
) -> Result<&'static mut Neighbor> {
    unsafe {
        // Check if already exists
        if let Some(nbr) = nbr_lookup(addr) {
            nbr.lladdr = *lladdr;
            nbr.state = state;
            nbr.is_router = is_router;
            return Ok(nbr);
        }

        // Find free slot
        for nbr in &mut NEIGHBOR_CACHE {
            if !nbr.is_used() {
                nbr.ipaddr = *addr;
                nbr.lladdr = *lladdr;
                nbr.state = state;
                nbr.is_router = is_router;
                return Ok(nbr);
            }
        }

        Err(Error::BufferTooSmall)
    }
}

/// Remove neighbor
pub fn nbr_rm(addr: &Ipv6Addr) -> Result<()> {
    unsafe {
        for nbr in &mut NEIGHBOR_CACHE {
            if nbr.is_used() && nbr.ipaddr == *addr {
                nbr.state = NbrState::Unused;
                return Ok(());
            }
        }
    }
    Err(Error::NeighborNotFound)
}

/// Process Neighbor Solicitation
pub fn process_ns(
    buffer: &mut UipBuffer,
    offset: usize,
    src: &Ipv6Addr,
    _dst: &Ipv6Addr,
) -> Result<()> {
    let payload = buffer.payload(offset)?;

    if payload.len() < core::mem::size_of::<NeighborSol>() {
        return Err(Error::InvalidPacket);
    }

    let ns = unsafe {
        &*(payload.as_ptr() as *const NeighborSol)
    };

    let target = &ns.target;

    // Check if we have this address
    let netif = ds6::get_netif();
    if let Some(addr_entry) = netif.addr_lookup(target) {
        if addr_entry.state == AddrState::Tentative {
            // Duplicate Address Detection failed
            // Should mark address as duplicate
            return Ok(());
        }

        // Send Neighbor Advertisement
        send_na(buffer, target, src, true, true)?;
    }

    Ok(())
}

/// Process Neighbor Advertisement
pub fn process_na(
    buffer: &mut UipBuffer,
    offset: usize,
    _src: &Ipv6Addr,
    _dst: &Ipv6Addr,
) -> Result<()> {
    let payload = buffer.payload(offset)?;

    if payload.len() < core::mem::size_of::<NeighborAdv>() {
        return Err(Error::InvalidPacket);
    }

    let na = unsafe {
        &*(payload.as_ptr() as *const NeighborAdv)
    };

    let target = &na.target;
    let is_router = (na.flags & NA_FLAG_ROUTER) != 0;
    let solicited = (na.flags & NA_FLAG_SOLICITED) != 0;
    let _override_flag = (na.flags & NA_FLAG_OVERRIDE) != 0;

    // Parse options to get link-layer address
    let opt_offset = core::mem::size_of::<NeighborAdv>();
    if let Some(lladdr) = parse_target_lladdr(&payload[opt_offset..]) {
        // Update neighbor cache
        let state = if solicited {
            NbrState::Reachable
        } else {
            NbrState::Stale
        };

        let _ = nbr_add(target, &lladdr, is_router, state);
    }

    Ok(())
}

/// Process Router Solicitation
pub fn process_rs(
    _buffer: &mut UipBuffer,
    _offset: usize,
    _src: &Ipv6Addr,
    _dst: &Ipv6Addr,
) -> Result<()> {
    // Routers would send RA in response
    // For now, just acknowledge receipt
    Ok(())
}

/// Process Router Advertisement
pub fn process_ra(
    buffer: &mut UipBuffer,
    offset: usize,
    _src: &Ipv6Addr,
    _dst: &Ipv6Addr,
) -> Result<()> {
    let payload = buffer.payload(offset)?;

    if payload.len() < core::mem::size_of::<RouterAdv>() {
        return Err(Error::InvalidPacket);
    }

    let ra = unsafe {
        &*(payload.as_ptr() as *const RouterAdv)
    };

    // Update default router list and process options
    // This is simplified - full implementation would:
    // 1. Update default router list
    // 2. Process prefix information options
    // 3. Update MTU if present
    // 4. Update reachable time and retrans timer

    let netif = ds6::get_netif();
    netif.cur_hop_limit = ra.cur_hop_limit;

    Ok(())
}

/// Send Neighbor Advertisement
fn send_na(
    _buffer: &mut UipBuffer,
    _target: &Ipv6Addr,
    _dest: &Ipv6Addr,
    _solicited: bool,
    _override_flag: bool,
) -> Result<()> {
    // Implementation would construct and send NA
    // This is a placeholder
    Ok(())
}

/// Send Neighbor Solicitation
pub fn send_ns(_target: &Ipv6Addr) -> Result<()> {
    // Implementation would construct and send NS
    // This is a placeholder
    Ok(())
}

/// Parse Target Link-Layer Address option
fn parse_target_lladdr(options: &[u8]) -> Option<LinkAddr> {
    let mut offset = 0;

    while offset + 2 <= options.len() {
        let opt_type = options[offset];
        let opt_len = options[offset + 1] as usize * 8;  // Length in units of 8 bytes

        if opt_len == 0 || offset + opt_len > options.len() {
            break;
        }

        if opt_type == Nd6Option::TargetLinkLayerAddr as u8 {
            // Extract link-layer address
            let mut lladdr = LinkAddr::new();
            let addr_len = core::cmp::min(opt_len - 2, 8);
            lladdr.addr[..addr_len].copy_from_slice(&options[offset + 2..offset + 2 + addr_len]);
            lladdr.len = addr_len as u8;
            return Some(lladdr);
        }

        offset += opt_len;
    }

    None
}
