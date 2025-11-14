//! ICMPv6 implementation

use crate::types::*;
use crate::checksum;
use crate::buffer::UipBuffer;

// Logging (C functions)
extern "C" {
    fn tcpip_rust_log_info(msg: *const u8);
    fn tcpip_rust_log_warn(msg: *const u8);
    fn tcpip_rust_log_err(msg: *const u8);
}

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

/// ICMPv6 message types
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Icmpv6Type {
    DestUnreach = 1,
    PacketTooBig = 2,
    TimeExceeded = 3,
    ParamProblem = 4,
    EchoRequest = 128,
    EchoReply = 129,
    RouterSol = 133,
    RouterAdv = 134,
    NeighborSol = 135,
    NeighborAdv = 136,
    Redirect = 137,
}

impl Icmpv6Type {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            1 => Some(Icmpv6Type::DestUnreach),
            2 => Some(Icmpv6Type::PacketTooBig),
            3 => Some(Icmpv6Type::TimeExceeded),
            4 => Some(Icmpv6Type::ParamProblem),
            128 => Some(Icmpv6Type::EchoRequest),
            129 => Some(Icmpv6Type::EchoReply),
            133 => Some(Icmpv6Type::RouterSol),
            134 => Some(Icmpv6Type::RouterAdv),
            135 => Some(Icmpv6Type::NeighborSol),
            136 => Some(Icmpv6Type::NeighborAdv),
            137 => Some(Icmpv6Type::Redirect),
            _ => None,
        }
    }
}

/// ICMPv6 header
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct Icmpv6Header {
    pub type_: u8,
    pub code: u8,
    pub chksum: [u8; 2],
}

impl Icmpv6Header {
    pub const SIZE: usize = 4;

    pub fn checksum(&self) -> u16 {
        u16::from_be_bytes(self.chksum)
    }

    pub fn set_checksum(&mut self, chksum: u16) {
        self.chksum = chksum.to_be_bytes();
    }
}

/// ICMPv6 Echo message
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct Icmpv6Echo {
    pub type_: u8,
    pub code: u8,
    pub chksum: [u8; 2],
    pub id: [u8; 2],
    pub seqno: [u8; 2],
}

impl Icmpv6Echo {
    pub const SIZE: usize = 8;
}

/// ICMPv6 Destination Unreachable codes
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum DestUnreachCode {
    NoRoute = 0,
    AdminProhibited = 1,
    BeyondScope = 2,
    AddrUnreachable = 3,
    PortUnreachable = 4,
    SrcAddrFailedPolicy = 5,
    RejectRoute = 6,
}

/// ICMPv6 Time Exceeded codes
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum TimeExceededCode {
    HopLimitExceeded = 0,
    FragmentReassemblyTimeExceeded = 1,
}

/// Initialize ICMPv6
pub fn init() {
    // Nothing to initialize for now
}

/// Process ICMPv6 message
pub fn process_input(
    buffer: &mut UipBuffer,
    offset: usize,
    src: &Ipv6Addr,
    dst: &Ipv6Addr,
) -> Result<()> {
    log_info!("[ICMPv6] Processing ICMPv6 message");

    let payload = buffer.payload(offset)?;

    if payload.len() < Icmpv6Header::SIZE {
        log_warn!("[ICMPv6] Packet too small for ICMPv6 header");
        return Err(Error::InvalidPacket);
    }

    let header = unsafe {
        &*(payload.as_ptr() as *const Icmpv6Header)
    };

    log_info!("[ICMPv6] Verifying checksum");

    // Verify checksum
    let payload_len = payload.len() as u32;
    if !checksum::verify_checksum(
        src,
        dst,
        payload_len,
        IpProto::Icmpv6 as u8,
        payload,
        header.checksum(),
    ) {
        log_warn!("[ICMPv6] Checksum verification FAILED");
        return Err(Error::InvalidChecksum);
    }

    log_info!("[ICMPv6] Checksum OK, dispatching message type");

    // Dispatch based on type
    match Icmpv6Type::from_u8(header.type_) {
        Some(Icmpv6Type::EchoRequest) => {
            log_info!("[ICMPv6] Type: Echo Request (ping)");
            process_echo_request(buffer, offset, src, dst)
        }
        Some(Icmpv6Type::NeighborSol) => {
            log_info!("[ICMPv6] Type: Neighbor Solicitation");
            crate::nd6::process_ns(buffer, offset, src, dst)
        }
        Some(Icmpv6Type::NeighborAdv) => {
            log_info!("[ICMPv6] Type: Neighbor Advertisement");
            crate::nd6::process_na(buffer, offset, src, dst)
        }
        Some(Icmpv6Type::RouterAdv) => {
            log_info!("[ICMPv6] Type: Router Advertisement");
            crate::nd6::process_ra(buffer, offset, src, dst)
        }
        Some(Icmpv6Type::RouterSol) => {
            log_info!("[ICMPv6] Type: Router Solicitation");
            crate::nd6::process_rs(buffer, offset, src, dst)
        }
        _ => {
            log_warn!("[ICMPv6] Unknown ICMPv6 message type");
            // Unknown or unimplemented ICMPv6 type
            Ok(())
        }
    }
}

/// Process Echo Request and generate Echo Reply
fn process_echo_request(
    buffer: &mut UipBuffer,
    offset: usize,
    _src: &Ipv6Addr,
    _dst: &Ipv6Addr,
) -> Result<()> {
    log_info!("[ICMPv6] Processing Echo Request");

    // First, get and swap addresses
    let (new_src, new_dst) = {
        let ip_header = buffer.ipv6_header_mut()?;
        let orig_src = ip_header.srcipaddr;
        let orig_dst = ip_header.destipaddr;
        ip_header.srcipaddr = orig_dst;
        ip_header.destipaddr = orig_src;
        (orig_dst, orig_src)
    };

    log_info!("[ICMPv6] Swapped source and destination addresses");

    // Now modify the payload
    let payload = buffer.payload_mut(offset)?;

    if payload.len() < Icmpv6Echo::SIZE {
        log_warn!("[ICMPv6] Echo packet too small");
        return Err(Error::InvalidPacket);
    }

    // Change type to Echo Reply
    payload[0] = Icmpv6Type::EchoReply as u8;

    log_info!("[ICMPv6] Changed type to Echo Reply");

    // Recalculate checksum
    let new_checksum = checksum::calculate_checksum(
        &new_src,
        &new_dst,
        payload.len() as u32,
        IpProto::Icmpv6 as u8,
        payload,
    );

    payload[2] = (new_checksum >> 8) as u8;
    payload[3] = (new_checksum & 0xff) as u8;

    log_info!("[ICMPv6] Echo Reply ready to send");

    Ok(())
}

/// Send ICMPv6 error message
pub fn send_error(
    _buffer: &mut UipBuffer,
    _type_: Icmpv6Type,
    _code: u8,
    _param: u32,
) -> Result<()> {
    // Implementation would construct and send error message
    // For now, just a placeholder
    Ok(())
}

/// Send ICMPv6 Destination Unreachable
pub fn send_dest_unreach(buffer: &mut UipBuffer, code: DestUnreachCode) -> Result<()> {
    send_error(buffer, Icmpv6Type::DestUnreach, code as u8, 0)
}

/// Send ICMPv6 Time Exceeded
pub fn send_time_exceeded(buffer: &mut UipBuffer, code: TimeExceededCode) -> Result<()> {
    send_error(buffer, Icmpv6Type::TimeExceeded, code as u8, 0)
}

/// Send ICMPv6 Packet Too Big
pub fn send_packet_too_big(buffer: &mut UipBuffer, mtu: u32) -> Result<()> {
    send_error(buffer, Icmpv6Type::PacketTooBig, 0, mtu)
}
