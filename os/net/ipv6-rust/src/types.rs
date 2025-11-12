//! Core IPv6 data types

/// IPv6 address (128 bits)
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Ipv6Addr {
    pub u8: [u8; 16],
}

impl Ipv6Addr {
    pub const fn new(bytes: [u8; 16]) -> Self {
        Self { u8: bytes }
    }

    pub const fn unspecified() -> Self {
        Self { u8: [0; 16] }
    }

    pub const fn loopback() -> Self {
        Self { u8: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1] }
    }

    pub fn is_unspecified(&self) -> bool {
        self.u8.iter().all(|&b| b == 0)
    }

    pub fn is_loopback(&self) -> bool {
        &self.u8[..15] == &[0u8; 15] && self.u8[15] == 1
    }

    pub fn is_multicast(&self) -> bool {
        self.u8[0] == 0xff
    }

    pub fn is_link_local(&self) -> bool {
        self.u8[0] == 0xfe && (self.u8[1] & 0xc0) == 0x80
    }

    pub fn is_global(&self) -> bool {
        (self.u8[0] & 0xe0) == 0x20
    }

    /// Check if address matches a prefix
    pub fn matches_prefix(&self, prefix: &Ipv6Addr, prefix_len: u8) -> bool {
        if prefix_len > 128 {
            return false;
        }

        let full_bytes = (prefix_len / 8) as usize;
        let remaining_bits = prefix_len % 8;

        // Check full bytes
        if self.u8[..full_bytes] != prefix.u8[..full_bytes] {
            return false;
        }

        // Check remaining bits if any
        if remaining_bits > 0 {
            let mask = !((1u8 << (8 - remaining_bits)) - 1);
            if (self.u8[full_bytes] & mask) != (prefix.u8[full_bytes] & mask) {
                return false;
            }
        }

        true
    }
}

/// IPv6 header (40 bytes)
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct Ipv6Header {
    pub vtc: u8,           // Version (4 bits) + Traffic Class (4 bits)
    pub tcflow: u8,        // Traffic Class (4 bits) + Flow Label (4 bits)
    pub flow: [u8; 2],     // Flow Label (16 bits)
    pub len: [u8; 2],      // Payload Length (16 bits)
    pub next: u8,          // Next Header
    pub hlim: u8,          // Hop Limit
    pub srcipaddr: Ipv6Addr,
    pub destipaddr: Ipv6Addr,
}

impl Ipv6Header {
    pub const SIZE: usize = 40;

    pub fn version(&self) -> u8 {
        self.vtc >> 4
    }

    pub fn payload_len(&self) -> u16 {
        u16::from_be_bytes([self.len[0], self.len[1]])
    }

    pub fn set_payload_len(&mut self, len: u16) {
        let bytes = len.to_be_bytes();
        self.len[0] = bytes[0];
        self.len[1] = bytes[1];
    }
}

/// IPv6 Protocol Numbers
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum IpProto {
    HopByHop = 0,
    Icmpv6 = 58,
    Tcp = 6,
    Udp = 17,
    Ipv6Route = 43,
    Ipv6Frag = 44,
    Ipv6NoNxt = 59,
    Ipv6Opts = 60,
}

impl IpProto {
    pub fn from_u8(val: u8) -> Option<Self> {
        match val {
            0 => Some(IpProto::HopByHop),
            58 => Some(IpProto::Icmpv6),
            6 => Some(IpProto::Tcp),
            17 => Some(IpProto::Udp),
            43 => Some(IpProto::Ipv6Route),
            44 => Some(IpProto::Ipv6Frag),
            59 => Some(IpProto::Ipv6NoNxt),
            60 => Some(IpProto::Ipv6Opts),
            _ => None,
        }
    }
}

/// Extension Header
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct ExtHeader {
    pub next: u8,
    pub len: u8,
}

/// Fragment Header
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct FragHeader {
    pub next: u8,
    pub reserved: u8,
    pub offset_res_m: [u8; 2],  // Offset (13 bits) + Reserved (2 bits) + M flag (1 bit)
    pub id: [u8; 4],            // Fragment ID
}

impl FragHeader {
    pub fn offset(&self) -> u16 {
        u16::from_be_bytes(self.offset_res_m) >> 3
    }

    pub fn more_fragments(&self) -> bool {
        (self.offset_res_m[1] & 0x01) != 0
    }

    pub fn id(&self) -> u32 {
        u32::from_be_bytes(self.id)
    }
}

/// UDP header
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct UdpHeader {
    pub srcport: [u8; 2],
    pub destport: [u8; 2],
    pub len: [u8; 2],
    pub chksum: [u8; 2],
}

impl UdpHeader {
    pub const SIZE: usize = 8;

    pub fn src_port(&self) -> u16 {
        u16::from_be_bytes(self.srcport)
    }

    pub fn dest_port(&self) -> u16 {
        u16::from_be_bytes(self.destport)
    }

    pub fn length(&self) -> u16 {
        u16::from_be_bytes(self.len)
    }
}

/// TCP header
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct TcpHeader {
    pub srcport: [u8; 2],
    pub destport: [u8; 2],
    pub seqno: [u8; 4],
    pub ackno: [u8; 4],
    pub offset_flags: [u8; 2],
    pub window: [u8; 2],
    pub chksum: [u8; 2],
    pub urgp: [u8; 2],
}

impl TcpHeader {
    pub const MIN_SIZE: usize = 20;

    pub fn src_port(&self) -> u16 {
        u16::from_be_bytes(self.srcport)
    }

    pub fn dest_port(&self) -> u16 {
        u16::from_be_bytes(self.destport)
    }

    pub fn data_offset(&self) -> u8 {
        self.offset_flags[0] >> 4
    }
}

/// Link-layer address
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct LinkAddr {
    pub addr: [u8; 8],
    pub len: u8,
}

impl LinkAddr {
    pub const fn new() -> Self {
        Self {
            addr: [0; 8],
            len: 0,
        }
    }
}

/// Error types
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidPacket,
    BufferTooSmall,
    InvalidAddress,
    InvalidChecksum,
    UnsupportedProtocol,
    FragmentationError,
    RouteNotFound,
    NeighborNotFound,
}

pub type Result<T> = core::result::Result<T, Error>;
