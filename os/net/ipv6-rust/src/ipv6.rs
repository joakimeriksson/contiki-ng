//! IPv6 packet processing

use crate::types::*;
use crate::buffer::UipBuffer;
use crate::{checksum, icmpv6, ds6};

/// Minimum IPv6 MTU
pub const IPV6_MIN_MTU: usize = 1280;

/// Default hop limit
pub const DEFAULT_HOP_LIMIT: u8 = 64;

/// Process an incoming IPv6 packet
pub fn process_input(data: &mut [u8]) -> Result<()> {
    let mut buffer = UipBuffer::new(data);

    // Verify minimum packet size
    if buffer.len() < Ipv6Header::SIZE {
        return Err(Error::InvalidPacket);
    }

    // Parse IPv6 header
    let ip_header = buffer.ipv6_header()?;

    // Verify version
    if ip_header.version() != 6 {
        return Err(Error::InvalidPacket);
    }

    // Check payload length
    let payload_len = ip_header.payload_len() as usize;
    if Ipv6Header::SIZE + payload_len > buffer.len() {
        return Err(Error::InvalidPacket);
    }

    // Get addresses
    let src = ip_header.srcipaddr;
    let dst = ip_header.destipaddr;

    // Check if packet is for us
    if !is_for_us(&dst) {
        // Forward packet if we're a router
        return forward_packet(&mut buffer);
    }

    // Check hop limit
    if ip_header.hlim == 0 {
        icmpv6::send_time_exceeded(&mut buffer, icmpv6::TimeExceededCode::HopLimitExceeded)?;
        return Err(Error::InvalidPacket);
    }

    // Process extension headers and get final next header
    let (next_header, payload_offset) = process_extension_headers(&mut buffer)?;

    // Dispatch to upper layer protocol
    match IpProto::from_u8(next_header) {
        Some(IpProto::Icmpv6) => {
            icmpv6::process_input(&mut buffer, payload_offset, &src, &dst)
        }
        Some(IpProto::Udp) => {
            process_udp(&mut buffer, payload_offset, &src, &dst)
        }
        Some(IpProto::Tcp) => {
            process_tcp(&mut buffer, payload_offset, &src, &dst)
        }
        Some(IpProto::Ipv6NoNxt) => {
            // No next header - packet ends here
            Ok(())
        }
        _ => {
            // Unsupported protocol
            Err(Error::UnsupportedProtocol)
        }
    }
}

/// Check if destination address is for us
fn is_for_us(addr: &Ipv6Addr) -> bool {
    let netif = ds6::get_netif();

    // Check unicast addresses
    if netif.addr_lookup(addr).is_some() {
        return true;
    }

    // Check multicast addresses
    for maddr in &netif.maddrs {
        if maddr.is_used && maddr.ipaddr == *addr {
            return true;
        }
    }

    // Check all-nodes multicast (FF02::1)
    if addr.u8[0] == 0xff && addr.u8[1] == 0x02 &&
       addr.u8[15] == 0x01 &&
       addr.u8[2..15].iter().all(|&b| b == 0) {
        return true;
    }

    false
}

/// Forward IPv6 packet
fn forward_packet(buffer: &mut UipBuffer) -> Result<()> {
    // Get IP header
    let ip_header = buffer.ipv6_header_mut()?;

    // Decrement hop limit
    if ip_header.hlim <= 1 {
        icmpv6::send_time_exceeded(buffer, icmpv6::TimeExceededCode::HopLimitExceeded)?;
        return Err(Error::InvalidPacket);
    }
    ip_header.hlim -= 1;

    // Look up route and forward
    // This would integrate with routing table
    // For now, just return error
    Err(Error::RouteNotFound)
}

/// Process IPv6 extension headers
/// Returns (final next header, offset to payload)
fn process_extension_headers(buffer: &UipBuffer) -> Result<(u8, usize)> {
    let ip_header = buffer.ipv6_header()?;
    let mut next_header = ip_header.next;
    let mut offset = Ipv6Header::SIZE;

    // Process extension headers in order
    loop {
        match IpProto::from_u8(next_header) {
            Some(IpProto::HopByHop) | Some(IpProto::Ipv6Opts) => {
                // Hop-by-Hop or Destination Options
                let payload = buffer.payload(offset)?;
                if payload.len() < 2 {
                    return Err(Error::InvalidPacket);
                }

                let ext_hdr = unsafe {
                    &*(payload.as_ptr() as *const ExtHeader)
                };

                next_header = ext_hdr.next;
                // Length is in units of 8 bytes, excluding first 8 bytes
                let hdr_len = 8 + (ext_hdr.len as usize * 8);
                offset += hdr_len;
            }
            Some(IpProto::Ipv6Route) => {
                // Routing header
                let payload = buffer.payload(offset)?;
                if payload.len() < 4 {
                    return Err(Error::InvalidPacket);
                }

                let ext_hdr = unsafe {
                    &*(payload.as_ptr() as *const ExtHeader)
                };

                next_header = ext_hdr.next;
                let hdr_len = 8 + (ext_hdr.len as usize * 8);
                offset += hdr_len;
            }
            Some(IpProto::Ipv6Frag) => {
                // Fragment header
                let payload = buffer.payload(offset)?;
                if payload.len() < 8 {
                    return Err(Error::InvalidPacket);
                }

                let frag_hdr = unsafe {
                    &*(payload.as_ptr() as *const FragHeader)
                };

                next_header = frag_hdr.next;
                offset += 8;

                // Handle fragmentation
                if frag_hdr.offset() != 0 || frag_hdr.more_fragments() {
                    // This is a fragment - would need reassembly
                    return Err(Error::FragmentationError);
                }
            }
            _ => {
                // Not an extension header, must be upper layer
                break;
            }
        }

        // Safety check
        if offset >= buffer.len() {
            return Err(Error::InvalidPacket);
        }
    }

    Ok((next_header, offset))
}

/// Process UDP packet
fn process_udp(
    buffer: &mut UipBuffer,
    offset: usize,
    src: &Ipv6Addr,
    dst: &Ipv6Addr,
) -> Result<()> {
    let payload = buffer.payload(offset)?;

    if payload.len() < UdpHeader::SIZE {
        return Err(Error::InvalidPacket);
    }

    let udp_hdr = unsafe {
        &*(payload.as_ptr() as *const UdpHeader)
    };

    // Verify checksum if present (0 means no checksum for IPv4, but mandatory for IPv6)
    let checksum = u16::from_be_bytes(udp_hdr.chksum);
    if checksum != 0 {
        if !checksum::verify_checksum(
            src,
            dst,
            payload.len() as u32,
            IpProto::Udp as u8,
            payload,
            checksum,
        ) {
            return Err(Error::InvalidChecksum);
        }
    }

    // Would dispatch to UDP connection handler
    // For now, just acknowledge receipt
    Ok(())
}

/// Process TCP packet
fn process_tcp(
    buffer: &mut UipBuffer,
    offset: usize,
    src: &Ipv6Addr,
    dst: &Ipv6Addr,
) -> Result<()> {
    let payload = buffer.payload(offset)?;

    if payload.len() < TcpHeader::MIN_SIZE {
        return Err(Error::InvalidPacket);
    }

    let tcp_hdr = unsafe {
        &*(payload.as_ptr() as *const TcpHeader)
    };

    // Verify checksum
    let checksum = u16::from_be_bytes(tcp_hdr.chksum);
    if !checksum::verify_checksum(
        src,
        dst,
        payload.len() as u32,
        IpProto::Tcp as u8,
        payload,
        checksum,
    ) {
        return Err(Error::InvalidChecksum);
    }

    // Would dispatch to TCP connection handler
    // For now, just acknowledge receipt
    Ok(())
}

/// Prepare an IPv6 packet for sending
pub fn prepare_output(
    buffer: &mut UipBuffer,
    src: &Ipv6Addr,
    dst: &Ipv6Addr,
    next_header: u8,
    payload_len: u16,
) -> Result<()> {
    if buffer.len() < Ipv6Header::SIZE {
        return Err(Error::BufferTooSmall);
    }

    let ip_header = buffer.ipv6_header_mut()?;

    // Set version and traffic class
    ip_header.vtc = 0x60;  // Version 6, TC = 0
    ip_header.tcflow = 0;
    ip_header.flow = [0, 0];

    // Set payload length
    ip_header.set_payload_len(payload_len);

    // Set next header and hop limit
    ip_header.next = next_header;
    ip_header.hlim = DEFAULT_HOP_LIMIT;

    // Set addresses
    ip_header.srcipaddr = *src;
    ip_header.destipaddr = *dst;

    Ok(())
}

/// Send an IPv6 packet
pub fn send_packet(buffer: &mut UipBuffer) -> Result<()> {
    // Would send to link layer
    // For now, just validate
    if buffer.len() < Ipv6Header::SIZE {
        return Err(Error::BufferTooSmall);
    }

    let ip_header = buffer.ipv6_header()?;
    if ip_header.version() != 6 {
        return Err(Error::InvalidPacket);
    }

    // Would pass to sicslowpan for 6LoWPAN compression
    // then to MAC layer
    Ok(())
}
