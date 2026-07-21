use std::time::Instant;

pub const TELEMETRY_MAGIC: [u8; 4] = *b"PDL1";
pub const TELEMETRY_VERSION: u16 = 1;
pub const TELEMETRY_FLAG_CAMERA_ONLINE: u32 = 1 << 0;
pub const TELEMETRY_FLAG_GIMBAL_ONLINE: u32 = 1 << 1;

pub const VIDEO_0310_FLAG_RESET: u8 = 1 << 0;
pub const VIDEO_0310_PAYLOAD_BYTES_MSB_MASK: u8 = 1 << 1;
pub const VIDEO_0310_RESERVED_MASK: u8 = 0xFC;
pub const VIDEO_0310_HEADER_BYTES: usize = 3;
pub const VIDEO_0310_PAYLOAD_BYTES: usize = 297;
pub const VIDEO_0310_PACKET_BYTES: usize = VIDEO_0310_HEADER_BYTES + VIDEO_0310_PAYLOAD_BYTES;
pub const VIDEO_CODEC_H264: u8 = 1;

#[allow(dead_code)]
#[derive(Clone, Debug, Default)]
pub struct VehicleTelemetry {
    pub version: u16,
    pub struct_bytes: u16,
    pub flags: u32,
    pub unix_ms: u64,
    pub frame_seq: u32,
    pub image_width: u16,
    pub image_height: u16,
    pub fps_x100: u16,
    pub gain_x100: u16,
    pub exposure_us: u32,
    pub gimbal_mode: u8,
    pub bullet_count: u16,
    pub yaw: f32,
    pub yaw_vel: f32,
    pub pitch: f32,
    pub pitch_vel: f32,
    pub bullet_speed: f32,
    pub quaternion: [f32; 4],
    pub status_text: String,
}

impl VehicleTelemetry {
    pub fn camera_online(&self) -> bool {
        (self.flags & TELEMETRY_FLAG_CAMERA_ONLINE) != 0
    }

    pub fn gimbal_online(&self) -> bool {
        (self.flags & TELEMETRY_FLAG_GIMBAL_ONLINE) != 0
    }

    pub fn fps(&self) -> f32 {
        self.fps_x100 as f32 / 100.0
    }

    pub fn gain(&self) -> f32 {
        self.gain_x100 as f32 / 100.0
    }

    pub fn gimbal_mode_str(&self) -> &'static str {
        match self.gimbal_mode {
            0 => "IDLE",
            1 => "AUTO_AIM",
            2 => "SMALL_BUFF",
            3 => "BIG_BUFF",
            _ => "UNKNOWN",
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct MetadataSnapshot {
    pub enabled: bool,
    pub connected: bool,
    pub messages_received: u64,
    pub last_message_at: Option<Instant>,
    pub last_error: Option<String>,
    pub telemetry: Option<VehicleTelemetry>,
}

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct Video0310Chunk {
    pub codec: u8,
    pub flags: u8,
    pub sequence: u32,
    pub sequence_modulus: u32,
    pub stream_ms: u32,
    pub payload: Vec<u8>,
}

pub fn decode_custom_byte_block(payload: &[u8]) -> Option<Vec<u8>> {
    let mut cursor = 0_usize;

    while cursor < payload.len() {
        let key = decode_varint(payload, &mut cursor)?;
        let field_number = key >> 3;
        let wire_type = key & 0x07;

        match (field_number, wire_type) {
            (1, 2) => {
                let length = decode_varint(payload, &mut cursor)? as usize;
                if cursor + length > payload.len() {
                    return None;
                }
                return Some(payload[cursor..cursor + length].to_vec());
            }
            (_, 0) => {
                let _ = decode_varint(payload, &mut cursor)?;
            }
            (_, 1) => {
                cursor = cursor.checked_add(8)?;
                if cursor > payload.len() {
                    return None;
                }
            }
            (_, 2) => {
                let length = decode_varint(payload, &mut cursor)? as usize;
                cursor = cursor.checked_add(length)?;
                if cursor > payload.len() {
                    return None;
                }
            }
            (_, 5) => {
                cursor = cursor.checked_add(4)?;
                if cursor > payload.len() {
                    return None;
                }
            }
            _ => return None,
        }
    }

    None
}

pub fn parse_vehicle_telemetry(data: &[u8]) -> Option<VehicleTelemetry> {
    if data.len() < 74 {
        return None;
    }

    let mut cursor = 0_usize;

    let magic = read_exact::<4>(data, &mut cursor)?;
    if magic != TELEMETRY_MAGIC {
        return None;
    }

    let version = read_u16_le(data, &mut cursor)?;
    if version != TELEMETRY_VERSION {
        return None;
    }

    let struct_bytes = read_u16_le(data, &mut cursor)?;
    let struct_len = struct_bytes as usize;
    if struct_len == 0 || struct_len > data.len() || struct_len < 74 {
        return None;
    }

    let flags = read_u32_le(data, &mut cursor)?;
    let unix_ms = read_u64_le(data, &mut cursor)?;
    let frame_seq = read_u32_le(data, &mut cursor)?;
    let image_width = read_u16_le(data, &mut cursor)?;
    let image_height = read_u16_le(data, &mut cursor)?;
    let fps_x100 = read_u16_le(data, &mut cursor)?;
    let gain_x100 = read_u16_le(data, &mut cursor)?;
    let exposure_us = read_u32_le(data, &mut cursor)?;
    let gimbal_mode = read_u8(data, &mut cursor)?;
    let _reserved0 = read_u8(data, &mut cursor)?;
    let bullet_count = read_u16_le(data, &mut cursor)?;
    let yaw = read_f32_le(data, &mut cursor)?;
    let yaw_vel = read_f32_le(data, &mut cursor)?;
    let pitch = read_f32_le(data, &mut cursor)?;
    let pitch_vel = read_f32_le(data, &mut cursor)?;
    let bullet_speed = read_f32_le(data, &mut cursor)?;

    let mut quaternion = [0.0_f32; 4];
    for item in &mut quaternion {
        *item = read_f32_le(data, &mut cursor)?;
    }

    let text_len = struct_len.saturating_sub(cursor);
    let status_text = decode_status_text(&data[cursor..cursor + text_len]);

    Some(VehicleTelemetry {
        version,
        struct_bytes,
        flags,
        unix_ms,
        frame_seq,
        image_width,
        image_height,
        fps_x100,
        gain_x100,
        exposure_us,
        gimbal_mode,
        bullet_count,
        yaw,
        yaw_vel,
        pitch,
        pitch_vel,
        bullet_speed,
        quaternion,
        status_text,
    })
}

pub fn parse_video_0310_chunk(data: &[u8]) -> Option<Video0310Chunk> {
    if data.len() != VIDEO_0310_PACKET_BYTES {
        return None;
    }

    let mut cursor = 0_usize;
    let header = read_u8(data, &mut cursor)?;
    if (header & VIDEO_0310_RESERVED_MASK) != 0 {
        return None;
    }

    let flags = header & VIDEO_0310_FLAG_RESET;
    let sequence = read_u8(data, &mut cursor)? as u32;
    let payload_len = read_u8(data, &mut cursor)? as usize
        | (usize::from((header & VIDEO_0310_PAYLOAD_BYTES_MSB_MASK) != 0) << 8);

    if payload_len > VIDEO_0310_PAYLOAD_BYTES || cursor != VIDEO_0310_HEADER_BYTES {
        return None;
    }

    let payload_end = VIDEO_0310_HEADER_BYTES.checked_add(payload_len)?;
    let payload = data.get(VIDEO_0310_HEADER_BYTES..payload_end)?.to_vec();

    Some(Video0310Chunk {
        codec: VIDEO_CODEC_H264,
        flags,
        sequence,
        sequence_modulus: 256,
        stream_ms: 0,
        payload,
    })
}

fn decode_status_text(bytes: &[u8]) -> String {
    let end = bytes
        .iter()
        .position(|value| *value == 0)
        .unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_string()
}

fn decode_varint(data: &[u8], cursor: &mut usize) -> Option<u64> {
    let mut shift = 0_u32;
    let mut value = 0_u64;

    while *cursor < data.len() && shift < 64 {
        let byte = data[*cursor];
        *cursor += 1;
        value |= ((byte & 0x7F) as u64) << shift;
        if (byte & 0x80) == 0 {
            return Some(value);
        }
        shift += 7;
    }

    None
}

fn read_u8(data: &[u8], cursor: &mut usize) -> Option<u8> {
    let value = *data.get(*cursor)?;
    *cursor += 1;
    Some(value)
}

fn read_u16_le(data: &[u8], cursor: &mut usize) -> Option<u16> {
    let bytes = read_exact::<2>(data, cursor)?;
    Some(u16::from_le_bytes(bytes))
}

fn read_u32_le(data: &[u8], cursor: &mut usize) -> Option<u32> {
    let bytes = read_exact::<4>(data, cursor)?;
    Some(u32::from_le_bytes(bytes))
}

fn read_u64_le(data: &[u8], cursor: &mut usize) -> Option<u64> {
    let bytes = read_exact::<8>(data, cursor)?;
    Some(u64::from_le_bytes(bytes))
}

fn read_f32_le(data: &[u8], cursor: &mut usize) -> Option<f32> {
    let bytes = read_exact::<4>(data, cursor)?;
    Some(f32::from_le_bytes(bytes))
}

fn read_exact<const N: usize>(data: &[u8], cursor: &mut usize) -> Option<[u8; N]> {
    let end = cursor.checked_add(N)?;
    let slice = data.get(*cursor..end)?;
    let mut bytes = [0_u8; N];
    bytes.copy_from_slice(slice);
    *cursor = end;
    Some(bytes)
}

#[cfg(test)]
mod tests {
    use super::{
        TELEMETRY_FLAG_CAMERA_ONLINE, TELEMETRY_FLAG_GIMBAL_ONLINE, VIDEO_0310_HEADER_BYTES,
        VIDEO_0310_PACKET_BYTES, VIDEO_0310_PAYLOAD_BYTES_MSB_MASK, VIDEO_CODEC_H264,
        decode_custom_byte_block, parse_vehicle_telemetry, parse_video_0310_chunk,
    };

    #[test]
    fn decodes_custom_byte_block_field_one() {
        let payload = vec![0x0A, 0x03, 0x11, 0x22, 0x33];
        let decoded = decode_custom_byte_block(&payload).expect("payload should decode");
        assert_eq!(decoded, vec![0x11, 0x22, 0x33]);
    }

    #[test]
    fn parses_vehicle_telemetry_v1() {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(b"PDL1");
        bytes.extend_from_slice(&1_u16.to_le_bytes());
        bytes.extend_from_slice(&138_u16.to_le_bytes());
        bytes.extend_from_slice(
            &(TELEMETRY_FLAG_CAMERA_ONLINE | TELEMETRY_FLAG_GIMBAL_ONLINE).to_le_bytes(),
        );
        bytes.extend_from_slice(&123456_u64.to_le_bytes());
        bytes.extend_from_slice(&42_u32.to_le_bytes());
        bytes.extend_from_slice(&1280_u16.to_le_bytes());
        bytes.extend_from_slice(&720_u16.to_le_bytes());
        bytes.extend_from_slice(&5999_u16.to_le_bytes());
        bytes.extend_from_slice(&850_u16.to_le_bytes());
        bytes.extend_from_slice(&12000_u32.to_le_bytes());
        bytes.push(1_u8);
        bytes.push(0_u8);
        bytes.extend_from_slice(&7_u16.to_le_bytes());
        bytes.extend_from_slice(&1.0_f32.to_le_bytes());
        bytes.extend_from_slice(&2.0_f32.to_le_bytes());
        bytes.extend_from_slice(&3.0_f32.to_le_bytes());
        bytes.extend_from_slice(&4.0_f32.to_le_bytes());
        bytes.extend_from_slice(&5.0_f32.to_le_bytes());
        for value in [1.0_f32, 0.0_f32, 0.0_f32, 0.0_f32] {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        let mut text = [0_u8; 64];
        text[..9].copy_from_slice(b"HELLO 123");
        bytes.extend_from_slice(&text);

        let telemetry = parse_vehicle_telemetry(&bytes).expect("telemetry should parse");
        assert!(telemetry.camera_online());
        assert!(telemetry.gimbal_online());
        assert_eq!(telemetry.frame_seq, 42);
        assert_eq!(telemetry.image_width, 1280);
        assert_eq!(telemetry.image_height, 720);
        assert_eq!(telemetry.bullet_count, 7);
        assert_eq!(telemetry.status_text, "HELLO 123");
    }

    #[test]
    fn parses_video_0310_chunk() {
        let payload = b"abc123";
        let mut bytes = vec![0_u8; VIDEO_0310_PACKET_BYTES];
        bytes[0] = 1;
        bytes[1] = 42;
        bytes[2] = payload.len() as u8;
        bytes[VIDEO_0310_HEADER_BYTES..VIDEO_0310_HEADER_BYTES + payload.len()]
            .copy_from_slice(payload);

        let chunk = parse_video_0310_chunk(&bytes).expect("video chunk should parse");
        assert_eq!(chunk.codec, VIDEO_CODEC_H264);
        assert_eq!(chunk.flags, 1);
        assert_eq!(chunk.sequence, 42);
        assert_eq!(chunk.sequence_modulus, 256);
        assert_eq!(chunk.payload, payload);
    }

    #[test]
    fn parses_video_0310_chunk_with_297_byte_payload() {
        let payload = vec![0x5A_u8; 297];
        let mut bytes = vec![0_u8; VIDEO_0310_PACKET_BYTES];
        bytes[0] = 1 | VIDEO_0310_PAYLOAD_BYTES_MSB_MASK;
        bytes[1] = 7;
        bytes[2] = 41;
        bytes[VIDEO_0310_HEADER_BYTES..VIDEO_0310_HEADER_BYTES + payload.len()]
            .copy_from_slice(&payload);

        let chunk = parse_video_0310_chunk(&bytes).expect("video chunk should parse");
        assert_eq!(chunk.codec, VIDEO_CODEC_H264);
        assert_eq!(chunk.flags, 1);
        assert_eq!(chunk.sequence, 7);
        assert_eq!(chunk.sequence_modulus, 256);
        assert_eq!(chunk.stream_ms, 0);
        assert_eq!(chunk.payload, payload);
    }
}
