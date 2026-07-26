import struct
SYNC_BYTE = 0xA5
PKT_LEN = 7
def compute_checksum(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc

def send_command(vel_x, vel_y, w, dribbler, kicker, kicker_mode, angle_mode)->bytes:
    """Send command packet with sync byte and XOR checksum."""
    header = struct.pack('<B5eB?',
                         SYNC_BYTE,
                         vel_x, vel_y, w,
                         dribbler, kicker,
                         kicker_mode, angle_mode)
    checksum = compute_checksum(header)
    packet = header + struct.pack('B', checksum)
    return packet

