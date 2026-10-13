"""CRSF framing and payload decoding without ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass


FRAME_TYPE_LINK_STATISTICS = 0x14
FRAME_TYPE_RC_CHANNELS_PACKED = 0x16
MAX_FRAME_SIZE = 64
CHANNEL_COUNT = 16
CHANNEL_MIN = 172
CHANNEL_MAX = 1811
MICROSECONDS_MIN = 988
MICROSECONDS_MAX = 2012


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class StreamParser:
    """Extract checksum-valid CRSF frames from a byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0
        self.valid_frames = 0

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames: list[bytes] = []
        while len(self.buffer) >= 4:
            payload_length = self.buffer[1]
            if payload_length < 2 or payload_length > MAX_FRAME_SIZE - 2:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue

            frame_length = payload_length + 2
            if len(self.buffer) < frame_length:
                break

            frame = bytes(self.buffer[:frame_length])
            if crc8_dvb_s2(frame[2:-1]) != frame[-1]:
                del self.buffer[0]
                self.crc_errors += 1
                self.discarded_bytes += 1
                continue

            frames.append(frame)
            self.valid_frames += 1
            del self.buffer[:frame_length]
        return frames


def decode_channels(payload: bytes) -> tuple[list[int], list[int]]:
    if len(payload) != 22:
        raise ValueError(f"expected 22 channel bytes, received {len(payload)}")
    packed = int.from_bytes(payload, byteorder="little")
    raw = [(packed >> (index * 11)) & 0x07FF for index in range(CHANNEL_COUNT)]
    microseconds = [channel_to_microseconds(value) for value in raw]
    return raw, microseconds


def channel_to_microseconds(raw_value: int) -> int:
    input_span = CHANNEL_MAX - CHANNEL_MIN
    output_span = MICROSECONDS_MAX - MICROSECONDS_MIN
    return round(MICROSECONDS_MIN + (raw_value - CHANNEL_MIN) * output_span / input_span)


def signed_byte(value: int) -> int:
    return value - 256 if value >= 128 else value


@dataclass(frozen=True)
class LinkStatistics:
    uplink_rssi_1_dbm: int
    uplink_rssi_2_dbm: int
    uplink_link_quality: int
    uplink_snr_db: int
    active_antenna: int
    rf_mode: int
    tx_power_code: int
    downlink_rssi_dbm: int
    downlink_link_quality: int
    downlink_snr_db: int

    @classmethod
    def decode(cls, payload: bytes) -> "LinkStatistics":
        if len(payload) < 10:
            raise ValueError(f"expected at least 10 link-statistics bytes, received {len(payload)}")
        return cls(
            uplink_rssi_1_dbm=-payload[0],
            uplink_rssi_2_dbm=-payload[1],
            uplink_link_quality=payload[2],
            uplink_snr_db=signed_byte(payload[3]),
            active_antenna=payload[4],
            rf_mode=payload[5],
            tx_power_code=payload[6],
            downlink_rssi_dbm=-payload[7],
            downlink_link_quality=payload[8],
            downlink_snr_db=signed_byte(payload[9]),
        )
