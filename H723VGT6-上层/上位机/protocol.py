# -*- coding: utf-8 -*-
"""H723 upper-computer protocol.

The wire format mirrors User/Driver/PcLink/pc_protocol.c. All integer and
float fields are little-endian. Position commands are consumed by the
existing M3508/M2006 position loops in the upper firmware. The extended
position command also carries editable MIT and cascaded PID parameters.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass


SYNC = b"\xA5\x5A"
VERSION = 3
MAX_PAYLOAD = 128
HEADER_SIZE = 8
FRAME_OVERHEAD = 10

MSG_HEARTBEAT = 0x01
MSG_ESTOP = 0x02
MSG_HANDSHAKE = 0x03
MSG_UPPER_CMD = 0x10
MSG_UPPER_POSITION_CMD = 0x12
MSG_ROBOT_STATE = 0x20
MSG_ACK = 0x7E

# The payload is deliberately fixed so an ACK from another protocol cannot
# accidentally turn a newly opened serial port into a ready control link.
HANDSHAKE_MAGIC = b"H723"

ENABLE_ARM = 1 << 0
ENABLE_CONVEYOR = 1 << 1
ENABLE_GRIPPER = 1 << 2

UPPER_PAYLOAD_SIZE = 34
EXTENDED_POSITION_PAYLOAD_SIZE = 122
_PAYLOAD_FORMAT = "<H8f"
_EXTENDED_POSITION_FORMAT = "<H30f"


def crc16(data: bytes) -> int:
    """Modbus CRC16 used by the firmware parser."""

    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value >> 1) ^ 0xA001) if value & 1 else value >> 1
    return value


def encode_frame(msg_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if not 0 <= msg_type <= 0xFF:
        raise ValueError("message type must fit in one byte")
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("sequence must fit in uint16")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds protocol limit")
    header = struct.pack("<BBHH", VERSION, msg_type, sequence, len(payload))
    return SYNC + header + payload + struct.pack("<H", crc16(header + payload))


def build_handshake_frame(sequence: int) -> bytes:
    """Build the request used to establish a PC-to-board control session."""

    return encode_frame(MSG_HANDSHAKE, sequence, HANDSHAKE_MAGIC)


def build_velocity_payload(
    enable_mask: int,
    j_position: float,
    j_velocity: float,
    j_kp: float,
    j_kd: float,
    m3508_velocity_1: float,
    m3508_velocity_2: float,
    conveyor_velocity: float,
    gripper_velocity: float,
) -> bytes:
    return struct.pack(
        _PAYLOAD_FORMAT,
        enable_mask,
        j_position,
        j_velocity,
        j_kp,
        j_kd,
        m3508_velocity_1,
        m3508_velocity_2,
        conveyor_velocity,
        gripper_velocity,
    )


def build_position_payload(
    enable_mask: int,
    j_position: float,
    j_velocity: float,
    j_kp: float,
    j_kd: float,
    m3508_position_1: float,
    m3508_position_2: float,
    conveyor_position: float,
    gripper_position: float,
) -> bytes:
    return struct.pack(
        _PAYLOAD_FORMAT,
        enable_mask,
        j_position,
        j_velocity,
        j_kp,
        j_kd,
        m3508_position_1,
        m3508_position_2,
        conveyor_position,
        gripper_position,
    )


def build_extended_position_payload(
    enable_mask: int,
    j_position: float,
    j_velocity: float,
    j_kp: float,
    j_kd: float,
    j_tau: float,
    j_torque_limit: float,
    m3508_position_1: float,
    m3508_position_2: float,
    conveyor_position: float,
    gripper_position: float,
    *pid_values: float,
) -> bytes:
    """Build a position command carrying MIT and cascaded PID settings.

    The twenty trailing values are ordered as M3508 speed/position PID and
    M2006 speed/position PID, with five values per loop: Kp, Ki, Kd,
    integral limit, output limit. Values use the DJI_H723_VOFA GUI units.
    """

    if len(pid_values) != 20:
        raise ValueError("extended position payload requires 20 PID values")
    return struct.pack(
        _EXTENDED_POSITION_FORMAT,
        enable_mask,
        j_position,
        j_velocity,
        j_kp,
        j_kd,
        j_tau,
        j_torque_limit,
        m3508_position_1,
        m3508_position_2,
        conveyor_position,
        gripper_position,
        *pid_values,
    )


@dataclass(frozen=True)
class Frame:
    msg_type: int
    sequence: int
    payload: bytes


class FrameParser:
    """Incremental parser that tolerates noise and split USB packets."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.valid_count = 0
        self.crc_error_count = 0
        self.length_error_count = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            sync_at = self._buffer.find(SYNC)
            if sync_at < 0:
                self._buffer.clear()
                break
            if sync_at:
                del self._buffer[:sync_at]
            if len(self._buffer) < HEADER_SIZE:
                break
            version, msg_type, sequence, payload_len = struct.unpack_from(
                "<BBHH", self._buffer, 2
            )
            if version != VERSION or payload_len > MAX_PAYLOAD:
                self.length_error_count += 1
                del self._buffer[:2]
                continue
            frame_len = FRAME_OVERHEAD + payload_len
            if len(self._buffer) < frame_len:
                break
            raw = bytes(self._buffer[:frame_len])
            expected_crc = struct.unpack_from("<H", raw, HEADER_SIZE + payload_len)[0]
            if crc16(raw[2:HEADER_SIZE + payload_len]) != expected_crc:
                self.crc_error_count += 1
                del self._buffer[:2]
                continue
            frames.append(Frame(msg_type, sequence, raw[HEADER_SIZE:HEADER_SIZE + payload_len]))
            self.valid_count += 1
            del self._buffer[:frame_len]
        return frames


def decode_robot_state(payload: bytes) -> dict[str, int]:
    if len(payload) != 20:
        raise ValueError("robot state payload must be 20 bytes")
    state, remote_active, tick_ms, last_rx, sent, send_fail, blocked = struct.unpack(
        "<BBIHIII", payload
    )
    return {
        "state": state,
        "remote_active": remote_active,
        "tick_ms": tick_ms,
        "last_rx_sequence": last_rx,
        "sent_count": sent,
        "send_fail_count": send_fail,
        "protocol_block_count": blocked,
    }
