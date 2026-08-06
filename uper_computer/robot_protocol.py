from __future__ import annotations

import struct
from dataclasses import dataclass


VELOCITY_HEADER = bytes((0xA5, 0x5A))
ACTION_HEADER = bytes((0xA5, 0x5B))
YAW_HEADER = bytes((0xA5, 0x5C))

ACTION_LOWER = 0x00
ACTION_LIFT = 0x01
ACTION_M2006_FORWARD = 0x02
ACTION_FRONT_FOLD = 0x03
ACTION_FRONT_FLAT = 0x04
ACTION_FRONT_DOWN = 0x05
ACTION_REAR_FOLD = 0x06
ACTION_REAR_FLAT = 0x07
ACTION_REAR_DOWN = 0x08

YAW_FRAME_LENGTH = 5
YAW_SCALE = 100.0


@dataclass(frozen=True)
class VelocityCommand:
    vx_mm_s: int = 0
    vy_mm_s: int = 0
    wz_mrad_s: int = 0


def calculate_checksum(payload: bytes) -> int:
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum


def build_velocity_frame(command: VelocityCommand) -> bytes:
    payload = struct.pack(
        "<hhh",
        command.vx_mm_s,
        command.vy_mm_s,
        command.wz_mrad_s,
    )
    return VELOCITY_HEADER + payload + bytes((calculate_checksum(payload),))


def build_action_frame(action: int) -> bytes:
    if not ACTION_LOWER <= action <= ACTION_REAR_DOWN:
        raise ValueError("unknown action")
    return ACTION_HEADER + bytes((action,))


class YawFrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, payload: bytes) -> list[float]:
        yaw_values = []
        self._buffer.extend(payload)

        while len(self._buffer) >= len(YAW_HEADER):
            header_index = self._buffer.find(YAW_HEADER)
            if header_index < 0:
                if self._buffer[-1] == YAW_HEADER[0]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break

            if header_index > 0:
                del self._buffer[:header_index]
            if len(self._buffer) < YAW_FRAME_LENGTH:
                break

            frame = self._buffer[:YAW_FRAME_LENGTH]
            if frame[4] != calculate_checksum(frame[2:4]):
                del self._buffer[0]
                continue

            yaw_cdeg = struct.unpack("<h", frame[2:4])[0]
            yaw_values.append(yaw_cdeg / YAW_SCALE)
            del self._buffer[:YAW_FRAME_LENGTH]

        return yaw_values
