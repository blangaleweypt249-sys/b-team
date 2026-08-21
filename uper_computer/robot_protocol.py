from __future__ import annotations

import struct
from dataclasses import dataclass


VELOCITY_HEADER = bytes((0xA5, 0x5A))
ACTION_HEADER = bytes((0xA5, 0x5B))
YAW_HEADER = bytes((0xA5, 0x5C))
ROAD_HEADER = bytes((0xA5, 0x5D))
ROAD_RESET_HEADER = bytes((0xA5, 0x5E))
DT35_HEADER = bytes((0xAA,))
PNP_HEADER = bytes((0xAB,))
DT35_ADDR_F = 0x41
DT35_ADDR_L = 0x40
# Keep numeric aliases for callers that still use the original names.
DT35_ADDR_40 = DT35_ADDR_F
DT35_ADDR_41 = DT35_ADDR_L
PNP_ADDR_F = 0x40
PNP_ADDR_B = 0x41

SC_LINK_HEADER = bytes((0xAA, 0x55))
SC_LINK_PERCEPTION_LENGTH = 37
SC_LINK_POSE_LENGTH = 29
SC_LINK_PERCEPTION_TYPE = 0x10
SC_LINK_POSE_TYPE = 0x11
SC_LINK_FLAG_BLOCK_VALID = 1 << 0
SC_LINK_FLAG_BALL_VALID = 1 << 2
SC_LINK_FLAG_POSE_VALID = 1 << 0

ACTION_LOWER = 0x00
ACTION_LIFT = 0x01
ACTION_M2006_FORWARD = 0x02
ACTION_FRONT_FLAT = 0x03
ACTION_FRONT_DOWN = 0x04
ACTION_REAR_FLAT = 0x05
ACTION_REAR_DOWN = 0x06
ACTION_M2006_COAST = 0x07
ACTION_ALIGN_BLOCK_PNP = 0x0C

YAW_FRAME_LENGTH = 5
YAW_SCALE = 100.0
ROAD_FRAME_LENGTH = 16
DT35_FRAME_LENGTH = 5
PNP_FRAME_LENGTH = 5


@dataclass(frozen=True)
class VelocityCommand:
    vx_mm_s: int = 0
    vy_mm_s: int = 0
    wz_mrad_s: int = 0


@dataclass(frozen=True)
class RoadFrame:
    valid: bool
    x_m: float
    y_m: float
    distance_m: float


@dataclass(frozen=True)
class ScVisionFrame:
    frame_type: str
    sequence: int
    flags: int
    timestamp_ms: int
    block_xyz_m: tuple[float, float, float] | None
    ball_xyz_m: tuple[float, float, float] | None
    pose_xyzyaw: tuple[float, float, float, float] | None
    raw: bytes


def calculate_checksum(payload: bytes) -> int:
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return checksum


def calculate_crc16_ccitt(payload: bytes) -> int:
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_velocity_frame(command: VelocityCommand) -> bytes:
    payload = struct.pack(
        "<hhh",
        command.vx_mm_s,
        command.vy_mm_s,
        command.wz_mrad_s,
    )
    return VELOCITY_HEADER + payload + bytes((calculate_checksum(payload),))


def build_action_frame(action: int) -> bytes:
    if not ACTION_LOWER <= action <= ACTION_ALIGN_BLOCK_PNP:
        raise ValueError("unknown action")
    return ACTION_HEADER + bytes((action,))


def build_road_reset_frame() -> bytes:
    return ROAD_RESET_HEADER + bytes((1,))


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


class RoadFrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, payload: bytes) -> list[RoadFrame]:
        frames = []
        self._buffer.extend(payload)

        while len(self._buffer) >= len(ROAD_HEADER):
            header_index = self._buffer.find(ROAD_HEADER)
            if header_index < 0:
                if self._buffer[-1] == ROAD_HEADER[0]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break
            if header_index > 0:
                del self._buffer[:header_index]
            if len(self._buffer) < ROAD_FRAME_LENGTH:
                break

            frame = self._buffer[:ROAD_FRAME_LENGTH]
            if frame[15] != calculate_checksum(frame[2:15]):
                del self._buffer[0]
                continue

            x_mm, y_mm, distance_mm = struct.unpack_from("<iii", frame, 3)
            frames.append(RoadFrame(
                valid=bool(frame[2] & 1),
                x_m=x_mm / 1000.0,
                y_m=y_mm / 1000.0,
                distance_m=distance_mm / 1000.0,
            ))
            del self._buffer[:ROAD_FRAME_LENGTH]

        return frames


class Dt35FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, payload: bytes) -> list[tuple[int, int]]:
        frames = []
        self._buffer.extend(payload)

        while len(self._buffer) >= DT35_FRAME_LENGTH:
            header_index = self._buffer.find(DT35_HEADER)
            if header_index < 0:
                self._buffer.clear()
                break
            if header_index > 0:
                del self._buffer[:header_index]
            if len(self._buffer) < DT35_FRAME_LENGTH:
                break

            frame = self._buffer[:DT35_FRAME_LENGTH]
            checksum = 0
            for byte in frame[:4]:
                checksum ^= byte
            if frame[1] not in (DT35_ADDR_F, DT35_ADDR_L) or frame[4] != checksum:
                del self._buffer[0]
                continue

            distance_cm = frame[2] | (frame[3] << 8)
            frames.append((frame[1], distance_cm))
            del self._buffer[:DT35_FRAME_LENGTH]

        return frames


class PnpFrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, payload: bytes) -> list[tuple[int, int]]:
        frames = []
        self._buffer.extend(payload)

        while len(self._buffer) >= PNP_FRAME_LENGTH:
            header_index = self._buffer.find(PNP_HEADER)
            if header_index < 0:
                self._buffer.clear()
                break
            if header_index > 0:
                del self._buffer[:header_index]
            if len(self._buffer) < PNP_FRAME_LENGTH:
                break

            frame = self._buffer[:PNP_FRAME_LENGTH]
            checksum = calculate_checksum(frame[:4])
            trigger = frame[2] | (frame[3] << 8)
            if (
                frame[1] not in (PNP_ADDR_F, PNP_ADDR_B)
                or trigger not in (0, 1)
                or frame[4] != checksum
            ):
                del self._buffer[0]
                continue

            frames.append((frame[1], trigger))
            del self._buffer[:PNP_FRAME_LENGTH]

        return frames


class ScVisionFrameParser:
    """Parse the AA 55 CRC16 frames produced by the small computer link."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.valid_count = 0
        self.invalid_count = 0

    def reset(self) -> None:
        self._buffer.clear()
        self.valid_count = 0
        self.invalid_count = 0

    def feed(self, payload: bytes) -> list[ScVisionFrame]:
        frames: list[ScVisionFrame] = []
        self._buffer.extend(payload)

        while len(self._buffer) >= len(SC_LINK_HEADER):
            header_index = self._buffer.find(SC_LINK_HEADER)
            if header_index < 0:
                if self._buffer[-1] == SC_LINK_HEADER[0]:
                    self._buffer[:] = self._buffer[-1:]
                else:
                    self._buffer.clear()
                break
            if header_index > 0:
                del self._buffer[:header_index]
            if len(self._buffer) < 3:
                break

            frame_type = self._buffer[2]
            if frame_type == SC_LINK_PERCEPTION_TYPE:
                frame_length = SC_LINK_PERCEPTION_LENGTH
            elif frame_type == SC_LINK_POSE_TYPE:
                frame_length = SC_LINK_POSE_LENGTH
            else:
                del self._buffer[0]
                self.invalid_count += 1
                continue

            if len(self._buffer) < frame_length:
                break

            frame = bytes(self._buffer[:frame_length])
            crc_received = struct.unpack_from("<H", frame, frame_length - 4)[0]
            crc_expected = calculate_crc16_ccitt(frame[2:frame_length - 4])
            if frame[-2:] != bytes((0x0D, 0x0A)) or crc_received != crc_expected:
                del self._buffer[0]
                self.invalid_count += 1
                continue

            sequence = frame[3]
            flags = frame[4]
            timestamp_ms = struct.unpack_from("<I", frame, 5)[0]
            if frame_type == SC_LINK_PERCEPTION_TYPE:
                block = struct.unpack_from("<fff", frame, 9) \
                    if flags & SC_LINK_FLAG_BLOCK_VALID else None
                ball = struct.unpack_from("<fff", frame, 21) \
                    if flags & SC_LINK_FLAG_BALL_VALID else None
                parsed = ScVisionFrame(
                    "perception", sequence, flags, timestamp_ms,
                    block, ball, None, frame,
                )
            else:
                pose = struct.unpack_from("<ffff", frame, 9) \
                    if flags & SC_LINK_FLAG_POSE_VALID else None
                parsed = ScVisionFrame(
                    "pose", sequence, flags, timestamp_ms,
                    None, None, pose, frame,
                )

            frames.append(parsed)
            self.valid_count += 1
            del self._buffer[:frame_length]

        return frames
