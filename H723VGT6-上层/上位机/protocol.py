# -*- coding: utf-8 -*-
"""H723 upper-computer protocol.

The wire format mirrors User/Driver/PcLink/pc_protocol.c. All integer and
float fields are little-endian. Position commands are consumed by the
existing M3508/M2006 position loops in the upper firmware. Extended position
commands also carry editable PID parameters for explicitly enabled motors.
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
MSG_MOTOR_ACTION = 0x13
MSG_FLASH_INFO_REQUEST = 0x14
MSG_AUX_CONTROL = 0x15
MSG_ROBOT_STATE = 0x20
MSG_MOTOR_ACTION_RESULT = 0x21
MSG_DJI_TELEMETRY = 0x22
MSG_FLASH_INFO = 0x23
MSG_ACK = 0x7E
MSG_FAULT = 0x7F

MOTOR_ACTION_J4310_SAVE_ZERO = 1
MOTOR_ACTION_J4310_AUTO_RETURN = 2
MOTOR_ACTION_J4310_ENABLE = 3

# The payload is deliberately fixed so an ACK from another protocol cannot
# accidentally turn a newly opened serial port into a ready control link.
HANDSHAKE_MAGIC = b"H723"

# PC_MSG_AUX_CONTROL payload: output state bits followed by an update mask.
# The PC command enters H723 over UART4; H723 forwards the resulting state
# frame over UART5 to the F103 receiver's UART2.
# bit0/1/2 are the arm, push-block and gripper cylinders; bit3 is the
# receiver-board PB8/PB9 electronic-stop output.
AUX_CONTROL_PAYLOAD_SIZE = 2
AUX_OUTPUT_ARM_CYLINDER = 1 << 0
AUX_OUTPUT_PUSH_CYLINDER = 1 << 1
AUX_OUTPUT_GRIPPER_CYLINDER = 1 << 2
AUX_OUTPUT_ESTOP = 1 << 3

ENABLE_ARM = 1 << 0
ENABLE_CONVEYOR = 1 << 1
ENABLE_GRIPPER = 1 << 2
ENABLE_DJI_SYNC = 1 << 3
ENABLE_J4310_ONLY = 1 << 4
ENABLE_M3508_ONLY = 1 << 5
COMMAND_J4310_STOP = 1 << 6

UPPER_PAYLOAD_SIZE = 34
POSITION_TORQUE_PAYLOAD_SIZE = 42
EXTENDED_POSITION_PAYLOAD_SIZE = 122
_PAYLOAD_FORMAT = "<H8f"
_POSITION_TORQUE_FORMAT = "<H10f"
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


def build_aux_control_payload(output_bits: int, update_mask: int = 0x0F) -> bytes:
    """Build a masked cylinder/electronic-stop state command."""

    if not 0 <= output_bits <= 0x0F:
        raise ValueError("auxiliary output bits must fit in the low nibble")
    if not 0 <= update_mask <= 0x0F:
        raise ValueError("auxiliary update mask must fit in the low nibble")
    return struct.pack("<BB", output_bits, update_mask)


def build_motor_action_payload(
    action: int,
    can_bus: int,
    node_id: int,
    value: int | None = None,
) -> bytes:
    """Build a one-shot motor action payload."""

    if not 0 <= action <= 0xFF:
        raise ValueError("action must fit in one byte")
    if not 1 <= can_bus <= 3:
        raise ValueError("CAN bus must be 1, 2, or 3")
    if not 0 <= node_id <= 0xFF:
        raise ValueError("node ID must fit in one byte")
    if value is None:
        return struct.pack("<BBB", action, can_bus, node_id)
    if not 0 <= value <= 0xFF:
        raise ValueError("action value must fit in one byte")
    return struct.pack("<BBBB", action, can_bus, node_id, value)


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


def build_position_torque_payload(
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
) -> bytes:
    """Build a position command without any startup/test PID parameters."""

    return struct.pack(
        _POSITION_TORQUE_FORMAT,
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
    """Build an explicit position command with editable DJI PID values."""

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
                # A serial read may end after the first sync byte. Keep that
                # byte so the following read can complete the frame prefix.
                if self._buffer and self._buffer[-1] == SYNC[0]:
                    self._buffer[:] = self._buffer[-1:]
                else:
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


def decode_flash_info(payload: bytes) -> dict[str, int | bool]:
    """Decode the one-shot external Flash information response."""

    if len(payload) != 20:
        raise ValueError("Flash info payload must be 20 bytes")
    (
        init_status,
        initialized,
        jedec_id,
        capacity_kb,
        sector_count,
        page_size_byte,
        sector_size_byte,
    ) = struct.unpack("<BBIIIHI", payload)
    return {
        "init_status": init_status,
        "initialized": bool(initialized),
        "jedec_id": jedec_id,
        "capacity_kb": capacity_kb,
        "sector_count": sector_count,
        "page_size_byte": page_size_byte,
        "sector_size_byte": sector_size_byte,
    }


def decode_robot_state(payload: bytes) -> dict[str, object]:
    if len(payload) not in (20, 25, 50, 80, 84, 89, 101):
        raise ValueError(
            "robot state payload must be 20, 25, 50, 80, 84, 89, or 101 bytes"
        )
    state, remote_active, tick_ms, last_rx, sent, send_fail, blocked = struct.unpack(
        "<BBIHIII", payload[:20]
    )
    result = {
        "state": state,
        "remote_active": remote_active,
        "tick_ms": tick_ms,
        "last_rx_sequence": last_rx,
        "sent_count": sent,
        "send_fail_count": send_fail,
        "protocol_block_count": blocked,
    }
    if len(payload) in (25, 50, 80, 84, 89, 101):
        result["j4310_position_rad"] = struct.unpack_from("<f", payload, 20)[0]
        result["j4310_position_valid"] = payload[24]
    else:
        result["j4310_position_rad"] = 0.0
        result["j4310_position_valid"] = 0
    if len(payload) in (50, 80, 84):
        (
            bus_rx_frames,
            accepted_frames,
            rejected_format_frames,
            rejected_master_id_frames,
            rejected_feedback_id_frames,
        ) = struct.unpack_from("<5I", payload, 25)
        last_can_id, last_dlc, last_data0, last_result = struct.unpack_from(
            "<HBBB", payload, 45
        )
        result["j4310_rx_diagnostic"] = {
            "bus_rx_frames": bus_rx_frames,
            "accepted_frames": accepted_frames,
            "rejected_format_frames": rejected_format_frames,
            "rejected_master_id_frames": rejected_master_id_frames,
            "rejected_feedback_id_frames": rejected_feedback_id_frames,
            "last_can_id": last_can_id,
            "last_dlc": last_dlc,
            "last_data0": last_data0,
            "last_result": last_result,
        }
    else:
        result["j4310_rx_diagnostic"] = None
    if len(payload) in (80, 84):
        (
            attempted_frames,
            queued_frames,
            failed_frames,
            enable_frames,
            mit_frames,
            disable_frames,
        ) = struct.unpack_from("<6I", payload, 50)
        (
            last_can_id,
            last_dlc,
            last_data7,
            enable_confirmed,
            feedback_state,
        ) = struct.unpack_from("<HBBBB", payload, 74)
        result["j4310_tx_diagnostic"] = {
            "attempted_frames": attempted_frames,
            "queued_frames": queued_frames,
            "failed_frames": failed_frames,
            "enable_frames": enable_frames,
            "mit_frames": mit_frames,
            "disable_frames": disable_frames,
            "last_can_id": last_can_id,
            "last_dlc": last_dlc,
            "last_data7": last_data7,
            "enable_confirmed": bool(enable_confirmed),
            "feedback_state": feedback_state,
        }
    else:
        result["j4310_tx_diagnostic"] = None
    if len(payload) == 84:
        available, enabled, active, stage = struct.unpack_from(
            "<BBBB", payload, 80
        )
        result["j4310_auto_return"] = {
            "available": bool(available),
            "enabled": bool(enabled),
            "active": bool(active),
            "stage": stage,
        }
    else:
        result["j4310_auto_return"] = None
    diagnostics = []
    if len(payload) in (89, 101):
        for offset in range(25, 89, 16):
            model, can_bus, node_id, flags = struct.unpack_from(
                "<BBBB", payload, offset
            )
            rotor_rad, zero_rotor_rad, relative_output_rad = struct.unpack_from(
                "<3f", payload, offset + 4
            )
            diagnostics.append(
                {
                    "model": model,
                    "can_bus": can_bus,
                    "node_id": node_id,
                    "feedback_received": bool(flags & 0x01),
                    "zero_valid": bool(flags & 0x02),
                    "feedback_fresh": bool(flags & 0x04),
                    "rotor_position_rad": rotor_rad,
                    "zero_rotor_position_rad": zero_rotor_rad,
                    "relative_output_position_rad": relative_output_rad,
                }
            )
    result["dji_diagnostics"] = diagnostics
    result["fdcan_rx_counts"] = (
        struct.unpack_from("<3I", payload, 89) if len(payload) == 101 else None
    )
    return result


def decode_dji_telemetry(payload: bytes) -> dict[str, object]:
    """Decode one short, independently refreshable DJI feedback frame."""

    if len(payload) != 28:
        raise ValueError("DJI telemetry payload must be 28 bytes")
    model, can_bus, node_id, flags = struct.unpack_from("<BBBB", payload)
    rotor_rad, zero_rotor_rad, relative_output_rad = struct.unpack_from(
        "<3f", payload, 4
    )
    return {
        "diagnostic": {
            "model": model,
            "can_bus": can_bus,
            "node_id": node_id,
            "feedback_received": bool(flags & 0x01),
            "zero_valid": bool(flags & 0x02),
            "feedback_fresh": bool(flags & 0x04),
            "rotor_position_rad": rotor_rad,
            "zero_rotor_position_rad": zero_rotor_rad,
            "relative_output_position_rad": relative_output_rad,
        },
        "fdcan_rx_counts": struct.unpack_from("<3I", payload, 16),
    }


def decode_motor_event(payload: bytes) -> dict[str, int]:
    if len(payload) != 8:
        raise ValueError("motor event payload must be 8 bytes")
    value, can_bus, node_id, code, tick_ms = struct.unpack("<BBBBI", payload)
    return {
        "value": value,
        "can_bus": can_bus,
        "node_id": node_id,
        "code": code,
        "tick_ms": tick_ms,
    }


def decode_motor_action_result(payload: bytes) -> dict[str, int]:
    """Decode the board acknowledgement for a one-shot motor action."""

    if len(payload) != 8:
        raise ValueError("motor action result payload must be 8 bytes")
    action, can_bus, node_id, status, tick_ms = struct.unpack("<BBBBI", payload)
    return {
        "action": action,
        "can_bus": can_bus,
        "node_id": node_id,
        "status": status,
        "tick_ms": tick_ms,
    }
