# -*- coding: utf-8 -*-

import math
import queue
import struct
import tempfile
import tkinter as tk
import unittest
from pathlib import Path
from tkinter import ttk
from unittest import mock

import main
from main import (
    DJI_REDUCTION_RATIOS,
    GATE_M2006_ID,
    GRIPPER_M2006_ID,
    J4310_RAW_MAX_DEG,
    M2006_NODE_IDS,
    POSITION_MAX_DEG,
    POSITION_MIN_DEG,
    SerialTransport,
    UpperConsole,
)
from protocol import (
    COMMAND_J4310_STOP,
    ENABLE_CONVEYOR,
    ENABLE_GRIPPER,
    ENABLE_J4310_ONLY,
    ENABLE_M3508_ONLY,
    FRAME_OVERHEAD,
    HANDSHAKE_MAGIC,
    HEADER_SIZE,
    MSG_ACK,
    MSG_ESTOP,
    MSG_FLASH_INFO,
    MSG_FLASH_INFO_REQUEST,
    MSG_HEARTBEAT,
    MOTOR_ACTION_J4310_SAVE_ZERO,
    MOTOR_ACTION_J4310_AUTO_RETURN,
    MSG_MOTOR_ACTION,
    MSG_MOTOR_ACTION_RESULT,
    MSG_DJI_TELEMETRY,
    MSG_ROBOT_STATE,
    MSG_UPPER_POSITION_CMD,
    Frame,
    FrameParser,
    decode_dji_telemetry,
    decode_flash_info,
    decode_motor_action_result,
    decode_robot_state,
    encode_frame,
)


class UpperConsoleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = tk.Tk()
        self.root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            self.app = UpperConsole(self.root)
        self.root.update_idletasks()

    def tearDown(self) -> None:
        self.app.close()

    def test_m3508_position_targets_are_independent(self) -> None:
        self.app.m3508_position_1.set(10.0)
        self.app.m3508_position_2.set(20.0)
        self.app.conveyor_position.set(30.0)
        self.app.gripper_position.set(40.0)

        self.assertEqual(self.app._values()[4:], (10.0, 20.0, 30.0, 40.0))

    def test_m2006_node_ids_match_the_mechanisms(self) -> None:
        self.assertEqual(GATE_M2006_ID, 1)
        self.assertEqual(GRIPPER_M2006_ID, 2)
        self.assertEqual(M2006_NODE_IDS["conveyor"], GATE_M2006_ID)
        self.assertEqual(M2006_NODE_IDS["gripper"], GRIPPER_M2006_ID)

    def test_each_page_uses_only_its_enable_bits(self) -> None:
        self.app.arm_enable.set(True)
        self.app.m3508_enable.set(True)
        self.assertEqual(
            self.app._mask_for_page("机械臂"),
            ENABLE_J4310_ONLY | ENABLE_M3508_ONLY,
        )

        self.app.conveyor_enable.set(True)
        self.app.gripper_enable.set(True)
        self.assertEqual(self.app._mask_for_page("开关门"), ENABLE_CONVEYOR)
        self.assertEqual(self.app._mask_for_page("夹爪"), ENABLE_GRIPPER)

    def test_position_frame_keeps_protocol_motor_targets(self) -> None:
        self.app.m3508_enable.set(True)
        values = (0.0, 2.0, 20.0, 0.5, 10.0, 20.0, 30.0, 40.0)

        frame = self.app._build_position_frame("机械臂", values)
        payload_size = struct.unpack_from("<H", frame, 6)[0]
        payload = frame[HEADER_SIZE:FRAME_OVERHEAD + payload_size - 2]
        unpacked = struct.unpack("<H30f", payload)

        self.assertEqual(unpacked[0], ENABLE_M3508_ONLY)
        for actual, expected in zip(unpacked[7:11], values[4:]):
            self.assertAlmostEqual(actual, math.radians(expected), places=6)

    def test_each_m2006_page_sends_only_its_own_node(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.conveyor_position.set(0.0)
        self.app.gripper_position.set(45.0)
        self.app.conveyor_enable.set(True)
        self.app.gripper_enable.set(True)

        self.app.send_page_now("开关门")

        gate = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(gate[0], ENABLE_CONVEYOR)
        self.assertAlmostEqual(gate[9], 0.0, places=7)
        self.assertFalse(self.app.gripper_enable.get())

        self.app.conveyor_position.set(30.0)
        self.app.gripper_position.set(0.0)
        self.app.gripper_enable.set(True)
        self.app.send_page_now("夹爪")

        gripper = self._decode_position_frame(
            transport.write.call_args.args[0]
        )
        self.assertEqual(gripper[0], ENABLE_GRIPPER)
        self.assertAlmostEqual(gripper[10], 0.0, places=7)
        self.assertFalse(self.app.conveyor_enable.get())

    @staticmethod
    def _decode_position_frame(frame: bytes) -> tuple[float, ...]:
        payload_size = struct.unpack_from("<H", frame, 6)[0]
        payload = frame[HEADER_SIZE:FRAME_OVERHEAD + payload_size - 2]
        return struct.unpack("<H30f", payload)

    def test_input_values_only_stage_targets(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True

        self.app.j_position.set(30.0)
        self.app.m3508_position_1.set(-35.0)
        self.app.m3508_position_2.set(35.0)
        self.app.conveyor_position.set(40.0)
        self.app.gripper_position.set(-45.0)

        transport.write.assert_not_called()
        self.assertEqual(self.app.j_position.get(), 30.0)
        self.assertEqual(self.app.m3508_position_1.get(), -35.0)
        self.assertEqual(self.app.m3508_position_2.get(), 35.0)
        self.assertEqual(self.app.conveyor_position.get(), 40.0)
        self.assertEqual(self.app.gripper_position.get(), -45.0)
        self.assertFalse(self.app.arm_enable.get())
        self.assertFalse(self.app.m3508_enable.get())
        self.assertFalse(self.app.conveyor_enable.get())
        self.assertFalse(self.app.gripper_enable.get())

    def test_real_slider_mouse_drag_sends_and_enables(self) -> None:
        self.root.deiconify()
        self.root.update()
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        scale = self.app.j_position_controls[1]
        middle_y = scale.winfo_height() // 2

        scale.event_generate(
            "<ButtonPress-1>", x=scale.winfo_width() // 2, y=middle_y
        )
        scale.event_generate(
            "<B1-Motion>", x=scale.winfo_width() * 3 // 4, y=middle_y
        )
        scale.event_generate(
            "<ButtonRelease-1>", x=scale.winfo_width() * 3 // 4, y=middle_y
        )

        sent = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(sent[0], ENABLE_J4310_ONLY)
        self.assertGreater(sent[1], 0.1)
        self.assertTrue(self.app.arm_enable.get())
        self.assertIsNone(self.app._drag_target)

    def test_each_slider_directly_controls_its_motor(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        cases = (
            ("j4310", ENABLE_J4310_ONLY, self.app.arm_enable),
            ("m3508", ENABLE_M3508_ONLY, self.app.m3508_enable),
            ("conveyor", ENABLE_CONVEYOR, self.app.conveyor_enable),
            ("gripper", ENABLE_GRIPPER, self.app.gripper_enable),
        )

        for target, expected_mask, enable_var in cases:
            with self.subTest(target=target):
                self.app._disable_all_outputs()
                transport.reset_mock()

                self.app._start_slider_drag(target)

                sent = self._decode_position_frame(
                    transport.write.call_args.args[0]
                )
                self.assertEqual(sent[0], expected_mask)
                self.assertTrue(enable_var.get())
                self.app._finish_slider_drag(target)

    def test_drag_period_sends_latest_slider_position(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        self.app.conveyor_enable.set(True)
        self.app._start_slider_drag("conveyor")
        transport.reset_mock()
        self.app.conveyor_position.set(75.0)
        self.app.last_heartbeat = 1.0
        send_after_id = self.app._send_after_id

        with (
            mock.patch("main.time.monotonic", return_value=1.01),
            mock.patch.object(self.root, "after"),
        ):
            self.app._periodic_send()
        self.app._send_after_id = send_after_id

        sent = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(sent[0], ENABLE_CONVEYOR)
        self.assertAlmostEqual(sent[9], math.radians(75.0), places=6)

    def test_manual_output_controls_remain_available(self) -> None:
        self.assertEqual(self.app.j4310_enable_check.cget("text"), "使能 J4310 输出")
        self.assertEqual(self.app.j4310_send_button.cget("text"), "发送目标")
        self.assertEqual(self.app.j4310_stop_button.cget("text"), "停止发送")
        self.assertEqual(self.app.m3508_enable_check.cget("text"), "允许 M3508 输出")
        self.assertEqual(self.app.m3508_send_button.cget("text"), "发送目标")
        self.assertEqual(self.app.m3508_stop_button.cget("text"), "停止发送")

    def test_space_key_sends_estop_once_until_released(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.arm_enable.set(True)
        self.app.m3508_enable.set(True)
        self.app.conveyor_enable.set(True)
        self.app.gripper_enable.set(True)

        self.assertEqual(self.app._on_space_estop(mock.Mock()), "break")
        frame = transport.write.call_args.args[0]
        self.assertEqual(frame[3], MSG_ESTOP)
        self.assertEqual(frame[HEADER_SIZE:HEADER_SIZE + 1], b"\x01")
        self.assertFalse(self.app.arm_enable.get())
        self.assertFalse(self.app.m3508_enable.get())
        self.assertFalse(self.app.conveyor_enable.get())
        self.assertFalse(self.app.gripper_enable.get())

        self.app._on_space_estop(mock.Mock())
        self.assertEqual(transport.write.call_count, 1)
        self.app._on_space_estop_released(mock.Mock())
        self.app._on_space_estop(mock.Mock())
        self.assertEqual(transport.write.call_count, 2)

    def test_j4310_and_m3508_send_buttons_use_independent_masks(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True

        self.app.arm_enable.set(True)
        self.app.send_arm_group_now("j4310")
        j4310_frame = self._decode_position_frame(
            transport.write.call_args.args[0]
        )

        self.app.m3508_enable.set(True)
        self.app.send_arm_group_now("m3508")
        m3508_frame = self._decode_position_frame(
            transport.write.call_args.args[0]
        )

        self.assertEqual(j4310_frame[0], ENABLE_J4310_ONLY)
        self.assertEqual(
            m3508_frame[0], ENABLE_J4310_ONLY | ENABLE_M3508_ONLY
        )

    def test_stopping_j4310_keeps_last_sent_m3508_target(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.m3508_position_1.set(45.0)
        self.app.m3508_position_2.set(-30.0)
        self.app.m3508_enable.set(True)
        self.app.send_arm_group_now("m3508")
        self.app.arm_enable.set(True)
        self.app.send_arm_group_now("j4310")

        self.app.m3508_position_1.set(90.0)
        self.app.m3508_position_2.set(90.0)
        self.app.stop_arm_group("j4310")

        stopped = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(
            stopped[0], ENABLE_M3508_ONLY | COMMAND_J4310_STOP
        )
        self.assertAlmostEqual(stopped[7], math.radians(45.0), places=6)
        self.assertAlmostEqual(stopped[8], math.radians(-30.0), places=6)
        self.assertFalse(self.app.arm_enable.get())
        self.assertTrue(self.app.m3508_enable.get())

    def test_clearing_j4310_enable_does_not_send(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.arm_enable.set(True)
        self.app.m3508_enable.set(True)

        self.app.arm_enable.set(False)
        self.root.update_idletasks()

        self.assertFalse(self.app.arm_enable.get())
        self.assertTrue(self.app.m3508_enable.get())
        transport.write.assert_not_called()

    def test_clearing_m3508_enable_does_not_send(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.arm_enable.set(True)
        self.app.m3508_enable.set(True)

        self.app.m3508_enable.set(False)
        self.root.update_idletasks()

        self.assertTrue(self.app.arm_enable.get())
        self.assertFalse(self.app.m3508_enable.get())
        transport.write.assert_not_called()

    def test_new_arm_target_does_not_restore_revoked_group(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.m3508_enable.set(True)
        self.app.send_arm_group_now("m3508")

        self.app.m3508_enable.set(False)
        self.app.arm_enable.set(True)
        self.app.send_arm_group_now("j4310")

        sent = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(sent[0], ENABLE_J4310_ONLY)

    def test_dji_relative_targets_accept_both_directions(self) -> None:
        self.app.m3508_position_1.set(-90.0)
        self.app.m3508_position_2.set(90.0)
        self.app.conveyor_position.set(-45.0)

        self.assertEqual(self.app.m3508_position_1.get(), -90.0)
        self.assertEqual(self.app.m3508_position_2.get(), 90.0)
        self.assertEqual(self.app.conveyor_position.get(), -45.0)

    def test_all_position_sliders_use_full_rotation_range(self) -> None:
        self.assertEqual(POSITION_MIN_DEG, -360.0)
        self.assertEqual(POSITION_MAX_DEG, 360.0)
        self.assertEqual(
            set(self.app.position_slider_controls),
            {"j4310", "m3508", "conveyor", "gripper"},
        )
        for controls in self.app.position_slider_controls.values():
            for control in controls:
                self.assertEqual(float(control.cget("from")), POSITION_MIN_DEG)
                self.assertEqual(float(control.cget("to")), POSITION_MAX_DEG)

    def test_m3508_has_two_inputs_and_one_shared_slider(self) -> None:
        controls = self.app.m3508_position_controls

        self.assertEqual(len(controls), 3)
        self.assertIsInstance(controls[0], ttk.Spinbox)
        self.assertIsInstance(controls[1], ttk.Spinbox)
        self.assertIsInstance(controls[2], ttk.Scale)

    def test_slider_value_change_without_drag_does_not_send(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.m3508_enable.set(True)

        for position in (35.0, -35.0):
            with self.subTest(position=position):
                self.app._on_m3508_sync_slider_value(str(position))
                transport.write.assert_not_called()
                self.assertEqual(
                    self.app.m3508_position_1.get(), -position
                )
                self.assertEqual(
                    self.app.m3508_position_2.get(), position
                )

    def test_m3508_sync_slider_keeps_opposite_targets_with_equal_magnitude(self) -> None:
        for value in (-360.0, -12.5, 0.0, 42.0, 360.0):
            self.app._on_m3508_sync_slider_value(str(value))
            self.assertEqual(
                self.app.m3508_position_1.get(), -value if value else 0.0
            )
            self.assertEqual(self.app.m3508_position_2.get(), value)

    def test_missing_m3508_feedback_keeps_angle_placeholders(self) -> None:
        diagnostics = [
            {
                "model": 1,
                "can_bus": 2,
                "node_id": node_id,
                "feedback_received": False,
                "zero_valid": False,
                "feedback_fresh": False,
                "rotor_position_rad": 0.0,
                "zero_rotor_position_rad": 0.0,
                "relative_output_position_rad": 0.0,
            }
            for node_id in (1, 2)
        ]

        self.app._update_dji_diagnostics(diagnostics, (0, 0, 0))

        self.assertEqual(
            self.app.dji_diagnostic_vars[(1, 2, 1)].get(),
            "M3508 #1\n"
            "当前输出轴角度: -- deg\n"
            "相对标定零点角度: -- deg\n"
            "是否标零: 否\n"
            "反馈状态: 未收到反馈",
        )
        self.assertEqual(
            self.app.dji_diagnostic_vars[(1, 2, 2)].get(),
            "M3508 #2\n"
            "当前输出轴角度: -- deg\n"
            "相对标定零点角度: -- deg\n"
            "是否标零: 否\n"
            "反馈状态: 未收到反馈",
        )

        self.app._update_dji_diagnostics(diagnostics, (0, 17, 0))

        self.assertNotIn(
            "FDCAN", self.app.dji_diagnostic_vars[(1, 2, 1)].get()
        )
        self.assertNotIn(
            "0x202", self.app.dji_diagnostic_vars[(1, 2, 2)].get()
        )

    def test_missing_m2006_feedback_keeps_angle_placeholders(self) -> None:
        diagnostics = [
            {
                "model": 2,
                "can_bus": 3,
                "node_id": node_id,
                "feedback_received": False,
                "zero_valid": False,
                "feedback_fresh": False,
                "rotor_position_rad": 0.0,
                "zero_rotor_position_rad": 0.0,
                "relative_output_position_rad": 0.0,
            }
            for node_id in (1, 2)
        ]

        self.app._update_dji_diagnostics(diagnostics, (0, 0, 0))

        self.assertEqual(
            self.app.dji_diagnostic_vars[(2, 3, 1)].get(),
            "开关门 M2006\n"
            "当前输出轴角度: -- deg\n"
            "相对标定零点角度: -- deg\n"
            "是否标零: 否\n"
            "反馈状态: 未收到反馈",
        )

        self.app._update_dji_diagnostics(diagnostics, (0, 0, 23))

        self.assertNotIn(
            "原始 RX", self.app.dji_diagnostic_vars[(2, 3, 1)].get()
        )
        self.assertNotIn(
            "检查", self.app.dji_diagnostic_vars[(2, 3, 2)].get()
        )

    def test_received_motor_shows_current_output_before_zero(self) -> None:
        diagnostic = {
            "model": 2,
            "can_bus": 3,
            "node_id": 1,
            "feedback_received": True,
            "zero_valid": False,
            "feedback_fresh": True,
            "rotor_position_rad": 1.25,
            "zero_rotor_position_rad": 0.0,
            "relative_output_position_rad": 0.0,
        }

        self.app._update_dji_diagnostics([diagnostic], clear_missing=False)

        feedback_text = self.app.dji_diagnostic_vars[(2, 3, 1)].get()
        current_output_deg = math.degrees(1.25 / DJI_REDUCTION_RATIOS[2])
        self.assertIn(
            f"当前输出轴角度: {current_output_deg:.2f} deg", feedback_text
        )
        self.assertIn("相对标定零点角度: -- deg", feedback_text)
        self.assertIn("是否标零: 否", feedback_text)
        self.assertIn("反馈状态: 正常，等待稳定标零", feedback_text)

    def test_begin_handshake_clears_stale_motor_feedback(self) -> None:
        self.app.dji_diagnostic_vars[(2, 3, 1)].set(
            "开关门 M2006\n当前电机反馈角度: 88.00 deg"
        )

        self.app._begin_handshake(10.0)

        feedback_text = self.app.dji_diagnostic_vars[(2, 3, 1)].get()
        self.assertIn("当前输出轴角度: -- deg", feedback_text)
        self.assertIn("相对标定零点角度: -- deg", feedback_text)
        self.assertIn("是否标零: --", feedback_text)
        self.assertIn("反馈状态: 等待握手后反馈", feedback_text)

    def test_m3508_send_button_uses_two_independent_targets(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.m3508_enable.set(True)
        self.app.m3508_position_1.set(12.0)
        self.app.m3508_position_2.set(-34.0)

        self.app.send_arm_group_now("m3508")

        sent = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(sent[0], ENABLE_M3508_ONLY)
        self.assertAlmostEqual(sent[7], math.radians(12.0), places=6)
        self.assertAlmostEqual(sent[8], math.radians(-34.0), places=6)

    def test_minimum_window_keeps_page_and_log_in_separate_regions(self) -> None:
        self.root.deiconify()
        self.root.geometry("1180x900")
        self.root.update()
        page = self.app.page_frames["机械臂"]
        canvas = next(
            widget for widget in page.winfo_children() if isinstance(widget, tk.Canvas)
        )
        log_frame = self.app.log_text.master

        self.assertLessEqual(
            page.winfo_y() + page.winfo_height(),
            log_frame.winfo_y(),
        )
        self.assertGreater(int(canvas.cget("scrollregion").split()[3]), canvas.winfo_height())

    def test_mousewheel_over_page_control_scrolls_the_control_page(self) -> None:
        self.root.deiconify()
        self.root.geometry("1180x900")
        self.root.update()
        page = self.app.page_frames["机械臂"]
        canvas = next(
            widget for widget in page.winfo_children() if isinstance(widget, tk.Canvas)
        )
        canvas.yview_moveto(0.0)

        self.app.m3508_position_1_spinbox.event_generate(
            "<MouseWheel>", delta=-120
        )
        self.root.update()

        self.assertGreater(canvas.yview()[0], 0.0)

    def test_m3508_feedback_panels_are_horizontal(self) -> None:
        first, second = self.app.m3508_feedback_labels

        self.assertEqual(int(first.grid_info()["row"]), 0)
        self.assertEqual(int(second.grid_info()["row"]), 0)
        self.assertEqual(int(first.grid_info()["column"]), 0)
        self.assertEqual(int(second.grid_info()["column"]), 1)

    def test_log_panel_uses_reduced_height(self) -> None:
        self.assertEqual(int(self.app.log_text.master.cget("height")), 150)

    def test_handshake_retries_keep_the_original_sequence(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport

        self.app._send_handshake()
        first_sequence = self.app.handshake_sequence
        first_frame = transport.write.call_args.args[0]
        self.app._send_handshake()

        self.assertIsNotNone(first_sequence)
        self.assertEqual(self.app.handshake_sequence, first_sequence)
        self.assertEqual(transport.write.call_args.args[0], first_frame)
        self.assertEqual(first_sequence, 0x1234)
        self.assertEqual(
            first_frame,
            bytes.fromhex(
                "A5 5A 03 03 34 12 04 00 48 37 32 33 56 81"
            ),
        )

    def test_handshake_automatically_queries_flash_once(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshake_sequence = 0x1234

        self.app._accept_handshake_ack(
            Frame(MSG_ACK, 0x1234, HANDSHAKE_MAGIC)
        )

        written = transport.write.call_args.args[0]
        self.assertTrue(self.app.handshaken)
        self.assertEqual(written[3], MSG_FLASH_INFO_REQUEST)
        self.assertEqual(struct.unpack_from("<H", written, 6)[0], 0)
        self.assertEqual(self.app.flash_info_var.get(), "Flash: 查询中...")

    def test_manual_flash_query_uses_binary_protocol(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True

        self.assertTrue(self.app.request_flash_info())

        written = transport.write.call_args.args[0]
        self.assertEqual(written[3], MSG_FLASH_INFO_REQUEST)
        self.assertEqual(struct.unpack_from("<H", written, 6)[0], 0)

    def test_handshake_timeout_keeps_wireless_cdc_open(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = False
        self.app.handshake_started_at = 1.0
        self.app.last_handshake = 1.0
        self.app.handshake_sequence = 0x1234
        send_after_id = self.app._send_after_id

        with (
            mock.patch("main.time.monotonic", return_value=5.0),
            mock.patch.object(self.app, "disconnect") as disconnect,
            mock.patch.object(self.app, "_send_handshake") as send_handshake,
            mock.patch.object(self.root, "after"),
        ):
            self.app._periodic_send()
        self.app._send_after_id = send_after_id

        disconnect.assert_not_called()
        send_handshake.assert_called_once_with()
        self.assertTrue(self.app.handshake_timeout_reported)
        self.assertEqual(self.app.handshake_sequence, 0x1234)

    def test_disconnect_keeps_wireless_cdc_open(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True

        self.app.disconnect()

        transport.close.assert_not_called()
        self.assertFalse(self.app.connection_requested)
        self.assertFalse(self.app.handshaken)
        self.assertEqual(self.app.status_var.get(), "未连接")

    def test_reconnect_reuses_open_wireless_cdc(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.matches.return_value = True
        self.app.transport = transport
        selected = self.app.port_var.get()
        self.app._port_lookup = {selected: "COM9"}

        with mock.patch("main.time.monotonic", return_value=10.0):
            self.app.toggle_connection()

        transport.open.assert_not_called()
        transport.discard_input.assert_called_once_with()
        self.assertTrue(self.app.connection_requested)
        self.assertEqual(self.app.handshake_started_at, 10.0)

    def test_missing_board_state_restarts_handshake_without_revoking_enable(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        self.app.last_board_response = 1.0
        self.app.arm_enable.set(True)
        send_after_id = self.app._send_after_id

        with (
            mock.patch("main.time.monotonic", return_value=2.1),
            mock.patch.object(self.root, "after"),
        ):
            self.app._periodic_send()
        self.app._send_after_id = send_after_id

        self.assertFalse(self.app.handshaken)
        self.assertTrue(self.app.arm_enable.get())
        self.assertEqual(self.app.handshake_started_at, 2.1)
        self.assertEqual(self.app.status_var.get(), "主控无响应，重新握手中...")

    def test_robot_state_refreshes_board_response_time(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.rx_queue = queue.Queue()
        transport.error_queue = queue.Queue()
        payload = bytearray(101)
        struct.pack_into("<BBIHIIIfB", payload, 0, 1, 0, 100, 7, 8, 9, 10,
                         1.25, 1)
        transport.rx_queue.put(encode_frame(MSG_ROBOT_STATE, 3, bytes(payload)))
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        poll_after_id = self.app._poll_after_id

        with (
            mock.patch("main.time.monotonic", return_value=12.5),
            mock.patch.object(self.root, "after"),
        ):
            self.app._poll_io()
        self.app._poll_after_id = poll_after_id

        self.assertEqual(self.app.last_board_response, 12.5)

    def test_serial_failure_resets_connection_for_retry(self) -> None:
        transport = mock.Mock()
        transport.connected = False
        transport.rx_queue = queue.Queue()
        transport.error_queue = queue.Queue()
        transport.error_queue.put("串口接收失败: 设备已移除")
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        self.app.arm_enable.set(True)
        poll_after_id = self.app._poll_after_id

        with mock.patch.object(self.root, "after"):
            self.app._poll_io()
        self.app._poll_after_id = poll_after_id

        self.assertFalse(self.app.connection_requested)
        self.assertFalse(self.app.handshaken)
        self.assertFalse(self.app.arm_enable.get())
        self.assertEqual(str(self.app.connect_button.cget("state")), "normal")
        self.assertEqual(
            self.app.status_var.get(),
            "无线 DAP 已断开，请重新插入后连接",
        )

    def test_disconnected_session_does_not_send_estop(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.connection_requested = False
        self.app.handshaken = False

        self.app.estop()

        transport.write.assert_not_called()

    def test_periodic_send_never_sends_a_motor_target(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        self.app.arm_enable.set(True)
        self.app.m3508_enable.set(True)
        self.app.last_heartbeat = 0.0
        send_after_id = self.app._send_after_id

        with (
            mock.patch("main.time.monotonic", return_value=1.0),
            mock.patch.object(self.root, "after"),
        ):
            self.app._periodic_send()
        self.app._send_after_id = send_after_id

        written = transport.write.call_args.args[0]
        self.assertEqual(written[3], MSG_HEARTBEAT)
        self.assertNotEqual(written[3], MSG_UPPER_POSITION_CMD)
        self.assertEqual(transport.write.call_count, 1)

    def test_all_enable_checkboxes_do_not_send_motor_targets(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True

        for check in self.app.output_enable_checks.values():
            check.invoke()

        transport.write.assert_not_called()
        self.assertTrue(self.app.arm_enable.get())
        self.assertTrue(self.app.m3508_enable.get())
        self.assertTrue(self.app.conveyor_enable.get())
        self.assertTrue(self.app.gripper_enable.get())

    def test_j4310_waits_for_send_target_button(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        self.app.j_position.set(30.0)

        self.app.j4310_enable_check.invoke()
        transport.write.assert_not_called()

        self.app.j4310_send_button.invoke()

        sent = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(sent[0], ENABLE_J4310_ONLY)
        self.assertAlmostEqual(sent[1], math.radians(30.0), places=6)
        self.assertEqual(transport.write.call_count, 1)

    def test_j4310_target_stays_in_absolute_motor_range(self) -> None:
        self.assertAlmostEqual(J4310_RAW_MAX_DEG, math.degrees(12.5))

    def test_j4310_permanent_calibration_sends_motor_action(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.handshaken = True

        self.app.permanently_calibrate_j4310()

        frame = transport.write.call_args.args[0]
        self.assertEqual(frame[3], MSG_MOTOR_ACTION)
        self.assertEqual(
            frame[HEADER_SIZE:HEADER_SIZE + 3],
            bytes((MOTOR_ACTION_J4310_SAVE_ZERO, 1, 0x06)),
        )

    def test_j4310_auto_return_button_persists_board_setting(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.j4310_auto_return_status_received = True
        self.app.j4310_auto_return_storage_ready = True
        self.app.j4310_auto_return_enabled = False
        self.app._update_j4310_auto_return_button()

        self.app.j4310_auto_return_button.invoke()

        frame = transport.write.call_args.args[0]
        self.assertEqual(frame[3], MSG_MOTOR_ACTION)
        self.assertEqual(
            frame[HEADER_SIZE:HEADER_SIZE + 4],
            bytes((MOTOR_ACTION_J4310_AUTO_RETURN, 1, 0x06, 1)),
        )
        self.assertTrue(self.app.j4310_auto_return_pending)
        self.assertEqual(
            self.app.j4310_auto_return_button.cget("text"),
            "重启归零：设置中",
        )

        self.app._handle_motor_action_result(
            struct.pack(
                "<BBBBI",
                MOTOR_ACTION_J4310_AUTO_RETURN,
                1,
                0x06,
                0,
                123,
            )
        )
        self.assertTrue(self.app.j4310_auto_return_enabled)
        self.assertIsNone(self.app.j4310_auto_return_pending)
        self.assertEqual(
            self.app.j4310_auto_return_button.cget("text"),
            "重启归零：开启",
        )

    def test_j4310_auto_return_button_explains_missing_flash(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.j4310_auto_return_status_received = True
        self.app.j4310_auto_return_storage_ready = False
        self.app._update_j4310_auto_return_button()

        self.assertEqual(
            str(self.app.j4310_auto_return_button.cget("state")), "normal"
        )
        self.assertEqual(
            self.app.j4310_auto_return_button.cget("text"),
            "重启归零：Flash未就绪",
        )

        self.app.j4310_auto_return_button.invoke()

        transport.write.assert_not_called()
        self.assertEqual(
            self.app.status_var.get(),
            "J4310 重启归零不可用：Flash 未就绪",
        )

    def test_j4310_status_hides_bus_counters_and_shows_flash(self) -> None:
        rx_diagnostic = {
            "bus_rx_frames": 38105,
            "accepted_frames": 38105,
            "rejected_format_frames": 0,
            "rejected_master_id_frames": 0,
            "rejected_feedback_id_frames": 0,
            "last_can_id": 0x016,
            "last_dlc": 8,
            "last_data0": 0x16,
            "last_result": 1,
        }
        tx_diagnostic = {
            "attempted_frames": 38106,
            "queued_frames": 38106,
            "failed_frames": 0,
            "enable_frames": 1,
            "mit_frames": 38105,
            "disable_frames": 0,
            "last_can_id": 0x006,
            "last_dlc": 8,
            "last_data7": 0xFF,
            "enable_confirmed": True,
            "feedback_state": 1,
        }

        self.app._update_j4310_diagnostic(rx_diagnostic, tx_diagnostic)
        self.app._apply_j4310_auto_return_state(
            {"storage_ready": True, "enabled": False, "active": False,
             "stage": 0}
        )

        text = self.app.j4310_bus_label.cget("text")
        self.assertEqual(text, "反馈状态: 电机已使能\nFlash 是否就绪: 是")
        self.assertNotIn("TX 尝试", text)
        self.assertNotIn("最近 TX", text)
        self.assertNotIn("RX 38105", text)
        self.assertNotIn("拒绝明细", text)

    def test_stop_page_clears_enable_and_sends_zero_mask(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.arm_enable.set(True)
        self.app.m3508_enable.set(True)

        self.app.stop_page("机械臂")

        frame = transport.write.call_args.args[0]
        payload_size = struct.unpack_from("<H", frame, 6)[0]
        payload = frame[HEADER_SIZE:FRAME_OVERHEAD + payload_size - 2]
        self.assertEqual(frame[3], MSG_UPPER_POSITION_CMD)
        self.assertEqual(
            struct.unpack_from("<H", payload)[0], COMMAND_J4310_STOP
        )
        self.assertFalse(self.app.arm_enable.get())
        self.assertFalse(self.app.m3508_enable.get())

    def test_permanent_calibration_result_is_reported(self) -> None:
        payload = struct.pack(
            "<BBBBI", MOTOR_ACTION_J4310_SAVE_ZERO, 1, 0x06, 0, 123
        )

        self.app._handle_motor_action_result(payload)

        self.assertEqual(self.app.status_var.get(), "J4310 永久标定成功")

    def test_motor_fault_is_appended_to_motor_log(self) -> None:
        payload = struct.pack("<BBBBI", 1, 2, 0x01, 0xF0, 123)

        with tempfile.TemporaryDirectory() as temp_dir:
            log_path = Path(temp_dir) / "电机日志.log"
            log_path.write_text("已有记录\n", encoding="utf-8")
            with mock.patch.object(main, "MOTOR_LOG_PATH", log_path):
                self.app._handle_motor_fault(payload)

            contents = log_path.read_text(encoding="utf-8")

        self.assertTrue(contents.startswith("已有记录\n"))
        self.assertIn("M3508 | FDCAN2 | ID=0x01", contents)
        self.assertIn("error=0xF0 (反馈超时/离线)", contents)
        self.assertIn("board_tick_ms=123", contents)

    def test_failed_motor_action_is_appended_to_motor_log(self) -> None:
        payload = struct.pack(
            "<BBBBI", MOTOR_ACTION_J4310_SAVE_ZERO, 1, 0x06, 2, 456
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            log_path = Path(temp_dir) / "电机日志.log"
            with mock.patch.object(main, "MOTOR_LOG_PATH", log_path):
                self.app._handle_motor_action_result(payload)

            contents = log_path.read_text(encoding="utf-8")

        self.assertEqual(self.app.status_var.get(), "J4310 永久标定失败")
        self.assertIn("J4310 | FDCAN1 | ID=0x06", contents)
        self.assertIn("action=0x01, status=0x02 (永久标定失败", contents)
        self.assertIn("board_tick_ms=456", contents)

    def test_decode_permanent_calibration_result(self) -> None:
        payload = struct.pack(
            "<BBBBI", MOTOR_ACTION_J4310_SAVE_ZERO, 1, 0x06, 1, 456
        )

        result = decode_motor_action_result(payload)

        self.assertEqual(result["action"], MOTOR_ACTION_J4310_SAVE_ZERO)
        self.assertEqual(result["can_bus"], 1)
        self.assertEqual(result["node_id"], 0x06)
        self.assertEqual(result["status"], 1)
        self.assertEqual(result["tick_ms"], 456)

    def test_disconnect_does_not_send_a_motor_command(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.handshaken = True

        self.app.disconnect()

        transport.write.assert_not_called()
        transport.close.assert_not_called()

    def test_decode_robot_state_includes_j4310_output_position(self) -> None:
        payload = struct.pack("<BBIHIIIfB", 1, 0, 100, 7, 8, 9, 10, 1.25, 1)
        frame = encode_frame(MSG_ROBOT_STATE, 3, payload)
        parsed = FrameParser().feed(frame)[0]

        state = decode_robot_state(parsed.payload)

        self.assertEqual(state["j4310_position_valid"], 1)
        self.assertAlmostEqual(state["j4310_position_rad"], 1.25)

    def test_decode_flash_info_includes_init_result_and_geometry(self) -> None:
        payload = struct.pack(
            "<BBIIIHI", 0, 1, 0xEF4015, 2048, 512, 256, 4096
        )

        info = decode_flash_info(payload)

        self.assertEqual(info["init_status"], 0)
        self.assertTrue(info["initialized"])
        self.assertEqual(info["jedec_id"], 0xEF4015)
        self.assertEqual(info["capacity_kb"], 2048)
        self.assertEqual(info["sector_count"], 512)
        self.assertEqual(info["page_size_byte"], 256)
        self.assertEqual(info["sector_size_byte"], 4096)

    def test_flash_info_response_is_displayed(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.rx_queue = queue.Queue()
        transport.error_queue = queue.Queue()
        payload = struct.pack(
            "<BBIIIHI", 3, 0, 0, 0, 0, 0, 4096
        )
        transport.rx_queue.put(encode_frame(MSG_FLASH_INFO, 7, payload))
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        poll_after_id = self.app._poll_after_id

        with (
            mock.patch("main.time.monotonic", return_value=12.5),
            mock.patch.object(self.root, "after"),
        ):
            self.app._poll_io()
        self.app._poll_after_id = poll_after_id

        text = self.app.flash_info_var.get()
        self.assertIn("未初始化", text)
        self.assertIn("未知型号", text)
        self.assertIn("w25q_init_status=3 W25Q_ERROR_UNSUPPORTED_DEVICE", text)
        self.assertIn("JEDEC=0x000000", text)
        self.assertIn("容量=0 KB", text)
        self.assertEqual(self.app.last_board_response, 12.5)

    def test_decode_robot_state_includes_j4310_rx_diagnostic(self) -> None:
        payload = struct.pack(
            "<BBIHIIIfB5IHBBB",
            1, 0, 100, 7, 8, 9, 10, 1.25, 1,
            123, 100, 4, 5, 6, 0x016, 8, 0x06, 1,
        )

        state = decode_robot_state(payload)
        diagnostic = state["j4310_rx_diagnostic"]

        self.assertEqual(diagnostic["bus_rx_frames"], 123)
        self.assertEqual(diagnostic["accepted_frames"], 100)
        self.assertEqual(diagnostic["rejected_format_frames"], 4)
        self.assertEqual(diagnostic["rejected_master_id_frames"], 5)
        self.assertEqual(diagnostic["rejected_feedback_id_frames"], 6)
        self.assertEqual(diagnostic["last_can_id"], 0x016)
        self.assertEqual(diagnostic["last_dlc"], 8)
        self.assertEqual(diagnostic["last_data0"], 0x06)
        self.assertEqual(diagnostic["last_result"], 1)

    def test_decode_robot_state_includes_j4310_tx_diagnostic(self) -> None:
        payload = bytearray(80)
        struct.pack_into(
            "<BBIHIIIfB5IHBBB",
            payload,
            0,
            2, 1, 100, 7, 8, 9, 10, 1.25, 1,
            123, 100, 4, 5, 6, 0x016, 8, 0x16, 1,
        )
        struct.pack_into(
            "<6IHBBBB",
            payload,
            50,
            120, 118, 2, 5, 110, 3, 0x006, 8, 0xFC, 1, 1,
        )

        state = decode_robot_state(bytes(payload))
        diagnostic = state["j4310_tx_diagnostic"]

        self.assertEqual(diagnostic["attempted_frames"], 120)
        self.assertEqual(diagnostic["queued_frames"], 118)
        self.assertEqual(diagnostic["failed_frames"], 2)
        self.assertEqual(diagnostic["enable_frames"], 5)
        self.assertEqual(diagnostic["mit_frames"], 110)
        self.assertEqual(diagnostic["disable_frames"], 3)
        self.assertEqual(diagnostic["last_can_id"], 0x006)
        self.assertEqual(diagnostic["last_dlc"], 8)
        self.assertEqual(diagnostic["last_data7"], 0xFC)
        self.assertTrue(diagnostic["enable_confirmed"])
        self.assertEqual(diagnostic["feedback_state"], 1)

    def test_decode_robot_state_includes_j4310_auto_return(self) -> None:
        payload = bytearray(84)
        struct.pack_into(
            "<BBIHIIIfB", payload, 0, 2, 1, 100, 7, 8, 9, 10,
            1.25, 1,
        )
        struct.pack_into("<BBBB", payload, 80, 1, 1, 1, 2)

        state = decode_robot_state(bytes(payload))

        self.assertEqual(
            state["j4310_auto_return"],
            {
                "storage_ready": True,
                "enabled": True,
                "active": True,
                "stage": 2,
            },
        )

    def test_decode_robot_state_includes_four_dji_diagnostics(self) -> None:
        payload = bytearray(101)
        struct.pack_into("<BBIHIIIfB", payload, 0, 1, 0, 100, 7, 8, 9, 10,
                         1.25, 1)
        entries = (
            (1, 2, 1, 7, 1.0, 0.5, 0.1),
            (1, 2, 2, 7, 2.0, 1.5, 0.2),
            (2, 3, 1, 7, 3.0, 2.5, 0.3),
            (2, 3, 2, 1, 4.0, 0.0, 0.0),
        )
        for index, entry in enumerate(entries):
            struct.pack_into("<BBBB3f", payload, 25 + index * 16, *entry)
        struct.pack_into("<3I", payload, 89, 11, 22, 33)

        state = decode_robot_state(bytes(payload))

        diagnostics = state["dji_diagnostics"]
        self.assertEqual(len(diagnostics), 4)
        self.assertEqual(diagnostics[2]["model"], 2)
        self.assertEqual(diagnostics[2]["can_bus"], 3)
        self.assertEqual(diagnostics[2]["node_id"], 1)
        self.assertTrue(diagnostics[2]["zero_valid"])
        self.assertAlmostEqual(
            diagnostics[2]["relative_output_position_rad"], 0.3
        )
        self.assertFalse(diagnostics[3]["zero_valid"])
        self.assertEqual(state["fdcan_rx_counts"], (11, 22, 33))

    def test_decode_short_dji_telemetry(self) -> None:
        payload = struct.pack(
            "<BBBB3f3I", 2, 3, 1, 7, 1.0, 0.5, 0.25, 11, 22, 33
        )

        telemetry = decode_dji_telemetry(payload)

        diagnostic = telemetry["diagnostic"]
        self.assertEqual(diagnostic["model"], 2)
        self.assertEqual(diagnostic["can_bus"], 3)
        self.assertEqual(diagnostic["node_id"], 1)
        self.assertTrue(diagnostic["feedback_received"])
        self.assertTrue(diagnostic["zero_valid"])
        self.assertTrue(diagnostic["feedback_fresh"])
        self.assertAlmostEqual(diagnostic["relative_output_position_rad"], 0.25)
        self.assertEqual(telemetry["fdcan_rx_counts"], (11, 22, 33))

    def test_short_state_does_not_clear_split_dji_feedback(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.rx_queue = queue.Queue()
        transport.error_queue = queue.Queue()
        dji_payload = struct.pack(
            "<BBBB3f3I", 2, 3, 1, 7, 1.0, 0.5, 0.25, 11, 22, 33
        )
        state_payload = struct.pack(
            "<BBIHIIIfB", 1, 0, 100, 7, 8, 9, 10, 1.25, 1
        )
        transport.rx_queue.put(
            encode_frame(MSG_DJI_TELEMETRY, 3, dji_payload) +
            encode_frame(MSG_ROBOT_STATE, 4, state_payload)
        )
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        poll_after_id = self.app._poll_after_id

        with (
            mock.patch("main.time.monotonic", return_value=12.5),
            mock.patch.object(self.root, "after"),
        ):
            self.app._poll_io()
        self.app._poll_after_id = poll_after_id

        feedback_text = self.app.dji_diagnostic_vars[(2, 3, 1)].get()
        self.assertIn("是否标零: 是", feedback_text)
        current_output_deg = math.degrees(0.5 / 36.0 + 0.25)
        self.assertIn(
            f"当前输出轴角度: {current_output_deg:.2f} deg", feedback_text
        )
        self.assertIn(
            f"相对标定零点角度: {math.degrees(0.25):.2f} deg",
            feedback_text,
        )
        self.assertIn("反馈状态: 正常", feedback_text)
        self.assertEqual(self.app.last_board_response, 12.5)

    def test_four_dji_diagnostics_are_shown_independently(self) -> None:
        diagnostics = [
            {
                "model": model,
                "can_bus": can_bus,
                "node_id": node_id,
                "feedback_received": True,
                "zero_valid": True,
                "feedback_fresh": True,
                "rotor_position_rad": float(node_id),
                "zero_rotor_position_rad": float(node_id) + 0.25,
                "relative_output_position_rad": float(node_id) / 10.0,
            }
            for model, can_bus, node_id in (
                (1, 2, 1), (1, 2, 2), (2, 3, 1), (2, 3, 2)
            )
        ]

        self.app._update_dji_diagnostics(diagnostics)

        for key, variable in self.app.dji_diagnostic_vars.items():
            text = variable.get()
            model = key[0]
            node_id = key[2]
            relative_rad = float(node_id) / 10.0
            current_rad = (
                (float(node_id) + 0.25) / DJI_REDUCTION_RATIOS[model] +
                relative_rad
            )
            self.assertIn(
                f"当前输出轴角度: {math.degrees(current_rad):.2f} deg",
                text,
            )
            self.assertIn(
                "相对标定零点角度: "
                f"{math.degrees(relative_rad):.2f} deg",
                text,
            )
            self.assertIn("是否标零: 是", text)
            self.assertIn("反馈状态: 正常", text)

    def test_parser_keeps_a_split_first_sync_byte(self) -> None:
        parser = FrameParser()
        ack = encode_frame(MSG_ACK, 0x1234, HANDSHAKE_MAGIC)

        self.assertEqual(parser.feed(ack[:1]), [])
        frames = parser.feed(ack[1:])

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].msg_type, MSG_ACK)
        self.assertEqual(frames[0].sequence, 0x1234)
        self.assertEqual(frames[0].payload, HANDSHAKE_MAGIC)


class SerialTransportTest(unittest.TestCase):
    @mock.patch("main.serial.Serial")
    def test_open_disables_dtr_and_rts_before_open(self, serial_factory) -> None:
        port = serial_factory.return_value
        port.is_open = False
        transport = SerialTransport()

        transport.open("COM9", 115200)

        serial_factory.assert_called_once_with()
        self.assertFalse(port.dtr)
        self.assertFalse(port.rts)
        port.open.assert_called_once_with()

    def test_read_failure_releases_stale_device_handle(self) -> None:
        port = mock.Mock()
        port.is_open = True
        port.in_waiting = 0
        port.read.side_effect = OSError("设备已移除")
        transport = SerialTransport()
        transport._serial = port

        transport._reader(port)

        self.assertFalse(transport.connected)
        port.close.assert_called_once_with()
        self.assertIn("设备已移除", transport.error_queue.get_nowait())

    def test_write_failure_releases_stale_device_handle(self) -> None:
        port = mock.Mock()
        port.is_open = True
        port.write.side_effect = OSError("设备已移除")
        transport = SerialTransport()
        transport._serial = port

        self.assertFalse(transport.write(b"test"))

        self.assertFalse(transport.connected)
        port.close.assert_called_once_with()
        self.assertIn("设备已移除", transport.error_queue.get_nowait())


if __name__ == "__main__":
    unittest.main()
