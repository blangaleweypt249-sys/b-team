# -*- coding: utf-8 -*-

import json
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
    GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG,
    GRIPPER_M2006_ID,
    J4310_POSITION_MAX_DEG,
    J4310_POSITION_MIN_DEG,
    J4310_RAW_MAX_DEG,
    M3508_SYNC_POSITION_MAX_DEG,
    M3508_SYNC_POSITION_MIN_DEG,
    M2006_POSITION_MAX_DEG,
    M2006_POSITION_MIN_DEG,
    M2006_NODE_IDS,
    POSITION_MAX_DEG,
    POSITION_MIN_DEG,
    SerialTransport,
    UpperConsole,
)
from protocol import (
    AUX_OUTPUT_ARM_CYLINDER,
    AUX_OUTPUT_ESTOP,
    AUX_OUTPUT_GRIPPER_CYLINDER,
    AUX_OUTPUT_PUSH_CYLINDER,
    COMMAND_J4310_STOP,
    ENABLE_CONVEYOR,
    ENABLE_GRIPPER,
    ENABLE_J4310_ONLY,
    ENABLE_M3508_ONLY,
    FRAME_OVERHEAD,
    HANDSHAKE_MAGIC,
    HEADER_SIZE,
    MSG_ACK,
    MSG_AUX_CONTROL,
    MSG_ESTOP,
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
    decode_motor_action_result,
    decode_robot_state,
    encode_frame,
)


class UpperConsoleTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.parameter_path_patch = mock.patch.object(
            main,
            "MOTOR_PARAMETER_PATH",
            Path(self.temp_dir.name) / "motor_parameters.json",
        )
        self.parameter_path_patch.start()
        self.root = tk.Tk()
        self.root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            self.app = UpperConsole(self.root)
        self.root.update_idletasks()

    def tearDown(self) -> None:
        self.app.close()
        self.parameter_path_patch.stop()
        self.temp_dir.cleanup()

    def test_j4310_defaults_use_online_tuning_baseline(self) -> None:
        self.assertEqual(self.app._values()[:4], (90.0, 0.0, 30.0, 0.95))
        self.assertEqual(
            (self.app.j_tau.get(), self.app.j_torque_limit.get()),
            (0.0, 10.0),
        )

    def test_numeric_input_validator_enforces_configured_range(self) -> None:
        self.assertTrue(UpperConsole._validate_numeric_edit("-10", "-10", "10"))
        self.assertTrue(UpperConsole._validate_numeric_edit("10", "-10", "10"))
        self.assertTrue(UpperConsole._validate_numeric_edit("-", "-10", "10"))
        self.assertFalse(
            UpperConsole._validate_numeric_edit("10.01", "-10", "10")
        )
        self.assertFalse(
            UpperConsole._validate_numeric_edit("-10.01", "-10", "10")
        )
        self.assertFalse(UpperConsole._validate_numeric_edit("nan", "-10", "10"))

    def test_j4310_torque_limit_above_tmax_is_not_sent(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.j_torque_limit.set(20.0)

        with mock.patch("main.messagebox.showerror") as showerror:
            sent = self.app.send_arm_group_now("j4310")

        self.assertFalse(sent)
        transport.write.assert_not_called()
        showerror.assert_called_once()
        message = showerror.call_args.args[1]
        self.assertIn("J4310 力矩限幅", message)
        self.assertIn("0.1 .. 10 Nm", message)

    def test_j4310_tau_must_fit_selected_torque_limit(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.j_tau.set(6.0)
        self.app.j_torque_limit.set(5.0)

        with mock.patch("main.messagebox.showerror") as showerror:
            sent = self.app.send_arm_group_now("j4310")

        self.assertFalse(sent)
        transport.write.assert_not_called()
        self.assertIn("超过当前力矩限幅", showerror.call_args.args[1])

    def test_stop_command_is_not_blocked_by_invalid_target(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.j_torque_limit.set(20.0)

        with mock.patch("main.messagebox.showerror") as showerror:
            self.app.stop_page("机械臂")

        transport.write.assert_called_once()
        showerror.assert_not_called()

    def test_parameter_range_dialog_lists_j4310_torque_limits(self) -> None:
        with mock.patch("main.messagebox.showinfo") as showinfo:
            self.app.show_parameter_ranges()

        showinfo.assert_called_once()
        message = showinfo.call_args.args[1]
        self.assertIn("tau：-10 .. 10 Nm", message)
        self.assertIn("力矩限幅：0.1 .. 10 Nm", message)

    def test_m3508_defaults_match_original_parameters(self) -> None:
        self.assertEqual(
            tuple(var.get() for var in self.app.m3508_speed_pid_vars.values()),
            (120.0, 80.0, 0.0, 57.2958, 2458.0),
        )
        self.assertEqual(
            tuple(var.get() for var in self.app.m3508_position_pid_vars.values()),
            (100.0, 0.0, 0.0, 0.0, 150.0),
        )

    def test_save_button_is_immediately_right_of_stop_button(self) -> None:
        for stop_button, save_button in (
            (self.app.j4310_stop_button, self.app.j4310_save_button),
            (self.app.m3508_stop_button, self.app.m3508_save_button),
            (
                self.app.page_action_buttons["闸门"][1],
                self.app.page_save_buttons["闸门"],
            ),
            (
                self.app.page_action_buttons["夹爪"][1],
                self.app.page_save_buttons["夹爪"],
            ),
        ):
            siblings = stop_button.master.pack_slaves()
            self.assertEqual(
                siblings.index(save_button), siblings.index(stop_button) + 1
            )

    def test_save_j4310_persists_and_immediately_sends(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.j_position.set(25.0)
        self.app.j_kp.set(12.0)

        self.assertTrue(self.app.save_motor_parameters("j4310"))

        transport.write.assert_called_once()
        with main.MOTOR_PARAMETER_PATH.open("r", encoding="utf-8") as stream:
            saved = json.load(stream)
        self.assertEqual(saved["j4310"]["position_deg"], 25.0)
        self.assertEqual(saved["j4310"]["kp"], 12.0)
        self.assertEqual(self.app.status_var.get(), "机械臂 参数已保存并立即应用")

    def test_m2006_parameters_are_saved_and_restored_independently(self) -> None:
        self.app.m2006_speed_pid_vars_by_motor["conveyor"]["kp"].set(111.0)
        self.app.m2006_speed_pid_vars_by_motor["gripper"]["kp"].set(222.0)
        self.assertTrue(self.app.save_motor_parameters("conveyor"))
        self.assertTrue(self.app.save_motor_parameters("gripper"))

        other_root = tk.Toplevel(self.root)
        other_root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            other_app = UpperConsole(other_root)
        try:
            self.assertEqual(
                other_app.m2006_speed_pid_vars_by_motor["conveyor"]["kp"].get(),
                111.0,
            )
            self.assertEqual(
                other_app.m2006_speed_pid_vars_by_motor["gripper"]["kp"].get(),
                222.0,
            )
        finally:
            other_app.close()

    def test_remote_action_parameters_are_saved_and_restored(self) -> None:
        self.app.remote_action_angles["取地面块"]["m3508"].set(321.0)
        self.app.remote_action_angles["收块"]["j4310"].set(123.0)
        self.app.remote_action_angles["收块"]["j4310_second"].set(234.0)
        self.app.remote_action_angles["闸门"]["angle_off"].set(45.0)

        self.app.save_remote_action_parameters()

        self.assertEqual(
            self.app.status_var.get(), "遥控动作参数已保存，下次启动自动恢复"
        )
        other_root = tk.Toplevel(self.root)
        other_root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            other_app = UpperConsole(other_root)
        try:
            self.assertEqual(
                other_app.remote_action_angles["取地面块"]["m3508"].get(),
                321.0,
            )
            self.assertEqual(
                other_app.remote_action_angles["收块"]["j4310"].get(),
                123.0,
            )
            self.assertEqual(
                other_app.remote_action_angles["收块"]["j4310_second"].get(),
                234.0,
            )
            self.assertEqual(
                other_app.remote_action_angles["闸门"]["angle_off"].get(),
                45.0,
            )
        finally:
            other_app.close()

    def test_remote_action_defaults_match_physical_remote(self) -> None:
        expected = {
            "取地面块": (500.0, 90.0, 1050.0, 90.0),
            "取台阶块": (0.0, 90.0, 850.0, 90.0),
            "收块": (0.0, 165.0, 0.0, 240.0),
            "翻转/存块": (0.0, 40.0, 0.0, -20.0),
            "闸门": (180.0, 60.0),
            "夹爪": (55.0, 125.0),
        }

        for action, values in expected.items():
            with self.subTest(action=action):
                self.assertEqual(
                    tuple(
                        variable.get()
                        for variable in self.app.remote_action_angles[
                            action
                        ].values()
                    ),
                    values,
                )

    def test_previous_gate_label_is_restored(self) -> None:
        previous = {
            "remote_actions": {
                "\u5f00\u5173\u95e8": {
                    "angle_on": 170.0,
                    "angle_off": 50.0,
                },
            },
            "remote_actions_version": main.REMOTE_ACTION_PARAMETER_VERSION,
        }
        main.MOTOR_PARAMETER_PATH.write_text(
            json.dumps(previous, ensure_ascii=False), encoding="utf-8"
        )

        other_root = tk.Toplevel(self.root)
        other_root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            other_app = UpperConsole(other_root)
        try:
            self.assertEqual(
                tuple(
                    variable.get()
                    for variable in other_app.remote_action_angles["闸门"].values()
                ),
                (170.0, 50.0),
            )
        finally:
            other_app.close()

    def test_legacy_remote_actions_do_not_override_new_defaults(self) -> None:
        legacy = {
            "remote_actions": {
                "翻转/存块": {
                    "m3508": 0.0,
                    "j4310": 30.0,
                    "m3508_second": 0.0,
                    "j4310_second": -10.0,
                },
                "闸门": {
                    "angle_on": 175.0,
                    "angle_off": 65.0,
                },
            },
            "remote_actions_version": 3,
        }
        main.MOTOR_PARAMETER_PATH.write_text(
            json.dumps(legacy, ensure_ascii=False), encoding="utf-8"
        )

        other_root = tk.Toplevel(self.root)
        other_root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            other_app = UpperConsole(other_root)
        try:
            self.assertEqual(
                other_app.remote_action_angles["翻转/存块"]["j4310"].get(),
                40.0,
            )
            self.assertEqual(
                other_app.remote_action_angles["闸门"]["angle_on"].get(),
                180.0,
            )
            self.assertEqual(
                tuple(
                    variable.get()
                    for variable in other_app.remote_action_angles["闸门"].values()
                ),
                (180.0, 60.0),
            )
        finally:
            other_app.close()

    def test_version_five_keeps_other_actions_but_resets_gate_angles(self) -> None:
        previous = {
            "remote_actions": {
                "取地面块": {
                    "m3508": 321.0,
                    "j4310": 90.0,
                    "m3508_second": 1000.0,
                    "j4310_second": 90.0,
                },
                "闸门": {
                    "angle_on": 175.0,
                    "angle_off": 65.0,
                },
            },
            "remote_actions_version": 5,
        }
        main.MOTOR_PARAMETER_PATH.write_text(
            json.dumps(previous, ensure_ascii=False), encoding="utf-8"
        )

        other_root = tk.Toplevel(self.root)
        other_root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            other_app = UpperConsole(other_root)
        try:
            self.assertEqual(
                other_app.remote_action_angles["取地面块"]["m3508"].get(),
                321.0,
            )
            self.assertEqual(
                tuple(
                    variable.get()
                    for variable in other_app.remote_action_angles["闸门"].values()
                ),
                (180.0, 60.0),
            )
        finally:
            other_app.close()

    def test_version_seven_resets_pd11_to_two_stage_defaults(self) -> None:
        previous = {
            "remote_actions": {
                "收块": {
                    "m3508": 10.0,
                    "j4310": 180.0,
                },
            },
            "remote_actions_version": 7,
        }
        main.MOTOR_PARAMETER_PATH.write_text(
            json.dumps(previous, ensure_ascii=False), encoding="utf-8"
        )

        other_root = tk.Toplevel(self.root)
        other_root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            other_app = UpperConsole(other_root)
        try:
            self.assertEqual(
                tuple(
                    variable.get()
                    for variable in other_app.remote_action_angles["收块"].values()
                ),
                (0.0, 165.0, 0.0, 240.0),
            )
        finally:
            other_app.close()

    def test_version_eight_resets_gripper_action_order(self) -> None:
        previous = {
            "remote_actions": {
                "夹爪": {
                    "angle_on": 125.0,
                    "angle_off": 45.0,
                },
            },
            "remote_actions_version": 8,
        }
        main.MOTOR_PARAMETER_PATH.write_text(
            json.dumps(previous, ensure_ascii=False), encoding="utf-8"
        )

        other_root = tk.Toplevel(self.root)
        other_root.withdraw()
        with mock.patch.object(UpperConsole, "refresh_ports"):
            other_app = UpperConsole(other_root)
        try:
            self.assertEqual(
                tuple(
                    variable.get()
                    for variable in other_app.remote_action_angles["夹爪"].values()
                ),
                (55.0, 125.0),
            )
        finally:
            other_app.close()

    def test_pd13_and_pd12_actions_are_synchronous(self) -> None:
        expected = {
            "取地面块": ((500.0, 90.0), (1050.0, 90.0)),
            "取台阶块": ((0.0, 90.0), (850.0, 90.0)),
        }
        for action, targets in expected.items():
            sent_values = []
            with self.subTest(action=action), mock.patch.object(
                self.app,
                "_send_position_values",
                side_effect=lambda *args, **kwargs: (
                    sent_values.append(args[1]) or True
                ),
            ):
                self.assertTrue(self.app.execute_remote_action(action))
                self.assertTrue(self.app.execute_remote_action(action))
            self.assertEqual(
                [(values[4], values[0]) for values in sent_values],
                list(targets),
            )
            self.assertEqual(self.app.remote_action_states[action], 0)

    def test_pd8_first_delays_j4310_but_second_is_synchronous(self) -> None:
        self.app._sent_arm_values = (
            45.0, 0.0, 5.0, 0.5, 321.0, 321.0, 0.0, 0.0
        )
        with (
            mock.patch.object(
                self.app, "_send_position_values", return_value=True
            ) as send,
            mock.patch.object(
                self.root, "after", return_value="pd8-first"
            ) as after,
        ):
            self.assertTrue(self.app.execute_remote_action("翻转/存块"))
        first = send.call_args.args[1]
        self.assertEqual((first[4], first[5], first[0]), (0.0, 0.0, 45.0))
        self.assertEqual(send.call_args.kwargs["enable_mask"], ENABLE_M3508_ONLY)
        self.assertEqual(after.call_args.args[0], 500)
        self.assertEqual(after.call_args.args[2], "翻转/存块")
        self.assertEqual(after.call_args.args[3], "第一次动作")
        delayed_values = after.call_args.args[4]
        self.assertEqual((delayed_values[4], delayed_values[0]), (0.0, 40.0))

        with mock.patch.object(
            self.app, "_send_position_values", return_value=True
        ) as delayed_send:
            self.app._complete_remote_arm_j4310_delay(
                "翻转/存块", "第一次动作", delayed_values
            )
        self.assertEqual(
            delayed_send.call_args.kwargs["enable_mask"], ENABLE_J4310_ONLY
        )

        with (
            mock.patch.object(
                self.app, "_send_position_values", return_value=True
            ) as second_send,
            mock.patch.object(self.root, "after") as after,
        ):
            self.assertTrue(self.app.execute_remote_action("翻转/存块"))
        second = second_send.call_args.args[1]
        self.assertEqual((second[4], second[5], second[0]), (0.0, 0.0, -20.0))
        self.assertEqual(
            second_send.call_args.kwargs["enable_mask"],
            ENABLE_M3508_ONLY | ENABLE_J4310_ONLY,
        )
        after.assert_not_called()

    def test_pd8_first_makes_next_pd9_return_zero_once(self) -> None:
        self.app._sent_arm_values = (
            45.0, 0.0, 5.0, 0.5, 321.0, 321.0, 0.0, 0.0
        )
        sent_values = []
        with (
            mock.patch.object(
                self.app,
                "_send_position_values",
                side_effect=lambda *args, **kwargs: (
                    sent_values.append(args[1]) or True
                ),
            ),
            mock.patch.object(
                self.root, "after", return_value="pd8-first"
            ) as after,
        ):
            self.assertTrue(self.app.execute_remote_action("翻转/存块"))
            self.assertTrue(self.app.remote_pd9_zero_pending)
            self.assertTrue(self.app.execute_remote_action("闸门"))
            self.assertFalse(self.app.remote_pd9_zero_pending)
            self.assertTrue(self.app.execute_remote_action("闸门"))
            self.assertTrue(self.app.execute_remote_action("闸门"))

        self.assertEqual(
            [values[6] for values in sent_values[-3:]],
            [0.0, 180.0, 60.0],
        )
        self.assertEqual(self.app.remote_action_states["闸门"], 0)

    def test_pd8_second_resets_next_pd9_to_single_normal_action(self) -> None:
        sent_values = []
        self.app.remote_action_states["闸门"] = 1
        with (
            mock.patch.object(
                self.app,
                "_send_position_values",
                side_effect=lambda *args, **kwargs: (
                    sent_values.append(args[1]) or True
                ),
            ),
            mock.patch.object(
                self.root, "after", return_value="pd8-first"
            ) as after,
        ):
            self.assertTrue(self.app.execute_remote_action("翻转/存块"))
            self.assertTrue(self.app.execute_remote_action("翻转/存块"))
            self.assertFalse(self.app.remote_pd9_zero_pending)
            self.assertTrue(self.app.execute_remote_action("闸门"))

        self.assertEqual(sent_values[-1][6], 180.0)
        self.assertEqual(after.call_count, 1)

    def test_pd9_zero_resets_existing_toggle_phase_to_180(self) -> None:
        self.app.remote_action_states["闸门"] = 1
        sent_values = []
        with (
            mock.patch.object(
                self.app,
                "_send_position_values",
                side_effect=lambda *args, **kwargs: (
                    sent_values.append(args[1]) or True
                ),
            ),
            mock.patch.object(self.root, "after", return_value="pd8-first"),
        ):
            self.assertTrue(self.app.execute_remote_action("翻转/存块"))
            self.assertTrue(self.app.execute_remote_action("闸门"))
            self.assertTrue(self.app.execute_remote_action("闸门"))

        self.assertEqual([values[6] for values in sent_values[-2:]], [0.0, 180.0])

    def test_pd11_toggles_between_first_and_second_collect_actions(self) -> None:
        sent_values = []
        with (
            mock.patch.object(
                self.app,
                "_send_position_values",
                side_effect=lambda *args, **kwargs: (
                    sent_values.append((args[1], kwargs["enable_mask"])) or True
                ),
            ),
            mock.patch.object(
                self.root,
                "after",
                side_effect=("pd11-first-1", "pd11-first-2"),
            ) as after,
        ):
            self.assertTrue(self.app.execute_remote_action("收块"))
            self.assertTrue(self.app.execute_remote_action("收块"))

        self.assertEqual(len(sent_values), 2)
        for values, enable_mask in sent_values:
            self.assertEqual((values[4], values[5]), (0.0, 0.0))
            self.assertEqual(enable_mask, ENABLE_M3508_ONLY)
        self.assertEqual(after.call_count, 2)
        expected = (
            ("第一次动作", 165.0),
            ("第二次动作", 240.0),
        )
        for call, (state_text, j4310_angle) in zip(after.call_args_list, expected):
            self.assertEqual(call.args[0], 500)
            self.assertEqual(call.args[2], "收块")
            self.assertEqual(call.args[3], state_text)
            self.assertEqual((call.args[4][4], call.args[4][0]), (0.0, j4310_angle))
        self.assertEqual(self.app.remote_action_states["收块"], 0)
        self.assertEqual(
            self.app.remote_action_buttons["收块"].cget("text"), "执行"
        )

    def test_remote_controls_use_save_and_execute_button_labels(self) -> None:
        self.assertEqual(self.app.remote_parameters_save_button.cget("text"), "保存参数")
        self.assertEqual(self.app.remote_zero_button.cget("text"), "全部归零")
        zero_grid = self.app.remote_zero_button.grid_info()
        self.assertEqual(int(zero_grid["row"]), 9)
        self.assertEqual(int(zero_grid["column"]), 7)
        for button in self.app.aux_buttons.values():
            self.assertEqual(button.cget("text"), "执行")

    def test_aux_buttons_send_uart_bridge_output_state(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True

        self.assertEqual(self.app.aux_output_bits, AUX_OUTPUT_ARM_CYLINDER)
        self.assertIn("全部关闭", self.app.aux_status_var.get())

        expected_payloads = (
            (AUX_OUTPUT_ARM_CYLINDER, 0x00),
            (AUX_OUTPUT_PUSH_CYLINDER, 0x02),
            (AUX_OUTPUT_GRIPPER_CYLINDER, 0x06),
            (AUX_OUTPUT_ESTOP, 0x0E),
        )
        for output_bit, expected_bits in expected_payloads:
            self.app.toggle_aux_output(output_bit)
            frame = FrameParser().feed(transport.write.call_args.args[0])[0]
            self.assertEqual(frame.msg_type, MSG_AUX_CONTROL)
            self.assertEqual(frame.payload, bytes((expected_bits, output_bit)))

        self.app.aux_output_bits = next(iter(self.app.aux_buttons))
        self.app._update_aux_buttons()

        for button in self.app.aux_buttons.values():
            self.assertEqual(button.cget("text"), "执行")

    def test_remote_arm_actions_send_equal_m3508_targets(self) -> None:
        for action in ("取地面块", "翻转/存块"):
            with self.subTest(action=action):
                self.app.remote_action_angles[action]["m3508"].set(321.0)
                with mock.patch.object(
                    self.app, "_send_position_values", return_value=True
                ) as send:
                    self.assertTrue(self.app.execute_remote_action(action))

                values = send.call_args.args[1]
                self.assertEqual(values[4], 321.0)
                self.assertEqual(values[5], 321.0)

    def test_remote_zero_sends_all_motor_targets_in_one_frame(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.remote_action_states["取地面块"] = True
        self.app.remote_action_buttons["取地面块"].configure(text="第二次动作")

        self.assertTrue(self.app.return_all_motors_to_zero())

        transport.write.assert_called_once()
        frame = transport.write.call_args.args[0]
        payload_size = struct.unpack_from("<H", frame, 6)[0]
        self.assertEqual(payload_size, 42)
        payload = frame[HEADER_SIZE:HEADER_SIZE + payload_size]
        sent = struct.unpack("<H10f", payload)
        self.assertEqual(
            sent[0],
            ENABLE_J4310_ONLY
            | ENABLE_M3508_ONLY
            | ENABLE_CONVEYOR
            | ENABLE_GRIPPER,
        )
        zero_fields = (
            sent[1], sent[2], sent[5], sent[7], sent[8], sent[9], sent[10]
        )
        for target in zero_fields:
            self.assertAlmostEqual(target, 0.0, places=7)
        self.assertEqual(self.app.status_var.get(), "全部电机归零目标已发送")
        self.assertEqual(self.app.remote_action_states, {})
        for button in self.app.remote_action_buttons.values():
            self.assertEqual(button.cget("text"), "执行")
        self.assertEqual(self.app.j_position.get(), 0.0)
        self.assertEqual(self.app.m3508_position_1.get(), 0.0)
        self.assertEqual(self.app.m3508_position_2.get(), 0.0)
        self.assertEqual(self.app.conveyor_position.get(), 0.0)
        self.assertEqual(self.app.gripper_position.get(), 0.0)

    def test_remote_zero_does_not_change_state_when_not_connected(self) -> None:
        self.app.remote_action_states["取地面块"] = True

        self.assertFalse(self.app.return_all_motors_to_zero())

        self.assertEqual(self.app.remote_action_states, {"取地面块": True})
        self.assertEqual(self.app.status_var.get(), "未连接，目标未发送")

    def test_m2006_defaults_match_saved_parameters(self) -> None:
        self.assertEqual(
            tuple(var.get() for var in self.app.m2006_speed_pid_vars.values()),
            (350.0, 250.0, 0.0, 0.5, 10000.0),
        )
        self.assertEqual(
            tuple(var.get() for var in self.app.m2006_position_pid_vars.values()),
            (900.0, 500.0, 5.0, 0.002, 50.0),
        )

    def test_j4310_last_enabled_feedback_angle_is_retained(self) -> None:
        self.app._apply_j4310_position_feedback(True, math.radians(42.5))
        self.assertEqual(
            self.app.j4310_output_position_var.get(),
            "当前输出轴角度: 42.50 deg",
        )

        self.app._apply_j4310_position_feedback(False, 0.0)

        self.assertEqual(
            self.app.j4310_output_position_var.get(),
            "当前输出轴角度: 42.50 deg",
        )

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
        self.assertEqual(self.app._mask_for_page("闸门"), ENABLE_CONVEYOR)
        self.assertEqual(self.app._mask_for_page("夹爪"), ENABLE_GRIPPER)

    def test_position_frame_keeps_protocol_motor_targets(self) -> None:
        self.app.m3508_enable.set(True)
        values = (0.0, 2.0, 20.0, 0.5, 10.0, 20.0, 30.0, 40.0)

        frame = self.app._build_position_frame("机械臂", values)
        payload_size = struct.unpack_from("<H", frame, 6)[0]
        payload = frame[HEADER_SIZE:FRAME_OVERHEAD + payload_size - 2]
        unpacked = struct.unpack("<H30f", payload)

        self.assertEqual(
            unpacked[0], ENABLE_J4310_ONLY | ENABLE_M3508_ONLY
        )
        expected_targets = (
            *values[4:7],
            values[7] * GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG,
        )
        for actual, expected in zip(unpacked[7:11], expected_targets):
            self.assertAlmostEqual(actual, math.radians(expected), places=6)

    def test_gripper_output_target_is_scaled_to_motor_angle(self) -> None:
        self.app.gripper_enable.set(True)
        values = (0.0, 0.0, 5.0, 0.5, 0.0, 0.0, 0.0, 90.0)

        frame = self.app._build_position_frame("夹爪", values)
        unpacked = self._decode_position_frame(frame)

        self.assertEqual(GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG, 2.0)
        self.assertEqual(unpacked[0], ENABLE_GRIPPER)
        self.assertAlmostEqual(unpacked[10], math.radians(180.0), places=6)

    def test_m2006_position_limits_are_sent_at_both_endpoints(self) -> None:
        for output_deg in (M2006_POSITION_MIN_DEG, M2006_POSITION_MAX_DEG):
            with self.subTest(output_deg=output_deg):
                values = (0.0, 0.0, 5.0, 0.5, 0.0, 0.0,
                          output_deg, output_deg)

                gate = self._decode_position_frame(
                    self.app._build_position_frame("闸门", values)
                )
                gripper = self._decode_position_frame(
                    self.app._build_position_frame("夹爪", values)
                )

                self.assertEqual(gate[0], ENABLE_CONVEYOR)
                self.assertAlmostEqual(
                    gate[9], math.radians(output_deg), places=6
                )
                self.assertEqual(gripper[0], ENABLE_GRIPPER)
                self.assertAlmostEqual(
                    gripper[10],
                    math.radians(
                        output_deg * GRIPPER_MOTOR_DEG_PER_OUTPUT_DEG
                    ),
                    places=6,
                )

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

        self.app.send_page_now("闸门")

        gate = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(gate[0], ENABLE_CONVEYOR)
        self.assertAlmostEqual(gate[9], 0.0, places=7)
        self.assertTrue(self.app.gripper_enable.get())

        self.app.conveyor_position.set(30.0)
        self.app.gripper_position.set(0.0)
        self.app.gripper_enable.set(True)
        self.app.send_page_now("夹爪")

        gripper = self._decode_position_frame(
            transport.write.call_args.args[0]
        )
        self.assertEqual(gripper[0], ENABLE_GRIPPER)
        self.assertAlmostEqual(gripper[10], 0.0, places=7)
        self.assertTrue(self.app.conveyor_enable.get())

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
        self.app.m3508_position_1.set(35.0)
        self.app.m3508_position_2.set(35.0)
        self.app.conveyor_position.set(40.0)
        self.app.gripper_position.set(-45.0)

        transport.write.assert_not_called()
        self.assertEqual(self.app.j_position.get(), 30.0)
        self.assertEqual(self.app.m3508_position_1.get(), 35.0)
        self.assertEqual(self.app.m3508_position_2.get(), 35.0)
        self.assertEqual(self.app.conveyor_position.get(), 40.0)
        self.assertEqual(self.app.gripper_position.get(), -45.0)
        self.assertTrue(self.app.arm_enable.get())
        self.assertTrue(self.app.m3508_enable.get())
        self.assertTrue(self.app.conveyor_enable.get())
        self.assertTrue(self.app.gripper_enable.get())

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
        self.assertGreater(sent[1], 0.0)
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

    def test_manual_enable_controls_are_removed(self) -> None:
        self.assertFalse(hasattr(self.app, "j4310_enable_check"))
        self.assertFalse(hasattr(self.app, "m3508_enable_check"))
        self.assertEqual(self.app.j4310_send_button.cget("text"), "发送目标")
        self.assertEqual(self.app.j4310_stop_button.cget("text"), "停止发送")
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
        self.app.m3508_position_2.set(30.0)
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
        self.assertAlmostEqual(stopped[8], math.radians(30.0), places=6)
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
        self.app.m3508_position_1.set(0.0)
        self.app.m3508_position_2.set(90.0)
        self.app.conveyor_position.set(-45.0)

        self.assertEqual(self.app.m3508_position_1.get(), 0.0)
        self.assertEqual(self.app.m3508_position_2.get(), 90.0)
        self.assertEqual(self.app.conveyor_position.get(), -45.0)

    def test_position_sliders_use_motor_specific_ranges(self) -> None:
        self.assertEqual(J4310_POSITION_MIN_DEG, -90.0)
        self.assertEqual(J4310_POSITION_MAX_DEG, 270.0)
        self.assertEqual(POSITION_MIN_DEG, 0.0)
        self.assertEqual(POSITION_MAX_DEG, 1200.0)
        self.assertEqual(M3508_SYNC_POSITION_MIN_DEG, 0.0)
        self.assertEqual(M3508_SYNC_POSITION_MAX_DEG, 1200.0)
        self.assertEqual(M2006_POSITION_MIN_DEG, -360.0)
        self.assertEqual(M2006_POSITION_MAX_DEG, 360.0)
        range_text = self.app._parameter_range_text()
        self.assertIn("M3508 独立输入：0 .. 1200 deg", range_text)
        self.assertIn("M3508 同步输入 / 滑块：0 .. 1200 deg", range_text)
        self.assertNotIn("J4310=0°", range_text)
        self.assertEqual(
            set(self.app.position_slider_controls),
            {"j4310", "m3508", "conveyor", "gripper"},
        )
        for control in self.app.position_slider_controls["j4310"]:
            self.assertEqual(float(control.cget("from")), J4310_POSITION_MIN_DEG)
            self.assertEqual(float(control.cget("to")), J4310_POSITION_MAX_DEG)
        m3508_controls = self.app.position_slider_controls["m3508"]
        for control in m3508_controls[:2]:
            self.assertEqual(float(control.cget("from")), POSITION_MIN_DEG)
            self.assertEqual(float(control.cget("to")), POSITION_MAX_DEG)
        for control in m3508_controls[2:]:
            self.assertEqual(
                float(control.cget("from")),
                M3508_SYNC_POSITION_MIN_DEG,
            )
            self.assertEqual(
                float(control.cget("to")),
                M3508_SYNC_POSITION_MAX_DEG,
            )
        for key in ("conveyor", "gripper"):
            controls = self.app.position_slider_controls[key]
            for control in controls:
                self.assertEqual(
                    float(control.cget("from")), M2006_POSITION_MIN_DEG
                )
                self.assertEqual(
                    float(control.cget("to")), M2006_POSITION_MAX_DEG
                )

    def test_j4310_slider_endpoints_pass_send_validation(self) -> None:
        for position_deg in (J4310_POSITION_MIN_DEG,
                             J4310_POSITION_MAX_DEG):
            values = (position_deg, 0.0, 5.0, 0.5,
                      0.0, 0.0, 0.0, 0.0)
            with self.subTest(position_deg=position_deg):
                self.assertEqual(
                    self.app._send_parameter_errors(
                        "机械臂",
                        values,
                        True,
                        {"enable_mask": ENABLE_J4310_ONLY},
                    ),
                    [],
                )

    def test_m3508_input_endpoints_pass_send_validation(self) -> None:
        for position_deg in (POSITION_MIN_DEG, POSITION_MAX_DEG):
            values = (0.0, 0.0, 5.0, 0.5,
                      position_deg, position_deg, 0.0, 0.0)
            with self.subTest(position_deg=position_deg):
                self.assertEqual(
                    self.app._send_parameter_errors(
                        "机械臂",
                        values,
                        True,
                        {"enable_mask": ENABLE_M3508_ONLY},
                    ),
                    [],
                )

    def test_m3508_sync_slider_endpoints_pass_send_validation(self) -> None:
        for position_deg in (
            M3508_SYNC_POSITION_MIN_DEG,
            M3508_SYNC_POSITION_MAX_DEG,
        ):
            self.app._on_m3508_sync_slider_value(str(position_deg))
            with self.subTest(position_deg=position_deg):
                self.assertEqual(
                    self.app._send_parameter_errors(
                        "机械臂",
                        self.app._values(),
                        True,
                        {"enable_mask": ENABLE_M3508_ONLY},
                    ),
                    [],
                )

    def test_m3508_max_is_independent_of_j4310_position(self) -> None:
        for j4310_deg in (0.0, 90.0):
            values = (j4310_deg, 0.0, 5.0, 0.5,
                      POSITION_MAX_DEG, POSITION_MAX_DEG, 0.0, 0.0)
            with self.subTest(j4310_deg=j4310_deg):
                self.assertEqual(
                    self.app._send_parameter_errors(
                        "机械臂",
                        values,
                        True,
                        {"enable_mask": ENABLE_M3508_ONLY},
                    ),
                    [],
                )

    def test_m3508_has_independent_and_sync_inputs_with_shared_slider(self) -> None:
        controls = self.app.m3508_position_controls

        self.assertEqual(len(controls), 4)
        self.assertIsInstance(controls[0], ttk.Spinbox)
        self.assertIsInstance(controls[1], ttk.Spinbox)
        self.assertIsInstance(controls[2], ttk.Spinbox)
        self.assertIsInstance(controls[3], ttk.Scale)

        self.root.deiconify()
        self.root.update()
        self.assertIs(controls[0].master, controls[2].master)
        self.assertGreater(controls[2].winfo_x(), controls[0].winfo_x())

    def test_m3508_sync_input_stages_equal_targets_without_sending(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.rx_queue = queue.Queue()
        transport.error_queue = queue.Queue()
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        sync_input = self.app.m3508_sync_input_spinbox

        self.root.deiconify()
        self.root.update()
        sync_input.focus_force()
        self.root.update()
        sync_input.delete(0, "end")
        sync_input.insert(0, "125.5")
        sync_input.event_generate("<Return>")
        self.root.update()

        self.assertEqual(self.app.m3508_position_1.get(), 125.5)
        self.assertEqual(self.app.m3508_position_2.get(), 125.5)
        transport.write.assert_not_called()

        self.app.send_arm_group_now("m3508")
        sent = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(sent[0], ENABLE_M3508_ONLY)
        self.assertAlmostEqual(sent[7], math.radians(125.5), places=6)
        self.assertAlmostEqual(sent[8], math.radians(125.5), places=6)

    def test_slider_value_change_without_drag_does_not_send(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.m3508_enable.set(True)

        for position in (35.0, 360.0):
            with self.subTest(position=position):
                self.app._on_m3508_sync_slider_value(str(position))
                transport.write.assert_not_called()
                self.assertEqual(
                    self.app.m3508_position_1.get(), position
                )
                self.assertEqual(
                    self.app.m3508_position_2.get(), position
                )

    def test_m3508_sync_slider_keeps_equal_targets(self) -> None:
        for value in (0.0, 42.0, 360.0, 960.0, 1200.0):
            self.app._on_m3508_sync_slider_value(str(value))
            self.assertEqual(
                self.app.m3508_position_1.get(), value
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
            "闸门 M2006\n"
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
            "闸门 M2006\n当前电机反馈角度: 88.00 deg"
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
        self.app.m3508_position_2.set(34.0)

        self.app.send_arm_group_now("m3508")

        sent = self._decode_position_frame(transport.write.call_args.args[0])
        self.assertEqual(sent[0], ENABLE_M3508_ONLY)
        self.assertAlmostEqual(sent[7], math.radians(12.0), places=6)
        self.assertAlmostEqual(sent[8], math.radians(34.0), places=6)

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

    def test_handshake_does_not_query_flash(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshake_sequence = 0x1234

        self.app._accept_handshake_ack(
            Frame(MSG_ACK, 0x1234, HANDSHAKE_MAGIC)
        )

        self.assertTrue(self.app.handshaken)
        transport.write.assert_not_called()

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

    def test_missing_board_state_does_not_restart_handshake(self) -> None:
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

        self.assertTrue(self.app.handshaken)
        self.assertTrue(self.app.arm_enable.get())
        self.assertEqual(self.app.handshake_started_at, 0.0)

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

    def test_outputs_start_allowed_without_enable_checkboxes(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True

        transport.write.assert_not_called()
        self.assertTrue(self.app.arm_enable.get())
        self.assertTrue(self.app.m3508_enable.get())
        self.assertTrue(self.app.conveyor_enable.get())
        self.assertTrue(self.app.gripper_enable.get())

    def test_j4310_send_button_directly_sends_target(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.connection_requested = True
        self.app.handshaken = True
        self.app.j_position.set(30.0)

        self.app.send_arm_group_now("j4310")

        parsed = FrameParser().feed(transport.write.call_args.args[0])[0]
        self.assertEqual(parsed.msg_type, MSG_UPPER_POSITION_CMD)
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

    def test_j4310_auto_return_button_sets_runtime_setting(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        transport.write.return_value = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.j4310_auto_return_status_received = True
        self.app.j4310_auto_return_available = True
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

    def test_j4310_auto_return_button_reports_unsupported_board(self) -> None:
        transport = mock.Mock()
        transport.connected = True
        self.app.transport = transport
        self.app.handshaken = True
        self.app.j4310_auto_return_status_received = True
        self.app.j4310_auto_return_available = False
        self.app._update_j4310_auto_return_button()

        self.assertEqual(
            str(self.app.j4310_auto_return_button.cget("state")), "disabled"
        )
        self.assertEqual(
            self.app.j4310_auto_return_button.cget("text"),
            "重启归零：不可用",
        )

        self.app.toggle_j4310_auto_return()

        transport.write.assert_not_called()
        self.assertEqual(
            self.app.status_var.get(),
            "J4310 重启归零不可用：板端不支持",
        )

    def test_j4310_status_hides_bus_counters(self) -> None:
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
            {"available": True, "enabled": False, "active": False,
             "stage": 0}
        )

        text = self.app.j4310_bus_label.cget("text")
        self.assertEqual(text, "反馈状态: 电机已使能")
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
                "available": True,
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
