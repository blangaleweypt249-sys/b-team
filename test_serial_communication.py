#!/usr/bin/env python3
"""虚拟串口通信测试脚本 — 模拟下位机(STM32)端。

使用方法:
  1. 运行本脚本, 它会创建一个虚拟串口并打印设备路径
  2. 在另一个终端启动 serial_gateway, 指向该路径:
     ros2 run competition_gateway serial_gateway --ros-args -p serial_device:=/dev/pts/XX
  3. 或直接用 common_launch 启动, 加参数 serial_device:=/dev/pts/XX

协议格式:
  发送帧 (ROS → STM32):
    位置帧 (24字节): AA 55 11 [seq] [flags] [x:4] [y:4] [z:4] [w:4] [checksum] 0D 0A
    感知帧 (44字节): AA 55 10 [seq] [flags] [red:12] [blue:12] [ball:12] [checksum] 0D 0A
  接收帧 (STM32 → ROS):
    状态帧 (8字节):  55 AA 20 [state] [error] [checksum] 0D 0A
"""

import os
import pty
import struct
import sys
import termios
import time
import select
import signal

# 协议常量
TX_HEADER = (0xAA, 0x55)
RX_HEADER = (0x55, 0xAA)
FRAME_TAIL = (0x0D, 0x0A)
TX_POSITION_TYPE = 0x11
TX_PERCEPTION_TYPE = 0x10
RX_STATUS_TYPE = 0x20

POSITION_FRAME_SIZE = 24
PERCEPTION_FRAME_SIZE = 44
STATUS_FRAME_SIZE = 8


def checksum(data: bytes) -> int:
    """计算8位累加校验和"""
    return sum(data) & 0xFF


def encode_status(state: int, error: int) -> bytes:
    """编码状态帧 (8字节)"""
    frame = bytearray(STATUS_FRAME_SIZE)
    frame[0] = RX_HEADER[0]
    frame[1] = RX_HEADER[1]
    frame[2] = RX_STATUS_TYPE
    frame[3] = state & 0xFF
    frame[4] = error & 0xFF
    frame[5] = checksum(frame[2:5])
    frame[6] = FRAME_TAIL[0]
    frame[7] = FRAME_TAIL[1]
    return bytes(frame)


def decode_position_frame(frame: bytes) -> dict:
    """解码位置帧 (24字节)"""
    if len(frame) != POSITION_FRAME_SIZE:
        return None
    if frame[0] != TX_HEADER[0] or frame[1] != TX_HEADER[1]:
        return None
    if frame[2] != TX_POSITION_TYPE:
        return None
    if frame[22] != FRAME_TAIL[0] or frame[23] != FRAME_TAIL[1]:
        return None

    # 校验和
    expected_cs = checksum(frame[2:21])
    if frame[21] != expected_cs:
        return {"error": "校验和不匹配", "expected": expected_cs, "got": frame[21]}

    seq = frame[3]
    flags = frame[4]
    x, y, z, w = struct.unpack_from('<ffff', frame, 5)

    return {
        "type": "位置帧",
        "sequence": seq,
        "flags": flags,
        "field_valid": bool(flags & 0x01),
        "x": x, "y": y, "z": z, "w": w,
    }


def decode_perception_frame(frame: bytes) -> dict:
    """解码感知帧 (44字节)"""
    if len(frame) != PERCEPTION_FRAME_SIZE:
        return None
    if frame[0] != TX_HEADER[0] or frame[1] != TX_HEADER[1]:
        return None
    if frame[2] != TX_PERCEPTION_TYPE:
        return None
    if frame[42] != FRAME_TAIL[0] or frame[43] != FRAME_TAIL[1]:
        return None

    # 校验和
    expected_cs = checksum(frame[2:41])
    if frame[41] != expected_cs:
        return {"error": "校验和不匹配", "expected": expected_cs, "got": frame[41]}

    seq = frame[3]
    flags = frame[4]
    red_x, red_y, red_z, blue_x, blue_y, blue_z, ball_x, ball_y, ball_z = \
        struct.unpack_from('<fffffffff', frame, 5)

    return {
        "type": "感知帧",
        "sequence": seq,
        "flags": flags,
        "red_valid": bool(flags & 0x01),
        "blue_valid": bool(flags & 0x02),
        "ball_valid": bool(flags & 0x04),
        "red": (red_x, red_y, red_z),
        "blue": (blue_x, blue_y, blue_z),
        "ball": (ball_x, ball_y, ball_z),
    }


def try_decode_frame(buf: bytearray):
    """从缓冲区尝试解析一帧, 返回 (result, frame_size) 或 (None, 0)"""
    if len(buf) < 5:
        return None, 0

    # 找帧头
    if buf[0] != TX_HEADER[0] or buf[1] != TX_HEADER[1]:
        return None, 1  # 丢弃1字节继续找

    frame_type = buf[2]
    if frame_type == TX_POSITION_TYPE and len(buf) >= POSITION_FRAME_SIZE:
        return decode_position_frame(bytes(buf[:POSITION_FRAME_SIZE])), POSITION_FRAME_SIZE
    elif frame_type == TX_PERCEPTION_TYPE and len(buf) >= PERCEPTION_FRAME_SIZE:
        return decode_perception_frame(bytes(buf[:PERCEPTION_FRAME_SIZE])), PERCEPTION_FRAME_SIZE
    elif frame_type in (TX_POSITION_TYPE, TX_PERCEPTION_TYPE):
        return None, 0  # 帧头正确但数据不够, 等待更多数据
    else:
        return None, 1  # 未知类型, 丢弃


def print_position(data):
    """打印位置帧"""
    print(f"\n{'='*60}")
    print(f"  [位置帧] seq={data['sequence']} flags=0x{data['flags']:02X} "
          f"field_valid={'是' if data['field_valid'] else '否'}")
    print(f"  机器人位置: x={data['x']:.3f}m, y={data['y']:.3f}m, z={data['z']:.3f}m")
    print(f"  旋转四元数w: {data['w']:.3f}")
    print(f"{'='*60}")


def print_perception(data):
    """打印感知帧"""
    print(f"\n{'='*60}")
    print(f"  [感知帧] seq={data['sequence']} flags=0x{data['flags']:02X}")
    print(f"  红块: {'有效' if data['red_valid'] else '无效'}  "
          f"({data['red'][0]:.3f}, {data['red'][1]:.3f}, {data['red'][2]:.3f})m")
    print(f"  蓝块: {'有效' if data['blue_valid'] else '无效'}  "
          f"({data['blue'][0]:.3f}, {data['blue'][1]:.3f}, {data['blue'][2]:.3f})m")
    print(f"  金球: {'有效' if data['ball_valid'] else '无效'}  "
          f"({data['ball'][0]:.3f}, {data['ball'][1]:.3f}, {data['ball'][2]:.3f})m")
    print(f"{'='*60}")


def set_raw(fd):
    """手动设置 raw 模式 — 关闭回显和终端处理"""
    import termios as t
    attr = t.tcgetattr(fd)
    # iflag: 关闭输入处理
    attr[0] &= ~(t.IGNBRK | t.BRKINT | t.PARMRK | t.ISTRIP |
                 t.INLCR | t.IGNCR | t.ICRNL | t.IXON)
    # oflag: 关闭输出处理
    attr[1] &= ~t.OPOST
    # lflag: 关闭回显、规范模式、信号
    attr[3] &= ~(t.ECHO | t.ECHONL | t.ICANON | t.ISIG | t.IEXTEN)
    t.tcsetattr(fd, t.TCSANOW, attr)


def main():
    # 创建虚拟串口对
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    # 设置两端为 raw 模式, 避免回显和终端处理
    try:
        set_raw(master_fd)
        set_raw(slave_fd)
    except Exception as e:
        print(f"警告: raw 模式设置失败: {e}", file=sys.stderr)

    os.close(slave_fd)
    print(f"\n虚拟串口已创建: {slave_name}")
    print(f"请用以下命令启动 serial_gateway:")
    print(f"  ros2 run competition_gateway serial_gateway "
          f"--ros-args -p serial_device:={slave_name}")
    print(f"\n或用 launch 启动:")
    print(f"  ros2 launch common_launch_pkg serial_gateway.launch.py "
          f"serial_device:={slave_name}")
    print(f"\n本脚本将自动发送状态帧 (state=1, error=0) 并解析收到的数据帧。")
    print(f"按 Ctrl+C 退出。\n")
    print(f"{'='*60}")
    print(f"等待数据...")
    print(f"{'='*60}\n")

    # 状态帧发送间隔
    last_status_send = 0
    status_interval = 0.1  # 100ms
    state_counter = 0

    receive_buf = bytearray()
    frame_count = 0

    try:
        while True:
            # 使用 select 等待数据, 超时 50ms
            readable, _, _ = select.select([master_fd], [], [], 0.05)

            # 发送状态帧
            now = time.time()
            if now - last_status_send >= status_interval:
                state = (state_counter % 3) + 1  # 在 1,2,3 之间循环
                error = 0
                status_frame = encode_status(state, error)
                os.write(master_fd, status_frame)
                last_status_send = now
                state_counter += 1

            # 读取数据
            if readable:
                try:
                    data = os.read(master_fd, 256)
                    if data:
                        # 十六进制调试输出
                        hex_str = ' '.join(f'{b:02X}' for b in data[:48])
                        if len(data) > 48:
                            hex_str += f' ... ({len(data)} bytes)'
                        print(f"  [RAW] {hex_str}")

                        receive_buf.extend(data)

                        # 尝试解析帧
                        while len(receive_buf) >= 5:
                            result, consumed = try_decode_frame(receive_buf)
                            if consumed == 0:
                                break  # 数据不够, 等待

                            if consumed > 0:
                                del receive_buf[:consumed]

                            if result is not None:
                                if 'error' in result:
                                    print(f"  [解析错误] {result['error']}")
                                elif result['type'] == '位置帧':
                                    frame_count += 1
                                    print_position(result)
                                elif result['type'] == '感知帧':
                                    frame_count += 1
                                    print_perception(result)

                                # 打印帧统计
                                print(f"  [统计] 已接收 {frame_count} 帧")

                except OSError:
                    pass

    except KeyboardInterrupt:
        print(f"\n\n退出。共接收 {frame_count} 帧。")
    finally:
        os.close(master_fd)


if __name__ == '__main__':
    main()
