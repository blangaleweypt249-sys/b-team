# STM32F405 + VESC + U12 KV120

## Hardware

- CAN1 RX/TX: PB8/PB9
- CAN bitrate: 1 Mbit/s
- VESC controller ID: 67
- Motor: U12 KV120, 36N42P (42 poles, 21 pole pairs)

## 1000 RPM example

VESC speed control uses electrical RPM (eRPM):

`1000 motor RPM * 21 pole pairs = 21000 eRPM`

The example sends the command every 20 ms:

- Extended CAN ID: `0x343` (`CAN_PACKET_SET_RPM << 8 | 67`)
- DLC: `4`
- Data: `00 00 52 08` (21000, signed big-endian int32)

Configure the VESC for FOC, 42 motor poles, CAN ID 67, and 1 Mbit/s.
Set voltage and current limits in VESC Tool for the actual battery, controller,
cooling, and mechanical load; do not copy the motor's 180-second peak rating
directly into the continuous-current limit.

## USART1 command bridge

A serial console on **USART1** lets you drive the VESC over CAN without
re-flashing:

- Pins: **PA9 (TX) / PA10 (RX)**, AF7
- Format: **115200 8N1** (divider computed from the configured APB2 clock at
  runtime — the clock configuration is not modified)
- Line format: ASCII, comma separated, terminated by `\n`

| Command | Meaning | Wire payload |
|---------|---------|--------------|
| `R,<id>,<erpm>` | set electrical RPM | CAN_PACKET_SET_RPM (3) |
| `C,<id>,<mA>` | set current (mA) | CAN_PACKET_SET_CURRENT (1) |
| `D,<id>,<duty*1e5>` | set duty (50000 = 0.50) | CAN_PACKET_SET_DUTY (0) |
| `B,<id>,<mA>` | set brake current (mA) | CAN_PACKET_SET_CURRENT_BRAKE (2) |
| `P,<id>,<kp>,<ki>,<kd>` | set speed PID | **stub / reserved** (multi-frame COMM) |

Examples (controller id 67):

    R,67,21000        # 21000 eRPM  (1000 motor RPM * 21 pole pairs)
    C,67,2000         # 2 A
    C,67,-2000        # 2 A reverse
    D,67,50000        # 50% duty
    B,67,1500         # 1.5 A braking

Each line is answered over USART1 with `OK` or `ERR ...`.

Notes:
- The existing VESC/CAN code (`vesc_can_start`, `vesc_can_set_erpm`, CAN config)
  is unchanged; only new send helpers and the USART layer were added.
- `P` (PID) is a placeholder. Setting PID over CAN needs the VESC multi-frame
  COMM protocol and is not implemented yet; the function
  `vesc_can_set_speed_pid()` is stubbed for future use.
- The `.ioc` was intentionally not edited; if you regenerate with CubeMX, enable
  USART1 (PA9/PA10) there as well.
