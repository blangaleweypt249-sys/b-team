# E32-433T30D 遥控链路配置

> 本文记录旧发送工程 `培训遥控/遥控发送` 的固定 16 字节协议，不是当前启用的
> `培训遥控/u-remote control` 双 ID 协议。当前发送端与接收端对接以
> `培训遥控/u-remote control/遥控器输入与下位机对接说明.md` 为准。

## 模块参数

两只 E32-433T30D 由各自 MCU 在每次上电时自动读取并配置为完全相同的无线参数：

| 参数 | 设置 |
| --- | --- |
| UART | 115200 baud，8 数据位，无校验，1 停止位 |
| 空中速率 | 9.6 kbps（本批实机支持并已验证） |
| 传输方式 | 透明传输 |
| 信道 | 两端统一为信道 6（实机旧工程可通信配置） |
| 地址 | 两端统一为地址 6，六字节配置完全比较 |
| FEC | 两端保持一致，建议开启 |
| IO 驱动 | 推挽输出 |
| 发射功率 | 按现场距离设置；使用 30 dBm 时必须保证电源能力和天线连接 |

固件上电后先将 `M0=1、M1=1`，把 MCU 串口临时切换为 `9600、8N1`，发送
`C1 C1 C1` 读取模块参数。当前目标为完整配置 `C0 00 06 3C 06 44`，两端六个字节
必须完全一致。任意字节不一致时写入目标并再次读取校验。校验通过后，MCU 串口切回
`115200、8N1`，再将 `M0=0、M1=0` 切换到透明传输模式，并等待 AUX 稳定为高。

发送端只有在上述流程全部成功后才发送遥控帧；配置失败时 50 ms 任务只累计跳过
次数，不会把控制数据送入无线模块。接收端也只有配置成功后才启动 USART3 单字节
中断接收，因此不会用未知参数接收并执行控制数据。

参数字节含义：本批旧版 E32 使用 `0x3C` 对应 `115200/8N1 + 9.6 kbps`，`0x06` 为信道 6，
`0x44` 为透明传输、推挽 IO、FEC 开启、30 dBm。实机拒绝写入 `0x3D`，因此不再强制使用 19.2 kbps。

发送端可观察 `remote_lora_config_status`、`remote_lora_config_attempts`、
`remote_lora_config_readback[6]` 和 `remote_tx_config_skips`。接收端对应变量为
`remote_link_lora_config_status`、`remote_link_lora_config_attempts` 和
`remote_link_lora_config_readback[6]`。状态值 `2` 表示配置完成并已进入透明模式；
其余非零终态分别用于定位 AUX、UART、读取、写入、回读校验或退出配置模式失败。

## 接线

接收端 `D:\桌面\暑期培训\F103C8T6` 的 H1：

| H1 | STM32F103 | E32 |
| --- | --- | --- |
| 1 | PB0 | M0 |
| 2 | PB1 | M1 |
| 3 | PB10 / USART3_TX | RXD |
| 4 | PB11 / USART3_RX | TXD |
| 5 | PB12 | AUX |
| 6 | 5V | VCC |
| 7 | GND | GND |

接收端 H2（SWD 烧录）：

| H2 | STM32F103 | 功能 |
| --- | --- | --- |
| 1 | PA13 | SWDIO |
| 2 | PA14 | SWCLK |
| 3 | 3V3 | 调试器参考电压 |
| 4 | GND | 地 |

接收端 H3（SPI1，主机、Mode 0、软件 NSS、4.5 Mbit/s）：

| H3 | STM32F103 | 功能 |
| --- | --- | --- |
| 1 | GND | 地 |
| 2 | PA5 | SPI1_SCK |
| 3 | PA6 | SPI1_MISO |
| 4 | PA7 | SPI1_MOSI |

H3 没有单独的片选脚，需要时从 U1/U4 选择一个 GPIO 作为 CS。

接收端 H4（USART2，115200 baud、8N1）：

| H4 | STM32F103 | 功能 |
| --- | --- | --- |
| 1 | PA2 | USART2_TX |
| 2 | PA3 | USART2_RX |
| 3 | 3V3 | 电源 |
| 4 | GND | 地 |

接收端除 `PB14`、`PA8`、`PA10`、`PA11`、`PA12` 外的其余引出 GPIO 均配置为输入、无上下拉：

| U1 | STM32F103 | U1 | STM32F103 |
| --- | --- | --- | --- |
| 1 | GND | 7 | PB7 |
| 2 | 5V | 8 | PB6 |
| 3 | PB3 | 9 | PB9 |
| 4 | PA15 | 10 | PB8 |
| 5 | PB5 | 11 | GND |
| 6 | PB4 | 12 | 3V3 |

| U4 | STM32F103 | U4 | STM32F103 |
| --- | --- | --- | --- |
| 1 | GND | 7 | PA9 |
| 2 | GND | 8 | PA10 |
| 3 | 5V | 9 | PB15 |
| 4 | 5V | 10 | PA8 |
| 5 | PA11 | 11 | PB13 |
| 6 | PA12 | 12 | PB14 |

`PB14`、`PA8`、`PA10`、`PA11`、`PA12` 配置为推挽输出，上电初始均为低电平。接收端收到合法遥控帧后，
只要检测到对应拨动开关的稳定状态相对上一帧发生变化，就将输出电平翻转一次：

| 遥控端拨动开关 | 协议状态 | 接收端输出 |
| --- | --- | --- |
| PE4 | `switch_state[0]` / SW1 | PB14 |
| PE3 | `switch_state[1]` / SW2 | PA8 |
| PE1 | `switch_state[2]` / SW3 | PA10 |
| PD5 | `switch_state[5]` / SW6 | PA11、PA12 同时翻转 |

接收端上电后的第一帧只用于记录开关初始状态，不触发输出翻转。

发送端 `培训遥控` 的 H2：

| H2 | STM32F407 | E32 |
| --- | --- | --- |
| 7 | PD14 | M0 |
| 6 | PD15 | M1 |
| 5 | PC6 / USART6_TX | RXD |
| 4 | PC7 / USART6_RX | TXD |
| 3 | PC8 | AUX |
| 2 | 5V | VCC |
| 1 | GND | GND |

E32-433T30D 是 1 W 模块。必须先接好 433 MHz 天线，并使用能够承受发射峰值电流的
5 V 电源；不要使用普通 USB-TTL 的小电流 3.3 V 输出给模块供电。

## 遥控数据帧

发送周期为 50 ms（20 Hz），固定 16 字节。任务使用绝对周期调度，AUX 为低时跳过本周期，
只保留最新遥控状态。

| 字节 | 内容 |
| --- | --- |
| 0..1 | 帧头 `A5 5A` |
| 2 | 协议版本 `01` |
| 3 | 发送序号，8 位循环 |
| 4..5 | 左肩键 ADC，小端序 |
| 6..7 | 右肩键 ADC，小端序 |
| 8..11 | 左 X、左 Y、右 X、右 Y |
| 12 | 6 个拨动开关位图 |
| 13 | KEY1..KEY8 位图 |
| 14 bit0..3 | KEY9..KEY12 位图 |
| 14 bit7 | ADC 采集故障标志 |
| 15 | CRC8，计算范围为字节 0..14，多项式 `0x07` |

接收端连续 500 ms 未收到合法 CRC 帧即判定失联。第一帧只建立按键和拨动开关状态基准；
随后任一数字按键从松开变为按下，或任一拨动开关状态变化时，`PC13` 点亮 100 ms 后熄灭一次。
按键松开、摇杆/肩键 ADC 变化以及普通连续收帧不触发 PC13。应用代码应通过
`RemoteLink_GetSnapshot()` 读取数据并检查返回值，返回 0 时进入安全状态，不应继续执行
上一次遥控命令。

发送端可观察 `remote_tx_attempts`、`remote_tx_frames`、`remote_tx_busy_skips`、`remote_tx_errors`；接收端可观察
`remote_link_valid_frames`、`remote_link_crc_errors`、`remote_link_lost_frames`、
`remote_link_uart_errors`。

## 接收端 USART2 原始帧转发

F103 接收端通过 USART2（PA2 TX，115200/8N1）向电脑转发从 LoRa 收到的原始协议帧。
每帧固定 16 字节，内容与发送端无线帧完全一致：从 `A5 5A` 开始，以 CRC8 结束。
USART2 不再输出 `LORA OK`、`LORA WAIT`、配置诊断、字段文本或 VOFA/JustFloat 数据。
电脑端应使用二进制方式按 `A5 5A` 帧头和 16 字节长度拆帧，并自行校验版本与 CRC8。

USB 转 TTL 的 RX 接 F103 `PA2`，GND 共地。串口工具设置为 `115200、8N1` 并按二进制接收；
不需要连接 USB 转 TTL 的 TX。

## 发送端 USB1/CH340 输出

遥控器板载 `USB1` 连接 CH340 和 STM32 USART1。当前发送端调试任务中的 VOFA/JustFloat
调用已注释停用，不再从 USART1 周期输出曲线或 LoRa 诊断；电脑数据统一从接收端 USART2
获取原始 LoRa 帧。
