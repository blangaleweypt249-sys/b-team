# H723VGT6 板级与 FreeRTOS 初始配置

配置依据：`D:\桌面\网表文件\暑期培训\主控\H723VGT6.tel`。

## 时钟与内核

- MCU：STM32H723VGT6，LQFP100。
- HSE：25 MHz 有源晶振，连接 `PH0/PH1`。
- Cortex-M7：CPU 550 MHz，HCLK 275 MHz，APB1~APB4 137.5 MHz。
- PLL2 给 FDCAN 和 SPI1 提供 100 MHz 内核时钟。
- 已启用 I-Cache 和 D-Cache，HAL Tick 使用 TIM6，FreeRTOS Tick 使用 SysTick。
- 默认数据区是可被 DMA1 访问的 AXI SRAM `0x24000000`；不要把 DMA 缓冲区放进
  DTCM `0x20000000`。

## 网表对应关系

| 功能 | MCU 引脚 | 外部器件/网络 | 初始配置 |
| --- | --- | --- | --- |
| 调试 | PA13 / PA14 | DIO / CLK | SWD |
| FDCAN1 | PA11 RX / PA12 TX | CAN1_R / CAN1_T | 1 Mbps，经典 CAN |
| FDCAN2 | PB12 RX / PB13 TX | CAN2_R / CAN2_T | 1 Mbps，经典 CAN |
| FDCAN3 | PD12 RX / PD13 TX | CAN3_R / CAN3_T | 1 Mbps，经典 CAN |
| RS485-1 | PB15 RX / PB14 TX | MAX13488 U9 | USART1，115200 8N1，RX/TX DMA |
| RS485-2 | PA3 RX / PA2 TX | MAX13488 U10 | USART2，460800 8N1，RX/TX DMA，仅保留 485 通道 |
| USB 串口 | PA1 RX / PA0 TX | CH340N TXD/RXD | UART4，115200 8N1，RX/TX DMA，普通 UART 默认控制通道 |
| 遥控输入 | PD2 RX / PC12 TX | H3.24 / H3.19 | UART5，2000000 8N1，RX 循环 DMA / TX 中断 |
| 普通 UART | PE7 RX / PE8 TX | H1.18 / H1.17 | UART7，115200 8N1，中断收发 |
| 普通 UART | PE0 RX / PE1 TX | H2.10 / H2.7 | UART8，115200 8N1，中断收发 |
| 普通 UART | PD14 RX / PD15 TX | H3.7 / H3.10 | UART9，115200 8N1，中断收发 |
| 普通 UART | PB11 RX / PB10 TX | H3.4 / H3.1 | USART3，115200 8N1，中断收发 |
| 普通 UART | PC7 RX / PC6 TX | H3.12 / H3.9 | USART6，115200 8N1，中断收发 |
| 普通 UART | PE2 RX / PE3 TX | H2.8 / H2.5 | USART10，115200 8N1，中断收发 |
| W25Q128 | PA4 / PA5 / PA6 / PA7 | NSS/SCK/MISO/MOSI | SPI1 Mode 0，25 MHz，RX/TX DMA |
| 辅助输出转发 | PC12 UART5_TX | 抬升 H723 USART6_RX | 2 Mbps，8N1；经抬升端 115200 UART 转发到 F103 USART2_RX |
| 状态灯 | PC13 | LED1 | 推挽输出，默认低 |
| 蜂鸣器 | PC15 | Q3 驱动 | 推挽输出，默认低 |

默认机器人拓扑是：FDCAN1 连接一台 J4310（电机 ID `0x06`、Master ID `0x16`），FDCAN2 连接两台
M3508/C620（节点 1、2），FDCAN3 连接两台 M2006/C610（节点 1、2）。每条 DJI 总线共用
`0x200`/`0x1FF` 组控制帧，未配置的槽位为零。
各电机上下文按“型号 + 总线 + 节点”隔离，目标、反馈和 PID 不跨路由共享。

三路接收过滤器均接收标准 ID `0x000..0x7FF`，但拒绝扩展帧和远程帧。
三路控制周期均为 1 ms（1 kHz）。按 135 bit/帧保守估算，FDCAN1 持续 2 帧/ms，
占用 27.0%；FDCAN2、FDCAN3 各 3 帧/ms，占用 40.5%，均保留超过 20% 的持续带宽余量。
2 kHz 不作为本工程控制频率：FreeRTOS Tick 和 M3508 闭环时间步长均为 1 ms，提升频率
需要同时改时间基准、超时语义和 PID 参数，不能仅提高发送次数。

网表中的 LED2、LED3 是电源指示，BOOT0 和 NRST 由硬件电路管理。只接到排针、
尚未分配业务的 GPIO 保持复位状态，避免初始工程抢占后续外设复用。

## DMA 分配

| DMA1 Stream | 请求 | 模式 |
| --- | --- | --- |
| 0 / 1 | UART4 RX / TX | RX Circular / TX Normal |
| 2 / 3 | USART1 RX / TX | RX Circular / TX Normal |
| 4 / 5 | USART2 RX / TX | RX Circular / TX Normal |
| 6 / 7 | SPI1 RX / TX | Normal / Normal |

| DMA2 Stream | 请求 | 模式 |
| --- | --- | --- |
| 0 | UART5 RX | Circular |

UART5 使用 `HAL_UARTEx_ReceiveToIdle_DMA()` 接收遥控字节流，同时使用 TX 中断发送辅助输出帧。
其他普通 UART 使用 `HAL_UARTEx_ReceiveToIdle_IT()` 和 TX 中断，不占用 DMA1；USART1/2 的
RS485 DMA 配置保持不变。

H723 收到上位机 `MSG_AUX_CONTROL (0x15)` 后，组装
`A5 5A 01 02 01 seq control_bits crc8` 固定 8 字节帧，通过 UART5 以 2 Mbps 发送到抬升 H723。`control_bits` 低四位为目标状态、高四位为本次更新掩码；旧帧的高四位为 0 时保持 PB3 明确更新、其余三路按状态变化更新的兼容语义。
抬升端保持帧内容不变，通过另一组 115200 UART 发送到 F103 USART2_RX。

DMA、UART、SPI 和 FDCAN 中断优先级均为 5，可以调用 FreeRTOS 的 ISR 安全接口。
FDCAN 使用片上 Message RAM，不使用 DMA1/2。

## FreeRTOS 架构

- FreeRTOS V10.6.2 + CMSIS-RTOS2，抢占式调度，1 kHz Tick。
- `heap_4` 为 64 KiB，启用方法 2 栈溢出检查和 malloc 失败 Hook。
- 当前生成工程保留 `appTask`、`commRxTask` 和 `monitorTask` 作为兼容运行骨架；它们不代表
  机构拆分后的最终任务设计已经实现。
- 目标任务框架为 `upperSafetyTask`、`commRxTask`、`motorTxTask`、`upperArmTask`、
  `upperConveyorTask`、`upperGripperTask` 和 `monitorTask`，相对优先级固定为
  `upperSafetyTask > commRxTask > motorTxTask > 机构业务任务 > monitorTask`。
- 本轮不创建上述目标任务的源文件，不填写任务函数，也不预设未经最坏执行时间和栈水位
  验证的具体优先级枚举与栈大小。

板端框架使用通用回调注册通信入口，Application 不直接接触 HAL 句柄：

```c
void CommRuntime_SetHandlers(comm_uart_handler_t uart_handler,
                             comm_can_handler_t can_handler,
                             void *user_data);

void App_Control1ms(void);
```

Type-C 的 D+/D- 实际连接 CH340N，H723 侧通过 UART4 PA0/PA1 与其通信。小电脑看到的是
USB 虚拟串口，H723 不启用原生 USB CDC。除此之外，H1/H2/H3 排针上的普通 UART 均可用
USB-TTL 转换器连接电脑。帧格式、超时策略、电机拓扑和目标任务职责见
[上层代码框架说明.md](../上层代码框架说明.md)。

## UART5 抬升桥接链路

上层 H723 UART5 与抬升 H723 USART6 使用 `2000000 8N1` 全双工交叉连接；F103 USART2
与抬升端另一组 UART 使用 `115200 8N1` 全双工交叉连接，所有控制板必须共地。抬升端在
两组串口之间保持字节内容和顺序不变。

上层接收方向按 `A5 5A` 帧头和固定 10 字节长度重组拆包/粘包，帧格式为：

```text
A5 5A left_x left_y right_x right_y primary_keys primary_switch key_bits switch_bits
```

- `primary_keys` 解析 bit1/bit2，分别对应左侧 PC1/PC0；其余左侧按键位继续忽略。
- `primary_switch` bit0 对应 PE0，bit1 是 PD6 模式位镜像，其他位忽略。
- `key_bits` bit0..5 对应遥控器 PD13、PD12、PD11、PD8、PD9、PD10，bit6..7 被屏蔽。
- `switch_bits` bit0、1、2、4、5 对应 PE4、PE3、PE1、PD6、PD5，其他位被屏蔽；PD6 bit4 作为兼容副本保留。
- PE0、PD6 使用 `primary_switch` 中的当前面板档位，不使用变化沿翻转内部模式。
- PE0=`1`选择自动、PE0=`0`选择手动；PD6=`1`选择存三、PD6=`0`选择存二。
- 当前电平 `11` 为存三自动、`01` 为存三手动、`10` 为存二自动、`00` 为存二手动。
- 存二自动保留存三自动的分支结构；两个自动模式的 PD13 分支二均下发 `M3508=1050 deg/J4310=70 deg`，PE4 关闭后 M3508 回 `0 deg`，等待 500 ms 后分别收尾到 J4310=`180 deg`（存二）和 `165 deg`（存三）。存二自动的其他 PD13 自动收尾角度为 `180 deg`；存三自动的 PD13 分支一收尾为 `165 deg`。PD12 第一次 PE4 关闭后执行 M3508=`0 deg`、等待 500 ms 后 J4310=`180 deg`。
- 自动模式下 PD13、PD12 在第一次 PE4 关闭前连续按两次时，第二次按键直接进入各自分支二；与第二次按键同帧到达的 PE4 边沿不会立即收尾刚进入的分支二。
- 自动模式下每次按下 PD13 或 PD12，都先发送固定的 PE4 打开命令（状态 bit0=`0`，连续发送 3 帧），再进入对应的第一分支、第二分支或 PD8 翻转子模式；该动作不读取旧输出状态，也不执行翻转。
- PD8 翻转子模式在存二自动、存三自动中共用：按 PD13 首段执行 `M3508=500 deg/J4310=90 deg`，按 PD12 首段执行 `M3508=0 deg/J4310=90 deg`，两者都会自动打开 PE4；等待手动关闭 PE4（`0 -> 1`）后均先等待 `200 ms`，再将闸门和 M3508 回到 `0 deg`、J4310 到 `180 deg`。
- 关闭 PE4 后，PD13/PD12 首段分别等待再次按对应按键，随后 M3508 到 `500 deg`、J4310 到 `40 deg`，再等待 `1.5 s` 自动打开 PE4；收尾后仅结束当前子流程，继续保持 PD8 翻转模式。PD12 的 `M3508=850 deg/J4310=90 deg` 第二段保持原时序，不增加这些首段动作。
- 每次启动 PD13 或 PD12 翻转首段时，自动存二、自动存三均同时执行夹爪首次 PD10 目标 `55 deg`，并将下一次实际 PD10 推进到 `125 deg`。
- 首段尚未手动关闭 PE4 时，再按 PD13 或 PD12 会推进当前活动子流程的下一目标：PD13 为 `M3508=1000 deg/J4310=90 deg`，PD12 为 `M3508=850 deg/J4310=90 deg`。翻转模式中按 PD13 会把下一次 PD12 复位为第一段，按 PD12 会把下一次 PD13 复位为第一段。
- PD13、PD12 的分支一收尾期间会锁存同键按下请求，即系统记住 500 ms 等待期间发生的按键；分支一完成后无需再按一次，已经锁存的请求会立即进入分支二。没有锁存请求时，分支一结束后的下一次同键按下才进入分支二。
- 存二自动的 PD13 分支一为：第一次按下先清除 PD12、PC0 当前流程但保留两者的已按历史，再到 `M3508=500 deg/J4310=90 deg`；按键入口已经固定先打开 PE4，随后等待手动关闭 PE4（UART3 PE4 `0 -> 1`），关闭时 M3508 立即回 `0 deg`，`500 ms` 后 J4310 到 `180 deg`，分支一结束，下一次 PD13 直接进入分支二。第一次手动关闭 PE4 前再次按 PD13，则立即进入分支二；若 PD13 是存二自动本模式第一次存放类操作，同时执行第一次 PD10 的夹爪动作（55°），并将下一次实际 PD10 推进到 125°。存三自动仍使用原有 PD13 起始联动。
- 按 PD13 会清除 PD12 当前分支和待执行请求，使 PD12 下一次从第一段开始；按 PD12 对 PD13 执行相同复位，首次自动联动历史仍按模式独立保留。
- 存三手动 PD11 第一次执行 `0 deg/165 deg`、第二次执行 `0 deg/240 deg`、第三次回到第一段并继续交替；存二手动 PD11 始终只执行第一段 `0 deg/165 deg`。
- 存二自动的 PD11 流程为：按下时到 `M3508=500 deg/J4310=90 deg` 且闸门到 `0 deg`；`1200 ms` 后自动打开 PE4，并同时将 M3508 回到 `0 deg`、J4310 移到 `180 deg`，流程结束。存三自动仍使用原有 `60 deg -> 1200/1700/3500/3700 ms` 完整流程。PE4 自动动作以完整辅助帧连续发送 3 次；随后普通 PD9 从 `180 deg` 开始与 `60 deg` 交替。
- PC1 在四种模式中使用同一流程：按下沿先使能闸门 M2006 并下发 `80 deg` 位置目标；实际机构角度进入闭区间 `79..81 deg` 后立即禁用闸门，效果与上位机“闸门停止发送”一致。之后按 PD9 会取消尚未完成的 PC1 待失能状态、重新使能闸门，并继续原有 `180 deg/60 deg` 交替控制。
- PC0 只在存二自动、存三自动中生效，两种自动模式使用完全相同的三分支循环：第一分支起步 `500/90 deg`，第二分支起步 `0/90 deg`，第三分支起步 `850/90 deg`，完成后按 `第一 -> 第二 -> 第三 -> 第一` 循环；存二手动、存三手动均忽略 PC0。
- 存二自动的三个 PC0 分支中段为：手动关闭 PE4 后到 `0/90 deg`；再按 PC0 将闸门固定到 `180 deg`，`500 ms` 后到 `0/-20 deg`；手动打开 PE4 后立即依次下发闸门 `68 deg`、M3508=`0 deg`、J4310=`90 deg`，随后将普通 PD9 的下一次动作复位为 `180 deg` 并推进到下一分支。存三自动打开 PE4 后直接执行同样的收尾动作，不再等待额外延时。第一次手动关闭 PE4 前连续按 PC0，不执行中段，而是立即切换并下发下一分支的起步目标，可连续按键循环切换三个分支。
- PC0 进入第一分支时自动打开 PE4；若 PC0 是存二自动本模式第一次存放类操作，同时执行第一次 PD10 的夹爪动作（55°），并将下一次实际 PD10 推进到 125°；存三自动保留原有首次夹爪联动。存二自动或存三自动中按下 PD13、PD12、PD11，原按键功能照常执行，同时取消 PC0 当前流程并将该模式下一次 PC0 固定复位到第一分支。
- PE0或PD6当前档位使模式发生切换时，清除旧模式中未完成的流程、延时和六个按键的单双次状态，新模式从各按键第一段开始。
- 各模式单独保留 PD13、PD12、PC0 的首次按键历史用于 PE4 起始联动；离开自动模式再返回时，PD13/PD12 仍从第一段执行。存二自动中 PC0、PD13、PD12 三者第一次触发存放类操作时，夹爪按第一次 PD10 的逻辑联动到 55°（启用堵转保护），且下一次实际按 PD10 执行第二段 125°；存三自动保留原有行为。
- `UpperEntry_GetSecondaryRemoteControl()` 返回 PC0/PC1、PE0、右侧按键、开关和在线状态；当前已关闭远控
  超时看门狗，没有新帧时继续保留最后一次合法状态。
- `UpperEntry_GetSecondaryRemoteDiagnostics()` 可读取合法帧、忽略帧和重同步统计。

## 电机启动与控制安全

UART4 固定作为正式上位机命令与 ACK 链路，配置为 115200 8N1。正式固件不编译 VOFA
电流、速度、位置或自整定测试桥，也不解析相关文本命令。H723 上电、复位和电机驱动初始化后
只接收反馈并建立本次启动的软件零点，不发送使能、电流、速度、位置、停止或测试帧。
M3508、M2006 和 J4310 的在线参数调整默认关闭。只有完成握手后由操作者明确发送的
正式控制命令，才允许进入电机发送链路；正常角度命令仅建立从当前反馈位置到用户目标的一段轨迹。

J4310 和闸门 M2006 的位置目标带保守堵转恢复。新目标先保留 `500 ms` 启动窗口，随后只有
目标误差仍分别不小于 `15 deg`/`12 deg`、反馈速度不大于 `1 deg/s`，且反馈转矩/电流分别
不小于 `3 N·m`/`3 A`，三个条件连续保持 `3000 ms` 才确认堵转；反馈中断或任一条件恢复都会
重新计时。确认后取消尚未完成的遥控延时流程，J4310 单次复位到 `90 deg`，闸门单次复位到
`80 deg`。保护复位目标本身不重复检测；下一条正常按键目标会重新启用检测。闸门复位后普通
PD9 的下一次动作固定为 `180 deg`，再下一次为 `60 deg`，之后继续交替。

## W25Q128 SPI 驱动

W25Qxx 的 C 驱动已放在工程内 `User/Driver/Flash`，其端口配置与本板一致：`hspi1`、
`FLASH_CS_GPIO_Port` 和 `FLASH_CS_Pin`。Flash 在 `commRxTask` 启动后通过 SPI1 初始化，
因此驱动可以使用 CMSIS-RTOS2 延时，擦除或自动擦除写入不会阻塞 1 ms 控制任务。
初始化结果保存在 `w25q_init_status`，可在调试器中查看；不再通过 UART 输出 Flash 状态或接收
Flash 文本命令。应用代码通过 `W25Q_PortGetDevice()` 获取句柄，并调用 `W25Q_ReadData()`、
`W25Q_WriteData()`、`W25Q_EraseSector()` 等接口访问外置 Flash。UART4 只用于普通上位机协议。

UART4/UART5/USART1/USART2 接收使用 256-byte、32-byte 对齐、独占 Cache line 的循环 DMA
缓冲区；其余普通 UART 使用同规格的中断接收缓冲区。
`CommRuntime_PcTransmit()` 会在启动 TX DMA 前 Clean D-Cache；调用者必须保证发送
完成前缓冲区不被修改或释放。

SPI DMA 使用应用提供的缓冲区时，按以下顺序调用：

1. TX 前调用 `DmaCache_PrepareTx()`。
2. RX 启动前调用 `DmaCache_PrepareRx()`。
3. RX 完成后、CPU 读取前调用 `DmaCache_CompleteRx()`。

缓冲区应按 32 bytes 对齐并独占完整 Cache line。若后续改用 MPU non-cacheable DMA 区，
必须同步修改链接布局，不能只改 MPU 属性。

## 工程可移植性

工程使用本地 `Middlewares/Third_Party/FreeRTOS` 和 `User/Driver/Flash/W25Qxx.c`，没有指向
上级 `Common` 或工程目录外的 W25Qxx 源码。ST 的 H7 V1.13.0 配置明确对 H7 使用
`GCC/ARM_CM4F` FreeRTOS portable layer；这是该固件包的官方配置，不是 MCU 被识别成
Cortex-M4。

在其他电脑编译需要 Keil ArmClang 6 和对应 STM32H7xx DFP。发送时只需发送完整的
`H723VGT6-上层` 工程，而不是只发送 `MDK-ARM`。
