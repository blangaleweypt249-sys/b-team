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
| RS485-2 | PA3 RX / PA2 TX | MAX13488 U10 | USART2，115200 8N1，RX/TX DMA |
| USB 串口 | PA1 RX / PA0 TX | CH340N TXD/RXD | UART4，115200 8N1，RX/TX DMA |
| W25Q128 | PA4 / PA5 / PA6 / PA7 | NSS/SCK/MISO/MOSI | SPI1 Mode 0，25 MHz，RX/TX DMA |
| 状态灯 | PC13 | LED1 | 推挽输出，默认低 |
| 蜂鸣器 | PC15 | Q3 驱动 | 推挽输出，默认低 |

FDCAN1 连接两台 M3508/C620（节点 1、2）和一台 J4310（电机 ID 3、Master ID 0）；
FDCAN2 连接另外两台 M3508/C620（节点 1、2）；FDCAN3 连接两台 M2006/C610，传送带为
节点 1，夹爪为节点 2。三条总线上的 DJI 节点各自共用本总线的一个 `0x200` 组控制帧，
节点 3、4 对应槽位始终为零。M3508 的运行上下文按“总线 + 节点”隔离，FDCAN1 和
FDCAN2 的同号节点不会共享目标、反馈或 PID。

接收过滤器按实际拓扑精确配置：FDCAN1 只接收 J4310 Master ID 0 以及 `0x201`、`0x202`，
FDCAN2 和 FDCAN3 只接收 `0x201`、`0x202`；其他标准帧、扩展帧和远程帧均拒绝。
三路控制周期均为 1 ms。按 135 bit/帧保守估算，FDCAN1 持续 5 帧/ms，占用 67.5%；
FDCAN2、FDCAN3 各 3 帧/ms，占用 40.5%，均保留超过 20% 的持续带宽余量。FDCAN1
每周期先将 M3508 `0x200` 组帧加入发送 FIFO，再加入 J4310 控制或状态帧。占用率仅用于
设计评估，初始化不设置 80% 带宽门限，超过 80% 的合法电机配置也允许启动。

网表中的 LED2、LED3 是电源指示，BOOT0 和 NRST 由硬件电路管理。只接到排针、
尚未分配业务的 GPIO 保持复位状态，避免初始工程抢占后续外设复用。

## DMA 分配

| DMA1 Stream | 请求 | 模式 |
| --- | --- | --- |
| 0 / 1 | UART4 RX / TX | RX Circular / TX Normal |
| 2 / 3 | USART1 RX / TX | RX Circular / TX Normal |
| 4 / 5 | USART2 RX / TX | RX Circular / TX Normal |
| 6 / 7 | SPI1 RX / TX | Normal / Normal |

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

void App_Periodic10ms(void);
void App_Control1ms(void);
```

Type-C 的 D+/D- 实际连接 CH340N，H723 侧通过 UART4 PA0/PA1 与其通信。小电脑看到的是
USB 虚拟串口，H723 不启用原生 USB CDC。帧格式、超时策略、电机拓扑和目标任务职责见
[上层代码框架说明.md](../上层代码框架说明.md)。

## W25Q128 与 VOFA 串口工具

W25Qxx 的 C 驱动已放在工程内 `User/Driver/Flash`，其端口配置与本板一致：`hspi1`、
`FLASH_CS_GPIO_Port` 和 `FLASH_CS_Pin`。Flash 在 `commRxTask` 启动后初始化，
因此驱动可以使用 CMSIS-RTOS2 延时，擦除或自动擦除写入不会阻塞 1 ms 控制任务。

在 VOFA 中选择 CH340 对应串口，配置 `115200 8N1`，文本命令以回车或换行结束。
地址无前缀时按十进制解析，也可以使用 `0x` 前缀输入十六进制地址；写入数据字节按
十六进制解析。支持以下命令：

固件复位后会主动发送 `FLASH READY COMM=<通信初始化状态> INIT=<Flash 初始化状态>
JEDEC_ID=<0x9F 返回值> DEVICE_STATUS=<0x90 读取状态> DEVICE_ID=<0x90 返回值>
MODEL=<型号>`。如果上层业务初始化失败，则发送
`FLASH BOOT ERROR UPPER_INIT` 后停机。这两条启动消息都不依赖接收命令或换行设置。

```text
FLASH
HELP
INFO
ERASE 0x000000 [sector_count]
WRITE 0x000000 12 34 AB CD
READ 0x000000 4
EXIT
```

W25Q128 正常时，`INFO` 应包含 `JEDEC_ID=0xEF4018 DEVICE_STATUS=0
DEVICE_ID=0xEF17 MODEL=W25Q128 CAPACITY_KB=16384`。`ERASE` 地址必须
按 `0x1000`（4 KiB）对齐；`WRITE` 最多写入 256 Byte，并由驱动自动处理跨页和必要的
扇区擦除。Flash 文本命令会临时占用 UART4，发送 `EXIT` 后恢复原有二进制上位机通信。

三路 UART 接收使用 256-byte、32-byte 对齐、独占 Cache line 的循环 DMA 缓冲区。
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
