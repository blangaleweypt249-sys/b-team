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
| 普通 UART | PD2 RX / PC12 TX | H3.24 / H3.19 | UART5，115200 8N1，中断收发 |
| 普通 UART | PE7 RX / PE8 TX | H1.18 / H1.17 | UART7，115200 8N1，中断收发 |
| 普通 UART | PE0 RX / PE1 TX | H2.10 / H2.7 | UART8，115200 8N1，中断收发 |
| 普通 UART | PD14 RX / PD15 TX | H3.7 / H3.10 | UART9，115200 8N1，中断收发 |
| 普通 UART | PB11 RX / PB10 TX | H3.4 / H3.1 | USART3，115200 8N1，中断收发 |
| 普通 UART | PC7 RX / PC6 TX | H3.12 / H3.9 | USART6，115200 8N1，中断收发 |
| 普通 UART | PE2 RX / PE3 TX | H2.8 / H2.5 | USART10，115200 8N1，中断收发 |
| W25Q128 | PA4 / PA5 / PA6 / PA7 | NSS/SCK/MISO/MOSI | SPI1 Mode 0，25 MHz，RX/TX DMA |
| 状态灯 | PC13 | LED1 | 推挽输出，默认低 |
| 蜂鸣器 | PC15 | Q3 驱动 | 推挽输出，默认低 |

默认机器人拓扑是：FDCAN1 连接一台 J4310（电机 ID 3、Master ID 0），FDCAN2 连接两台
M3508/C620（节点 1、2），FDCAN3 连接两台 M2006/C610（节点 1、2）。进入 VOFA 模式后，
上位机可以把 M2006 或 M3508 会话放到 FDCAN1/2/3 的任意组合；每个“型号 + 总线”最多一组
会话，每组最多 8 个 ID。每条总线仍共用 `0x200`/`0x1FF` 组控制帧，未配置的槽位为零。
各电机上下文按“型号 + 总线 + 节点”隔离，目标、反馈和 PID 不跨路由共享。

为支持可选路由，三路接收过滤器均接收标准 ID `0x000..0x7FF`，但拒绝扩展帧和远程帧；
VOFA 桥只把当前会话所选的 `0x201..0x208` 反馈交给对应型号驱动，J4310 仍由原机器人链路处理。
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

普通 UART 使用 `HAL_UARTEx_ReceiveToIdle_IT()` 和 TX 中断，不占用 DMA1；USART1/2 的
RS485 DMA 配置保持不变。

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
USB 虚拟串口，H723 不启用原生 USB CDC。除此之外，H1/H2/H3 排针上的普通 UART 均可用
USB-TTL 转换器连接电脑。帧格式、超时策略、电机拓扑和目标任务职责见
[上层代码框架说明.md](../上层代码框架说明.md)。

## DJI VOFA 上位机模式

所有普通 UART（UART4、UART5、UART7、UART8、UART9、USART3、USART6、USART10）
都可以作为命令与 ACK 链路，统一为 115200 8N1。板子从哪一路普通 UART 收到命令，就从哪一路
返回 ACK 和普通状态；默认未收到命令前使用 UART4。JustFloat 遥测以 115200 8N1 广播到除
当前控制 UART 外的所有普通 UART，因此上位机可在“遥测串口”下拉框选择任意另接的 USB-TTL
适配器。USART1/USART2 仍是 MAX13488 485 通道，不参与普通控制和遥测广播。命令格式在电机
型号后增加 CAN 路由，例如：

```text
VOFA M3508 CAN 2 START IDS 2 1 2 PERIOD 20
VOFA M2006 CAN 3 START IDS 2 1 2 PERIOD 20
```

固件维护 `2 种 DJI 型号 × 3 路 FDCAN` 共 6 个独立会话，并在每条总线上合并生成 DJI 组控制帧。
不同路由可同时运行；同一 CAN 上重复占用同一 ID 会返回忙状态。未显式写 `CAN` 的旧命令仍兼容：
M3508 默认 FDCAN2，M2006 默认 FDCAN3。JustFloat 本身不带路由标签，因此只上传最近一次
被 `FOCUS` 选中的会话，其他会话的控制周期不受影响。VOFA 模式会暂停普通机器人控制，全部会话退出后恢复接收；
机器人若要重新运动仍需重新下发目标。
上位机切换已有路由时会发送 `FOCUS`，只切换遥测焦点，不改变电机目标。

## W25Q128 与 Flash 串口工具

W25Qxx 的 C 驱动已放在工程内 `User/Driver/Flash`，其端口配置与本板一致：`hspi1`、
`FLASH_CS_GPIO_Port` 和 `FLASH_CS_Pin`。Flash 在 `commRxTask` 启动后初始化，
因此驱动可以使用 CMSIS-RTOS2 延时，擦除或自动擦除写入不会阻塞 1 ms 控制任务。

在串口终端中选择任意已接入的普通 UART，配置 `115200 8N1`，Flash 文本命令以回车或换行结束。
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
扇区擦除。Flash 文本命令会临时占用最近收到命令的普通 UART，发送 `EXIT` 后恢复原有二进制上位机通信。

UART4/USART1/USART2 接收使用 256-byte、32-byte 对齐、独占 Cache line 的循环 DMA 缓冲区；
其余普通 UART 使用同规格的中断接收缓冲区。
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
