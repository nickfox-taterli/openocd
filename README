# OpenOCD 个人改动简报

> 基于`openocd-cubeide-r7`分支自 `e7ba36bfe` 起所有自定义提交都会定期提交到这个简报里.

---

## Renesas RA2L1 闪存驱动

为 Renesas RA2L1 新增完整的片内闪存编程支持.

| 项目 | 详情 |
|------|------|
| Code Flash | 256 KB @ `0x00000000`, 2 KB 擦除块, 4 字节编程单元 |
| Data Flash | 8 KB @ `0x40100000`, 1 KB 擦除块, 1 字节编程单元 |
| Flash 控制器基址 | `0x407EC000` (FLCN) |
| 工作区 | 默认 24 KB SRAM |

**核心功能**:
- 双路径写入: 快速 Loader 路径 (目标端执行) + DAP 直写回退路径
- PRCR 寄存器保护/解锁机制
- 完整的错误检测与状态寄存器 (FSTATR/FASTAT/FCMDR) 诊断
- 支持 CMSIS-DAP / SWD 调试探头

---

## Renesas RA6M5 闪存驱动

为 Renesas RA6M5 新增片内 Code Flash 和 Data Flash 编程支持.

| 项目 | 详情 |
|------|------|
| Code Flash | 2 MB @ `0x00000000`, 8 KB / 32 KB 混合擦除块, 128 字节编程单元 |
| Data Flash | 8 KB @ `0x08000000`, 64 字节擦除块, 4 字节编程单元 |
| 工作区 | 默认 128 KB SRAM |

**核心功能**:
- **双路径写入**:
  - *快速路径*: 同步 Loader 算法, 利用全部可用工作区进行批量传输
  - *回退路径*: DAP 直接写入, Loader 失败时自动降级
- **FMEPROT 保护管理**: 写 `0xD900` 解锁 → 执行操作 → 写 `0xD901` 锁定 (此为关键修复, 缺失时快速路径会触发 Command Lock 错误)
- **缓存一致性**: 编程期间自动保存/恢复 FCACHEE 和 CC_ACTL 寄存器
- **中断管理**: Loader 端 `cpsid i` 禁中断, BKPT 返回宿主机

---

## Renesas RA6M5 XSPI (外部 QSPI) 闪存驱动

为 RA6M5 外部 QSPI 接口新增独立闪存驱动, 支持大容量外部 Flash 编程.

| 项目 | 详情 |
|------|------|
| 映射基址 | `0x60000000` |
| 默认页大小 | 256 字节 |
| 默认扇区大小 | 4 KB |
| 最大寻址 | 支持 3 字节 (≤16 MB) 和 4 字节 (>16 MB) 地址模式 |
| 超时 | 状态轮询 3 ms, 扇区擦除 4 ms |

**支持的 SPI 命令**:
| 命令码 | 功能 |
|--------|------|
| `0x06` | Write Enable |
| `0x05` | Read Status Register |
| `0x02` | Page Program |
| `0x20` | Sector Erase |
| `0x9F` | Read JEDEC ID |

**核心功能**:
- **快速路径**: Loader 算法直接在目标端执行, 禁中断优化, 适合大容量外部 Flash
- **回退路径**: DAP 直接驱动 QSPI 控制器 (SFMCOM/SFMCMD), 无需目标工作区
- **自动地址模式检测**: 根据配置切换 3/4 字节地址模式
- **硬件初始化**: 自动配置 QSPI 引脚和控制器时序

---

## STM32H7R 板级配置与 stmqspi 优化

针对 STM32H7R/S7 OCTOSPI 控制器的可靠性优化, 并新增 ART-Pi2 开发板配置.

### stmqspi 驱动优化 (`src/flash/nor/stmqspi.c`)

| 改动 | 说明 |
|------|------|
| 新增 `stmqspi_dr_read8()` / `stmqspi_dr_write8()` | 以 32 位访问 OCTOSPI_DR 再提取/注入低字节, 解决部分目标 byte 访问不可靠导致超时的问题 |
| Abort 流程改进 | 先检查控制器是否空闲, 避免在空闲态强制 ABORT 导致 H7R 上 BUSY 位卡死 |
| 全局替换直接 DR 访问 | `read_status_reg`, SFDP 读取, Flash ID 读取等函数统一使用新的 8 位封装函数 |

### 新增板级配置

**文件**: `tcl/board/rt-thread-stm32h7r-base-board.cfg`

- 目标: STM32H7R7 (ART-Pi2 开发板)
- 接口: ST-Link DAP, DAPdirect SWD
- XSPI2 外部 Flash: 映射至 `0x70000000`, 适配 W35T51NWTBIE
- 包含完整的 QSPI GPIO / 时钟 / 控制器初始化脚本
- Reset 事件自动调用 `qspi_init()`
