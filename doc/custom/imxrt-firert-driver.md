# i.MXRT FlexSPI NOR Flash Driver (firert)

## Overview

`firert` is an OpenOCD flash driver for i.MXRT1052 (and compatible) series,
targeting external NOR flash connected via FlexSPI. It uses a target-side stub
architecture: a small firmware is loaded into on-chip RAM, where it directly
operates the FlexSPI peripheral for flash erase/program operations.

## Architecture

```
OpenOCD host                    i.MXRT1052 target (RAM)
+-------------------+           +---------------------+
| firert_flash      |  SWD/DAP  | firert_stub         |
|  .probe()    -----|---------->|  read JEDEC ID      |
|  .erase()    -----|-- cmd --> |  erase (4K/32K/64K/ |
|  .write()    -----|-- mb  --> |   chip)              |
|                   |           |  page program        |
+-------------------+           +---------------------+
                                       |
                                       v
                                FlexSPI -> NOR Flash
                                (W25Q256JV or compat.)
```

### Key components

| File | Purpose |
|------|---------|
| `src/flash/nor/imxrt_firert.c` | OpenOCD flash driver (host side) |
| `contrib/loaders/flash/imxrt_firert_stub/stub.c` | Target-side stub firmware |
| `contrib/loaders/flash/imxrt_firert_stub/protocol.h` | Mailbox protocol definition |
| `contrib/loaders/flash/imxrt_firert_stub/linker.ld` | Stub linker script |
| `src/flash/nor/imxrt_firert_stub_bin.inc` | Pre-built stub binary (C array) |
| `tcl/board/imxrt1052_firert_cmsisdap.cfg` | Board config entry point |
| `tcl/target/imxrt1052_firert.cfg` | Target/chip config |

### Memory map

| Region | Address | Size | Usage |
|--------|---------|------|-------|
| FlexSPI flash | `0x60000000` | 32 MB | XIP flash (W25Q256JV) |
| Work area | `0x20200000` | 128 KB | Mailbox (first 256 B) + work buffer |
| Stub code | `0x20202000` | 16 KB | Stub firmware (.text + .data + .bss) |
| Stub stack | `0x20205FF0` | - | Stack pointer |
| Stub data buffer | `0x20206000` | 64 KB | Data transfer buffer |

### Mailbox protocol

Located at `0x20200000` (`FIRET_MB_ADDR`):

```
Offset  Field       Description
0x00    magic       0x46525431 ("FRT1")
0x04    cmd         0=NONE, 1=JEDEC, 2=ERASE, 3=PROGRAM
0x08    addr        Flash offset (erase/program) or 0
0x0C    size        Byte count (program) or 0
0x10    arg         Erase kind: 1=4K, 2=32K, 3=64K, 4=CHIP
0x14    src         Source address in RAM (program data)
0x18    status      0x10=BOOTING, 0x11=READY, 0x12=BUSY, 0x13=DONE, 0x1F=ERROR
0x1C    result      JEDEC ID (from probe)
0x20    detail0     Extended status (SDK return code on error)
0x24    detail1     Extended status
```

Stub signals completion by executing `bkpt 0xab` (instruction `0xBEAB`).

## Supported operations

- **Probe**: Read JEDEC ID via FlexSPI, match against OpenOCD flash device table
- **Erase**: 4 KB / 32 KB / 64 KB block erase, or full chip erase
- **Program**: Page program (256-byte pages) with built-in read-back verify
- **Read**: Direct memory-mapped read via FlexSPI AHB (default_flash_read)
- **Protect**: Stub (no hardware write-protect support)

## Configuration

### Board config: `tcl/board/imxrt1052_firert_cmsisdap.cfg`

```tcl
source [find interface/cmsis-dap.cfg]
transport select swd

# Adapter speed (override via -c "set ADAPTER_KHZ N")
if { [info exists ADAPTER_KHZ] } {
    adapter speed $ADAPTER_KHZ
} else {
    adapter speed 1000
}

reset_config none
source [find target/imxrt1052_firert.cfg]
```

### SWD adapter speed

Override via command line:

```bash
# 1 MHz (default, most stable)
./src/openocd -s tcl -c "set ADAPTER_KHZ 1000" -f board/imxrt1052_firert_cmsisdap.cfg ...

# 2 MHz
./src/openocd -s tcl -c "set ADAPTER_KHZ 2000" -f board/imxrt1052_firert_cmsisdap.cfg ...

# 4 MHz
./src/openocd -s tcl -c "set ADAPTER_KHZ 4000" -f board/imxrt1052_firert_cmsisdap.cfg ...
```

> USB transfer warnings (`busy command USB transfer`) may appear at 2+ MHz
> with some CMSIS-DAP adapters. These are non-fatal and do not affect flash
> operation correctness.

## Test procedures

### Prerequisites

- OpenOCD built with firert driver (`./configure && make`)
- i.MXRT1052 board connected via CMSIS-DAP (SWD)
- Test binary (e.g. `dd if=/dev/urandom of=test.bin bs=1024 count=64`)

### Test 1: Probe and identify flash

Verify JEDEC ID detection and flash bank registration.

```bash
./src/openocd -s tcl \
  -f board/imxrt1052_firert_cmsisdap.cfg \
  -c "init; reset halt; flash probe 0; flash info 0; shutdown"
```

**Expected output:**

```
Info : imxrt stub boot: status=0x00000011 result=0x001940ef ...
Info : Found flash device 'win w25q256fv/jv' raw JEDEC 0x001940ef normalized 0x00ef4019
```

- `status=0x00000011` = FIRET_ST_READY
- `result=0x001940ef` = JEDEC ID (manufacturer 0xEF Winbond, type 0x40, capacity 0x19 = 256Mbit)
- `normalized 0x00ef4019` = device_id format `(mfgr<<16)|(cap<<8)|type)`

### Test 2: Sector erase (1 MB)

```bash
./src/openocd -s tcl -c "set ADAPTER_KHZ 1000" \
  -f board/imxrt1052_firert_cmsisdap.cfg \
  -c "init; reset halt; flash probe 0; flash erase_address 0x60200000 0x100000; shutdown"
```

**Expected:** Clean completion, no ERROR lines. For W25Q256JV, this performs
16 x 64 KB erase operations. Each appears as a halt/resume cycle in the log.

### Test 3: Program + verify (low address, 1 MHz)

```bash
./src/openocd -s tcl -c "set ADAPTER_KHZ 1000" \
  -f board/imxrt1052_firert_cmsisdap.cfg \
  -c "init; reset halt; flash probe 0; program test.bin 0x60200000 verify; shutdown"
```

**Expected:**

```
** Programming Started **
...
** Programming Finished **
** Verify Started **
** Verified OK **
```

### Test 4: Program + verify (mid address, 2 MHz)

```bash
./src/openocd -s tcl -c "set ADAPTER_KHZ 2000" \
  -f board/imxrt1052_firert_cmsisdap.cfg \
  -c "init; reset halt; flash probe 0; program test.bin 0x60300000 verify; shutdown"
```

**Expected:** Same as Test 3. USB transfer warnings at 2 MHz are acceptable.

### Test 5: Program + verify (mid address, 4 MHz)

```bash
./src/openocd -s tcl -c "set ADAPTER_KHZ 4000" \
  -f board/imxrt1052_firert_cmsisdap.cfg \
  -c "init; reset halt; flash probe 0; program test.bin 0x60300000 verify; shutdown"
```

**Expected:** Same as Test 3. Verifies high-speed stability.

### Test 6: Full chip erase (32 MB)

```bash
./src/openocd -s tcl -c "set ADAPTER_KHZ 1000" \
  -f board/imxrt1052_firert_cmsisdap.cfg \
  -c "init; reset halt; flash probe 0; flash erase_address 0x60000000 0x2000000; shutdown"
```

**Expected:** Uses chip erase command (0xC7), single stub operation. Allow up
to 3 minutes for large flash.

### Acceptance checklist

- [ ] `flash probe 0` returns stable JEDEC ID (e.g. `0x001940EF`)
- [ ] Sector erase completes without errors
- [ ] Program + verify passes at 1 MHz
- [ ] Program + verify passes at 2 MHz
- [ ] Program + verify passes at 4 MHz
- [ ] Full chip erase completes without errors
- [ ] High-address region (>= `0x60300000`) accessible

## Troubleshooting

### Stub boot failure (magic mismatch)

```
Error: i.MXRT stub mailbox magic mismatch: 0x????????
```

Possible causes:
- Work area overlap (check nothing else uses `0x20200000`-`0x2020FFFF`)
- Stub binary corrupted (rebuild: `make` in top-level)
- Target not properly halted before stub load

### USB transfer errors

```
Error: busy command USB transfer at 0
Error: USB write: late transfer competed
```

Non-fatal CMSIS-DAP USB timing issue. If operations fail consistently, reduce
adapter speed (`set ADAPTER_KHZ 1000`).

### JEDEC ID reads as 0x000000 or 0xFFFFFF

- FlexSPI not initialized: stub handles pinmux/clock/FlexSPI init, but check
  that the board's flash is connected to FlexSPI Port A1
- Flash chip not responding: check hardware connections

### Verify mismatch

```
Error: verify mismatch at flash offset 0x...
```

- Flash sector not erased before programming
- Flash timing issue at high speed (reduce ADAPTER_KHZ)
- Defective flash sector

## Building from source

```bash
# Configure (enable CMSIS-DAP driver)
./configure --enable-cmsis-dap

# Build
make -j$(nproc)

# Verify firert driver is linked
strings src/openocd | grep firert
```

## Supported flash chips

Any SPI NOR flash with standard command set (JEDEC compatible). Verified:

| Chip | JEDEC ID | Size |
|------|----------|------|
| Winbond W25Q256JV | `0x001940EF` / `0xEF4019` | 32 MB |

Other chips will work if their JEDEC ID is in OpenOCD's `flash_devices[]` table
(`src/flash/nor/spi.c`). Unknown chips fall back to 32 MB size with a warning.
