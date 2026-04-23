OUT := build
TP := third_party

CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

CFLAGS := -mcpu=cortex-m7 -mthumb -mfloat-abi=soft \
	-Os -ffreestanding -fno-builtin -fdata-sections -ffunction-sections \
	-fno-common -fomit-frame-pointer -Wall -Wextra -Wno-unused-parameter \
	-Wno-unused-function -DNDEBUG \
	-DCPU_MIMXRT1052CVL5B \
	-I$(TP)/devices/MIMXRT1052 \
	-I$(TP)/devices/MIMXRT1052/periph \
	-I$(TP)/Core/Include \
	-I$(TP)/Core/Include/CMSIS \
	-I.

LDFLAGS := -nostdlib -nostartfiles -Wl,--gc-sections -T linker.ld

SRCS := stub.c
OBJS := $(OUT)/stub.o

all: $(OUT)/firert_stub.bin

$(OUT):
	mkdir -p $(OUT)

$(OUT)/stub.o: stub.c protocol.h | $(OUT)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/firert_stub.elf: $(OBJS) linker.ld
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(OUT)/firert_stub.bin: $(OUT)/firert_stub.elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(OUT)

.PHONY: all clean
