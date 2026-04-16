/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_FLASH_NOR_SWM341_SFC_H
#define OPENOCD_FLASH_NOR_SWM341_SFC_H

#define SWM341_SFC_MAP_BASE                  0x70000000u
#define SWM341_SFC_DEFAULT_SIZE              0x01000000u
#define SWM341_SFC_DEFAULT_SECTOR_SIZE       0x00001000u
#define SWM341_SFC_DEFAULT_PAGE_SIZE         0x00000100u

#define SWM341_SYS_CLKEN1                    0x4000000Cu
#define SWM341_SYS_CLKEN0                    0x40000008u
#define SWM341_SYS_CLKEN0_GPIOD              (1u << 3)
#define SWM341_SYS_CLKEN1_SFC                (1u << 13)

#define SWM341_PORTD_BASE                    0x400A0030u
#define SWM341_PORT_FUNC0                    0x00u
#define SWM341_PORT_FUNC1                    0x04u
#define SWM341_PORT_INEN                     0x300u

#define SWM341_SFC_BASE                      0x4004A800u
#define SWM341_SFC_CFG                       (SWM341_SFC_BASE + 0x00u)
#define SWM341_SFC_TIM                       (SWM341_SFC_BASE + 0x04u)
#define SWM341_SFC_SR                        (SWM341_SFC_BASE + 0x08u)
#define SWM341_SFC_GO                        (SWM341_SFC_BASE + 0x14u)
#define SWM341_SFC_ADDR                      (SWM341_SFC_BASE + 0x18u)
#define SWM341_SFC_DATA                      (SWM341_SFC_BASE + 0x1Cu)
#define SWM341_SFC_CMDAHB                    (SWM341_SFC_BASE + 0x20u)
#define SWM341_SFC_CMD                       (SWM341_SFC_BASE + 0x24u)
#define SWM341_SFC_UNLOCK                    (SWM341_SFC_BASE + 0x3F4u)

#define SWM341_SFC_CFG_CMDTYPE_MASK          (0x0Fu << 0)
#define SWM341_SFC_CFG_CMDWREN               (1u << 5)
#define SWM341_SFC_CFG_CLKDIV_MASK           (0x03u << 6)
#define SWM341_SFC_CFG_DATA4L_PP_MASK        (1u << 9)
#define SWM341_SFC_CFG_DATA4L_RD_MASK        (0x03u << 10)
#define SWM341_SFC_CFG_WREN                  (1u << 12)

#define SWM341_SFC_SR_BUSY                   (1u << 0)

#define SWM341_SFC_TIM_WIP_CHK_ITV_MASK      (0xFFu << 0)
#define SWM341_SFC_TIM_WIP_CHK_LMT_MASK      (0xFFu << 8)

#define SWM341_SFC_CMD_WREN                  0x06u
#define SWM341_SFC_CMD_READ_JEDEC            0x9Fu
#define SWM341_SFC_CMD_READ_STATUS1          0x05u
#define SWM341_SFC_CMD_READ_STATUS2          0x35u
#define SWM341_SFC_CMD_WRITE_STATUS1         0x01u
#define SWM341_SFC_CMD_PAGE_PROGRAM          0x02u
#define SWM341_SFC_CMD_SECTOR_ERASE          0x20u
#define SWM341_SFC_CMD_CHIP_ERASE            0x60u

#define SWM341_SFC_LOADER_STACK_SIZE         256u

struct swm341_sfc_loader_cfg {
	uint32_t sfc_cfg_addr;
	uint32_t sfc_sr_addr;
	uint32_t map_base;
	uint32_t page_size;
	uint32_t last_error;
	uint8_t stack[SWM341_SFC_LOADER_STACK_SIZE];
};

#endif /* OPENOCD_FLASH_NOR_SWM341_SFC_H */
