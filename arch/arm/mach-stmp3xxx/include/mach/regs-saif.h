/*
 * STMP SAIF Register Definitions
 *
 * Copyright 2008-2009 Freescale Semiconductor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef __ARCH_ARM___SAIF_H
#define __ARCH_ARM___SAIF_H  1

#include <mach/stmp3xxx_regs.h>

#define REGS_SAIF_BASE (REGS_BASE + 0x42000)
#define REGS_SAIF1_BASE_PHYS (0x80042000)
#define REGS_SAIF2_BASE_PHYS (0x80046000)
#define REGS_SAIF_SIZE 0x00002000
HW_REGISTER(HW_SAIF_CTRL, REGS_SAIF_BASE, 0x00000000)
#define HW_SAIF_CTRL_ADDR (REGS_SAIF_BASE + 0x00000000)
#define BM_SAIF_CTRL_SFTRST 0x80000000
#define BM_SAIF_CTRL_CLKGATE 0x40000000
#define BP_SAIF_CTRL_BITCLK_MULT_RATE      27
#define BM_SAIF_CTRL_BITCLK_MULT_RATE 0x38000000
#define BF_SAIF_CTRL_BITCLK_MULT_RATE(v)  \
	(((v) << 27) & BM_SAIF_CTRL_BITCLK_MULT_RATE)
#define BM_SAIF_CTRL_BITCLK_BASE_RATE 0x04000000
#define BM_SAIF_CTRL_FIFO_ERROR_IRQ_EN 0x02000000
#define BM_SAIF_CTRL_FIFO_SERVICE_IRQ_EN 0x01000000
#define BP_SAIF_CTRL_DMAWAIT_COUNT      16
#define BM_SAIF_CTRL_DMAWAIT_COUNT 0x001F0000
#define BF_SAIF_CTRL_DMAWAIT_COUNT(v)  \
	(((v) << 16) & BM_SAIF_CTRL_DMAWAIT_COUNT)
#define BP_SAIF_CTRL_CHANNEL_NUM_SELECT      14
#define BM_SAIF_CTRL_CHANNEL_NUM_SELECT 0x0000C000
#define BF_SAIF_CTRL_CHANNEL_NUM_SELECT(v)  \
	(((v) << 14) & BM_SAIF_CTRL_CHANNEL_NUM_SELECT)
#define BM_SAIF_CTRL_BIT_ORDER 0x00001000
#define BM_SAIF_CTRL_DELAY 0x00000800
#define BM_SAIF_CTRL_JUSTIFY 0x00000400
#define BM_SAIF_CTRL_LRCLK_POLARITY 0x00000200
#define BM_SAIF_CTRL_BITCLK_EDGE 0x00000100
#define BP_SAIF_CTRL_WORD_LENGTH      4
#define BM_SAIF_CTRL_WORD_LENGTH 0x000000F0
#define BF_SAIF_CTRL_WORD_LENGTH(v)  \
	(((v) << 4) & BM_SAIF_CTRL_WORD_LENGTH)
#define BM_SAIF_CTRL_BITCLK_48XFS_ENABLE 0x00000008
#define BM_SAIF_CTRL_SLAVE_MODE 0x00000004
#define BM_SAIF_CTRL_READ_MODE 0x00000002
#define BM_SAIF_CTRL_RUN 0x00000001
HW_REGISTER(HW_SAIF_STAT, REGS_SAIF_BASE, 0x00000010)
#define HW_SAIF_STAT_ADDR (REGS_SAIF_BASE + 0x00000010)
#define BM_SAIF_STAT_PRESENT 0x80000000
#define BM_SAIF_STAT_DMA_PREQ 0x00010000
#define BM_SAIF_STAT_FIFO_UNDERFLOW_IRQ 0x00000040
#define BM_SAIF_STAT_FIFO_OVERFLOW_IRQ 0x00000020
#define BM_SAIF_STAT_FIFO_SERVICE_IRQ 0x00000010
#define BM_SAIF_STAT_BUSY 0x00000001
HW_REGISTER(HW_SAIF_DATA, REGS_SAIF_BASE, 0x00000020)
#define HW_SAIF_DATA_ADDR (REGS_SAIF_BASE + 0x00000020)
#define BP_SAIF_DATA_PCM_RIGHT      16
#define BM_SAIF_DATA_PCM_RIGHT 0xFFFF0000
#define BF_SAIF_DATA_PCM_RIGHT(v) \
	(((v) << 16) & BM_SAIF_DATA_PCM_RIGHT)
#define BP_SAIF_DATA_PCM_LEFT      0
#define BM_SAIF_DATA_PCM_LEFT 0x0000FFFF
#define BF_SAIF_DATA_PCM_LEFT(v)  \
	(((v) << 0) & BM_SAIF_DATA_PCM_LEFT)
HW_REGISTER_0(HW_SAIF_VERSION, REGS_SAIF_BASE, 0x00000030)
#define HW_SAIF_VERSION_ADDR (REGS_SAIF_BASE + 0x00000030)
#define BP_SAIF_VERSION_MAJOR      24
#define BM_SAIF_VERSION_MAJOR 0xFF000000
#define BF_SAIF_VERSION_MAJOR(v) \
	(((v) << 24) & BM_SAIF_VERSION_MAJOR)
#define BP_SAIF_VERSION_MINOR      16
#define BM_SAIF_VERSION_MINOR 0x00FF0000
#define BF_SAIF_VERSION_MINOR(v)  \
	(((v) << 16) & BM_SAIF_VERSION_MINOR)
#define BP_SAIF_VERSION_STEP      0
#define BM_SAIF_VERSION_STEP 0x0000FFFF
#define BF_SAIF_VERSION_STEP(v)  \
	(((v) << 0) & BM_SAIF_VERSION_STEP)
#endif /* __ARCH_ARM___SAIF_H */
