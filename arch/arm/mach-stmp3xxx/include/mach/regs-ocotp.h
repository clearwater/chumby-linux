/*
 * STMP OCOTP Register Definitions
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

#ifndef __ARCH_ARM___OCOTP_H
#define __ARCH_ARM___OCOTP_H  1

#include <mach/stmp3xxx_regs.h>

#define REGS_OCOTP_BASE (REGS_BASE + 0x2c000)
#define REGS_OCOTP_BASE_PHYS (0x8002C000)
#define REGS_OCOTP_SIZE 0x00002000
HW_REGISTER(HW_OCOTP_CTRL, REGS_OCOTP_BASE, 0x00000000)
#define HW_OCOTP_CTRL_ADDR (REGS_OCOTP_BASE + 0x00000000)
#define BP_OCOTP_CTRL_WR_UNLOCK      16
#define BM_OCOTP_CTRL_WR_UNLOCK 0xFFFF0000
#define BF_OCOTP_CTRL_WR_UNLOCK(v) \
	(((v) << 16) & BM_OCOTP_CTRL_WR_UNLOCK)
#define BV_OCOTP_CTRL_WR_UNLOCK__KEY 0x3E77
#define BM_OCOTP_CTRL_RELOAD_SHADOWS 0x00002000
#define BM_OCOTP_CTRL_RD_BANK_OPEN 0x00001000
#define BM_OCOTP_CTRL_ERROR 0x00000200
#define BM_OCOTP_CTRL_BUSY 0x00000100
#define BP_OCOTP_CTRL_ADDR      0
#define BM_OCOTP_CTRL_ADDR 0x0000001F
#define BF_OCOTP_CTRL_ADDR(v)  \
	(((v) << 0) & BM_OCOTP_CTRL_ADDR)
HW_REGISTER_0(HW_OCOTP_DATA, REGS_OCOTP_BASE, 0x00000010)
#define HW_OCOTP_DATA_ADDR (REGS_OCOTP_BASE + 0x00000010)
#define BP_OCOTP_DATA_DATA      0
#define BM_OCOTP_DATA_DATA 0xFFFFFFFF
#define BF_OCOTP_DATA_DATA(v)   (v)
/*
 *  multi-register-define name HW_OCOTP_CUSTn
 *	      base 0x00000020
 *	      count 4
 *	      offset 0x10
 */
HW_REGISTER_0_INDEXED(HW_OCOTP_CUSTn, REGS_OCOTP_BASE, 0x00000020, 0x10)
#define BP_OCOTP_CUSTn_BITS      0
#define BM_OCOTP_CUSTn_BITS 0xFFFFFFFF
#define BF_OCOTP_CUSTn_BITS(v)   (v)
/*
 *  multi-register-define name HW_OCOTP_CRYPTOn
 *	      base 0x00000060
 *	      count 4
 *	      offset 0x10
 */
HW_REGISTER_0_INDEXED(HW_OCOTP_CRYPTOn, REGS_OCOTP_BASE, 0x00000060, 0x10)
#define BP_OCOTP_CRYPTOn_BITS      0
#define BM_OCOTP_CRYPTOn_BITS 0xFFFFFFFF
#define BF_OCOTP_CRYPTOn_BITS(v)   (v)
/*
 *  multi-register-define name HW_OCOTP_HWCAPn
 *	      base 0x000000A0
 *	      count 6
 *	      offset 0x10
 */
HW_REGISTER_0_INDEXED(HW_OCOTP_HWCAPn, REGS_OCOTP_BASE, 0x000000a0, 0x10)
#define BP_OCOTP_HWCAPn_BITS      0
#define BM_OCOTP_HWCAPn_BITS 0xFFFFFFFF
#define BF_OCOTP_HWCAPn_BITS(v)   (v)
HW_REGISTER_0(HW_OCOTP_SWCAP, REGS_OCOTP_BASE, 0x00000100)
#define HW_OCOTP_SWCAP_ADDR (REGS_OCOTP_BASE + 0x00000100)
#define BP_OCOTP_SWCAP_BITS      0
#define BM_OCOTP_SWCAP_BITS 0xFFFFFFFF
#define BF_OCOTP_SWCAP_BITS(v)   (v)
HW_REGISTER_0(HW_OCOTP_CUSTCAP, REGS_OCOTP_BASE, 0x00000110)
#define HW_OCOTP_CUSTCAP_ADDR (REGS_OCOTP_BASE + 0x00000110)
#define BP_OCOTP_CUSTCAP_BITS      0
#define BM_OCOTP_CUSTCAP_BITS 0xFFFFFFFF
#define BF_OCOTP_CUSTCAP_BITS(v)   (v)
HW_REGISTER_0(HW_OCOTP_LOCK, REGS_OCOTP_BASE, 0x00000120)
#define HW_OCOTP_LOCK_ADDR (REGS_OCOTP_BASE + 0x00000120)
#define BM_OCOTP_LOCK_ROM7 0x80000000
#define BM_OCOTP_LOCK_ROM6 0x40000000
#define BM_OCOTP_LOCK_ROM5 0x20000000
#define BM_OCOTP_LOCK_ROM4 0x10000000
#define BM_OCOTP_LOCK_ROM3 0x08000000
#define BM_OCOTP_LOCK_ROM2 0x04000000
#define BM_OCOTP_LOCK_ROM1 0x02000000
#define BM_OCOTP_LOCK_ROM0 0x01000000
#define BM_OCOTP_LOCK_HWSW_SHADOW_ALT 0x00800000
#define BM_OCOTP_LOCK_CRYPTODCP_ALT 0x00400000
#define BM_OCOTP_LOCK_CRYPTOKEY_ALT 0x00200000
#define BM_OCOTP_LOCK_PIN 0x00100000
#define BM_OCOTP_LOCK_OPS 0x00080000
#define BM_OCOTP_LOCK_UN2 0x00040000
#define BM_OCOTP_LOCK_UN1 0x00020000
#define BM_OCOTP_LOCK_UN0 0x00010000
#define BP_OCOTP_LOCK_UNALLOCATED      11
#define BM_OCOTP_LOCK_UNALLOCATED 0x0000F800
#define BF_OCOTP_LOCK_UNALLOCATED(v)  \
	(((v) << 11) & BM_OCOTP_LOCK_UNALLOCATED)
#define BM_OCOTP_LOCK_ROM_SHADOW 0x00000400
#define BM_OCOTP_LOCK_CUSTCAP 0x00000200
#define BM_OCOTP_LOCK_HWSW 0x00000100
#define BM_OCOTP_LOCK_CUSTCAP_SHADOW 0x00000080
#define BM_OCOTP_LOCK_HWSW_SHADOW 0x00000040
#define BM_OCOTP_LOCK_CRYPTODCP 0x00000020
#define BM_OCOTP_LOCK_CRYPTOKEY 0x00000010
#define BM_OCOTP_LOCK_CUST3 0x00000008
#define BM_OCOTP_LOCK_CUST2 0x00000004
#define BM_OCOTP_LOCK_CUST1 0x00000002
#define BM_OCOTP_LOCK_CUST0 0x00000001
/*
 *  multi-register-define name HW_OCOTP_OPSn
 *	      base 0x00000130
 *	      count 4
 *	      offset 0x10
 */
HW_REGISTER_0_INDEXED(HW_OCOTP_OPSn, REGS_OCOTP_BASE, 0x00000130, 0x10)
#define BP_OCOTP_OPSn_BITS      0
#define BM_OCOTP_OPSn_BITS 0xFFFFFFFF
#define BF_OCOTP_OPSn_BITS(v)   (v)
/*
 *  multi-register-define name HW_OCOTP_UNn
 *	      base 0x00000170
 *	      count 3
 *	      offset 0x10
 */
HW_REGISTER_0_INDEXED(HW_OCOTP_UNn, REGS_OCOTP_BASE, 0x00000170, 0x10)
#define BP_OCOTP_UNn_BITS      0
#define BM_OCOTP_UNn_BITS 0xFFFFFFFF
#define BF_OCOTP_UNn_BITS(v)   (v)
/*
 *  multi-register-define name HW_OCOTP_ROMn
 *	      base 0x000001A0
 *	      count 8
 *	      offset 0x10
 */
HW_REGISTER_0_INDEXED(HW_OCOTP_ROMn, REGS_OCOTP_BASE, 0x000001a0, 0x10)
#define BP_OCOTP_ROMn_BITS      0
#define BM_OCOTP_ROMn_BITS 0xFFFFFFFF
#define BF_OCOTP_ROMn_BITS(v)   (v)
HW_REGISTER_0(HW_OCOTP_VERSION, REGS_OCOTP_BASE, 0x00000220)
#define HW_OCOTP_VERSION_ADDR (REGS_OCOTP_BASE + 0x00000220)
#define BP_OCOTP_VERSION_MAJOR      24
#define BM_OCOTP_VERSION_MAJOR 0xFF000000
#define BF_OCOTP_VERSION_MAJOR(v) \
	(((v) << 24) & BM_OCOTP_VERSION_MAJOR)
#define BP_OCOTP_VERSION_MINOR      16
#define BM_OCOTP_VERSION_MINOR 0x00FF0000
#define BF_OCOTP_VERSION_MINOR(v)  \
	(((v) << 16) & BM_OCOTP_VERSION_MINOR)
#define BP_OCOTP_VERSION_STEP      0
#define BM_OCOTP_VERSION_STEP 0x0000FFFF
#define BF_OCOTP_VERSION_STEP(v)  \
	(((v) << 0) & BM_OCOTP_VERSION_STEP)
#endif /* __ARCH_ARM___OCOTP_H */
