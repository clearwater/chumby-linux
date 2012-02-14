/*
	chumby_sense1.h
	bunnie -- June 2006 -- 1.4 -- linux 2.4.20
	bunnie -- March 2007 -- 2.0 -- port to Ironforge linux 2.6.16

	This file is part of the chumby sensor suite driver in the linux kernel.
	Copyright (c) Chumby Industries, 2007

	The sensor suite driver is free software; you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation; either version 2 of the
	License, or (at your option) any later version.

	The sensor suite driver is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with the Chumby; if not, write to the Free Software Foundation, Inc.,
	51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <linux/ioctl.h>

/* hysteresis defines */
#define DIM_LIGHT_TO_DARK	0
#define DIM_DARK_TO_LIGHT	1


/* local macros; eventually migrate to custom .h file */
#define FW_BEND_IS_OPEN  0x1
/* IOCTL commands */
#define SENSE1_LOOPBACK  0x0
#define SENSE1_HASH_ARG  0x1
/* etc... */

/* device numbers are allocated dynamically */
/* see /proc/devices */

#define SENSE1_IOCTL_MAGIC    'C'

/**
 *  Defines for each of the commands. Note that since we want to reduce
 *  the possibility that a user mode program gets out of sync with a given
 *  driver, we explicitly assign a value to each enumeration. This makes
 *  it more difficult to stick new ioctl's in the middle of the list.
 */
typedef enum {
	SENSE1_CMD_FIRST = 0x80,	/* just a placeholder, not a command */

	/* Insert new ioctls here */
	SENSE1_CMD_LAST,		/* another placeholder */
} SENSE1_CMD;

/*
 * Our ioctl commands
 */
#define SENSE1_IOCTL_TICKLE	_IO(SENSE1_IOCTL_MAGIC, SENSE1_CMD_TICKLE)	/* arg is int */
#define SENSE1_IOCTL_INCR	_IOWR(SENSE1_IOCTL_MAGIC, SENSE1_CMD_INCR, int)	/* arg is int */

/*
 * The following are for entries in /proc/sys/sense1
 */
#define CTL_CHUMSENSE1	0x4348554E	/* 'CHUO' in hex form */

enum {
	CTL_CHUMSENSE1_DEBUG_TRACE	= 101,
	CTL_CHUMSENSE1_DEBUG_IOCTL	= 102,
	CTL_CHUMSENSE1_DEBUG_ERROR	= 103,
	CTL_CHUMSENSE1_DIMLEVEL		= 105,
	CTL_CHUMSENSE1_HPIN		= 106,
	CTL_CHUMSENSE1_DEBOUNCE		= 107,
	CTL_CHUMSENSE1_SPKMUTE		= 111,
	CTL_CHUMSENSE1_DCPOWER		= 113,
	CTL_CHUMSENSE1_BTPOWER		= 114,
	CTL_CHUMSENSE1_DEBUG_CLEAR	= 116,
	CTL_CHUMSENSE1_RESETSERIAL	= 117,
	CTL_CHUMSENSE1_BRIGHTNESS	= 118,
};

/* data structures */
struct sense1_data {
	unsigned int dummy;
};

