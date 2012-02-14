/*
       chumby_timerx.h
       bunnie -- June 2006 -- 1.4 -- linux 2.4.20
       bunnie -- March 2007 -- 2.0 -- port to Ironforge linux 2.6.16

       This file is part of the chumby sensor suite driver in the linux kernel.
       Copyright (c) Chumby Industries, 2007

       The sensor suite driver is free software; you can redistribute it and/or modify
       it under the terms of the GNU General Public License as published by
       the Free Software Foundation; either version 2 of the License, or
       (at your option) any later version.

       The sensor suite driver is distributed in the hope that it will be useful,
       but WITHOUT ANY WARRANTY; without even the implied warranty of
       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
       GNU General Public License for more details.

       You should have received a copy of the GNU General Public License
       along with the Chumby; if not, write to the Free Software
       Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <linux/ioctl.h>


// device numbers are allocated dynamically
// see /proc/devices

/*
 * The following are for entries in /proc/sys/timerx
 */
#define CTL_CHUMTIMERX    0x43485551  /* 'CHUQ' in hex form */

enum
{
    CTL_CHUMTIMERX_DEBUG_TRACE    = 101,
    CTL_CHUMTIMERX_DEBUG_IOCTL    = 102,
    CTL_CHUMTIMERX_DEBUG_ERROR    = 103,
};

// data structures
struct timerx_data {
  unsigned long jiffies;
};

