/******************************************************************************
 * dmi_pri.h - Linux kernel module for PEGATRON DMI EEPROM
 *
 * Copyright 2008-2010 Pegatron Corporation.
 *
 * DESCRIPTION:
 *	- This is the private data header for PEGATRON DMI EEPROM driver
 *
 * modification history
 * --------------------
 * v1.0   2010/11/09, Vincent Hao create this file

 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/


#ifndef __DMI_PRI_H__
#define __DMI_PRI_H__

/* Size of EEPROM in bytes */
#define DMI_TOTAL_SIZE      256
#define DMI_TOD_SIZE        124
#define DMI_PEGA_OFFSET     (DMI_TOD_SIZE)
#define DMI_PEGA_SIZE       (DMI_TOTAL_SIZE-DMI_TOD_SIZE)


typedef struct
{
    char    *name;
    int     offset;
    int     len;
 
} Dmi_Privdata_Ent;

Dmi_Privdata_Ent Dmi_Privdata_Info [] =
{
//name                       offset                     len
//============================================================
{"G-Sensor-Calib",           0,                         12,     },  // 12 bytes, for x, y, z
{"Light-Sensor-Calib",       12,                        4,      },  // 4 bytes, scale, range
{"WiFi-Country-Code",        16,                        2,      },  // 2 bytes
{"Proximity-Calib",          18,                        2,      },  // 2 bytes
{NULL,                       0,                         0,      },
};

#endif

