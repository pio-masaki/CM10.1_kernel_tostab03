/******************************************************************************
 * dmieep.h - Linux kernel module for PEGATRON DMI EEPROM 
 *
 * Copyright 2008-2010 Pegatron Corporation.
 *
 * DESCRIPTION:
 *	- This is the linux driver for PEGATRON DMI EEPROM 
 *
 * modification history
 * --------------------
 * v1.0   2010/11/02, Vincent Hao create this file

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


#ifndef __DMIEEP_H__
#define __DMIEEP_H__


#define	DRIVER_NAME		    "dmieep"
#define	DRIVER_VERSION		"0.7"

/* IOCTL cmd define */
#define WR_CMD1			0x0
#define WR_CMD2			0x1
#define WR_INT_LT		0x2
#define	WR_INT_HT		0x3
#define RD_CMD1			0x4
#define RD_CMD2			0x5
#define	RD_INT_LT		0x6
#define	RD_INT_HT		0x7
#define	RD_DATA			0x8
#define ST_ENABLE		0x9

/* Size of EEPROM in bytes */
#define EEPROM_SIZE     256 
/* Each client has this additional data */
struct dmieep_data_t {
    u8              data[EEPROM_SIZE];       /* Register values */    

	struct mutex    mlock;
    int             data_ready;             // 1: ready, 0: no
	int             enabled;
    int             writable;
};

#if 0
//===================================================
#define DMIEEP_INDEX_GSENSOR            1
#define DMIEEP_INDEX_CAPSENSOR          2


int dmieep_query_data ( int index );
// query data
// in-index of data, see DMIEEP_INDEX_XXX
// return- >0 : len of data buffer required
//          0 : data not ready
//         -1: no such data, index error
//         -2: internal error

int dmieep_read_data ( int index, char * buf );
// actually read data
// in-index of data, see DMIEEP_INDEX_XXX
// return- >0: len of data been read, data has been filled in buf
//          0: data not ready
//         -1: no such data, index error
//         -2: internal error
//===================================================
#endif

//###################################################################
// New method to query the CPU UID, revision, etc.
#define DMIEEP_IOC_MAGIC	123
#define DMIEEP_G_DATA       _IOR(DMIEEP_IOC_MAGIC, 0, char[32])
#define DMIEEP_G_REVISION   _IOR(DMIEEP_IOC_MAGIC, 1, char[32])
#define DMIEEP_G_SKU        _IOR(DMIEEP_IOC_MAGIC, 2, int)
#define DMIEEP_G_GSENSOR    _IOR(DMIEEP_IOC_MAGIC, 3, char[12])
#define DMIEEP_S_GSENSOR    _IOW(DMIEEP_IOC_MAGIC, 4, char[12])
#define DMIEEP_G_LIGHT      _IOR(DMIEEP_IOC_MAGIC, 5, short[2])
#define DMIEEP_S_LIGHT      _IOW(DMIEEP_IOC_MAGIC, 6, short[2])
#define DMIEEP_G_PROXIMITY  _IOR(DMIEEP_IOC_MAGIC, 7, short)
#define DMIEEP_S_PROXIMITY  _IOW(DMIEEP_IOC_MAGIC, 8, short)

// Copy from fuse.h
enum tegra_revision {
    TEGRA_REVISION_UNKNOWN = 0,
    TEGRA_REVISION_A02,
    TEGRA_REVISION_A03,
    TEGRA_REVISION_A03p,
    TEGRA_REVISION_A04,
    TEGRA_REVISION_MAX,
};

static const char *tegra_revision_name[TEGRA_REVISION_MAX] = {
    [TEGRA_REVISION_UNKNOWN] = "unknown",
    [TEGRA_REVISION_A02] = "A02",
    [TEGRA_REVISION_A03] = "A03",
    [TEGRA_REVISION_A03p] = "A03 prime",
    [TEGRA_REVISION_A04] = "A04",
};
//###################################################################

#endif

