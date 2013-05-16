/******************************************************************************
 * dmi_api.h - Linux kernel module interface for PEGATRON DMI EEPROM
 *
 * Copyright 2008-2010 Pegatron Corporation.
 *
 * DESCRIPTION:
 *	- This is the private api header for PEGATRON DMI EEPROM driver
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


#ifndef __DMI_API_H__
#define __DMI_API_H__

#define DMIEEP_INDEX_MIN                1
//------------------------------------------
#define DMIEEP_INDEX_GSENSOR            1
#define DMIEEP_INDEX_LIGHTSENSOR        2
#define DMIEEP_INDEX_WIFI_COUNTRYCODE   3
#define DMIEEP_INDEX_PROXIMITY          4
//------------------------------------------
#define DMIEEP_INDEX_MAX                4


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
//         -1: parameter error
//         -2: no such data, index error

int dmieep_write_data ( int index, char * buf, int len );
// actually read data
// in-index of data, see DMIEEP_INDEX_XXX
// return- >0: len of data been written, should be equal to len
//          0: data not ready
//         -1: parameter error
//         -2: no such data, index error
//===========================
//
// dmieep driver supports sysfs for read onlu functionality
// location: /sys/devices/platform/tegra_dmieep
//      bt_mac_id: for colon separated bluetooth MAC address
//      cap_sensor_data: dump hex bytes of cap-sensor data
//      G_sensor_data: dump hex bytes of G-sensor data
//      light_sensor_data: dump hex bytes of light-sensor data
//
//===========================

#endif

