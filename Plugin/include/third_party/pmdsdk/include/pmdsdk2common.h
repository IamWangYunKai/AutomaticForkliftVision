/*****************************************************************************
 * PMDSDK 2
 *
 * Copyright (c) 2006-2013 PMD Technologies GmbH
 * All Rights Reserved.
 *
 * File: pmdsdk2common.h
 * Author: Martin Profittlich
 * Created: 20060530
 *
 *****************************************************************************/
#pragma once

#include <stddef.h>



/*!
    @ingroup Sdk
    @file pmdsdk2common.h
    @brief Common definitions and structs used by the PMDSDK2.
 */

/*!
    @addtogroup Common
    @{
 */

// Types

/// Single precision float
typedef float          PMDFloat32;
/// Double precision float
typedef double         PMDFloat64;

/// Standard float
typedef PMDFloat32     PMDFloat;

/// Unsigned 32-bit integer
typedef unsigned       PMDUInt32;
/// Signed 32-bit integer
typedef signed int     PMDSInt32;

/// Unsigned 16-bit integer
typedef unsigned short PMDUInt16;
/// Signed 16-bit integer
typedef signed short   PMDSInt16;

/// Unsigned 8-bit integer
typedef unsigned char  PMDUInt8;
/// Signed 8-bit integer
typedef signed char    PMDSInt8;

/*!
    @brief Enumeration for retrieving valid integration times and modulation
           frequencies
 */
typedef enum { AtLeast, AtMost, CloseTo } Proximity;

/*!
    @}
 */

// Resultcodes

/// \addtogroup StatusCodes
/// @{

/// Success
#define PMD_OK                0

/// Runtime error
#define PMD_RUNTIME_ERROR     1024
/// Generic error
#define PMD_GENERIC_ERROR     1025
/// Connection lost
#define PMD_DISCONNECTED      1026
/// An invalid value was given
#define PMD_INVALID_VALUE     1027
/// A timeout occurred
#define PMD_TIMEOUT_ERROR     1028

/// Program error
#define PMD_LOGIC_ERROR       2048
/// Handle not known (internal error)
#define PMD_UNKNOWN_HANDLE    2049
/// Requested functionality not implemented
#define PMD_NOT_IMPLEMENTED   2050
/// Index or value out of range
#define PMD_OUT_OF_BOUNDS     2051

/// Could not get resource
#define PMD_RESOURCE_ERROR    4096
/// Could not open file
#define PMD_FILE_NOT_FOUND    4097
/// Could not connect to or open the data source
#define PMD_COULD_NOT_OPEN    4098
/// Could not retrieve the requested data
#define PMD_DATA_NOT_FOUND    4099
/// There is no more data left
#define PMD_END_OF_DATA       4100
/// The device currently used and can not be accessed
#define PMD_DEVICE_IS_BUSY    4101

/*!
    @}
 */

// Versions

/*!
    @cond plugin_all_doc
    @addtogroup Versions
    @{
*/

/// Version 1.0.0 of the plugin interface
#define PMD_INTERFACE_VERSION_1_0_0 0x00010000
/// Version 1.1.0 of the plugin interface
#define PMD_INTERFACE_VERSION_1_1_0 0x00010100
/// Version 1.1.1 of the plugin interface
#define PMD_INTERFACE_VERSION_1_1_1 0x00010101
/// Version 1.2.0 of the plugin interface
#define PMD_INTERFACE_VERSION_1_2_0 0x00010200
/// Version 1.2.1 of the plugin interface
#define PMD_INTERFACE_VERSION_1_2_1 0x00010201
/// Version 1.3.0 of the plugin interface
#define PMD_INTERFACE_VERSION_1_3_0 0x00010300
/// Version 2.0.0 of the plugin interface
#define PMD_INTERFACE_VERSION_2_0_0 0x00020000
/// Version 3.0.0 of the plugin interface
#define PMD_INTERFACE_VERSION_3_0_0 0x00030000
/// The current version of the plugin interface
#define PMD_CURRENT_INTERFACE_VERSION PMD_INTERFACE_VERSION_2_0_0

/*!
    @}
    @endcond
 */
