/*****************************************************************************
 * PMDSDK 2
 *
 * Copyright (c) 2006-2013 PMD Technologies GmbH
 * All Rights Reserved.
 *
 *****************************************************************************/
#pragma once

#include <pmdsdk2common.h>



/*!
    @ingroup Sdk
    @file pmddatadescription.h
    @brief Definitions and structs which describe the data format of the PMDSDK2.
 */

/*!
    @brief Handle for a camera connection.
           This handle is used for all subsequent operations.
 */
typedef unsigned PMDHandle;

/*!
    @addtogroup DataCodes
    @{
*/

/// Unknown data
#define PMD_UNKNOWN_DATA            0x00000000u

// raw data ID system:
// 0x00 0x00 0bORWWWWWW 0bRRRRDABD
// R = Reserved, must be 0
// O = Byte order (1 = Big Endian)
// W = Bit width
// D = Difference
// A = Channel A
// B = Channel B


#define PMD_RAW_BITS_DIFFERENCE_POST 0x0001u
#define PMD_RAW_BITS_CHANNEL_B       0x0002u
#define PMD_RAW_BITS_CHANNEL_A       0x0004u
#define PMD_RAW_BITS_DIFFERENCE_PRE  0x0008u
#define PMD_RAW_BITS_DEPTH_12        0x0C00u
#define PMD_RAW_BITS_DEPTH_16        0x1000u
#define PMD_RAW_BITS_BIG_ENDIAN      0x8000u

/// A/B PMD data (e.g. pmd[vision] 3k-S)
#define PMD_RAW_L16_AB_REF          0x00001006u
#define PMD_RAW_L16_AB              (PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_CHANNEL_A | PMD_RAW_BITS_CHANNEL_B)
/// A/B PMD data (BigEndian) (e.g. CamCube)
#define PMD_RAW_B16_AB_REF          0x00009006u
#define PMD_RAW_B16_AB              (PMD_RAW_BITS_BIG_ENDIAN | PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_CHANNEL_A | PMD_RAW_BITS_CHANNEL_B)
/// Difference and A/B PMD data
#define PMD_RAW_L16_DAB_REF         0x0000100Eu
#define PMD_RAW_L16_DAB             (PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_DIFFERENCE_PRE | PMD_RAW_BITS_CHANNEL_A | PMD_RAW_BITS_CHANNEL_B)
/// Difference and A/B PMD data (Big Endian) (e.g. USB-L)
#define PMD_RAW_B16_DAB_REF         0x0000900Eu
#define PMD_RAW_B16_DAB             (PMD_RAW_BITS_BIG_ENDIAN | PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_DIFFERENCE_PRE | PMD_RAW_BITS_CHANNEL_A | PMD_RAW_BITS_CHANNEL_B)
/// A/B and difference PMD data
#define PMD_RAW_L16_ABD_REF         0x00001007u
#define PMD_RAW_L16_ABD             (PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_CHANNEL_A | PMD_RAW_BITS_CHANNEL_B | PMD_RAW_BITS_DIFFERENCE_POST)
/// A/B and difference PMD data (Big Endian) (e.g. pmd[vision] A2 w/o addition information)
#define PMD_RAW_B16_ABD_REF         0x00009007u
#define PMD_RAW_B16_ABD             (PMD_RAW_BITS_BIG_ENDIAN | PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_CHANNEL_A | PMD_RAW_BITS_CHANNEL_B | PMD_RAW_BITS_DIFFERENCE_POST)
/// Packed PMD difference data (e.g. pmd[vision] 19k)
#define PMD_RAW_P12_D_REF           0x00000C01u
#define PMD_RAW_P12_D               (PMD_RAW_BITS_DEPTH_12 | PMD_RAW_BITS_DIFFERENCE_POST)
/// PMD difference data (e.g. pmd[vision] 64)
#define PMD_RAW_L16_D_REF           0x00001001u
#define PMD_RAW_L16_D               (PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_DIFFERENCE_POST)

/// Difference data and A channel
#define PMD_RAW_B16_DA_REF          0x0000900cu
#define PMD_RAW_B16_DA              (PMD_RAW_BITS_BIG_ENDIAN | PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_DIFFERENCE_PRE | PMD_RAW_BITS_CHANNEL_A)
/// Difference data and A channel
#define PMD_RAW_L16_DA_REF          0x0000100cu
#define PMD_RAW_L16_DA              (PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_DIFFERENCE_PRE | PMD_RAW_BITS_CHANNEL_A)
/// A channel and difference data
#define PMD_RAW_B16_AD_REF          0x00009005u
#define PMD_RAW_B16_AD              (PMD_RAW_BITS_BIG_ENDIAN | PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_CHANNEL_A | PMD_RAW_BITS_DIFFERENCE_POST)
/// A channel and difference data
#define PMD_RAW_L16_AD_REF          0x00001005u
#define PMD_RAW_L16_AD              (PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_CHANNEL_A | PMD_RAW_BITS_DIFFERENCE_POST)
/// Difference data and B channel
#define PMD_RAW_B16_DB_REF          0x0000900au
#define PMD_RAW_B16_DB              (PMD_RAW_BITS_BIG_ENDIAN |PMD_RAW_BITS_DEPTH_16 |  PMD_RAW_BITS_DIFFERENCE_PRE | PMD_RAW_BITS_CHANNEL_B)
/// Difference data and B channel
#define PMD_RAW_L16_DB_REF          0x0000100au
#define PMD_RAW_L16_DB              (PMD_RAW_BITS_DEPTH_16 | PMD_RAW_BITS_DIFFERENCE_PRE | PMD_RAW_BITS_CHANNEL_B)

/// Subtype for inversed phases
#define PMD_REVERSED_PHASES         0x00000001u

/// Subtype for skipping of one subimage
#define PMD_SKIP_1_FRAME            0x04000000u
/// Subtype for skipping of two subimage
#define PMD_SKIP_2_FRAMES           0x08000000u
/// Subtype for skipping of three subimage
#define PMD_SKIP_3_FRAMES           0x0C000000u

// image data ID system:
// 0x00 0x01 0bORRRRTRR 0bIINNIIII
// O = Byte order (1 = Big Endian)
// R = Reserved, must be 0
// I = Interpretation
// T = Data type ( 0 - Float32, 1 - Int32, future: UInt8 )
// N = Number of values per pixel - 1

#define PMD_IMAGE_BITS_HEAD            0x00010000u
#define PMD_IMAGE_BITS_BIG_ENDIAN      0x8000u
#define PMD_IMAGE_BITS_LITTLE_ENDIAN      0x0000u

#define PMD_IMAGE_BITS_INT32           0x0400u
#define PMD_IMAGE_BITS_FLOAT32           0x0000u

#define PMD_IMAGE_BITS_1_VALUE_PER_PIXEL  0x0000u
#define PMD_IMAGE_BITS_2_VALUES_PER_PIXEL 0x0010u
#define PMD_IMAGE_BITS_3_VALUES_PER_PIXEL 0x0020u
#define PMD_IMAGE_BITS_4_VALUES_PER_PIXEL 0x0030u

#define PMD_IMAGE_BITS_FLAGS           0x0000u
#define PMD_IMAGE_BITS_DISTANCE        0x0001u
#define PMD_IMAGE_BITS_AMPLITUDE       0x0002u
#define PMD_IMAGE_BITS_SEGMENTATION    0x0003u
#define PMD_IMAGE_BITS_INTENSITY       0x0004u
#define PMD_IMAGE_BITS_REFLECTIVITY    0x0008u
#define PMD_IMAGE_BITS_XYZ_COORD       0x00e0u
#define PMD_IMAGE_BITS_X_COORD         0x00e3u
#define PMD_IMAGE_BITS_Y_COORD         0x00e5u
#define PMD_IMAGE_BITS_Z_COORD         0x00e6u

/// 32-bit floating point distances in meters
#define PMD_DISTANCE_LF32_REF       0x00010001u
#define PMD_DISTANCE_LF32           (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_FLOAT32 | PMD_IMAGE_BITS_1_VALUE_PER_PIXEL | PMD_IMAGE_BITS_DISTANCE)
/// 32-bit floating point amplitudes
#define PMD_AMPLITUDE_LF32_REF      0x00010002u
#define PMD_AMPLITUDE_LF32          (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_FLOAT32 | PMD_IMAGE_BITS_1_VALUE_PER_PIXEL | PMD_IMAGE_BITS_AMPLITUDE)
/// 32-bit floating point intensities
#define PMD_INTENSITY_LF32_REF      0x00010004u
#define PMD_INTENSITY_LF32          (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_FLOAT32 | PMD_IMAGE_BITS_1_VALUE_PER_PIXEL | PMD_IMAGE_BITS_INTENSITY)
/// 32-bit floating point reflectivities
#define PMD_REFLECTIVITY_LF32_REF   0x00010008u
#define PMD_REFLECTIVITY_LF32       (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_FLOAT32 | PMD_IMAGE_BITS_1_VALUE_PER_PIXEL | PMD_IMAGE_BITS_REFLECTIVITY)
#define PMD_FLAGS_L16        0x00010010u

/// Cartesian X coordinates
#define PMD_X_COORD_LF32_REF        0x00010020u
#define PMD_X_COORD_LF32            (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_FLOAT32 | PMD_IMAGE_BITS_1_VALUE_PER_PIXEL | PMD_IMAGE_BITS_X_COORD)
/// Cartesian Y coordinates
#define PMD_Y_COORD_LF32_REF        0x00010040u
#define PMD_Y_COORD_LF32            (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_FLOAT32 | PMD_IMAGE_BITS_1_VALUE_PER_PIXEL | PMD_IMAGE_BITS_Y_COORD)
/// Cartesian Z coordinates
#define PMD_Z_COORD_LF32_REF        0x00010080u
#define PMD_Z_COORD_LF32            (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_FLOAT32 | PMD_IMAGE_BITS_1_VALUE_PER_PIXEL | PMD_IMAGE_BITS_Z_COORD)
/// Cartesian XYZ coordinates
#define PMD_XYZ_COORD_LF32_REF      0x000100E0u
#define PMD_XYZ_COORD_LF32          (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_FLOAT32 | PMD_IMAGE_BITS_3_VALUES_PER_PIXEL | PMD_IMAGE_BITS_XYZ_COORD)

/// 32-bit boolean information
#define PMD_FLAGS_L32_REF           0x00010400u
#define PMD_FLAGS_L32               (PMD_IMAGE_BITS_HEAD | PMD_IMAGE_BITS_LITTLE_ENDIAN | PMD_IMAGE_BITS_INT32 | PMD_IMAGE_BITS_1_VALUE_PER_PIXEL | PMD_IMAGE_BITS_FLAGS)

/// n types of data packed together.
#define PMD_PACKAGED_DATA           0x00020000u
#define PMD_CSTRING                 0x00020001u

/// 16 bit distances in millimeters and amplitudes
#define PMD_DISTANCE_I16_AMPLITUDE_I16       0x00020003u
/// 16 bit distances in 100 micrometers and amplitudes
#define PMD_DISTANCE_100UM_I16_AMPLITUDE_I16 0x00020103u

/// A3 camera data v2.31
#define PMD_A3_DATA_2_31            0x00040003u
/// A3 camera data v4
#define PMD_A3_DATA_4               0x00040005u
/// A3 camera raw data
#define PMD_A3_RAWDATA_2_31         0x00040007u
/// A3 camera processed and raw data v4
#define PMD_A3_COMBINED_2_31        0x00040009u
/// A3 camera processed and raw data v4
#define PMD_A3_COMBINED_4           0x0004000Bu
/// A3 camera processed and raw data v4.2
#define PMD_A3_DATA_42              0x0004000Du

/// A3 sub type
#define PMD_A3_DISTANCE             0x00000001u
/// A3 sub type
#define PMD_A3_AMPLITUDE            0x00000002u
/// A3 sub type
#define PMD_A3_CONFIDENCE           0x00000004u
/// A3 sub type
#define PMD_A3_COORDINATES          0x00000008u
/// A3 sub type
#define PMD_A3_OBJECT_LIST          0x00000010u

/// O3 camera data V1.7
#define PMD_O3D_DATA_1_7            0x00050003u
/// O3 camera data V1.13
#define PMD_O3D_DATA_1_13           0x00050004u
/// S3/O3 camera data
#define PMD_O3_DATA_4040            0x00050005u
/// S3/O3 camera data
#define PMD_S3_DATA_4040            0x00050005u
/// S3/O3 camera data
#define PMD_O3NT_DATA_PRE           0x00050005u
/// O3D sub type
#define PMD_O3D_DISTANCE            0x00000002u
/// O3D sub type
#define PMD_O3D_INTENSITY           0x00000004u
/// O3D sub type
#define PMD_O3D_STDDEV              0x00000010u
/// O3D sub type
#define PMD_O3D_VIEW_X              0x00000020u
/// O3D sub type
#define PMD_O3D_VIEW_Y              0x00000040u
/// O3D sub type
#define PMD_O3D_VIEW_Z              0x00000080u
/// O3D sub type
#define PMD_O3D_X                   0x00000100u
/// O3D sub type
#define PMD_O3D_Y                   0x00000200u
/// O3D sub type
#define PMD_O3D_Z                   0x00000400u

/// CamCube data type (512 byte header for each phase image)
#define PMD_CAMCUBE_DATA_1_0        0x00060001u
/// CamCube data type (512 byte header for each phase image)
#define PMD_CAMCUBE_DATA_2_0        0x00060002u
/// CamCube data type (512 byte header for each phase image)
#define PMD_CAMCUBE_DATA_2_0_AD     0x00060003u
/// CamCube data type (512 byte header for each phase image)
#define PMD_CAMCUBE_DATA_2_0_DB     0x00060004u

/// ConceptCam data type (scrambled data)
#define PMD_CONCEPTCAM_DATA_1_0     0x00070001u
/// ConceptCam data type (scrambled data)
#define PMD_CONCEPTCAM_DATA_1_1     0x00070003u

/// Camboard data type (scrambled data)
#define PMD_CAMBOARD_DATA_1_0       0x00070002u
/// Camboard data type (scrambled data)
#define PMD_CAMBOARD_DATA_1_1       0x00070004u

/// Camboard nano data type
#define PMD_CAMBOARD_NANO_DATA      0x00070005u
/// Camboard nano data type
#define PMD_CAMBOARD_NANO_DATA_1_0  0x00070006u

/// Camboard Mod data type (scrambled data)
#define PMD_CAMBOARD_MOD_DATA       0x00070007u

/// Digicam data type
#define PMD_DIGICAM_DATA            0x00070008u

/// Camboard Plus data type
#define PMD_CAMBOARD_PLUS_DATA      0x00070009u

/// CamBoard pico data type
#define PMD_CAMBOARD_PICO_DATA      0x0007000Au
/// CamBoard pico data type
#define PMD_CAMBOARD_PICO_DATA_1_0  0x0007000Bu
/// CamBoard pico data type (with dark pixels)
#define PMD_CAMBOARD_PICO_DATA_1_1  0x0007000Du
/// CamBoard pico data type (header based)
#define PMD_CAMBOARD_PICO_DATA_1_2  0x0007000Eu

/// CamBoard ATV data type
#define PMD_CAMBOARD_ATV_DATA       0x0007000Cu



/// user defined data type
#define PMD_USER_DEFINED_0          0xFFFF0000u
/// user defined data type
#define PMD_USER_DEFINED_1          0xFFFF0001u
/// user defined data type
#define PMD_USER_DEFINED_2          0xFFFF0002u
/// user defined data type
#define PMD_USER_DEFINED_3          0xFFFF0003u
/// user defined data type
#define PMD_USER_DEFINED_4          0xFFFF0004u
/// user defined data type
#define PMD_USER_DEFINED_5          0xFFFF0005u
/// user defined data type
#define PMD_USER_DEFINED_6          0xFFFF0006u
/// user defined data type
#define PMD_USER_DEFINED_7          0xFFFF0007u
/// user defined data type
#define PMD_USER_DEFINED_8          0xFFFF0008u
/// user defined data type
#define PMD_USER_DEFINED_9          0xFFFF0009u
/// user defined data type
#define PMD_USER_DEFINED_10         0xFFFF000au
/// user defined data type
#define PMD_USER_DEFINED_11         0xFFFF000bu
/// user defined data type
#define PMD_USER_DEFINED_12         0xFFFF000cu
/// user defined data type
#define PMD_USER_DEFINED_13         0xFFFF000du
/// user defined data type
#define PMD_USER_DEFINED_14         0xFFFF000eu
/// user defined data type
#define PMD_USER_DEFINED_15         0xFFFF000fu

// pixel origin values
/// The top-right pixel is the first pixel
#define PMD_ORIGIN_TOP_RIGHT        0x00000000u
/// The top-left pixel is the first pixel
#define PMD_ORIGIN_TOP_LEFT         0x00000001u
/// The bottom-right pixel is the first pixel
#define PMD_ORIGIN_BOTTOM_RIGHT     0x00000002u
/// The bottom-left pixel is the first pixel
#define PMD_ORIGIN_BOTTOM_LEFT      0x00000003u

/// The pixel origin is on the right edge of the image
#define PMD_ORIGIN_RIGHT            0x00000000u
/// The pixel origin is on the left edge of the image
#define PMD_ORIGIN_LEFT             0x00000001u
/// The pixel origin is on the top edge of the image
#define PMD_ORIGIN_TOP              0x00000000u
/// The pixel origin is on the bottom edge of the image
#define PMD_ORIGIN_BOTTOM           0x00000002u

// pixel direction values
/// The second pixel is on the right or the left of the first pixel
#define PMD_DIRECTION_HORIZONTAL    0x00000000u
/// The second pixel is above or below the first pixel
#define PMD_DIRECTION_VERTICAL      0x00010000u

// sub header types
/// The PMDGenericData structure is used in the PMDDataDescription
#define PMD_GENERIC_DATA            0x00000001u
/// The PMDImageData structure is used in the PMDDataDescription
#define PMD_IMAGE_DATA              0x00000002u

/// if element size is variable use this macro.
#define PMD_UNKNOWN_SIZE            0x00000000u

/*!
    @}
 */

/*!
    @addtogroup Flags
    @{
*/

// Flags

/// invalid flag
#define PMD_FLAG_INVALID            0x00000001u
/// saturation flag
#define PMD_FLAG_SATURATED          0x00000002u
/// inconsistency flag
#define PMD_FLAG_INCONSISTENT       0x00000004u
/// low signal flag
#define PMD_FLAG_LOW_SIGNAL         0x00000008u
/// SBI flag
#define PMD_FLAG_SBI_ACTIVE         0x00000010u
/// User defined flag
#define PMD_FLAG_USER_0             0x80000000u
/// User defined flag
#define PMD_FLAG_USER_1             0x40000000u

/*!
    @}
 */

/*!
    @addtogroup Common
    @{
*/

extern "C" {

/// Generic data description
struct PMDGenericData
{
    /// Specific type of the data
    unsigned subType;
    /// Number of elements in the data
    unsigned numElem;
    /// Size of one element in bytes
    unsigned sizeOfElem;
};

/// Standard PMD image data.
struct PMDImageData
{
    /// Specific type of the data
    unsigned subType;
    /// Number of columns in the image
    unsigned numColumns;
    /// Number of rows in the image
    unsigned numRows;
    /// Number of sub images
    unsigned numSubImages;

    /// Integration times at which the data was captured
    int integrationTime[4];
    /// Modulation frequencies at which the data was captured
    int modulationFrequency[4];
    /// Offsets for up to four separate measurements in mm.
    int offset[4];

    /// Pixel aspect ratio. The ratio is pixelAspectRatio / 1000.0
    int pixelAspectRatio;
    /// Position/direction of the first pixel.
    int pixelOrigin;

    /// Time at which the data was captured. Most significant word.
    unsigned timeStampHi;
    /// Time at which the data was captured. Least significant word.
    unsigned timeStampLo;

    /// Reserved for future use.
    char reserved[24];

    /// Contains user-defined information. This will not be evaluated by the PMDSDK.
    unsigned userData0;
};

/// Layout description of a data block.
struct PMDDataDescription
{
    /// Unique ID of the plugin that generated the data
    unsigned PID;
    /// Unique ID of the data
    unsigned DID;
    /// Interpretation of the data
    unsigned type;
    /// Size of the data block in bytes
    unsigned size;

    /// Type of the sub header
    unsigned subHeaderType;
    /// Specific interpretation-dependent information
    union
    {
        /// Generic data
        struct PMDGenericData gen;
        /// Standard PMD image data
        struct PMDImageData img;
        /// Ensure fixed size
        char fillUpToSizeOfStructure[108];
    };
};

}

/*!
    @}
 */

#ifndef PMD_NO_DEPRECATED

/*!
    @cond plugin_all_doc
    @addtogroup Deprecated
    @{
*/

extern "C" {
typedef struct PMDDataDescription   DataDescription;
}
#define STANDARD_PMD_DATA           PMD_IMAGE_DATA
#define PMD_CAMBOARD_MIRA_DATA      PMD_CAMBOARD_PICO_DATA
#define PMD_CAMBOARD_MIRA_DATA_1_0  PMD_CAMBOARD_PICO_DATA_1_0

/*!
    @}
    @endcond
 */

#endif
