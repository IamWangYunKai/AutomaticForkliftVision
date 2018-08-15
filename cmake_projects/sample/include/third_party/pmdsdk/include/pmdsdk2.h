/*
 * PMDSDK 2
 *
 * File: pmdsdk2.h
 * Author: Martin Profittlich
 *
 * General header file for applications using the PMDSDK 2.0.
 * Contains all necessary definitions and prototypes.
 *
 * Copyright (c) 2006-2013 PMD Technologies GmbH.
 * All Rights Reserved.
 *
 */
#pragma once



#include <pmddatadescription.h>

#ifdef _WIN32
# ifndef DLLSPEC
#  define DLLSPEC __declspec(dllimport)
# endif
#else
# ifdef DLLSPEC
#  undef DLLSPEC
# endif
# define DLLSPEC
#endif

/*!
    @addtogroup Sdk
    @{

    @file pmdsdk2.h
    @brief The main include file for working with pmd sensors.
*/

extern "C" {

// FUNCTIONS

/*!
    @brief Connect to a PMD sensor or other data source
    @param hnd Empty PMDHandle structure. On success, this value
           will contain the handle for subsequent operations.
    @param rplugin Path of the camera plugin
    @param rparam Parameter for the camera plugin
    @param pplugin Path of the processing plugin. If this is NULL, no porcessing plugin will be loaded.
    @param pparam Parameter for the processing plugin
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdOpen (PMDHandle *hnd, const char *rplugin, const char *rparam, const char *pplugin, const char *pparam);

/*!
    @brief Connect to a pmd device or other data source without processing
    @param hnd Empty PMDHandle structure. On success, this value
           will contain the handle for subsequent operations.
    @param rplugin Path of the camera plugin
    @param rparam Parameter for the camera plugin
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdOpenSourcePlugin (PMDHandle *hnd, const char *rplugin, const char *rparam);

/*!
    @brief Disconnect and close the handle.
    @param hnd Handle of the connection.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdClose (PMDHandle hnd);

/*!
    @brief Disconnect and close all handles.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdCloseAll ();

/*!
    @brief Get an error description from the last error.
           Error messages are stored per handle. A new error associated with PMDHandle A
           does not overwrite the last error message associated with PMDHandle B.
    @param hnd Handle of the connection/plugin.
    @param error Memory to hold the error message.
    @param maxLen Maximum length of the error message, including the terminating zero byte.
    Longer messages will be truncated.
 */
DLLSPEC int pmdGetLastError (PMDHandle hnd, char *error, size_t maxLen);

/*!
    @brief Retrieve the a new frame from the camera.
           To obtain the actual data, use pmdGetSourceData, pmdGetDistances,
           pmdGetAmplitudes etc.
    @param hnd Handle of the connection.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdUpdate (PMDHandle hnd);

/*!
    @brief Set the integration time of the camera
    @param hnd Handle of the connection.
    @param idx Index of the integration time.
    @param t Integration time in microseconds.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdSetIntegrationTime (PMDHandle hnd, unsigned idx, unsigned t);

/*!
    @brief Get the integration time of the camera
    @param hnd Handle of the connection.
    @param idx Index of the integration time.
    @param t Pointer to a variable to contain the
           integration time in microseconds.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetIntegrationTime (PMDHandle hnd, unsigned *t, unsigned idx);

/*!
    @brief Get a supported integration time of the camera
    @param hnd Handle of the connection.
    @param idx Index of the integration time.
    @param t The desired integration time
    @param w Where to look for a valid integration time in respect to the desired
           integration time (CloseTo, AtLeast or AtMost)
    @param responsible Pointer to a variable to contain the
           integration time in microseconds.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetValidIntegrationTime (PMDHandle hnd, unsigned *result, unsigned idx, Proximity w, unsigned t);

/*!
    @brief Set the modulation frequency of the camera
    @param hnd Handle of the connection.
    @param idx Index of the modulation frequency.
    @param f Modulation frequency in Hz.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdSetModulationFrequency (PMDHandle hnd, unsigned idx, unsigned f);

/*!
    @brief Get the modulation frequency of the camera
    @param hnd Handle of the connection.
    @param idx Index of the modulation frequency.
    @param t Pointer to a variable to contain the
           modulation frequency in Hz.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetModulationFrequency (PMDHandle hnd, unsigned *f, unsigned idx);

/*!
    @brief Get a supported modulation frequency of the camera
    @param hnd Handle of the connection.
    @param idx Index of the modulation frequency.
    @param f The desired modulation frequency
    @param w Where to look for a valid modulation frequency in respect to the desired
           modulation frequency (CloseTo, AtLeast or AtMost)
    @param result Pointer to a variable to contain the
           modulation frequency in Hz.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetValidModulationFrequency (PMDHandle hnd, unsigned *result, unsigned idx, Proximity w, unsigned f);

/*!
    @brief Get the raw data from the current frame.
    @param hnd Handle of the connection.
    @param data Pointer to the memory to contain the address of the data.
    @param maxLen Maximum length in bytes for the data
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetSourceData (PMDHandle hnd, void *data, size_t maxLen);

/*!
    @brief Get the size in bytes of the current raw data frame.
    @param hnd Handle of the connection.
    @param size Will contain the size after the call
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetSourceDataSize (PMDHandle hnd, size_t *size);

/*!
    @brief Get the description of the current raw data frame.
    @param hnd Handle of the connection.
    @param dd Will contain the PMDDataDescription after the call.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetSourceDataDescription (PMDHandle hnd, struct PMDDataDescription *dd);

/*!
    @brief Get the distance data from the current frame.
    @param hnd Handle of the connection.
    @param data Pointer to a block of memory to contain the data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetDistances (PMDHandle hnd, float *data, size_t maxLen);

/*!
    @brief Get 3d-coordinates of the current frame.
    @param hnd Handle of the connection.
    @param data Pointer to a block of memory to contain the data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGet3DCoordinates (PMDHandle hnd, float *data, size_t maxLen);

/*!
    @brief Get the amplitude data from the current frame.
    @param hnd Handle of the connection.
    @param data Pointer to a block of memory to contain the data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetAmplitudes (PMDHandle hnd, float *data, size_t maxLen);

/*!
    @brief Get the intensity data from the current frame.
    @param hnd Handle of the connection.
    @param data Pointer to a block of memory to contain the data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetIntensities (PMDHandle hnd, float *data, size_t maxLen);

/*!
    @brief Get the flags from the current frame.
    @param hnd Handle of the connection.
    @param data Pointer to a block of memory to contain the data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdGetFlags (PMDHandle hnd, unsigned *data, size_t maxLen);

/*!
    @brief Execute an source plugin-specific command.
    @param hnd Handle of the connection.
    @param result Pointer to a block of memory to contain the result string.
    @param maxLen Maximum length of the result string, including terminating 0.
    @param cmd The command to be executed.
 */
DLLSPEC int pmdSourceCommand (PMDHandle hnd, char *result, size_t maxLen, const char *cmd);

/*!
    @brief Execute a processing plugin-specific command.
    @param hnd Handle of the connection.
    @param result Pointer to a block of memory to contain the result string.
    @param maxLen Maximum length of the result string, including terminating 0.
    @param cmd The command to be executed.
 */
DLLSPEC int pmdProcessingCommand (PMDHandle hnd, char *result, size_t maxLen, const char *cmd);

// ADDITIONAL PROCESSING


/*!
    @brief Open a PMDSDK processing plugin.
    @param hnd Handle of the connection.
    @param pplugin Filename of the plugin.
    @param pparam Parameters for the plugin.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdOpenProcessingPlugin (PMDHandle *hnd, const char *pplugin, const char *pparam);


/*!
    @brief Calculate the distances based on the source data.
    @param hnd Handle of the connection.
    @param data Array for the calculation result.
    @param maxLen Size of the result array in bytes.
    @param sourceDD Source data description.
    @param sourceData Source data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdCalcDistances (PMDHandle hnd, float *data, size_t maxLen, struct PMDDataDescription sourceDD, void *sourceData);

/*!
    @brief Calculate the amplitudes based on the source data.
    @param hnd Handle of the connection.
    @param data Array for the calculation result.
    @param maxLen Size of the result array in bytes.
    @param sourceDD Source data description.
    @param sourceData Source data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdCalcAmplitudes (PMDHandle hnd, float *data, size_t maxLen, struct PMDDataDescription sourceDD, void *sourceData);

/*!
    @brief Calculate the intensities based on the source data.
    @param hnd Handle of the connection.
    @param data Array for the calculation result.
    @param maxLen Size of the result array in bytes.
    @param sourceDD Source data description.
    @param sourceData Source data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdCalcIntensities (PMDHandle hnd, float *data, size_t maxLen, struct PMDDataDescription sourceDD, void *sourceData);

/*!
    @brief Calculate the 3D coordinates based on the source data.
    @param hnd Handle of the connection.
    @param data Array for the calculation result.
    @param maxLen Size of the result array in bytes.
    @param sourceDD Source data description.
    @param sourceData Source data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdCalc3DCoordinates (PMDHandle hnd, float *data, size_t maxLen, struct PMDDataDescription sourceDD, void *sourceData);

/*!
    @brief Calculate the flags based on the source data.
    @param hnd Handle of the connection.
    @param data Array for the calculation result.
    @param maxLen Size of the result array in bytes.
    @param sourceDD Source data description.
    @param sourceData Source data.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdCalcFlags (PMDHandle hnd, unsigned *data, size_t maxLen, struct PMDDataDescription sourceDD, void *sourceData);

// OUTPUT PLUGINS

/*!
    @brief Open an output plugin.
    @param hnd Empty PMDHandle structure. On success, this value
           will contain the handle of the plugin.
    @param oplugin Path of the output plugin
    @param oparam Parameter for the output plugin
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdOpenOutputPlugin (PMDHandle *hnd, const char *oplugin, const char *oparam);

/*!
    @brief Output a new frame (old style interface).
    @param hnd Handle of the output plugin.
    @param numIn Output count.
    @param ddIn Array of data descriptions.
    @param input Array of data arrays.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdOutput (PMDHandle hnd, unsigned numIn, struct PMDDataDescription *ddIn, void **input);

/*!
    @brief Check if the output plugin supports the data described by a PMDDataDescription.
    @param hnd Handle of the connection.
    @param numFmt Data description count.
    @param fmt Data description array.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdCanOutput (PMDHandle hnd, unsigned numFmt, struct PMDDataDescription *fmt);

/*!
    @brief Execute an output plugin-specific command.
    @param hnd Handle of the plugin.
    @param result Pointer to a block of memory to contain the result string.
    @param maxLen Maximum length of the result string, including terminating 0.
    @param cmd The command to be executed.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdOutputCommand (PMDHandle hnd, char *result, size_t maxLen, const char *cmd);




/*!
    @}
 */

#ifndef PMD_NO_DEPRECATED
// DEPRECATED

/*!
    @cond plugin_all_doc
*/

/*!
    @addtogroup Deprecated
    @{
*/

/*!
    @brief Connect to a firewire camera.
    @param hnd Handle of the connection.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdConnectFireWire (PMDHandle *hnd, unsigned index);

/*!
    @brief Connect to PMD A sample.
    @param hnd Handle of the connection.
    @return PMD_OK on success, errorcode otherwise
 */
DLLSPEC int pmdConnectASample (PMDHandle *hnd, const char *address);


/*!
    @brief Connect.
    @deprecated
 */
DLLSPEC int pmdConnect (PMDHandle *hnd, const char *rplugin, const char *rparam, const char *pplugin, const char *pparam);

/*!
    @brief Connect only raw.
    @deprecated
 */
DLLSPEC int pmdConnectOnlyRaw (PMDHandle *hnd, const char *rplugin, const char *rparam);

/*!
    @brief Disconnect the sensor.
    @deprecated
 */
DLLSPEC int pmdDisconnect (PMDHandle hnd);

/*!
    @brief Open an access plugin.
    @deprecated
 */
DLLSPEC int pmdOpenAccessPlugin (PMDHandle *hnd, const char *rplugin, const char *rparam);

/*!
    @brief Retrieve raw data.
    @deprecated
 */
DLLSPEC int pmdGetRawData (PMDHandle hnd, void *data, size_t maxLen);

/*!
    @brief Retrieve raw data size.
    @deprecated
 */
DLLSPEC int pmdGetRawDataSize (PMDHandle hnd, size_t *size);

/*!
    @brief Retrieve raw data description.
    @deprecated
 */
DLLSPEC int pmdGetRawDataDescription (PMDHandle hnd, struct PMDDataDescription *dd);

/*!
    @brief Execute platform command.
    @deprecated
 */
DLLSPEC int pmdPlatformCommand (PMDHandle hnd, char *result, size_t maxLen, const char *cmd);

/*!
    @brief Configure the processing.
    @deprecated
 */
DLLSPEC int pmdConfigureProcess (PMDHandle hnd, char *result, size_t maxLen, const char *cmd);

/*!
    @brief Retrieve info.
    @deprecated
 */
DLLSPEC int pmdGetInfo (PMDHandle hnd, char *result, size_t maxLen, const char *key);

/*!
    @}
 */

/*!
    @endcond
 */

#endif // PMD_NO_DEPRECATED

}

