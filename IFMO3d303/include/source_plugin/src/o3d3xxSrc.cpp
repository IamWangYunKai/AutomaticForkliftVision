/** @file
@brief This file represents functions which form an interface between
pmdaccess.dll and o3d3xx source plugin class

The functions in this file are directly called via pmdaccess.dll for
configuring and capturing images from camera.

@author Winy Kakani\n
Version: 1.0\n

Copyright (C) 2014 ifm electronic GmbH\n
See accompanied file LICENCE.TXT for license information.
*/
#include <string>
#include <map>
#include <string.h>
#include <vector>
#include <sstream>

#include <pmddatadescription.h>
#include <pmdsdk2common.h>

#include "err.h"
#include "o3d3xx_camera.hpp"

/**
Macro to enable selectively export/import of functions
*/
#ifdef WIN32
#ifndef pmdDllExport
#define pmdDllExport __declspec( dllexport )
#endif
#ifndef pmdDllImport
#define pmdDllImport __declspec( dllimport )
#endif
#pragma warning(disable:4251)
#pragma warning(disable:4996)
#else
#ifndef pmdDllExport
#define pmdDllExport
#endif
#ifndef pmdDllImport
#define pmdDllImport
#endif
#endif

/**< counter for number of connections */
unsigned g_did = 1;

/**< An string holding errors occurring during pmdSdk calling the source plugin functions*/
std::string globalErrorMessage;

/** @struct
* @brief structure to hold data required by pmd source plugin
*/
struct SrcPluginData
{
    /** Initialize member values to default */
    SrcPluginData () : errorMessage ("")
    {
        integrationTime = 0;
        frameWidth = 0;
        frameHeight = 0;
    };

    /** Free the memory allocated to plugin data */
    void cleanUp()
    {
        if (o3d3xxCamera != NULL)
        {
            free (o3d3xxCamera);
            o3d3xxCamera = NULL;
        }
    };
    std::string errorMessage;/**< Error message from source plugin */
    uint32_t integrationTime;/**< Integration time of camera*/
    uint32_t frameHeight;/**< Height of frame */
    uint32_t frameWidth;/**< Width of frame */

    /**< Object of source plugin class. Need to configure and get frame data from camera*/
    O3d3xxCameraOperations *o3d3xxCamera;

    /**<local datadescriptor of the current data */
    PMDDataDescription currentDD;
};

/** Map instance IDs to SrcPluginData structures */
std::map <unsigned, SrcPluginData *> g_data;

/**
@brief Checks that whether the plugin of referred hnd
is already loaded or not.
@param[in] hnd
The identifier of the plugin instance
@return boolean value indicating whether the plugin is loaded or not
*/
bool idOk (unsigned hnd)
{
    if (g_data.find (hnd) == g_data.end ())
    {
        return false;
    }
    return true;
}

extern "C" {
    /**
    @brief Returns the interface version of the source plugin
    @param[in] none
    @return Version number of source plugin
    */
    pmdDllExport unsigned pmdpInterfaceVersion ()
    {
        return PMD_INTERFACE_VERSION_1_3_0;
    }

    /**
    @brief Creates an instance of source plugin class's object and connects to the camera
    @param[in] handle identifier/handle of connection
    @param[in] param parameters of plugin instance
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpInitSourcePlugin (unsigned handle, const char *param)
    {
        std::string ipAddress;
        uint16_t port_number = 0;
        uint16_t pcic_tcp_port = 0;
        int applicationID = -1;

        /* Check if plugin instance is present or not, if not create it*/
        if (g_data.find (handle) != g_data.end ())
        {
            globalErrorMessage = "handle already exists";
            return PMD_INVALID_VALUE;
        }
        else
        {
            g_data[handle] =  new SrcPluginData();
        }

        // retrieving the appropriate plugin data for given handle
        SrcPluginData *dat = g_data[handle];

        /*Extract ip address, configuration port and Pcic port number from
         input parameter string*/
        std::vector<std::string> v1;

        std::istringstream is1 (param);

        std::string t;

        while (std::getline (is1, t, ':'))
        {
            if (t.empty())
            {
                v1.push_back (":");
            }
            else
            {
                v1.push_back (t);
            }
        }

        if (v1.size() < 3)
        {
            dat->errorMessage = "Invalid Parameters";
            return PMD_COULD_NOT_OPEN;
        }

        /*Convert port numbers to integer values */
        ipAddress = v1.at (0);
        port_number = atoi (v1.at (1).c_str());
        pcic_tcp_port = atoi (v1.at (2).c_str());

        if (applicationID == -1 &&  v1.size() == 4)
        {
            applicationID =  atoi (v1.at (3).c_str());
        }
        else
        {
            applicationID = 0;
        }

        /* Create object of o3d3xx camera source plugin class */
        dat->o3d3xxCamera = new O3d3xxCameraOperations();

        if ( (dat->o3d3xxCamera->connectToCamera (ipAddress, port_number, pcic_tcp_port , applicationID)) != 0)
        {
            dat->cleanUp();
            dat->errorMessage = "Cannot open camera";
            return PMD_COULD_NOT_OPEN;
        }

        dat->o3d3xxCamera->getResolution (& (dat->frameWidth), & (dat->frameHeight));
        dat->o3d3xxCamera->getIntegrationTime (& (dat->integrationTime), 0);

        dat->currentDD.img.integrationTime[0] = dat->integrationTime;
        dat->currentDD.img.numColumns = dat->frameWidth;
        dat->currentDD.img.numRows = dat->frameHeight;

        dat->currentDD.type = PMD_USER_DEFINED_0;
        return PMD_OK;
    }

    /**
    @brief Disconnects camera and unloads the plugin
    @param[in] hnd identifier/handle of connection
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpDeinitPlugin (unsigned hnd)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        g_data[hnd]->o3d3xxCamera->disconnectFromCamera();
        g_data[hnd]->cleanUp();
        delete g_data[hnd];
        g_data.erase (hnd);

        return PMD_OK;
    }

    /**
    @brief Returns the last occurred error to calling function
    @param[in] hnd identifier/handle of connection
    @param[in,out] msg string containing the error message
    @param[in] maxLen maximum length of error message. It is size of "msg" buffer passed as input argument. This is to be as an
                    additional check parameter, while copying buffer to error message, to eliminate memory leaks
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpGetLastError (unsigned hnd, char *msg, size_t maxLen)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        if (maxLen && msg)
        {
            g_data[hnd]->errorMessage = g_data[hnd]->o3d3xxCamera->getLastError();
            /*Retrieve the last error that occurred */
            strncpy (msg, g_data[hnd]->errorMessage.c_str (), maxLen);
            msg[maxLen - 1] = '\0';
        }
        return PMD_OK;
    }

    /**
    @brief Triggers and retrieves a data frame.
    @param[in] hnd identifier/handle of connection
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpUpdate (unsigned hnd)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        SrcPluginData *dat = g_data[hnd];
        g_did++;
        dat->o3d3xxCamera->updateCameraFrame();
        return PMD_OK;
    }

    /**
    @brief Copies a frame data from camera to output buffer. The buffer must be allocated.
    @param[in] hnd identifier/handle of connection
    @param[in,out] result output buffer, in which image data need to be copied.
    @return PMD_OK, if no error occurred, else relevant error code
    @note use the pmdGetSourceDataSize function to get the necessary buffer size.
    */
    pmdDllExport int pmdpGetSourceData (unsigned hnd, void *result)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }
        SrcPluginData *dat = g_data[hnd];
        dat->o3d3xxCamera->getPCICpacket (result);

        return PMD_OK;
    }

    /**
    @brief Get size of frame data buffer
    @param[in] hnd identifier/handle of connection
    @param[in,out] result output buffer, in which image data need to be copied.
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpGetSourceDataSize (unsigned hnd, size_t *result)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }
        SrcPluginData *dat = g_data[hnd];
        dat->o3d3xxCamera->getFrameDataSize (result);
        return PMD_OK;
    }

    /**
    @brief Get data descriptor of the frame data buffer.
    The processing plugin needs this to understand and parse the
    frame data buffer.
    @param[in] hnd identifier/handle of connection
    @param[in,out] result data descriptor of current image data buffer
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpGetSourceDataDescription (unsigned hnd, PMDDataDescription *result)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        SrcPluginData *dat = g_data[hnd];

        int c_channels = 1;
        //
        dat->currentDD.type = PMD_USER_DEFINED_0; /* USER DEFINE TYPE DATA for O3D3xx data*/
        dat->currentDD.subHeaderType = PMD_IMAGE_DATA;
        dat->currentDD.img.numSubImages = 6;/* Mentions number of images in image data
                                            buffer. We have six images*/
        /* Get resolution of image from camera. Store it in descriptor */
        dat->currentDD.img.numColumns = dat->frameWidth;
        dat->currentDD.img.numRows = dat->frameHeight;

        /* LightVis uses four integration time and frequencies.
        Set dummy values for other three integration times, frequencies and other fields */
        //   dat->currentDD.img.integrationTime[0] = dat->integrationTime;
        dat->currentDD.img.integrationTime[1] = -1;
        dat->currentDD.img.integrationTime[2] = -1;
        dat->currentDD.img.integrationTime[3] = -1;
        dat->currentDD.img.modulationFrequency[0] = 10000000;
        dat->currentDD.img.modulationFrequency[1] = -1;
        dat->currentDD.img.modulationFrequency[2] = -1;
        dat->currentDD.img.modulationFrequency[3] = -1;
        dat->currentDD.img.offset[0] = 0;
        dat->currentDD.img.offset[1] = 0;
        dat->currentDD.img.offset[2] = 0;
        dat->currentDD.img.offset[3] = 0;
        dat->currentDD.img.pixelOrigin = 0;
        dat->currentDD.img.pixelAspectRatio = 1000;
        dat->currentDD.img.timeStampLo = 0;
        dat->currentDD.img.timeStampHi = 0;

        /* Get Size of image data buffer */
        dat->o3d3xxCamera->getFrameDataSize (& (dat->currentDD.size));

        dat->currentDD.PID = hnd;
        dat->currentDD.DID = g_did;

        /*Output this descriptor to calling function */
        memcpy (result , &dat->currentDD, sizeof (PMDDataDescription));

        return PMD_OK;
    }

    /**
    @brief Sets Integration time to camera. Only the first exposure time is supported.
    @param[in] hnd identifier/handle of connection
    @param[in] idx index of integration time to be set
    @param[in] val value of integration time to set (in [us])
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpSetIntegrationTime (unsigned hnd, unsigned idx, unsigned val)
    {
        int32_t status = 0;
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        SrcPluginData *dat = g_data[hnd];
        if (idx != 0)
        {
            dat->errorMessage = "multiple integration times not supported" ;
            return PMD_NOT_IMPLEMENTED;
        }

        if ( (status = dat->o3d3xxCamera->setIntegrationTime (val, 0)) != 0)
        {
            dat->errorMessage = "Could not set the rqeuested integration time";
            return PMD_RUNTIME_ERROR;
        }
        dat->currentDD.img.integrationTime[idx] = val;
        return PMD_OK;
    }

    /**
    @brief Gets Integration time from camera. Only the first exposure time is supported.
    @param[in] hnd identifier/handle of connection
    @param[in,out] result value of integration time (in [us])
    @param[in] idx index of integration time which needs to be read from camera
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpGetIntegrationTime (unsigned hnd, unsigned *result, unsigned idx)
    {
        int32_t status = 0;
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        /*Get handle of current plugin*/
        SrcPluginData *dat = g_data[hnd];

        if (idx != 0)
        {
            *result = 0;
            dat->errorMessage = "multiple integration times not supported" ;
            return PMD_OK;
        }
        if ( (status = dat->o3d3xxCamera->getIntegrationTime ( (uint32_t *) result , idx) != 0))
        {
            dat->errorMessage = "Could not get integration time from camera" ;
            dat->errorMessage += dat->o3d3xxCamera->getLastError();
            return PMD_RUNTIME_ERROR;
        }

        dat->currentDD.img.integrationTime[idx] = *result;
        return PMD_OK;
    }

    /// @cond NOT_TO_BE_EXPOSED
    /**
    @brief Get a supported integration time of the camera. Our device supports all exposure times between 1 ... 10000 us.
    @param[in] hnd identifier/handle of connection
    @param[in,out] result value of supported integration time
    @param[in] index index of integration time which needs to be read from camera
    @param[in] proximity Where to look for a valid integration time in respect to the
                        desired integration time (CloseTo, AtLeast or AtMost)
    @param[in] us Pointer to a variable to contain the integration time in microseconds.
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpGetValidIntegrationTime (unsigned hnd, unsigned *result, unsigned index, Proximity proximity, unsigned us)
    {
        int32_t status = 0;
        /* Check if the plugin is already loaded or not.
        If not then raise an error*/
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        SrcPluginData *dat = g_data[hnd];

        if (index != 0)
        {
            *result = 0;
            dat->errorMessage = "multiple integration times not supported" ;
            return PMD_INVALID_VALUE;
        }
        if (us < 1)
        {
            *result = 1;
        }
        else if (us > 10000)
        {
            *result = 10000;
        }
        else
        {
            *result = us;
        }

        return PMD_OK;
    }

    /**
    @brief Sets Modulation frequency of camera
    @param[in] hnd identifier/handle of connection
    @param[in] idx index of modulation frequency to be set
    @param[in] val value of modulation frequency to set
    @return PMD_NOT_IMPLEMENTED, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpSetModulationFrequency (unsigned hnd, unsigned idx, unsigned val)
    {

        /* Check if the plugin is already loaded or not.
        If not then raise an error*/
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        g_data[hnd]->errorMessage = "function not implemented";
        return PMD_NOT_IMPLEMENTED;
    }

    /**
    @brief Gets Modulation frequency of camera
    @param[in] hnd identifier/handle of connection
    @param[in] idx index of modulation frequency to be get
    @param[in] val value of modulation frequency to get
    @return PMD_NOT_IMPLEMENTED, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpGetModulationFrequency (unsigned hnd, unsigned *result, unsigned idx)
    {
        /* Check if the plugin is already loaded or not.
        If not then raise an error*/
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        g_data[hnd]->errorMessage = "function not implemented";
        return PMD_NOT_IMPLEMENTED;
    }

    /**
    @brief Get a supported modulation frequency of the camera.
    @param[in] hnd identifier/handle of connection
    @param[in,out] result value of supported modulation frequency
    @param[in] index index of modulation frequency which needs to be read from camera
    @param[in] proximity Where to look for a valid modulation frequency in respect to the
                        desired modulation frequency (CloseTo, AtLeast or AtMost)
    @param[in] us Pointer to a variable to contain the modulation frequency in hertz.
    @return PMD_NOT_IMPLEMENTED, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpGetValidModulationFrequency (unsigned hnd, \
            unsigned *result, unsigned index, Proximity proximity, unsigned hz)
    {
        /* Check if the plugin is already loaded or not.
        If not then raise an error*/
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        g_data[hnd]->errorMessage = "function not implemented";
        return PMD_NOT_IMPLEMENTED;
    }

    /**
    @brief Execute an source plugin-specific command.
    @param[in] hnd identifier/handle of connection
    @param[in,out] result Pointer to a block of memory to contain the result string.
    @param[in] maxLen Maximum length of the result string, including terminating 0.
    @param[in] cmd The command to be executed.
    @return PMD_NOT_IMPLEMENTED, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpSourceCommand (unsigned hnd, char *result, size_t maxLen, const char *cmd)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        SrcPluginData *dat = g_data[hnd];

        std::string s (cmd);
        std::string command = s.substr (0, s.find (' '));
        std::string param   = s.substr (command.length(), s.length());
        // removing leading spaces
        if (!param.empty())
        {
            size_t pos = param.find_first_not_of (' ');
            param = param.substr (pos, param.length() - pos);
        }

        std::string response;
        int res = PMD_NOT_IMPLEMENTED;
        g_data[hnd]->errorMessage = "command not implemented";
        if (command == "GetCommandList")
        {
            memcpy (result , command.c_str(), 13);
            return PMD_OK;
        }
        else if (command == "DisconnectPCIC")
        {
            dat->o3d3xxCamera->disconnectPCIC();
            response = "OK";
            res = PMD_OK;
        }
        else if (command == "ConnectPCIC")
        {
            res = dat->o3d3xxCamera->connecttoPCIC();
            response = "OK";
            res = PMD_OK;
        }
        else if (command == "SetPCICDataConfigString")
        {
            dat->o3d3xxCamera->setPCICDataConfigString (param);
            res = PMD_OK;
            response = "OK";
        }
        else if (command == "GetPCICDataConfigString")
        {
            res = PMD_OK;
            response = dat->o3d3xxCamera->getPCICDataConfigString();
        }
        else if (command == "XmlrpcCommand")
        {
            res = dat->o3d3xxCamera->xmlrpcCommand (param.c_str(), response);
            res = (res != ERR_NONE) ? PMD_RUNTIME_ERROR : PMD_OK;
        }
        else if (command == "SetTriggerMode")
        {
            res = dat->o3d3xxCamera->setTriggerMode ( (char *) cmd);
            if (res == 0)
            {
                response = "OK";
            }
            else
            {
                response = "failed";
                res = PMD_NOT_IMPLEMENTED;
            }
        }
        strncpy (result, response.c_str(), maxLen);
        result[maxLen - 1] = '\0';
        return res;
    }
    ///@endcond
    /// @}
    /// @}

}
