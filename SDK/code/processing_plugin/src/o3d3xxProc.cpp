/** @file
@brief This file represents functions which are required by
pmdaccess.dll for processing frame data available from o3D3xx camera

This is an implementation of the processing plugin interface of the PMDSDK2 as defined in pmdsdk.h.

@author Winy Kakani\n
Version: 1.0\n

Copyright (C) 2014 ifm electronic GmbH\n
See accompanied file LICENCE.TXT for license information.
*/
#include <iostream>
#include <sstream>
#include <map>
#include <stdlib.h>
#include <string.h>

#include <pmddatadescription.h> /*Definitions and structures which describe the data format of the PMDSDK2. */
#include <pmdsdk2common.h> /*Common definitions and structures used by the PMDSDK2. */

/* Standard include files */
#if (defined(_MSC_VER) && _MSC_VER < 1600)
#include <pstdint.h>
#else
#include <stdint.h>
#endif

#include "err.h"

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

/** Defines the size of protocol
header at start of frame buffer. Current PCIC version is V3, which requires
"4 (Ticket) + 10 (<Länge>) + 2 (CR/LF) + 4 (Ticket) +  4(output description, i.e
"star/stop")= 24 Bytes"*/
#define PCIC_V3_PROTOCOL_HEADER_SIZE_START 24

/** Defines the size of protocol
header at the end of frame. It is 4(output description) + 2 (CR LF) = 6 bytes*/
#define PCIC_V3_PROTOCOL_HEADER_SIZE_STOP 6
/** Defines the size of image header present in
frame. It contains 9 fields each of 4 bytes*/
#define IMAGE_HEADER_SIZE (4*9)
/** Defines number of bytes in diagonistic data, in bytes */
#define DIAGNOSTIC_DATA_SIZE 20
/** Defines number of images outputted by camera in a frame */
#define NO_OF_IMAGES_IN_FRAME 6


/**
@struct
@brief Structure to store Image relevant data.
Stores pointers to images in frame buffer.
Also contains size of images as expected by the PMDSDK2.
*/
struct structImageData
{
    void *pNormalizedAmplitudeImage;/**< Pointer to start of amplitude image in frame buffer */
    void *pDistanceImage;/**< Pointer to start of distance image in frame buffer */
    void *pXCoordinateImage;/**< Pointer to start of X coordinate image in frame buffer */
    void *pYCoordinateImage;/**< Pointer to start of Y coordinate image in frame buffer */
    void *pZCoordinateImage;/**< Pointer to start of X coordinate image in frame buffer */
    void *pConfidenceImage;/**< Pointer to start of confidence image in frame buffer */

    /*Sizes of images in bytes as expected by PmdSdk */
    size_t amplitudeImageSize; /**< Size of amplitude image in bytes */
    size_t distanceImageSize; /**< Size of distance image in bytes */
    size_t xyzCoordinateImageSize;/**< Size of interleaved xyz coordinate image in bytes */
    size_t confidenceImageSize;/**< Size of confidence image in bytes */
};

/**
@struct
@brief Structure to store frame data description.
Stores information about configuration by which frame was captured and its
size details too.
*/
struct FrameDataDescriptor
{
    static const uint8_t noOfPhases = 4;/**< indicates number of phases used by camera,
                                        treated as constant*/
    uint32_t noOfRows;/**< indicates number of rows in frame */
    uint32_t noOfColumns;/**< indicates number of columns in frame */
    uint32_t integrationTime;/**< indicates integration time of camera */
    uint32_t frameSizeInBytes;/**< indicates size of frame data in bytes */
    uint32_t actualReadFrameBytes;/*Inidcates actually how many bytes of frame
                                  were read from caemra*/
    structImageData ImageData;
};

/** @struct
* @brief Structure to store image header of a image, during parsing of frame buffer
*/
struct imageHeader
{
    uint32_t chunkType; /**< Defines the type of image */
    uint32_t chunkSize; /**<  Size of the whole image chunk in bytes.
                        After this count of bytes the next chunk starts. */
    /* Header size and version are not implemented in PCIC header */
    uint32_t headerSize; /* Size of header in bytes */
    uint32_t headerVersion; /*  Version number of the header */
    uint32_t imageWidth; /**< Image width in pixel */
    uint32_t imageHeight; /**< Image height in pixel */
    uint32_t pixelFormat; /**<  Format of pixel, i.e. data type */
    uint32_t timeStamp; /**< Time stamp in microseconds */
    /* Frame count is not implemented in PCIC header */
    uint32_t frameCount; /**<  Frame count according to algorithm output */
};

/**
@brief Parses current frame, and if a certain image is present in frame, it
marks a pointer, pointing to image's offset in the frame. This is done for
all possible image types in a frame.
@param[in] none
@return ERR_NONE, if no error occurred, else relevant error code
*/
int32_t parseFrameData (unsigned hnd , void *frameData , int length);

/**
@brief Get distance image from current frame
@param [in,out] data pointer to output buffer that shall contain distance image
@param[in] maxLen maximum length of output buffer
@return ERR_NONE, if no error occurred, else relevant error code
*/
int32_t getDistanceImage (unsigned hnd, float *data, size_t maxLen);

/**
@brief Get normalized amplitude image from current frame
@param[in,out] data pointer to output buffer that shall contain amplitude image
@param[in] maxLen maximum length of output buffer
@return ERR_NONE, if no error occurred, else relevant error code
*/
int32_t getNormalizedAmplitudeImage (unsigned hnd, float *data, size_t maxLen);

/**
@brief Get interleaved X,Y,Z coordinate image from current frame
@param[in,out] data pointer to output buffer that shall contain interleaved X,Y,Z
coordinate images from current frame image
@param[in] maxLen maximum length of output buffer
@return ERR_NONE, if no error occurred, else relevant error code
*/
int32_t getXYZCoordinateImages (unsigned hnd, float *data, size_t maxLen);

/**
@brief Get X coordinate image from current frame
@param[in,out] data pointer to output buffer that shall contain X
coordinate image from current frame image
@param[in] maxLen maximum length of output buffer
@return ERR_NONE, if no error occurred, else relevant error code
*/
int32_t getXCoordinateImage (unsigned hnd, float *data, size_t maxLen);

/**
@brief Get Y coordinate image from current frame
@param[in,out] data pointer to output buffer that shall contain Y
coordinate image from current frame image
@param[in] maxLen maximum length of output buffer
@return ERR_NONE, if no error occurred, else relevant error code
*/
int32_t getYCoordinateImage (unsigned hnd, float *data, size_t maxLen);

/**
@brief Get Z coordinate image from current frame
@param[in,out] data pointer to output buffer that shall contain Z
coordinate image from current frame image
@param[in] maxLen maximum length of output buffer
@return ERR_NONE, if no error occurred, else relevant error code
*/
int32_t getZCoordinateImage (unsigned hnd, float *data, size_t maxLen);

/**
@brief Get confidence image from current frame
@param[in,out] data pointer to output buffer that shall contain amplitude image
@param[in] maxLen maximum length of output buffer
@return ERR_NONE, if no error occurred, else relevant error code
*/
int32_t getConfidenceImage (unsigned hnd, uint32_t *data, size_t maxLen);

/**< Global count for number of connections */
unsigned g_did = 1;

std::string globalErrorMessage;/**< An string holding errors occurring during
                               pmdSdk calling the source plugin functions*/

/**@struct
@brief structure to hold local data of processing plugin
*/
struct ProcPluginData
{
    /** Constructor */
    ProcPluginData ()
    {
        errorMessage = "";
    };
    /**< Error message from processing  plugin */
    std::string errorMessage;
    FrameDataDescriptor frameDataDescriptor;
};

/**< Map instance IDs to ProcPluginData structures */
std::map <unsigned, ProcPluginData *> g_data;

/**
@brief Checks that whether the plugin of referred handle is already loaded or not.
@param[in] hnd The identifier of the connection of plugin instance
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

/// @addtogroup lightvis_plugins LightVis Wrapper Plugins
/// @{
/// @addtogroup Proc_plugin Processing Plugin
/// @{
extern "C" {

    /**
    @brief Returns the interface version of the processing plugin
    @param[in] none
    @return Version number of processing plugin
    */
    pmdDllExport unsigned pmdpInterfaceVersion ()
    {
        return PMD_INTERFACE_VERSION_1_3_0;
    }

    /**
    @brief Initializes processing plugin
    @param[in] handle identifier/handle of connection
    @param[in] param parameters of plugin instance
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpInitProcessingPlugin (unsigned handle, const char *param)
    {
        ProcPluginData dat;

        /* Check is already the plugin is loaded or not */
        if (g_data.find (handle) != g_data.end ())
        {
            globalErrorMessage = "handle already exists";
            return PMD_INVALID_VALUE;
        }

        /*If not present then add the plugin instance to global list
        Create a connection to processing plugin */
        g_data[handle] =  new ProcPluginData();

        return PMD_OK;
    }

    /**
    @brief De-initializes the processing plugin and
    frees the plugin memory and does clean up.
    @param[in] hnd
    identifier/handle of connection
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpDeinitPlugin (unsigned hnd)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        if(g_data[hnd])
        {
            delete g_data[hnd];
        }
        g_data.erase (hnd);
        return PMD_OK;
    }

    /**
    @brief Returns the last occurred error to calling function
    @param[in] hnd identifier/handle of connection
    @param[in,out] msg string containing the error message
    @param[in] maxLen maximum length of error message. It is size of "msg"
                        buffer passed as input argument. This is to be as an
                        additional check parameter, while copying buffer to
                        error message, to eliminate memory leaks
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpGetLastError (unsigned hnd, char *msg, size_t maxLen)
    {
        /* Check if the plugin is already loaded or not.
        If not then raise an error*/
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        /* Retrieving handle of plugin */
        ProcPluginData *dat = g_data[hnd];

        /* Copy the error message of plugin to output error message */
        if (maxLen && msg)
        {
            strncpy (msg, dat->errorMessage.c_str (), maxLen);
            msg[maxLen - 1] = 0;
        }
        return PMD_OK;
    }

    /**
    @brief Copies the processed image into the output buffer.
    Extracts the requested image from input buffer and copies it to
    output buffer.
    @param[in] hnd identifier/handle of connection
    @param[in] numOut number of images expected as output
    @param[in,out] fmtOut descriptor of output images
    @param[in,out] output pointer to buffer which shall contain output images
    @param[in] numIn number of images given as input
    @param[in,out] fmtIn descriptor of input images
    @param[in,out] input pointer to buffer which shall contain input images
    @return PMD_OK, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpProcess (unsigned hnd, unsigned numOut,
                                  PMDDataDescription *fmtOut, void **output, unsigned numIn,
                                  PMDDataDescription *fmtIn, void **input)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        /* Retrieving handle of plugin */
        ProcPluginData *dat = g_data[hnd];

        dat->frameDataDescriptor.frameSizeInBytes = fmtIn->size;
        dat->frameDataDescriptor.noOfColumns = fmtIn->img.numColumns;
        dat->frameDataDescriptor.noOfRows = fmtIn->img.numRows;

        /*Check that the buffer passed as input should have at least one image*/
        if (numIn < 1)
        {
            dat->errorMessage = "need at least one input data block";
            return PMD_INVALID_VALUE;
        }

        int dataIdx = -1;
        // search for supported data block...
        for(unsigned i=0; i < numIn; i++)
        {
            if(fmtIn[i].type == PMD_USER_DEFINED_0)
            {
                dataIdx = i;
            }
        }
        if(-1 == dataIdx)
        {
            dat->errorMessage = "no supported input data found";
            return PMD_DATA_NOT_FOUND;
        }

        unsigned  dd_out_size = 0 ; /* Contains size of the output image */

        /* Calculate size of an image. Sizes of all images will be same as data type of
        each image requires 4 bytes (float32)*/
        int imageSize = (sizeof(float) * fmtIn[dataIdx].img.numColumns *
                         fmtIn[dataIdx].img.numRows);

        if (parseFrameData (hnd, input[dataIdx], 0) != PMD_OK) // TODO calculation of lenggth or remove the parameter if not neccessary
        {
            memset (output[0], 0, fmtOut[0].size);
            return PMD_DATA_NOT_FOUND;
        }
        for (unsigned i = 0; i < numOut ; ++i)
        {
            /* PmdSDK passes type of the image, it requires in the output buffer.
            Also is passed in the input frame buffer containing images appended \
            one after another. As per type of output image required by PmdSDK, \
            For e.g., suppose processing pluign requires amplitude image in the output
            buffer for current frame. Then copy amplitude image from input buffer\
            (that contains distance, amplitude, xyz coordinate image etc, appended
            one after another), to output buffer.*/

            switch (fmtOut[i].type)
            {
                case PMD_AMPLITUDE_LF32: /*Fetch amplitude image */
                    {
                        getNormalizedAmplitudeImage (hnd, (float *) output[i], imageSize);
                        dd_out_size = imageSize;
                    }
                    break;

                case PMD_DISTANCE_LF32:/*Fetch distance image */
                    {
                        getDistanceImage (hnd, (float *) output[i], imageSize);
                        dd_out_size = imageSize;
                    }
                    break;

                case PMD_INTENSITY_LF32: /*Fetch intensity image. As we do not have intensity image, outputting amplitude image instead*/
                    {
                        getNormalizedAmplitudeImage (hnd, (float *) output[i], imageSize);
                        dd_out_size = imageSize;
                    }
                    break;

                case PMD_XYZ_COORD_LF32:/*Fetch XYZ interleaved coordinate image */
                    {
                        getXYZCoordinateImages (hnd, (float *) output[i], 3 * imageSize);
                        dd_out_size = imageSize * 3;
                        break;
                    }
                case PMD_FLAGS_L32:/*Fetch confidence image */
                    {
                        getConfidenceImage (hnd, (uint32_t *) output[i], imageSize / 4);
                        dd_out_size = imageSize;
                        break;
                    }
            }
            /* Update the descriptor of output image */
            PMDDataDescription tempDescriptor = fmtOut[i];
            fmtOut[i] = fmtIn[dataIdx];
            fmtOut[i].type = tempDescriptor.type;
            fmtOut[i].PID = hnd;
            fmtOut[i].DID = g_did++;
            fmtOut[i].size = dd_out_size;
            fmtOut[i].img.numSubImages = 1;
        }
        return PMD_OK;
    }

    /// @cond NOT_TO_BE_EXPOSED
    /**
    @brief This function is used by pmdsdk to check whether a given type of image can be processed or not.
    This functionality is not supported in o3d3xx source plugin.
    */
    pmdDllExport int pmdpCanProcess (unsigned hnd, int *result, unsigned type, unsigned numFmt, PMDDataDescription *fmt)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        if (numFmt < 1)
        {
            g_data[hnd]->errorMessage = "need at least one input data block";
            return PMD_INVALID_VALUE;
        }

        bool found = false;
        for(unsigned i=0; i<numFmt; i++)
        {
            if(fmt[i].type == PMD_USER_DEFINED_0)
            {
                found = true;
                break;
            }
        }
        if(!found)
        {
            g_data[hnd]->errorMessage = "input data types not supported";
            return PMD_DATA_NOT_FOUND;
        }

        *result = fmt[0].img.numRows * fmt[0].img.numColumns;
        switch(type)
        {
        case PMD_XYZ_COORD_LF32:
            *result *= sizeof(float)*3;
            break;
        case PMD_DISTANCE_LF32:
        case PMD_AMPLITUDE_LF32:
        case PMD_INTENSITY_LF32:
            *result *= sizeof(float);
            break;
        case PMD_FLAGS_L32:
            *result *= sizeof(unsigned);
            break;
        default:
            *result = -1;
        }
        return PMD_OK;
    }

    /**
    @brief Execute an source plugin-specific command.
    @param[in] hnd identifier/handle of connection
    @param[in,out] result Pointer to a block of memory to contain the result string.
    @param[in] maxLen Maximum length of the result string, including terminating 0.
    @param[in] cmd The command to be executed.
    @return PMD_NOT_IMPLEMENTED, if no error occurred, else relevant error code
    */
    pmdDllExport int pmdpProcessingCommand (unsigned hnd, char *result, size_t maxLen, const char *cmd)
    {
        if (!idOk (hnd))
        {
            globalErrorMessage = "unknown handle";
            return PMD_UNKNOWN_HANDLE;
        }

        /*Get handle of current plugin*/
        ProcPluginData *dat = g_data[hnd];

        dat->errorMessage = "no commands implemented";
        return PMD_NOT_IMPLEMENTED;
    }
    ///@endcond
    /// @}
    /// @}
}

int32_t parseFrameData (unsigned hnd , void *frameData , int length)
{
    imageHeader o3d3xxImageHeader;/**< Structure to store image header, parsed during
                                  parsing of frame buffer*/

    int8_t *pFrameData;

    pFrameData = (int8_t *) frameData + PCIC_V3_PROTOCOL_HEADER_SIZE_START ;

#ifdef DEBUG
    char pcicheader[50] = "";
    strncpy (pcicheader, (char *) frameData, 24);
    printf ("Pcic header of current frame is %s\n", pcicheader);
#endif

    uint8_t noOfImages = 0;/*Indicates number of images in frame buffer*/

    ProcPluginData *dat = g_data[hnd];
#ifdef DEBUG
    uint32_t data = 0;
    int32_t xyzdata = 0;
    int8_t confidencedata = 0;
#endif
    /*We know as PCIC schema is fixed in this release, we expect six images \
    from camera. One by one parse the images in the frame and mark offest of
    starting of each image in frame. */
    while (noOfImages++ < NO_OF_IMAGES_IN_FRAME)
    {
        memcpy (& (o3d3xxImageHeader), pFrameData , IMAGE_HEADER_SIZE);

        switch (o3d3xxImageHeader.chunkType)
        {
            /*Image is normalized amplitude image*/
            case 101:
                dat->frameDataDescriptor.ImageData.pNormalizedAmplitudeImage = pFrameData + IMAGE_HEADER_SIZE;
#ifdef DEBUG
                memcpy (&data, dat->frameDataDescriptor.ImageData.pNormalizedAmplitudeImage, sizeof (uint32_t));
                printf ("Got normalized image, pixel value is %x\n", data);
#endif
                break;

            /*Image is distance image*/
            case 100:
                dat->frameDataDescriptor.ImageData.pDistanceImage = pFrameData + IMAGE_HEADER_SIZE;
#ifdef DEBUG
                memcpy (&data, dat->frameDataDescriptor.ImageData.pDistanceImage, sizeof (uint32_t));
                printf ("Got distance image, pixel value is %x\n", data);
#endif
                break;

            /*Image is X coordinate image*/
            case 200:
                dat->frameDataDescriptor.ImageData.pXCoordinateImage = pFrameData + IMAGE_HEADER_SIZE;
#ifdef DEBUG
                memcpy (&xyzdata, dat->frameDataDescriptor.ImageData.pXCoordinateImage, sizeof (uint32_t));
                printf ("Got X coordinate image, pixel value is %x\n", xyzdata);
#endif
                break;

            /*Image is Y coordinate image*/
            case 201:
                dat->frameDataDescriptor.ImageData.pYCoordinateImage = pFrameData + IMAGE_HEADER_SIZE;
#ifdef DEBUG
                memcpy (&xyzdata, dat->frameDataDescriptor.ImageData.pYCoordinateImage, sizeof (int32_t));
                printf ("Got Y coordinate image, pixel value is %x\n", xyzdata);
#endif
                break;

            /*Image is Z coordinate image*/
            case 202:
                dat->frameDataDescriptor.ImageData.pZCoordinateImage = pFrameData + IMAGE_HEADER_SIZE;
#ifdef DEBUG
                printf ("Size of Z image is %d\n", o3d3xxImageHeader.chunkSize);
                memcpy (&xyzdata, dat->frameDataDescriptor.ImageData.pZCoordinateImage, sizeof (int32_t));
                printf ("Got Z coordinate image, pixel value is %x\n", xyzdata);
#endif
                break;

            /*Image is confidence image*/
            case 300:
                dat->frameDataDescriptor.ImageData.pConfidenceImage = pFrameData + IMAGE_HEADER_SIZE;
#ifdef DEBUG
                memcpy (&confidencedata, dat->frameDataDescriptor.ImageData.pConfidenceImage, sizeof (int8_t));
                printf ("Got confidence image, pixel value is %d\n", confidencedata);
#endif
                break;

            default:
                return PMD_DATA_NOT_FOUND;
                break;
        }
        pFrameData += o3d3xxImageHeader.chunkSize;
    };

    /*Skipping Diagnostic chunk information. */
    pFrameData += IMAGE_HEADER_SIZE + DIAGNOSTIC_DATA_SIZE;

    /*Skipping last 4 bytes af <stop> tag. */
    pFrameData += PCIC_V3_PROTOCOL_HEADER_SIZE_STOP;

    return PMD_OK;
}

int32_t getNormalizedAmplitudeImage (unsigned hnd, float *data, size_t maxLen)
{
    ProcPluginData *dat = g_data[hnd];

    if (dat->frameDataDescriptor.ImageData.pNormalizedAmplitudeImage != NULL)
    {
        uint16_t *pImageParsing = NULL;/*temporary variable for parsing image data*/

        /*Taking a temporary pointer which points to image buffer*/
        pImageParsing = (uint16_t *) (dat->frameDataDescriptor.ImageData.pNormalizedAmplitudeImage);

        /*From frame buffer, extract amplitude image. */
        for (unsigned i = 0; i < dat->frameDataDescriptor.noOfColumns * dat->frameDataDescriptor.noOfRows; i++)
        {
            *data++ = (float) (* (pImageParsing++));
        }
    }

    RETURN_NOERROR;
}

int32_t getDistanceImage (unsigned hnd, float *data, size_t maxLen)
{
    ProcPluginData *dat = g_data[hnd];

    if (dat->frameDataDescriptor.ImageData.pDistanceImage != NULL)
    {
        uint16_t *pImageParsing = NULL;/*temporary variable for parsing image data*/

        /*Taking a temporary pointer which points to image buffer*/
        pImageParsing = (uint16_t *) (dat->frameDataDescriptor.ImageData.pDistanceImage);

        /*Extracting image from frame buffer. The values are
        in millimeters, converting into meters by dividing by 1000 */
        for (unsigned i = 0; i < dat->frameDataDescriptor.noOfColumns * dat->frameDataDescriptor.noOfRows; i++)
        {
            *data++ = (float) (* (pImageParsing++)) / 1000.0f;
        }
    }
    RETURN_NOERROR;
}

int32_t getXYZCoordinateImages (unsigned hnd, float *data, size_t maxLen)
{
    ProcPluginData *dat = g_data[hnd];

    if (dat->frameDataDescriptor.ImageData.pXCoordinateImage != NULL || \
            dat->frameDataDescriptor.ImageData.pYCoordinateImage != NULL || \
            dat->frameDataDescriptor.ImageData.pZCoordinateImage != NULL)
    {
#ifdef DEBUG
        printf ("Size of xyz buffer could be maximum = %d\n", maxLen);
#endif
        /*temporary variable for parsing image data*/
        int16_t *pXImageParsing = NULL;
        int16_t *pYImageParsing = NULL;
        int16_t *pZImageParsing = NULL;

        /*Taking a temporary pointer which points to X,Y,Z coordinate image bufferss*/
        pXImageParsing = (int16_t *) (dat->frameDataDescriptor.ImageData.pXCoordinateImage);
        pYImageParsing = (int16_t *) (dat->frameDataDescriptor.ImageData.pYCoordinateImage);
        pZImageParsing = (int16_t *) (dat->frameDataDescriptor.ImageData.pZCoordinateImage);

        /**Extracting a one pixel of each X,Y and Z coordinate image from frame buffer.
        Copying it into input buffer in interleaved format like X1Y1Z1X2Y2Z2X3Y3Z3...*/
        /*The values are in millimeters, converting into meters by dividing by 1000. */
        for (unsigned i = 0; i < dat->frameDataDescriptor.noOfColumns * dat->frameDataDescriptor.noOfRows; i++)
        {
            *data++ = (int16_t) (* (pXImageParsing++)) / 1000.0f;
            *data++ = (int16_t) (* (pYImageParsing++)) / 1000.0f;
            *data++ = (int16_t) (* (pZImageParsing++)) / 1000.0f;
        }
    }

    RETURN_NOERROR;
}

int32_t getXCoordinateImage (unsigned hnd, float *data, size_t maxLen)
{
    ProcPluginData *dat = g_data[hnd];

    if (dat->frameDataDescriptor.ImageData.pXCoordinateImage != NULL)
    {
        int16_t *pImageParsing = NULL;/*temporary variable for parsing image data*/

        /*Taking a temporary pointer which points to image buffer*/
        pImageParsing = (int16_t *) (dat->frameDataDescriptor.ImageData.pXCoordinateImage);

        /*Extracting a X coordinate image from image buffer. The values are
        in millimeters, converting into meters by dividing by 1000 */
        for (unsigned i = 0; i < dat->frameDataDescriptor.noOfColumns * dat->frameDataDescriptor.noOfRows; i++)
        {
            *data++ = (int16_t) (* (pImageParsing++)) / 1000.0f;
        }
    }
    RETURN_NOERROR;
}

int32_t getYCoordinateImage (unsigned hnd, float *data, size_t maxLen)
{
    ProcPluginData *dat = g_data[hnd];

    if (dat->frameDataDescriptor.ImageData.pYCoordinateImage != NULL)
    {
        int16_t *pImageParsing = NULL;/*temporary variable for parsing image data*/

        /*Taking a temporary pointer which points to image buffer*/
        pImageParsing = (int16_t *) (dat->frameDataDescriptor.ImageData.pYCoordinateImage);

        /*Extracting a  Y coordinate  image from image buffer. The values are
        in millimeters, converting into meters by dividing by 1000 */
        for (unsigned i = 0; i < dat->frameDataDescriptor.noOfColumns * dat->frameDataDescriptor.noOfRows; i++)
        {
            *data++ = (int16_t) (* (pImageParsing++)) / 1000.0f;
        }
    }
    RETURN_NOERROR;
}

int32_t getZCoordinateImage (unsigned hnd, float *data, size_t maxLen)
{
    ProcPluginData *dat = g_data[hnd];

    if (dat->frameDataDescriptor.ImageData.pZCoordinateImage != NULL)
    {
        int16_t *pImageParsing = NULL;/*temporary variable for parsing image data*/

        /*Taking a temporary pointer which points to image buffer*/
        pImageParsing = (int16_t *) (dat->frameDataDescriptor.ImageData.pZCoordinateImage);

        /*Extracting a  Z coordinate image from image buffer. The values are
        in millimeters, converting into meters by dividing by 1000 */
        for (unsigned i = 0; i < dat->frameDataDescriptor.noOfColumns * dat->frameDataDescriptor.noOfRows; i++)
        {
            *data++ = (int16_t) (* (pImageParsing++)) / 1000.0f;
        }
    }
    RETURN_NOERROR;
}

/**
@internal
@details
This function shows the confidence image for last captured frame. Each pixel
of confidence is 8 bit value. This 8 bit value is stored in the LSB of output
confidence image.
*/
int32_t getConfidenceImage (unsigned hnd, uint32_t *data, size_t maxLen)
{
    ProcPluginData *dat = g_data[hnd];

    if (dat->frameDataDescriptor.ImageData.pConfidenceImage != NULL)
    {
        uint8_t *pImageParsing = NULL;/*temporary variable for parsing image data*/

        /*Taking a temporary pointer which points to image buffer*/
        pImageParsing = (uint8_t *) (dat->frameDataDescriptor.ImageData.pConfidenceImage);

        /*Extracting a confidence image from image buffer. Convert it from uint8_t to uint32 value.*/
        for (unsigned i = 0; i < dat->frameDataDescriptor.noOfColumns * dat->frameDataDescriptor.noOfRows; i++)
        {
            *data++ = (*pImageParsing++);
        }
    }
    RETURN_NOERROR;
}
