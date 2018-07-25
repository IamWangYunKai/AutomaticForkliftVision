/** @file
@brief Sample code to show usage of O3D3xx Camera operations.

We have defined a class which is used to perform basic O3D3xx camera
operations. This file serve as a sample code to show how this class functions
can be used to configure camera and get frame and image data from the O3D3xx camera

@author Winy Kakani\n
Version: 1.0\n
Copyright (C) 2014 ifm electronic GmbH\n
See accompanied file LICENCE.TXT for license information.
*/
/* Definitions of xmlrpc wrapper C++ class required
for doing basic camera operations */
#include "o3d3xx_camera.hpp"

/// @cond NOT_TO_BE_EXPOSED
#define NO_OF_FRAMES_TO_CAPTURE 100/**Indicates total number of frames
to be captured from camera. */
#define IP_ADDRESS_OF_CAMERA "192.168.0.69"
#define XMLRPC_PORT 80
#define PCIC_PORT 50010
///@endcond

int main_()
{
    uint16_t status_code;/* Code mentioning status of function execution */

    /* Create an object of O3D3xx camera class, which allows to perform basic
    functions on camera*/
    O3d3xxCameraOperations cameraOperations;

    /** Connect to camera, request a session, do default configuration and start grabbing the frames*/
    if((status_code = cameraOperations.connectToCamera(IP_ADDRESS_OF_CAMERA, XMLRPC_PORT, PCIC_PORT)) != 0)
    {
        printf("Connect to camera : %s\nPress any key to exit\n",cameraOperations.fault_string.c_str());
        getchar();
        exit(1);
    }
    
    uint32_t integrationTime=0;
    /** Setting integration time of camera */
    integrationTime=2000;
    if((status_code = cameraOperations.setIntegrationTime(integrationTime, 0)) != 0)
    {
        printf("Could not set requested integration time\n");
    }
    else
    {
        printf("\n\nIntegration time is set to %d\n",integrationTime);
    }        
    printf("\n\nTo check that the integration time value is correctly set into camera, getting integration time from camera.\n\nPress Enter key to get integration time from camera \n");
    getchar();

    /** Getting integration time of camera */

    integrationTime = 0;
    if((status_code = cameraOperations.getIntegrationTime(&integrationTime , 0)) != 0)
    {
        printf("Could not get integration time from camera\n");
        
    }
    else
    {
        printf("Integration time of camera is %d\n",integrationTime);
    }
    printf("\n\nPress Enter key to get resolution of camera \n");
    getchar();
    
    /** Get resolution of camera*/
    uint32_t frameWidth=0;
    uint32_t frameHeight=0;
    if((status_code = cameraOperations.getResolution(&frameWidth,&frameHeight)) !=0)
    {
        printf("Could not get resolution of camera\nExiting program\n");
        getchar();
        exit(1);
    }
    printf("Resolution at camera is width = %d, height = %d\n",frameWidth,frameHeight);
    printf("\nPress Enter key to start capturing frames from camera \n");
    getchar();
    /*Calculate number of pixels in an image */
    uint32_t noOfPixelsInImage = frameHeight * frameWidth;


    /**Get size of frame and allocate buffers for storing frame*/
    /* Get size of frame buffer in bytes */
    size_t frameSize=0;
    //if((status_code = cameraOperations.getFrameDataSize(&frameSize)) !=0)
    //{
    //    printf("Could not get size of frame data\n");
    //}
    //else
    //{
    //    printf("\nSize of frame data = %d\n",frameSize);
    //}

    ///* Pointers to memory allocated to hold different images */
    //float *pAmplitudeImageData = NULL;
    //float *pDistanceImageData = NULL;
    //float *pXImageData = NULL;
    //float *pYImageData = NULL;
    //float *pZImageData = NULL;
    //float *pXYZImageData = NULL;
    //uint32_t *pConfidenceImageData = NULL;

    ///* Allocate memory for holding images of frame*/
    //if( (pAmplitudeImageData  = (float *)calloc(noOfPixelsInImage,sizeof(float)) ) == NULL || \
    //    (pDistanceImageData  = (float *)calloc(noOfPixelsInImage,sizeof(float)) ) == NULL || \
    //    (pXImageData  = (float *)calloc(noOfPixelsInImage,sizeof(float)) ) == NULL || \
    //    (pYImageData  = (float *)calloc(noOfPixelsInImage,sizeof(float)) ) == NULL || \
    //    (pZImageData  = (float *)calloc(noOfPixelsInImage,sizeof(float)) ) == NULL || \
    //    (pXYZImageData  = (float *)calloc(noOfPixelsInImage*3,sizeof(float)) ) == NULL || \
    //    (pConfidenceImageData  = (uint32_t *)calloc(noOfPixelsInImage,sizeof(uint32_t)) ) == NULL )
    //{
    //    printf("Memory could not be allocated for image buffers.\nExiting program\n");
    //    getchar();
    //    exit(1);
    //}
    //
    ///** Capture frames from camera. Also get distance, amplitude, XYZ coordinate interleaved image, X coordinate image, Y coordinate image, Z coordinate image and Confidence image from camera*/
    //uint16_t noOfFramesToCapture = 0;/* Index to count number of frames captured from camera*/
    //while( noOfFramesToCapture < NO_OF_FRAMES_TO_CAPTURE)
    //{
    //    
    //    /*Capture a frame from camera */
    //    if( (status_code = cameraOperations.getFrameData()) != 0)
    //    {
    //        printf("Not able to capture frame data from camera\n");
    //        getchar();
    //        exit(1);
    //    }
    //    else
    //    {
    //        printf("Got frame data from camera\n");
    //    }
    //    /*Getting Amplitude image data */
    //    if( status_code = cameraOperations.getNormalizedAmplitudeImage(pAmplitudeImageData, \
    //        (size_t)(noOfPixelsInImage *sizeof(float))) != 0)
    //    {
    //        printf("Cannot get amplitude image from camera\n");
    //    }
    //    else
    //    {
    //        printf("done getting amplitude image data, first pixel = %f, last pixel=%f \n", \
    //            pAmplitudeImageData[0],pAmplitudeImageData[noOfPixelsInImage-1]);
    //    }
    //    
    //    /*Getting Distance image data */
    //    if( (status_code = cameraOperations.getDistanceImage(pDistanceImageData, (size_t)(noOfPixelsInImage *sizeof(float)))) != 0)
    //    {
    //        printf("Cannot get distance image from camera\n");
    //    }
    //    else
    //    {
    //        printf("done getting distance image data, first pixel = %f, last pixel=%f \n", \
    //            pDistanceImageData[0],pDistanceImageData[noOfPixelsInImage-1]);
    //    }
    //    
    //    /*Getting X coordinate image data */
    //    if((status_code = cameraOperations.getXCoordinateImage(pXImageData, (size_t)(noOfPixelsInImage *sizeof(float)))) != 0)
    //    {
    //        printf("Cannot get X coordinate image from camera\n");
    //    }
    //    else
    //    {
    //        printf("done getting X coordinate image data, first pixel = %f, last pixel=%f \n", \
    //            pXImageData[0],pXImageData[noOfPixelsInImage-1]);
    //    }
    //    
    //    /*Getting Y coordinate image data */
    //    if( (status_code = cameraOperations.getYCoordinateImage(pYImageData, (size_t)(noOfPixelsInImage *sizeof(float)))) != 0)
    //    {
    //        printf("Cannot get Y coordinate from camera\n");
    //    }
    //    else
    //    {
    //        printf("done getting Y image data, first pixel = %f, last pixel=%f \n",pYImageData[0], \
    //            pYImageData[noOfPixelsInImage-1]);
    //    }

    //    /*Getting Z coordinate image data */
    //    if( (status_code = cameraOperations.getZCoordinateImage(pZImageData, (size_t)(noOfPixelsInImage *sizeof(float)))) != 0)
    //    {
    //        printf("Cannot get Z coordinate image from camera\n");
    //    }
    //    else
    //    {
    //        printf("done getting image Z data, first pixel = %f, last pixel=%f \n",pZImageData[0],pZImageData[noOfPixelsInImage-1]);
    //    }

    //    /*Getting confidence image data */
    //    if( (status_code = cameraOperations.getConfidenceImage(pConfidenceImageData, (size_t)(noOfPixelsInImage *sizeof(uint32_t)))) != 0)
    //    {
    //        printf("Cannot get Confidence image from camera\n");
    //    }
    //    else
    //    {
    //        printf("done getting confidence image data, first pixel = %x, last pixel=%x \n",pConfidenceImageData[0],pConfidenceImageData[noOfPixelsInImage-1]);
    //    }
    //    
    //    /*Getting XYZ coordinate image data */
    //    if( (status_code = cameraOperations.getXYZCoordinateImages(pXYZImageData, (size_t)(noOfPixelsInImage *sizeof(float)*3))) != 0)
    //    {
    //        printf("Cannot get XYZ coordinate image from camera\n");
    //    }
    //    else
    //    {
    //        printf("done getting XYZ image data, coordinates, X = %f, Y= %f, z= %f\n",pXYZImageData[0],pXYZImageData[1],pXYZImageData[2]);
    //    }
    //    
    //    noOfFramesToCapture++;
    //};
    //
    ///*To let user see the console messages, the console 
    //doesn't dismisses until user enters a character. */
    //printf("Captured %d frames from camera. \n\n Press Enter key to disconnect camera\n\n",NO_OF_FRAMES_TO_CAPTURE);
    //getchar();

    /*Disconnect from camera */
    cameraOperations.disconnectFromCamera();
    return 0;
}