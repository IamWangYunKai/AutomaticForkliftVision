/** @file
@brief This file represents the definition of class which contains functions to
operate the O3D3xx camera.

@author Winy Kakani\n\n
@Version: 1.0\n\n

Copyright (C) 2014 ifm electronic GmbH\n\n
See accompanied file LICENCE.TXT for license information.
*/

/*! \mainpage O3D3xx Camera Plugin

* \section introduction Introduction
* This source plugin is developed to communicate with O3D3xx camera.
* Commands are invoked via XMLRpc interface
* ONLY Windows 32 and 64 bit OS platforms are supported.
*
*\section prerequisites Prerequisites
* To connect to camera user needs following information.
*   - IP address of camera.
*   - Tcp port number of camera for xmlrpc communications.
*   - Tcp port of camera for pcic communication for getting
*    connected to camera.
*   .
*
* Also, user shall be aware of the valid range of integration time
* that can be set on the camera.
*
* \section operational_details Operational Details
* The camera is assumed to be running
* in free run mode, i.e. it is constantly capturing frames. There are two
* tcp ports on the camera. One tcp port is for
* xmlrpc communication. The other port is known as pcic tcp port, on which
* camera broadcasts the frame data to the connected client. At a time only
* one client should be connected to camera.\n\n
* This plugin provides functions to,
*   - Connect to camera.
*   - Get the resolution of camera.
*   - Get the integration time of camera.
*   - Set the integration time of camera.
*   - Get the size of frame in bytes.
*   - Get a frame from camera.
*   - Disconnect from camera.
*   .
*
*\section usage Usage
*
*\subsection sample_c_code Sample C code - shows usage of class functions via C code.
*

*Steps to use sample code:
*   -# If your camera ip is 192.168.0.69, xmlrpc port is 80 and pcic port is 50010, then directly go to step 4.
*   -# In solution window, open file "o3d3xx_camera_sample_usage_code.cpp" listed under "SampleO3D3xxCamera" project.
*   -# Steps to change default camera ip address, xmlrpc port number and pcic port number
*       -# If your camera ip address is not 192.168.0.69,
*       find following line of code,
*       #define IP_ADDRESS_OF_CAMERA "192.168.0.69"
*       Replace 192.168.0.69 with your camera's ip address.
*
*       -# If your xmlrpc port number is not 80, then
*       Find following line of code,
*       #define XMLRPC_PORT 80
*       Replace "80" with your xmlrpc port number.
*
*       -# If your pcic port number is not 50010, then
*       Find following line of code,
*       #define PCIC_PORT 50010
*       Replace "50010" with your pcic port number.
*       Save the solution.
*   -# Compile the solution.
*   -# After successful compilation of the project, the executable for the sample C code shall be generated as
*   "SampleO3D3xxCamera.exe" at "build/bin/<configuration_folder>".<configuration_folder> denotes the configuration
*   by which you build the solution.
*   Double click the executable. It shall demonstrate configuration of camera and grabbing of frames and image data.
*
*\subsection lightVis LightVis - shows usage of O3D3xx plugin via third party application.
*
* Steps to use LightVis to communicate with camera:
*   -# Compile the solution.
*   -# After, successful compilation of the project, open "build/bin/<configuration_folder>" folder.
*   <configuration_folder> denotes the configuration by which you built the solution.
*   -# If your camera ip is 192.168.0.69, xmlrpc port is 80 and pcic port is 50010, then directly go to step 5.
*   -# The "build/bin/<configuration_folder>" folder contains "lv.cfg" file.
*   Open "lv.cfg" file in editor. This is configuration file of LightVis. It gives the details of camera ip address,
*   xmlrpc port, pcic port, source plugin binary name and processing plugin binary name to LightVis.
*
*   -# Find following line in "lv.cfg" file,\n
*   "192.168.0.69:80:50010".
*       -# If your camera ip address is not 192.168.0.69,
*       Replace 192.168.0.69 with your camera's ip address.
*
*       -# If your xmlrpc port number is not 80, then
*       Replace "80" with your xmlrpc port number.
*
*       -# If your pcic port number is not 50010, then
*       Replace "50010" with your pcic port number.
*       Save the file.
*
*       Suppose your camera ip address is 192.168.0.1, xmlrpc port is "8080" and pcic port is "2000", then the "lv.cfg"
*       file should have\n
*       192.168.0.1:8080:2000
*   -# Double click on "LightVis.exe". This is the LightVis executable.
*   -# LightVis shall show images captured from camera.Also integration time can be configured via LightVis.
*/
#include <xmlrpc-c/client.h> // headers of our xmlrpc client library
#include <xmlrpc-c/base.h>
#include "config.h"
#include "o3d3xx_camera.hpp" // The xmlrpc wrapper class implementing basic camera operations

#include <iostream>
#include "clientsocket.hpp"
#include "err.h"

#include <string>
#include <sstream>
#include <iomanip>
#include <vector>


/// @cond NOT_TO_BE_EXPOSED
/** Maximum length of session identifier*/
#define MAX_SESSION_ID_LENGTH 40
/** Url specifying root bject of O3D3xx camera interface */
#define ROOT_OBJECT_URL "/api/rpc/v1/com.ifm.efector/"

/* Macros for symbolizing edit and run mode of camera */
#define EDIT_MODE 1
#define RUN_MODE 0
/// @endcond

/** Default integration time in [us] */
#define DEFAULT_INTEGRATION_TIME 1234

/**Default frame rate of camera*/
#define DEFAULT_FRAME_RATE 25

/*Maximum length of ip address */
#define MAX_IP_ADDRESS_LENGTH 15

#define XMLRPCCHECK()\
{\
    int return_code = checkIfFaultOccurred(); \
    if(  return_code != 0 ) \
        { \
        return return_code;\
        } \
}

/// @cond NOT_TO_BE_EXPOSED

/** @struct
@brief Structure defining private data of this class
*/
struct O3d3xxCameraOperations::PrivateData
{
    PrivateData()
    {
        cameraXmlrpcPortNumber = HTTP_PORT_NUMBER;
        sessionUrl = "";
        applicationId = -1;
        isConnected = false;
        frameSizeInBytes = 0;
        isApplicationCreated = false;
        triggeredMode = 0;
    }

    /**If a session is active, this url should be containing the
       session's url, else it should be empty, indicating that
       there is no active session.
       */
    string sessionUrl;

    /** This is id of application, which we create on camera.
        Parameters are configured at the application represented by this ID.
        Valid ID range: 0 to 32, -1 indicats an invalid ID.
        */
    int applicationId;

    /** IP address of camera */
    string cameraIpAddress;

    /** Port number for XML-RPC communication */
    uint16_t cameraXmlrpcPortNumber;

    /** Port number of frame data (Protocol: PCIC) */
    uint16_t cameraImageOutputPortNumber;

    /** XML-RPC represents runtime errors as <fault> elements. These contain
    <faultCode> and <faultString> elements. During calling of xmlrpc functions,
    any error occurs, its details are logged in this variable. In other words,
    this variable keeps track of last occurred error */
    xmlrpc_env environment;

    /** Data size of currentFrame > */
    size_t frameSizeInBytes;

    /** TCP Socket >*/
    ClientSocket *clientSocket;

    /** Flag indicating established connection. 0: not connected, 1: connection Ok */
    bool isConnected;

    /** Flag indicating, if a new application has been created on the camera by this plugin.
        0: new application created, 1: use active application on connect. */
    bool isApplicationCreated ;

    /** The trigger mode of the camera */
    int triggeredMode;

    /** Buffer holding current frame's data received from camera.*/
    std::vector<char> frameData;

    /** string for define PCIC communcation */
    std::string dataChannelConfigString;

    /** Description of error/fault occurred, if any, as reported by plugin instance*/
    std::string fault_string;
};

/**
@internal
@details
Compilers older than C++11, do not support to_string() function of
string class. Therefor define it here.
@param [in]
Value to be converted to string
@return
Converted string value
*/
template <typename T>
std::string to_string (T value)
{
    std::ostringstream os ;
    os << value ;
    return os.str() ;
}
///@endcond

O3d3xxCameraOperations::O3d3xxCameraOperations()
{
    mp_d = new PrivateData();
    mp_d->fault_string = "";
}

O3d3xxCameraOperations::~O3d3xxCameraOperations()
{
    if (mp_d->isConnected)
    {
        disconnectPCIC();
    }
    /* Free the allocated memory of private data */
    if (mp_d)
    {
        delete mp_d;
        mp_d = 0;
    }
}

int32_t O3d3xxCameraOperations::getFrameDataSize (size_t *dataSize)
{
    *dataSize = mp_d->frameSizeInBytes;
    RETURN_NOERROR;
}

void O3d3xxCameraOperations::setPCICDataConfigString (const std::string &s)
{
    mp_d->dataChannelConfigString = s;
}

std::string O3d3xxCameraOperations::getPCICDataConfigString()
{
    return mp_d->dataChannelConfigString;
}

int32_t O3d3xxCameraOperations::getResolution (uint32_t *frameWidth, uint32_t *frameHeight)
{
    std::string res;
    std::vector<std::string> param ;
    int ret;

    // Variables to store first/last column/row number of a frame
    int indexofFirstRow = 0;
    int indexofLastRow = 0;
    int indexofFirstColumn = 0;
    int indexofLastColumn = 0;

    bool selfCreatedSession = false;/* need to know this for cleanup of session */

    // non-empty session-url indicates that a session is already established. If not start
    // a session and start editing application, which we had created
    // during connect
    if (mp_d->sessionUrl.empty())
    {
        switchCameraToConfigurationMode();

        selfCreatedSession = true;

        // Creating an application and setting it to edit mode ...
        param.clear();
        res = "";
        param.push_back (to_string<int> (mp_d->applicationId));
        ret = xmlrpcCommandImplementation (mp_d->sessionUrl + "edit/", "editApplication", "(s)", param, res);
        if (ret != 0)
        {
            return ret;
        }
    }

    // Generating string representing application object's url
    string appimager_url = mp_d->sessionUrl + "edit/application/imager_001";

    // Get index of first/last row and first/last column from camera.and calculate the matrix resolution...

    // Get index of first row of frame
    param.clear();
    res = "";
    param.push_back ("ClippingTop");
    ret = xmlrpcCommandImplementation (appimager_url, "getParameter", "(s)", param, res);
    if (ret != 0)
    {
        return ret;
    }
    indexofFirstRow = atoi (res.c_str());

    // Get index of last row of frame
    param.clear();
    res = "";
    param.push_back ("ClippingBottom");
    ret = xmlrpcCommandImplementation (appimager_url, "getParameter", "(s)", param, res);
    if (ret != 0)
    {
        return ret;
    }
    indexofLastRow = atoi (res.c_str());

    // Get index of first column of frame
    param.clear();
    res = "";
    param.push_back ("ClippingLeft");
    ret = xmlrpcCommandImplementation (appimager_url, "getParameter", "(s)", param, res);
    if (ret != 0)
    {
        return ret;
    }
    indexofFirstColumn = atoi (res.c_str());

    // Get index of last column of frame
    param.clear();
    res = "";
    param.push_back ("ClippingRight");
    ret = xmlrpcCommandImplementation (appimager_url, "getParameter", "(s)", param, res);
    if (ret != 0)
    {
        return ret;
    }
    indexofLastColumn = atoi (res.c_str());

    *frameWidth = indexofLastColumn - indexofFirstColumn + 1; // indices are zero based --> +1
    *frameHeight = indexofLastRow - indexofFirstRow + 1;

    if (selfCreatedSession)
    {
        // Stop editing the application
        param.clear();
        res = "";
        ret = xmlrpcCommandImplementation (mp_d->sessionUrl + "edit/", "stopEditingApplication", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }

        exitFromCameraConfigurationMode();

    }

    RETURN_NOERROR;
}

int32_t O3d3xxCameraOperations::getIntegrationTime (uint32_t *integrationTime , unsigned int index)
{
    std::string res;
    std::vector<std::string> param ;
    int ret ;

    bool selfCreatedSession = true;

    // Check if already attached to a session.
    if (mp_d->sessionUrl.empty())
    {
        switchCameraToConfigurationMode();
        selfCreatedSession = true;

        // Setting application in edit mode
        param.clear();
        res = "";
        param.push_back (to_string<int> (mp_d->applicationId));
        ret = xmlrpcCommandImplementation (mp_d->sessionUrl + "edit/", "editApplication", "(s)", param, res);
        if (ret != 0)
        {
            return ret;
        }
    }

    string appimager_url = "";
    // Generating string representing application object's url
    appimager_url = mp_d->sessionUrl + "edit/application/imager_001";

    param.clear();
    res = "";
    param.push_back ("ExposureTime");
    ret = xmlrpcCommandImplementation (appimager_url, "getParameter", "(s)", param, res);

    *integrationTime = atoi (res.c_str());

    // If this is self created session, do cleanup
    if (selfCreatedSession)
    {
        // Stop editing the application
        param.clear();
        res = "";
        ret = xmlrpcCommandImplementation (mp_d->sessionUrl + "edit/", "stopEditingApplication", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }

        exitFromCameraConfigurationMode();
    }
    RETURN_NOERROR;
}

int32_t O3d3xxCameraOperations::setIntegrationTime (uint32_t integrationTime ,  unsigned int index)
{
    if (index != 0)
    {
        mp_d->fault_string = "Multiple exposure mode not supported" ;
        return 0;
    }

    std::string res;
    std::vector<std::string> param ;
    int ret ;
    bool selfCreatedSession = false;

    if (mp_d->sessionUrl.empty())
    {
        switchCameraToConfigurationMode();

        selfCreatedSession = true;

        // Setting application in edit mode
        param.clear();
        res = "";
        param.push_back (to_string<int> (mp_d->applicationId));
        ret = xmlrpcCommandImplementation (mp_d->sessionUrl + "edit/", "editApplication", "(s)", param, res);
        if (ret != 0)
        {
            return ret;
        }
    }

    string appimager_url = "";
    // string representing of application url
    appimager_url = mp_d->sessionUrl + "edit/application/imager_001";

    param.clear();
    res = "";
    param.push_back ("ExposureTime");
    param.push_back (to_string ( (unsigned long long) integrationTime));
    ret = xmlrpcCommandImplementation (appimager_url, "setParameter", "(ss)", param, res);
    if (ret != 0)
    {
        return ret;
    }

    // do cleanup
    if (selfCreatedSession)
    {
        std:: string edit_url = mp_d->sessionUrl + "edit/";

        // Save the application...
        param.clear();
        res = "";
        std::string app_url = "";
        app_url = edit_url + "application/";
        ret = xmlrpcCommandImplementation (app_url, "save", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }

        edit_url = "";
        //Generating string representing edit object's url
        edit_url = mp_d->sessionUrl + "edit/";

        // Stop editing the application
        param.clear();
        res = "";
        ret = xmlrpcCommandImplementation (edit_url, "stopEditingApplication", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }

        exitFromCameraConfigurationMode();
    }

    RETURN_NOERROR;
}

int O3d3xxCameraOperations::checkParameterValidity (string ip_address, uint16_t port_number, uint16_t pcic_tcp_port , unsigned int applicatioID)
{

    // IP address validation
    if (ip_address.length() > 15) //MAX_IP_ADDRESS_LENGTH)
    {
        mp_d->fault_string = "Invalid ip address given as input.";
        RETURN_ERROR (ERR_INVALID_ARG);
    }

    if (ip_address == "0.0.0.0")
    {
        mp_d->fault_string = "Invalid ip address \"0.0.0.0\" given as input. Value of octet should be between 1 to 255";
        RETURN_ERROR (ERR_INVALID_ARG);
    }

    std::size_t found = ip_address.find_first_of (".");
    string octet = "";
    uint8_t octetStartPos = 0;
    uint8_t noOfDots = 0;
    while (octetStartPos < ip_address.length())
    {
        octet = ip_address.substr (octetStartPos, found - octetStartPos);
        if (atoi (octet.c_str()) > 255)
        {
            mp_d->fault_string = "Invalid ip address given as input. Value of octet should be between 1 to 255";
            RETURN_ERROR (ERR_INVALID_ARG);
        }

        noOfDots++;
        octetStartPos = found + 1;
        found = ip_address.find_first_of (".", found + 1);
        if (found == std::string::npos)
        {
            found = ip_address.length();
        }
    }

    if (noOfDots != 4)
    {
        mp_d->fault_string = "Invalid ip address given as input. 4 octets required in ip address";
        RETURN_ERROR (ERR_INVALID_ARG);
    }

    // Test that the port numbers provided as input are valid,i.e non zero
    if ( (port_number != 80) || (pcic_tcp_port != 50010))
    {
        mp_d->fault_string = "Invalid port number given as input. Valid value is from 1 to 65535";
        RETURN_ERROR (ERR_INVALID_ARG);
    }

    mp_d->cameraIpAddress = ip_address;
    mp_d->cameraXmlrpcPortNumber = port_number;
    mp_d->cameraImageOutputPortNumber = pcic_tcp_port;
    if (applicatioID < 33)  // max numberofapplication
    {
        mp_d->applicationId = applicatioID;
    }
    else
    {
        mp_d->applicationId = 0;
    }
    RETURN_NOERROR;
}

int32_t O3d3xxCameraOperations::connectToCamera (string ip_address, \
        uint16_t port_number, uint16_t pcic_tcp_port , unsigned int applicatioID)
{
    string edit_url, app_url, appimager_url, device_url;
    std::string res;
    std::vector<std::string> param ;
    int ret;

    if (checkParameterValidity (ip_address, port_number, pcic_tcp_port, applicatioID) != 0)
    {
        RETURN_ERROR (ERR_INVALID_ARG);
    }

    // Initialize our xmlrpc error-handling environment.
    xmlrpc_env_init (& (mp_d->environment));

    /*Check if any error occurred*/
    XMLRPCCHECK()

    /* Start up our XML-RPC client.*/
    xmlrpc_client_init2 (& (mp_d->environment), \
                         XMLRPC_CLIENT_NO_FLAGS, "Xmlrpc-c Test Client", "1.0", NULL, 0);
    XMLRPCCHECK()

    /**Creating an application, configuring it with default values
    and activating it on camera*/

    int32_t status_code = switchCameraToConfigurationMode();
    if (status_code != 0)
    {
        // Clean up our error-handling environment.
        xmlrpc_env_clean (& (mp_d->environment));

        // Shutdown our XML-RPC client library
        xmlrpc_client_cleanup();

        mp_d->fault_string = "Camera device is not connected\n";

        //If camera not connected, return relevant error code
        RETURN_ERROR (status_code);
    }

    // Save and activate a default configuration on camera...

    edit_url = mp_d->sessionUrl + "edit/";
    if (mp_d->applicationId != 0) // user want to use active application on camera
    {
        device_url = "";
        // Generating string representing device object's url
        device_url = edit_url + "device/";

        param.clear();
        param.push_back ("ActiveApplication");
        ret = xmlrpcCommandImplementation (device_url, "getParameter", "(s)", param, res);
        if (ret != 0)
        {
            return ret;
        }
        mp_d->applicationId = atoi (res.c_str());

        // if application ID is zero means no active application
        if (mp_d->applicationId != 0)
        {
            // Setting application in edit mode
            param.clear();
            res = "";
            param.push_back (to_string<int> (mp_d->applicationId));

            ret = xmlrpcCommandImplementation (edit_url, "editApplication", "(s)", param, res);
            if (ret != 0)
            {
                return ret;
            }

            app_url = mp_d->sessionUrl + "edit/application/";

            param.clear();
            res = "";
            param.push_back ("TriggerMode");
            //getting the trigger mode
            ret = xmlrpcCommandImplementation (app_url, "getParameter", "(s)", param, res);
            if (ret != 0)
            {
                return ret;
            }

            mp_d->triggeredMode = atoi (res.c_str());

            param.clear();
            res = "";
            ret = xmlrpcCommandImplementation (edit_url, "stopEditingApplication", "()", param, res);
            if (ret != 0)
            {
                return ret;
            }

        }
    }

    if (mp_d->applicationId == 0) // default case
    {
        param.clear();
        res = "";
        //creating the application
        ret = xmlrpcCommandImplementation (edit_url, "createApplication", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }
        // storing the application ID
        mp_d->applicationId = atoi (res.c_str());

        mp_d->isApplicationCreated = true;
#ifdef DEBUG
        /* Printing the result value */
        printf ("Now, The application id  is %d\n", mp_d->applicationId);
#endif

        // Setting application in edit mode
        param.clear();
        res = "";
        param.push_back (to_string<int> (mp_d->applicationId));
        ret = xmlrpcCommandImplementation (edit_url, "editApplication", "(s)", param, res);
        if (ret != 0)
        {
            return ret;
        }

        app_url = mp_d->sessionUrl + "edit/application/";

        param.clear();
        res = "";
        param.push_back ("TriggerMode");
        //getting the trigger mode
        ret = xmlrpcCommandImplementation (app_url, "getParameter", "(s)", param, res);
        if (ret != 0)
        {
            return ret;
        }

        mp_d->triggeredMode = atoi (res.c_str());


        // Set integration time to camera as default
        ret = setIntegrationTime (DEFAULT_INTEGRATION_TIME , 0) ;
        if (ret != 0)
        {
            return ret;
        }

        /*Setting default frame rate */
        param.clear();
        res = "";
        param.push_back ("FrameRate");
        param.push_back (to_string ( (unsigned long long) DEFAULT_FRAME_RATE).c_str());

        appimager_url = "";
        // Generating string representing application imager object's url
        appimager_url = mp_d->sessionUrl + "edit/application/imager_001";

        ret = xmlrpcCommandImplementation (appimager_url, "setParameter", "(ss)", param, res);
        if (ret != 0)
        {
            // Clean up our error-handling environment.
            std::string err = mp_d->environment.fault_string;
            xmlrpc_env_clean (& (mp_d->environment));

            // Shutdown our XML-RPC client library
            xmlrpc_client_cleanup();

            mp_d->fault_string = err;

            //If camera not connected, return relevant error code
            RETURN_ERROR (status_code);
            return ret;
        }

        // Save the application
        param.clear();
        res = "";
        app_url = "";
        // Generating string representing application object's url
        app_url = edit_url + "application/";
        ret = xmlrpcCommandImplementation (app_url, "save", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }

        /* Validate the application */
        //param.clear();
        //res = "";
        //app_url="";
        ///*Generating string representing application object's url */
        //app_url = edit_url + "application/";
        //ret = xmlrpcCommandImplementation(app_url,"validate","()",param,res);
        //  if(ret != 0)
        //  {
        //      return ret;
        //  }

        // Stop editing the application
        param.clear();
        res = "";
        ret = xmlrpcCommandImplementation (edit_url, "stopEditingApplication", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }

#ifdef DEBUG
        printf ("stopped application\n");
#endif

        /*Activate the configuration on camera */
        param.clear();
        res = "";
        device_url = "";
        param.push_back ("ActiveApplication");
        param.push_back (to_string ( (unsigned long long) (mp_d->applicationId)));
        /* Generating string representing device object's url */
        device_url = edit_url + "device/";
        ret = xmlrpcCommandImplementation (device_url, "setParameter", "(ss)", param, res);
        if (ret != 0)
        {
            return ret;
        }

        // Save device configuration
        param.clear();
        res = "";
        ret = xmlrpcCommandImplementation (device_url, "save", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }
    }

    exitFromCameraConfigurationMode();

    RETURN_NOERROR;
}


int32_t O3d3xxCameraOperations::connecttoPCIC()
{
    try
    {
        if (mp_d->isConnected)
        {
            RETURN_NOERROR;
        }
        else
        {
            mp_d->clientSocket = new ClientSocket (mp_d->cameraIpAddress.c_str(), mp_d->cameraImageOutputPortNumber);
            if (!mp_d->dataChannelConfigString.empty())
            {
                std::string data ("1234"); // protocol version for c command
                data += mp_d->dataChannelConfigString + "\r\n"; // the command itself
                std::stringstream msg;
                msg << "1234L" << std::setfill ('0') << std::setw (9) << data.length() << "\r\n" << data; // adding envelope
                mp_d->clientSocket->sendFrame (msg.str().length(), msg.str().c_str());
                // check resonse
                char response[23];
                mp_d->clientSocket->receiveFrame (sizeof (response), &response);
                if (response[20] != '*')
                {
                    throw std::exception ("Sending PCIC config string not successfull");
                }
            }
            mp_d->isConnected = true;
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        RETURN_ERROR (ERR_MEMORY);
    }
    RETURN_NOERROR;
}

int32_t O3d3xxCameraOperations::disconnectPCIC()
{
    if (mp_d->clientSocket != NULL && mp_d->isConnected)
    {
        delete mp_d->clientSocket;
        this->mp_d->isConnected = false;
    }
    return 0;
}

int32_t O3d3xxCameraOperations::disconnectFromCamera()
{
    std::string res;
    std::vector<std::string> param ;
    int ret;

    if (mp_d->isApplicationCreated)
    {
        // Delete the default configuration and kill the client

        switchCameraToConfigurationMode();

        string edit_url = "";
        //Generating string representing application object's url
        edit_url = mp_d->sessionUrl + "edit/";
        param.clear();
        res = "";
        param.push_back (to_string ( (unsigned long long) (mp_d->applicationId)));
        // Deleting the application, that we created
        ret = xmlrpcCommandImplementation (edit_url, "deleteApplication", "(s)", param, res);
        if (ret != 0)
        {
            return ret;
        }

        // Save device configuration
        string device_url = "";
        // Generating string representing device object's url
        device_url = edit_url + "device/";

        param.clear();
        res = "";
        // Deleting the application, that we created
        ret = xmlrpcCommandImplementation (device_url, "save", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }

        exitFromCameraConfigurationMode();
    }

    mp_d->frameData.clear();
    // Shutdown our XML-RPC client library.
    xmlrpc_env_clean (&mp_d->environment);
    xmlrpc_client_cleanup();

    disconnectPCIC();

    RETURN_NOERROR;
}

/// @cond NOT_TO_BE_EXPOSED


uint32_t O3d3xxCameraOperations::updateCameraFrame()
{
    char frameLengthfrmHeader[10];
    char frameheader[16];

    if (mp_d->isConnected)
    {
        memset (frameheader, 0, 16);
        std::string  trigger = "1111L000000008\r\n1111T?\r\n";
        if (mp_d->triggeredMode == 2)
        {
            mp_d->clientSocket->sendFrame (trigger.length(), trigger.c_str());
        }

        int index = mp_d->applicationId;
        mp_d->clientSocket->receiveFrame (16, frameheader); // reading 1st 16byte of header

        //protocol tck 4byte 'L0000000000' 9 byte length and one byte for L and /r/n
        memcpy (frameLengthfrmHeader , frameheader + 5 , 9); // +5 is to get to the start of the length,
        // next 9 byte will content length value

        uint32_t frameSize = atoi (frameLengthfrmHeader) + 16; // making an integer value

        // handling memory for different mode
        if (frameSize > mp_d->frameSizeInBytes)
        {
            mp_d->frameData.clear();
            mp_d->frameData.resize (frameSize);
        }
        mp_d->frameSizeInBytes  = frameSize; // copying new data size value
        memcpy (& (mp_d->frameData[0]), frameheader, 16); // adding the intial data
        // receiveFrame
        mp_d->clientSocket->receiveFrame (mp_d->frameSizeInBytes - 16  , & (mp_d->frameData[16]));


        RETURN_NOERROR;
    }
    RETURN_ERROR (-1);
}


int32_t O3d3xxCameraOperations::switchCameraToConfigurationMode()
{
    std::string res;
    std::vector<std::string> param ;
    int ret ;
    if (mp_d->isConnected)
    {
        disconnectPCIC();
    }

    string serverUrl ("http://" + mp_d->cameraIpAddress + ":" + \
                      to_string ( (unsigned long long) (mp_d->cameraXmlrpcPortNumber)) + ROOT_OBJECT_URL);

    param.clear();
    res = "";
    param.push_back ("");
    param.push_back ("");
    //creating session
    ret = xmlrpcCommandImplementation (serverUrl, "requestSession", "(ss)", param, res);
    if (ret != 0)
    {
        return ret;
    }

    //Generating string representing session object's url
    mp_d->sessionUrl =  serverUrl + "session_" + res + "/";

    /* Setting operating mode to "edit" mode */
    param.clear();
    res = "";
    param.push_back (to_string<int> (EDIT_MODE));

    ret = xmlrpcCommandImplementation (mp_d->sessionUrl, "setOperatingMode", "(s)", param, res);
    if (ret != 0)
    {
        return ret;
    }

    RETURN_NOERROR;
}

int32_t O3d3xxCameraOperations::exitFromCameraConfigurationMode()
{
    std::string res;
    std::vector<std::string> param ;
    int  ret;

    // Setting operating mode to "run" mode
    param.clear();
    res = "";
    param.push_back (to_string<int> (RUN_MODE));
    ret = xmlrpcCommandImplementation (mp_d->sessionUrl, "setOperatingMode", "(s)", param, res);
    if (ret != 0)
    {
        return ret;
    }

    param.clear();
    res = "";

    ret = xmlrpcCommandImplementation (mp_d->sessionUrl, "cancelSession", "()", param, res);
    if (ret != 0)
    {
        return ret;
    }

    // As we have ended our session, set the session url variable to an empty string
    mp_d->sessionUrl = "";

    if (!mp_d->isConnected)
    {
        connecttoPCIC();
    }
    RETURN_NOERROR;
}

int32_t O3d3xxCameraOperations::checkIfFaultOccurred()
{
    if (mp_d->environment.fault_occurred)
    {
        fprintf (stdout, "ERROR: %s (%d)\n",
                 mp_d->environment.fault_string, \
                 mp_d->environment.fault_code);
        return mp_d->environment.fault_code;
    }

    RETURN_NOERROR;
}

std::string O3d3xxCameraOperations:: getLastError (void)
{
    return mp_d->fault_string;
}

uint32_t  O3d3xxCameraOperations::getPCICpacket (void *data)
{
    memcpy (data, & (mp_d->frameData[0]), mp_d->frameSizeInBytes);
    return 0;
}

std::vector<std::string> splitCommand (const char *cmd)
{
    std::istringstream ss (cmd);
    std::vector<std::string> el;
    do
    {
        std::string s;
        ss >> s;
        if (!s.length())
        {
            continue;
        }
        el.push_back (s);
    }
    while (ss);
    return el;
}

uint32_t O3d3xxCameraOperations::xmlrpcCommand (const char *cmd, std::string &res)
{
    std::vector<std::string> el = splitCommand (cmd);
    switch (el.size())
    {
        case 0:
            res = "empty xml url";
            RETURN_ERROR (1);
            break;
        case 1:
            res = "missing command";
            RETURN_ERROR (1);
            break;
        case 2:
            res = "missing format and parameters";
            RETURN_ERROR (1);
            break;
        default:
            break;
    }
    std::string url ("http://" + mp_d->cameraIpAddress + ":" + \
                     to_string ( (unsigned long long) (mp_d->cameraXmlrpcPortNumber)) + el[0]);
    el.erase (el.begin());
    std::string command = el[0];
    el.erase (el.begin());
    std::string format;
    if (el.size() > 0)
    {
        format = el[0];
        el.erase (el.begin());
    }
    return xmlrpcCommandImplementation (url, command, format, el, res);
}

std::string extractStringFromXmlRpc (const xmlrpc_value *val, xmlrpc_env &env, bool quotes = false)
{
    std::stringstream ss;
    const char *r;
    xmlrpc_read_string (&env, val, &r);
    if (quotes)
    {
        ss << "\"";
    }
    if (!env.fault_occurred)
    {
        ss << r;
    }
    if (quotes)
    {
        ss << "\"";
    }
    free ( (void *) r);
    return ss.str();
}

int extractIntFromXmlRpc (const xmlrpc_value *val, xmlrpc_env &env)
{
    int res;
    xmlrpc_read_int (&env, val, &res);
    return env.fault_occurred ? 0 : res;
}

xmlrpc_bool extractBoolFromXmlRpc (const xmlrpc_value *val, xmlrpc_env &env)
{
    xmlrpc_bool res;
    xmlrpc_read_bool (&env, val, &res);
    return env.fault_occurred ? 0 : res;
}

double extractDoubleFromXmlRpc (const xmlrpc_value *val, xmlrpc_env &env)
{
    double res;
    xmlrpc_read_double (&env, val, &res);
    return env.fault_occurred ? 0. : res;
}

std::string O3d3xxCameraOperations::extractArrayFromXmlRpc (xmlrpc_value *val, xmlrpc_env &env)
{
    std::string s;
    int num = xmlrpc_array_size (&env, val);
    for (int i = 0; i < num; i++)
    {
        xmlrpc_value *value = 0;
        xmlrpc_array_read_item (&env, val, i, &value);
        if (i > 0)
        {
            s += ",";
        }
        s += "[" + extractFromXmlRpc (value, env, true) + "]"; // can have different primitive types
        free (value);
    }
    return s;
}

std::string O3d3xxCameraOperations::extractStructFromXmlRpc (xmlrpc_value *val, xmlrpc_env &env)
{
    std::string s;
    int num = xmlrpc_struct_size (&env, val);
    s += "{";
    for (int i = 0; i < num; i++)
    {
        xmlrpc_value *key = 0, *value = 0;
        xmlrpc_struct_read_member (&env, val, i, &key, &value);
        if (i > 0)
        {
            s += ",";
        }
        s += extractStringFromXmlRpc (key, env, true);
        s += ":";
        s += extractFromXmlRpc (value, env, true); // can have different primitive types
        free (key);
        free (value);
    }
    s += "}";
    return s;
}

std::string O3d3xxCameraOperations::extractFromXmlRpc (xmlrpc_value *val, xmlrpc_env &env, bool quotes) throw (std::string)
{
    std::stringstream ss;
    switch (val->_type)
    {
        case XMLRPC_TYPE_INT:
            ss << extractIntFromXmlRpc (val, env);
            break;
        case XMLRPC_TYPE_BOOL:
            ss << extractBoolFromXmlRpc (val, env);
            break;
        case XMLRPC_TYPE_DOUBLE:
            ss << extractDoubleFromXmlRpc (val, env);
            break;
        case XMLRPC_TYPE_ARRAY:
            ss << extractArrayFromXmlRpc (val, env);
            break;
        case XMLRPC_TYPE_STRUCT:
            ss << extractStructFromXmlRpc (val, env);
            break;
        case XMLRPC_TYPE_STRING:
            ss << extractStringFromXmlRpc (val, env, quotes);
            break;
        default:
            throw std::string ("xmlrpc data type not supported");
            break;
    }
    return ss.str();
}

uint32_t O3d3xxCameraOperations::xmlrpcCommandImplementation (const std::string &url, const std::string &cmd, const std::string &format,
        const std::vector<std::string> &params, std::string &res)
{
    if (url.empty())
    {
        res = "empty url";
        RETURN_ERROR (1);
    }

    xmlrpc_value *response = 0;
    xmlrpc_env env;
    memset (&env, 0, sizeof (env));
    switch (params.size())
    {
        case 0:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str());
            break;
        case 1:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str());
            break;
        case 2:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str());
            break;
        case 3:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str());
            break;
        case 4:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str());
            break;
        case 5:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str());
            break;
        case 6:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str());
            break;
        case 7:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str());
            break;
        case 8:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str());
            break;
        case 9:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str(),
                                           params[8].c_str());
            break;
        case 10:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str(),
                                           params[8].c_str(), params[9].c_str());
            break;
        case 11:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str(),
                                           params[8].c_str(), params[9].c_str(), params[10].c_str());
            break;
        case 12:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str(),
                                           params[8].c_str(), params[9].c_str(), params[10].c_str(), params[11].c_str());
            break;
        case 13:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str(),
                                           params[8].c_str(), params[9].c_str(), params[10].c_str(), params[11].c_str(), params[12].c_str());
            break;
        case 14:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str(),
                                           params[8].c_str(), params[9].c_str(), params[10].c_str(), params[11].c_str(), params[12].c_str(),
                                           params[13].c_str());
            break;
        case 15:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str(),
                                           params[8].c_str(), params[9].c_str(), params[10].c_str(), params[11].c_str(), params[12].c_str(),
                                           params[13].c_str(), params[14].c_str());
            break;
        case 16:
            response = xmlrpc_client_call (& (env),
                                           url.c_str(), cmd.c_str(), format.c_str(), params[0].c_str(), params[1].c_str(), params[2].c_str(),
                                           params[3].c_str(), params[4].c_str(), params[5].c_str(), params[6].c_str(), params[7].c_str(),
                                           params[8].c_str(), params[9].c_str(), params[10].c_str(), params[11].c_str(), params[12].c_str(),
                                           params[13].c_str(), params[14].c_str(), params[15].c_str());
            break;
        default:
            res = "maximum number of parameters exceeded";
            RETURN_ERROR (1);
            break;
    }

    if (env.fault_occurred)
    {
        mp_d->fault_string = env.fault_string;

        int tempfaultCode = env.fault_code;
        xmlrpc_env_clean (&env);
        return env.fault_code;
    }

    try
    {
        res = extractFromXmlRpc (response, env);

    }
    catch (const std::string &exc)
    {
        mp_d->fault_string = exc;

        int tempfaultCode = env.fault_code;
        xmlrpc_env_clean (&env);
        return ERR_GENERIC;
    }

    if (env.fault_occurred)
    {
        mp_d->fault_string = env.fault_string;

        int tempfaultCode = env.fault_code;
        xmlrpc_env_clean (&env);
        return env.fault_code;
    }
    xmlrpc_DECREF (response);
    int tempfaultCode = env.fault_code;
    xmlrpc_env_clean (&env);

    RETURN_NOERROR;
}

uint32_t  O3d3xxCameraOperations::setTriggerMode (char *cmd)
{
    std::string res;
    std::vector<std::string> param ;
    std::vector<std::string> el = splitCommand (cmd);
    int ret;
    bool selfCreatedSession = false;

    if (atoi (el.at (1).c_str()) > 2)
    {
        return -1 ;
    }

    if (mp_d->sessionUrl == "")
    {
        switchCameraToConfigurationMode();
        selfCreatedSession = true;

        /*Setting application to edit mode */
        param.clear();
        res = "";
        param.push_back (to_string<int> (mp_d->applicationId));
        ret = xmlrpcCommandImplementation (mp_d->sessionUrl + "edit/", "editApplication", "(s)", param, res);
        if (ret != 0)
        {
            return ret;
        }
    }

    string app_url = "";

    /* Generating string representing application object's url */
    app_url = mp_d->sessionUrl + "edit/application/";
    mp_d->triggeredMode = atoi (el.at (1).c_str());

    param.clear();
    res = "";
    param.push_back ("TriggerMode");
    param.push_back (to_string ( (unsigned long long) (mp_d->triggeredMode)));
    /*setting the trigger mode */
    ret = xmlrpcCommandImplementation (app_url, "setParameter", "(ss)", param, res);
    if (ret != 0)
    {
        return ret;
    }

    param.clear();
    res = "";
    // saving the application
    ret = xmlrpcCommandImplementation (app_url, "save", "()", param, res);
    if (ret != 0)
    {
        return ret;
    }

    if (selfCreatedSession)
    {
        string edit_url = "";
        edit_url = mp_d->sessionUrl + "edit/";

        param.clear();
        res = "";
        ret = xmlrpcCommandImplementation (edit_url, "stopEditingApplication", "()", param, res);
        if (ret != 0)
        {
            return ret;
        }

        string edit_device = "";
        edit_device = mp_d->sessionUrl + "edit/device/";

        param.clear();
        res = "";
        param.push_back ("ActiveApplication");
        param.push_back (to_string ( (unsigned long long) (mp_d->applicationId)));
        ret = xmlrpcCommandImplementation (edit_device, "setParameter", "(ss)", param, res);
        if (ret != 0)
        {
            return ret;
        }

        exitFromCameraConfigurationMode();

        /* As we have ended session, set the session url variable to an empty string */
        mp_d->sessionUrl = "";
        if (!mp_d->isConnected)
        {
            connecttoPCIC();
        }
        //exitFromCameraConfigurationMode();
    }

    RETURN_NOERROR;
}

///@endcond
/* @} */
/* @} */