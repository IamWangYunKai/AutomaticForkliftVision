/* A simple synchronous XML-RPC client program written in C, as an example of
   an Xmlrpc-c client.  This invokes the sample.add procedure that the
   Xmlrpc-c example xmlrpc_sample_add_server.c server provides.  I.e. it adds
   two numbers together, the hard way.

   This sends the RPC to the server running on the local system ("localhost"),
   HTTP Port 8080.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <xmlrpc-c/base.h>
#include <xmlrpc-c/client.h>

#include "config.h"  /* information about this build environment */

#define NAME "Xmlrpc-c Test Client"
#define VERSION "1.0"

static void 
dieIfFaultOccurred (xmlrpc_env * const envP) {
    if (envP->fault_occurred) {
        fprintf(stderr, "ERROR: %s (%d)\n",
                envP->fault_string, envP->fault_code);
		printf(stdout,"ERROR: %s (%d)\n",
			envP->fault_string, envP->fault_code);
		getch();
        exit(1);
    }
}



int 
main(int           const argc, 
     const char ** const argv) {

    xmlrpc_env env;
    xmlrpc_value * resultP;
    xmlrpc_int32 sum;

	char url[200]=""; ///While accessing objects on xmlrpc server, we require 
	 //form a url of that object. This variable holds that object specific url

	char *result = (char *)malloc(50);
	int operating_mode = -1; //specifies operating mode of camera

	/*** Connection related settings ***/
	//url specifying address of xmlrpc server
	// const char * const serverUrl = "http://127.0.0.1:3000/RPC2";
	  //const char * const serverUrl = "http://172.29.169.133:3000/RPC2";
	//For connecting to O3D3xx camera
	 const char * const serverUrl = "http://192.168.0.26:80/api/rpc/v1/com.ifm.efector/";
   
	 
	 
	
    if (argc-1 > 0) {
        fprintf(stderr, "This program has no arguments\n");
        exit(1);
    }

    /* Initialize our error-handling environment. */
    xmlrpc_env_init(&env);

    /* Start up our XML-RPC client library. */
    xmlrpc_client_init2(&env, XMLRPC_CLIENT_NO_FLAGS, NAME, VERSION, NULL, 0);
    dieIfFaultOccurred(&env);

	///******* Getting a parameter's value (Name of sensor) *********/
    //printf("Making XMLRPC call to server url '%s' method '%s' "
      //     "to request the result parameter\n", serverUrl, "getParameter");

 //   /* Make the synchronous remote procedure call */
 //  resultP = xmlrpc_client_call(&env, serverUrl, "getParameter",
 //                                "(s)","Name");
 //   
	/////** Parse the result **/
	//dieIfFaultOccurred(&env);
	//xmlrpc_read_string(&env, resultP, &result);
	//	/* Printing the result value */
	//printf("The result is %s\n", result);
	//getch();

 //   resultP = xmlrpc_client_call(&env, serverUrl, "getParameter",
 //                                "(s)","OperatingMode");
	//	///** Parse the result **/
	//dieIfFaultOccurred(&env);
	//xmlrpc_read_string(&env, resultP, &result);
	//	/* Printing the result value */
	//printf("The operating modet is %s\n", result);
	//getch();
 

	
#if 1
	///******* Setting a parameter's value ( Name of sensor) *********/
      /* Requesting a new session  */
	//strcpy(result,"00000000000000000000000000000000");
    resultP = xmlrpc_client_call(&env, serverUrl, "requestSession",
		"(ss)","","");
			///** Parse the result **/
	dieIfFaultOccurred(&env);
	xmlrpc_read_string(&env, resultP, &result);
		/* Printing the result value */
	printf("The session id  is %s\n", result);
	getch();

	///Generating stirng representing session object's url
    strcpy(url,serverUrl);
	strcat(url,"session_");
	strcat(url,result); //concatenating the session id
	strcat(url,"/");
	
	printf("session url %s\n",url);

	///Setting operating mode to "edit" mode 
	resultP = xmlrpc_client_call(&env, url, "setOperatingMode",
		 "(i)",(xmlrpc_int)1);
 #endif
	resultP = xmlrpc_client_call(&env, serverUrl, "getParameter",
                                 "(s)","OperatingMode");
		///** Parse the result **/
	dieIfFaultOccurred(&env);
	xmlrpc_read_string(&env, resultP, &result);
		/* Printing the result value */
	printf("The operating modet is %s\n", result);
	getch();
	    

	/// Changing parameters of camera
	//strcat(url,"edit/device/");
	//printf("device url %s,-->%s\n",url,serverUrl);
 //  xmlrpc_client_call(&env, url, "setParameter",
 //                                "(ss)","Name","DefaultConfig");

 //   dieIfFaultOccurred(&env);
 //    resultP = xmlrpc_client_call(&env, url, "getParameter",
 //                                "(s)","Name");
 //  
	// dieIfFaultOccurred(&env);
	/////** Parse the result **/
	//xmlrpc_read_string(&env, resultP, &result);
 //   
 //
	///* Printing the result value */
	//printf("The new name is %s\n", result);
	//getch();

	/*** Disconnect from camera ***/
    /* Dispose of our result value. */
    xmlrpc_DECREF(resultP);

    /* Clean up our error-handling environment. */
    xmlrpc_env_clean(&env);
    
    /* Shutdown our XML-RPC client library. */
    xmlrpc_client_cleanup();

    return 0;
}

