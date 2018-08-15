/* A simple asynchronous XML-RPC client program written in C, as an example of
   Xmlrpc-c asynchronous RPC facilities.  This is the same as the 
   simpler synchronous client xmlprc_sample_add_client.c, except that
   it adds 3 different pairs of numbers with the summation RPCs going on
   simultaneously.

   Use this with xmlrpc_sample_add_server.  Note that that server
   intentionally takes extra time to add 1 to anything, so you can see
   our 5+1 RPC finish after our 5+0 and 5+2 RPCs.
*/

#include <stdlib.h>
#include <stdio.h>
#include <conio.h>

#include <xmlrpc-c/base.h>
#include <xmlrpc-c/client.h>

#include "config.h"  /* information about this build environment */



#define NAME "Xmlrpc-c Asynchronous Test Client"
#define VERSION "1.0"

static void 
die_if_fault_occurred(xmlrpc_env * const envP) {
    if (envP->fault_occurred) {
        fprintf(stderr, "Something failed. %s (XML-RPC fault code %d)\n",
                envP->fault_string, envP->fault_code);
        exit(1);
    }
}



static void 
handle_sample_add_response(const char *   const serverUrl,
                           const char *   const methodName,
                           xmlrpc_value * const paramArrayP,
                           void *         const user_data,
                           xmlrpc_env *   const faultP,
                           xmlrpc_value * const resultP) {

							   xmlrpc_env env;
							   xmlrpc_int addend, adder;

							   /* Initialize our error environment variable */
							   xmlrpc_env_init(&env);

							   /* Our first four arguments provide helpful context. Let's grab the
							   addends from our parameter array.
							   */
							   xmlrpc_decompose_value(&env, paramArrayP, "(ii)", &addend, &adder);
							   die_if_fault_occurred(&env);

							   printf("RPC with method '%s' at URL '%s' to add %d and %d "
								   "has completed\n", methodName, serverUrl, addend, adder);

							   if (faultP->fault_occurred)
								   printf("The RPC failed. %s\n", faultP->fault_string);
							   else {
								   xmlrpc_int sum;

								   xmlrpc_read_int(&env, resultP, &sum);
								   die_if_fault_occurred(&env);

								   printf("The sum is %d\n", sum);
							   }
}

static void 
handle_getParameter_response(const char *   const serverUrl,
                           const char *   const methodName,
                           xmlrpc_value * const paramArrayP,
                           void *         const user_data,
                           xmlrpc_env *   const faultP,
                           xmlrpc_value * const resultP) {

							   const char *stringValueP;
							   xmlrpc_env env;
							   xmlrpc_int addend;// adder;

							   xmlrpc_value * firstElementP;
							   char *ssss;

							   ssss = (char *)malloc(50);
							   /* Initialize our error environment variable */
							   xmlrpc_env_init(&env);

							   /* Our first four arguments provide helpful context. Let's grab the
							   addends from our parameter array.
							   */

							   printf("DEcomposing the result got from server\n");
							   //xmlrpc_decompose_value(&env, paramArrayP, "(s)", &addend);
							   
							    xmlrpc_array_read_item(&env,  paramArrayP, 0, &firstElementP);
							   die_if_fault_occurred(&env);
							   
							  //  memcpy(ssss,&((firstElementP->_block)._block),4);
							    
									xmlrpc_read_string(&env, firstElementP, &ssss);

							   printf("RPC with method '%s' at URL '%s' to add %s "
								   "has completed\n", methodName, serverUrl, ssss);

							   if (faultP->fault_occurred)
								   printf("The RPC failed. %s\n", faultP->fault_string);
							   else {
								   xmlrpc_int sum;

								  // xmlrpc_read_int(&env, resultP, &sum);
								  xmlrpc_read_string(&env, resultP, &ssss);
								   die_if_fault_occurred(&env);

								   printf("The name is %s\n", ssss);
							   }
}
                                                                                                                                                                                                                                                                                                                                                                                                     
int 
main(int           const argc, 
     const char ** const argv) {

    //const char * const serverUrl = "http://localhost:8080/RPC2";
		//const char * const methodName = "sample.add";

		 //For local host server
//   const char * const serverUrl = "http://127.0.0.1:3000/RPC2";
		 ///For Server on intranet
//		 const char * const serverUrl = "http://inpu7d0007:3000/RPC2";
	//const char * const serverUrl = "http://192.168.0.26:80/api/rpc/v1/com.ifm.efector/";
		 const char * const serverUrl = "http://192.168.0.26/api/rpc/v1/com.ifm.efector/";
		
		 const char * const methodName = "getParameter";
	//static char paramName[50] = "Name";
		// xmlrpc_value * p = paramName;
		  const char * p;

    xmlrpc_env env;
    xmlrpc_client * clientP;
	const char APIparamName[50] = "Name";
	//  xmlrpc_value *str ="Name";

	unsigned int major, minor, point;

	xmlrpc_value * paramName; // name of parameter to be passed to method call
	
	//xmlrpc_int32 temp_Var = 12321;



	
#if 1

	xmlrpc_version(&major, &minor, &point);

	printf("libxmlrpc version %u.%u.%u\n", major, minor, point);
	if (argc-1 > 0) {
        fprintf(stderr, "This program has no arguments\n");
        exit(1);
    }

    /* Initialize our error environment variable */
    xmlrpc_env_init(&env);

    /* Required before any use of Xmlrpc-c client library: */
    xmlrpc_client_setup_global_const(&env);
    die_if_fault_occurred(&env);

    xmlrpc_client_create(&env, XMLRPC_CLIENT_NO_FLAGS, NAME, VERSION, NULL, 0,
                         &clientP);
    die_if_fault_occurred(&env);



	

	//paramName = xmlrpc_build_value(&env,"s",APIparamName);

	//xmlrpc_read_string(&env,paramName,&p);

	///* request the remote procedure call */
	//xmlrpc_client_start_rpcf(&env, clientP, serverUrl, methodName,
	//	                              handle_getParameter_response, NULL,
	//	                              "(s)", paramName);

	//paramName = xmlrpc_build_value(&env,"i",temp_Var);
	

/*	paramName = xmlrpc_string_new(&env, "Name");
xmlrpc_client_start_rpcf(&env, clientP, serverUrl, methodName,
		handle_getParameter_response, NULL,
		"(s)",paramName);
		*/
	paramName = xmlrpc_int_new(&env,(xmlrpc_int32)5678);
	
/*	xmlrpc_client_start_rpcf(&env, clientP, serverUrl, methodName,
		handle_getParameter_response, NULL,
		"(i)",5678);*/

	xmlrpc_client_start_rpcf(&env, clientP, serverUrl, methodName,
		handle_getParameter_response, NULL,
		"(s)", "Name");


	die_if_fault_occurred(&env);
	//getch();

    //for (adder = 50; adder < 53; ++adder) {
    //    printf("Making XMLRPC call to server url '%s' method '%s' "
    //           "to request the sum "
    //           "of 5 and %d...\n", serverUrl, methodName, adder);

    //    /* request the remote procedure call */
    //    xmlrpc_client_start_rpcf(&env, clientP, serverUrl, methodName,
    //                              handle_sample_add_response, NULL,
    //                              "(ii)", (xmlrpc_int32) 5, adder);
    //    die_if_fault_occurred(&env);
    //}
    
    printf("RPCs all requested.  Waiting for & handling responses...\n");

    /* Wait for all RPCs to be done.  With some transports, this is also
       what causes them to go.
    */
    xmlrpc_client_event_loop_finish(clientP);

    printf("All RPCs finished.\n");

    xmlrpc_client_destroy(clientP);
    xmlrpc_client_teardown_global_const();
	
#endif
   getch();
	return 0;
}
