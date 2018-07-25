**************************************************************************************************************
O3D3xx Toolbox:
**************************************************************************************************************
O3D3xx Toolbox contains a driver and utilities for usage with O3D3xx cameras built and sold by IFM Efector.
The O3D cameras are time-of-flight 3D cameras based on the Photonic Mixer Device imager from PMD Technologies.
Currently, these include the O3D3xx series of cameras. This library is intended for providing hands on basic 
functinality of camera like connecting, configuring, grabing frames and disconnecting from camera.

To that end, the focus has been on providing following sample guide applications.
i.  LightVis - a GUI application, to visualize the camera output.
ii. Sample application - A C/C++ code, showing usage of basic functions of camera. 

**************************************************************************************************************
Supported Platforms:
**************************************************************************************************************
Only Windows OS 32 and 64 bit platforms are supported.

**************************************************************************************************************
Prerequisites:
**************************************************************************************************************
The following software have to be installed on the PC before you can build the library -
1) 	CMake (version 3.0.2 or higher from http://cmake.org/cmake/resources/software.html)

The following software is needed to build the Doxygen documentation -
1) 	Doxygen (version 1.8.7 or higher from http://www.stack.nl/~dimitri/doxygen/download.html)
2) 	Graphviz (version 2.38 or higher from http://www.graphviz.org/Download.php)
3) 	Miktex (version 2.9 or higher from http://miktex.org/download) - this is required only if the documentation
		in PDF is required.

NOTE: We have built and tested the code with the versions mentioned above. Higher versions 
should also work seamlessly, but this is not guranteed.

**************************************************************************************************************
Installation:
**************************************************************************************************************
Download the source code from ifm official site.
The source code folder shall contains code related files for "O3D3xx plugin development" project.
It mainly consists of O3D3xx source and processing plugin, along with dependent code and third party tools, test suites etc.

The folder structure is -->
Base Folder 
	--> processing_plugin - the files needed to build the processing plugin
	--> source_plugin - the files needed to build the source plugin
	--> samples - all the sample codes
	--> tests - all the unit test codes
	--> third_party - all the third party code
	
Once you have source code and want to know how to build the software, follow the steps given below -

Process for building the Source Code -
1) 	Open the CMake GUI (Start Menu -> All Programs -> CMake 2.8 -> CMake (cmake-gui)).
2) 	In the "Where is the Source code", point to the location of the source code.
3) 	In the "Where to build the binaries", point to the location to build the binaries.
	It is recommended that a separate folder called as "build" (this name can be changed)
	be created in a separate location on the PC for this purpose. If the build folder is 
	a subfolder of the code folder, then an error message is given to the user and no build
	is generated.
4)	Click on "Configure". Select the Visual Studio (VS) compiler version that you have installed on your PC.
	Please note the following mapping to be used -
	VS 2008 --> Visual Studio 9
	VS 2010 --> Visual Studio 10
	VS 2012 --> Visual Studio 11
	VS 2013 --> Visual Studio 12
5)	There are a couple of options in CMake which the user can select if required -
		O3D3XX_SAMPLE_CODES_BUILD_ALL - This option if selected (default ON),
			generates the sample code project.
		O3D3XX_UNIT_TESTS_BUILD_ALL - This option if selected (default ON),
			generates the unit test and Gtest projects.
		O3D3XX_DOXYGEN_DOCUMENTATION_GENERATE - This option if selected (default OFF),
			generates the doxygen documentation. For this option to work, please have
			a look at the software required for doxygen described in the section above.
		O3D3XX_DOXYGEN_DOCUMENTATION_CHM - This option if selected (default ON),
			generates the doxygen documentation in CHM format. For this option to work,
			please have a look at the software required for doxygen described in the section above.
		O3D3XX_DOXYGEN_DOCUMENTATION_PDF - This option if selected (default OFF),
			generates the doxygen documentation in PDF format. For this option to work,
			please have a look at the software required for doxygen described in the section above.
		O3D3XX_GENERATE_RELEASE_PACKAGE - This option if selected (default OFF),
			generates a project which builds the release package in the SOURCE_DIR/release folder.
			This is to be used for packaging the code and lightvis to be given for external customers.
			There are two zip files created - 
				LightVis_O3D3xx.zip - contains the Lightvis and all necessary files.
				pmdsdk2_o3d3xx_plugin_VERSION - contains the source code from VERSION to be given to the
					customers.
6)	Click on "Generate".
7)	This will build a VS solution in the "build" folder.
8)	Open the VS Solution in VS.
9)	You will see a list pf projects which are created as -
		ALL_BUILD - CMake project; do not touch
		ZERO_CHECK - CMake project; do not touch
		gtest - part of the gtest library
		gtest_main - part of the gtest library
		libxmlrpc - part of the open source XML-RPC library
		libxmlrpc_client - part of the open source XML-RPC library
		libxmlrpc_util - part of the open source XML-RPC library
		libxmlrpc_xmlparse - part of the open source XML-RPC library
		libxmlrpc_xmltok - part of the open source XML-RPC library
		O3D3xxCamera - Source plugin for the O3D3xx Camera
		O3D3xxProc - Processing plugin for the O3D3xx Camera
		SampleO3D3xxCamera - Sample code for using the O3D3xxCamera Class directly
10)	You are now ready to build all the projects.
11) After successful compilation of code, 
	i. 	If the doxygen documentation generation is selected, the technical reference document 
		named "O3D3xx_Plugin.chm" and / or "O3D3xx_Plugin.pdf" shall be available at, "build/docs".
	ii. The generated binaries shall be available at, "build/bin/<configuration_folder>".
		<configuration_folder> denotes the configuration by which you build the solution eg: Debug, Release etc.


**************************************************************************************************************
Usage:
**************************************************************************************************************
______________________________________________________________________________________________________________
Sample C code - shows usage of class functions via C code.
______________________________________________________________________________________________________________
User application can directly access O3D3xx plugin class functions to talk to camera. A sample C/C++ code
is given to demonstrate usage of O3D3xx plugin class functions.
Steps to use sample code:
	1)If your camera ip is 192.168.0.69, xmlrpc port is 80 and pcic port is 50010, then directly go to step 4.
	2)In solution window, open file "o3d3xx_camera_sample_usage_code.cpp" listed under "SampleO3D3xxCamera" project.
	3)Steps to change default camera ip address, xmlrpc port number and pcic port number
		a)If your camera ip address is not 192.168.0.69,
		find following line of code,
		#define IP_ADDRESS_OF_CAMERA "192.168.0.69"
		Replace 192.168.0.69 with your camera's ip address.

		b) If your xmlrpc port number is not 80, then
		Find following line of code,
		#define XMLRPC_PORT 80
		Replace "80" with your xmlrpc port number.

		c)If your pcic port number is not 50010, then
		Find following line of code,
		#define PCIC_PORT 50010
		Replace "50010" with your pcic port number.
		Save the solution.
 	4) Compile the solution.
	5) After successfull compilation of project, the executable for sample C code shall be generated by name 
	"SampleO3D3xxCamera.exe" at "build/bin/<configuration_folder>".<configuration_folder> denotes the configuration
	by which you build the solution. It could be Debug, Release etc.
	Double click the executable. It shall demonstrate configuration of camera and grabbing of frames and image data.

______________________________________________________________________________________________________________
LightVis - shows usage of O3D3xx plugin via third party application.
______________________________________________________________________________________________________________
Steps to use LightVis to communicate with camera:
	1)Compile the solution.
	2)After, successfull compilation of project, open "build/bin/<configuration_folder>" folder.
	<configuration_folder> denotes the configuration by which you built the solution. 
	It could be Debug, Release etc.
	3)If your camera ip is 192.168.0.69, xmlrpc port is 80 and pcic port is 50010, then directly go to step 5.
	4)The "build/bin/<configuration_folder>" folder contains "lv.cfg" file.
	Open "lv.cfg" file in editor. This is configuration file of LightVis. It gives the details of camera ip address,
	xmlrpc port, pcic port, source plugin binary name and processing plugin binary name to LightVis.
	
	Find following line in "lv.cfg" file,
	192.168.0.69:80:50010
	
	a)If your camera ip address is not 192.168.0.69,
	Replace 192.168.0.69 with your camera's ip address.
	
	b) If your xmlrpc port number is not 80, then
	Replace "80" with your xmlrpc port number.

	c)If your pcic port number is not 50010, then
	Replace "50010" with your pcic port number.
	Save the file.
	
	Suppose your camera ip addresss is 192.168.0.1, xmlrpc port is "8080" and pcic port is "2000", then the "lv.cfg"
	file should have
	192.168.0.1:8080:2000
	5)Double click on "LightVis.exe". This is the LightVis executable.
	6)LightVis shall show images captured from camera.
	7)Also integration time can be configured via LightVis.
**************************************************************************************************************

**************************************************************************************************************
Generating Unit Test Report in XML Format:
**************************************************************************************************************
Motivation:
	GTest code allows us the generate the Test Report in XML format. Once we have this in XML format, we can
	import it into any tool like Polarion. Also, this allows automated parsing of the results for tools like
	Jenkins Build Server.
	
Steps:
	1)	The GTest executable created (O3D3xxTests.exe) when executed, runs the unit tests and prints the results
		on the console.
	2)	To generate the report in XML file, the following command is needed to be executed on the command prompt -
			O3D3xxTests.exe --gtest_output="xml:PATH_TO_XML/XML_FILE_NAME.xml"
		where, PATH_TO_XML = the path to generate the XML file
		XML_FILE_NAME = the name of the XML file to be generated
	3)	Once the user runs this, then the file is generated in JUnit XML format which can be uploaded to tolls like
		Polarion.
**************************************************************************************************************