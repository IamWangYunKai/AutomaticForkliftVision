#include <iostream>
#include <vector>
#include <pmdsdk2.h>
#include <fstream>
#include <sstream>

// on error, prepend absolute path to files before plugin names
#define SOURCE_PLUGIN "O3D3xxCamera.W32.pap"
#define SOURCE_PARAM "169.254.130.223:80:50010" // Ip Address : port Number : XmlRPC port
#define PROC_PLUGIN "O3D3xxProc.W32.ppp"
#define PROC_PARAM ""

#define EXPOSURE 4000
#define WRITE_DATA true

using namespace std;

int main(void){
	PMDHandle hnd; // connection handle
	int res;
	PMDDataDescription dd;
	unsigned int integrationTime;
	vector<float> amp;
	vector<float> dist;
	vector<unsigned> flags;
	vector<float> xyz3Dcoordinate;
	int imgHeight;
	int imgWidth;
	char err[256];

	printf("\n ==============================O3D300 Sample Code============================");
	printf("\n Connecting to camera : \n");
	// connect to camera
	res = pmdOpen(&hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM);

	if (res != PMD_OK){
		fprintf(stderr, "Could not connect: \n");
		getchar();
		return 0;
	}
	printf("\n DONE");
	printf("\n -----------------------------------------------------------------------------");
	printf("\n Updating the framedata ");
	res = pmdUpdate(hnd); // to update the camera parameter and framedata
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 256);
		fprintf(stderr, "Could not updateData: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}
	printf("\n DONE");
	printf("\n -----------------------------------------------------------------------------");
	printf("\n Retrieving source data description\n");
	res = pmdGetSourceDataDescription(hnd, &dd);
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get data description: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}
	printf("\n DONE");

	imgWidth = dd.img.numColumns;
	imgHeight = dd.img.numRows;
	printf("\n -----------------------------------------------------------------------------");
	dist.resize(imgWidth * imgHeight);
	amp.resize(imgWidth * imgHeight);
	flags.resize(imgWidth * imgHeight);
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3); // value three is for 3 images
	printf("\n Obtaining different image data from camera viz amplitude and Distance Image  \n");

	res = pmdGetDistances(hnd, &dist[0], dist.size() * sizeof(float));
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get distance data: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}
	printf("\n Middle pixel Distance value: %fm\n", dist[(imgHeight / 2) * imgWidth + imgWidth / 2]);

	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get amplitude data: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}
	printf("\n Middle Amplitude: %f\n", amp[(dd.img.numRows / 2) * dd.img.numColumns + dd.img.numColumns / 2]);

	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get flag data: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}

	res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get xyz data: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}

	if (WRITE_DATA) {
		string filename = to_string(rand()) + ".pcd";
		ofstream outfile(filename);
		if (!outfile) {
			cout << "Unable to open otfile" << endl;;
			exit(1); // terminate with error
		}
		outfile << "# .PCD v0.7 - Point Cloud Data file format" << endl;
		outfile << "VERSION 0.7" << endl;
		outfile << "FIELDS x y z" << endl;
		outfile << "SIZE 4 4 4" << endl;
		outfile << "TYPE F F F" << endl;
		outfile << "COUNT 1 1 1" << endl;
		outfile << "WIDTH 16782" << endl;
		outfile << "HEIGHT 1" << endl;
		outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
		outfile << "POINTS 16782" << endl;
		outfile << "DATA ascii" << endl;
		for (int i = 0; i < imgHeight * imgWidth; i++) {
			if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
				outfile << xyz3Dcoordinate[i * 3 + 0] << " ";
				outfile << xyz3Dcoordinate[i * 3 + 1] << " ";
				outfile << xyz3Dcoordinate[i * 3 + 2] << endl;
			}
		}

		outfile.close(); // close file
	}
	/*******************************************************************************/
	printf("\n -----------------------------------------------------------------------------");
	printf("\n Setting and getting parameters viz : integrationTime \n");
	res = pmdSetIntegrationTime(hnd, 0, EXPOSURE);
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not set the integration time: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}

	res = pmdGetIntegrationTime(hnd, &integrationTime, 0);
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not set the integration time:%s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}
	printf("\n integrationTime: %d\n", integrationTime);
	printf("\n DONE");
	printf("\n -----------------------------------------------------------------------------");
	printf("\n Closing the connection \n");
	res = pmdClose(hnd);
	if (res != PMD_OK){
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not close the connection %s\n", err);

		return 0;
	}

	printf("\n Camera closed Successfully");
	printf("\n ================================================================================");
	getchar();
}