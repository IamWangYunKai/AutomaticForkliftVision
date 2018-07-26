#include <iostream>
#include <vector>
#include <pmdsdk2.h>
#include <fstream>

// on error, prepend absolute path to files before plugin names
#define SOURCE_PLUGIN "O3D3xxCamera.W32.pap"
#define SOURCE_PARAM "169.254.130.223:80:50010" // Ip Address : port Number : XmlRPC port
#define PROC_PLUGIN "O3D3xxProc.W32.ppp"
#define PROC_PARAM ""

#define EXPOSURE 2000

using namespace std;

int main(void)
{
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

	ofstream outfile("./data.txt");
	if (!outfile) {
		cout << "Unable to open otfile" << endl;;
		exit(1); // terminate with error
	}

	printf("\n ==============================O3D300 Sample Code============================");
	printf("\n Connecting to camera : \n");
	// connect to camera
	res = pmdOpen(&hnd, SOURCE_PLUGIN, SOURCE_PARAM, PROC_PLUGIN, PROC_PARAM);

	if (res != PMD_OK)
	{
		fprintf(stderr, "Could not connect: \n");
		getchar();
		return 0;
	}
	printf("\n DONE");


	printf("\n -----------------------------------------------------------------------------");
	printf("\n Updating the framedata ");
	res = pmdUpdate(hnd); // to update the camera parameter and framedata
	if (res != PMD_OK)
	{
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
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get data description: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}
	printf("\n DONE");

	imgWidth = dd.img.numColumns;
	imgHeight = dd.img.numRows;
	/*******************************************************************************/
	outfile << "imgWidth:" << imgWidth << endl;
	outfile << "imgHeight:" << imgHeight << endl;
	/*******************************************************************************/

	printf("\n -----------------------------------------------------------------------------");

	dist.resize(imgWidth * imgHeight);
	amp.resize(imgWidth * imgHeight);
	flags.resize(imgWidth * imgHeight);
	xyz3Dcoordinate.resize(imgHeight * imgWidth * 3); // value three is for 3 images

	printf("\n Obtaining different image data from camera viz amplitude and Distance Image  \n");

	res = pmdGetDistances(hnd, &dist[0], dist.size() * sizeof(float));
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get distance data: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}
	printf("\n Middle pixel Distance value: %fm\n", dist[(imgHeight / 2) * imgWidth + imgWidth / 2]);

	res = pmdGetAmplitudes(hnd, &amp[0], amp.size() * sizeof(float));
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get amplitude data: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}
	printf("\n Middle Amplitude: %f\n", amp[(dd.img.numRows / 2) * dd.img.numColumns + dd.img.numColumns / 2]);

	res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get flag data: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}

	/* 3D coordinates are stored in interleved way .i.e.
	for every pixel i
	xcordinate value = xyz3Dcoordinate[i+0]
	ycordinate value = xyz3Dcoordinate[i+1]
	zcordinate value = xyz3Dcoordinate[i+2]*/

	pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not get xyz data: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}

	//calculating the mean values of X/Y/Z/ Image
	float XSum = 0, YSum = 0, ZSum = 0;
	float ASum = 0, DSum = 0;
	unsigned counter = 0;
	for (int i = 0; i < imgHeight * imgWidth; i++)
	{
		if (!(flags[i] & 1)) // first bit is set to 1,if pixel is invalid
		{
			DSum += dist[i];
			ASum += amp[i];
			XSum += xyz3Dcoordinate[i * 3 + 0];
			YSum += xyz3Dcoordinate[i * 3 + 1];
			ZSum += xyz3Dcoordinate[i * 3 + 2];
			counter++;
		}
	}

	float DMean = counter ? (DSum / float(counter)) : 0.f;
	float AMean = counter ? (ASum / float(counter)) : 0.f;
	float XMean = counter ? (XSum / float(counter)) : 0.f;
	float YMean = counter ? (YSum / float(counter)) : 0.f;
	float ZMean = counter ? (ZSum / float(counter)) : 0.f;

	printf(" DMean = %fm \n AMean = %f \n", DMean, AMean);
	printf(" XMean = %fm \n YMean = %fm \n ZMean = %fm ", XMean, YMean, ZMean);

	printf("\n DONE");

	/*******************************************************************************/
	outfile << "dist | amp | x | y | z |" << endl;
	for (int i = 0; i < imgHeight * imgWidth; i++) {
		if (!(flags[i] & 1)) { // first bit is set to 1,if pixel is invalid
			outfile << dist[i] << " ";
			outfile << amp[i] << " ";
			outfile << xyz3Dcoordinate[i * 3 + 0] << " ";
			outfile << xyz3Dcoordinate[i * 3 + 1] << " ";
			outfile << xyz3Dcoordinate[i * 3 + 2] << endl;;
		}
	}

	outfile.close(); // close file
					 /*******************************************************************************/
	printf("\n -----------------------------------------------------------------------------");
	printf("\n Setting and getting parameters viz : integrationTime \n");
	res = pmdSetIntegrationTime(hnd, 0, EXPOSURE);
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not set the integration time: %s\n", err);
		pmdClose(hnd);
		getchar();
		return 0;
	}

	res = pmdGetIntegrationTime(hnd, &integrationTime, 0);
	if (res != PMD_OK)
	{
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
	if (res != PMD_OK)
	{
		pmdGetLastError(hnd, err, 128);
		fprintf(stderr, "Could not close the connection %s\n", err);

		return 0;
	}

	printf("\n Camera closed Successfully");
	printf("\n ================================================================================");
	getchar();
}