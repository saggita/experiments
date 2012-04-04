#include "../basic_initialize/btOpenCLUtils.h"


/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///original author: Erwin Coumans


#include <stdio.h>

cl_context			g_cxMainContext;
cl_command_queue	g_cqCommandQue;



int main(int argc, char* argv[])
{
	int ciErrNum = 0;
	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
	const char* vendorSDK = btOpenCLUtils_getSdkVendorName();
	int numPlatforms,i;
	void* glCtx=0;
	void* glDC = 0;
	int numDev =0;

	printf("This program was compiled using the %s OpenCL SDK\n",vendorSDK);
	
	numPlatforms = btOpenCLUtils_getNumPlatforms(&ciErrNum);
	printf("Num Platforms = %d\n", numPlatforms);

	

	for (i=0;i<numPlatforms;i++)
	{
		cl_platform_id platform = btOpenCLUtils_getPlatform(i,&ciErrNum);
		int numDevices,j;
		cl_context context;
		printf("================================\n");
		printf("Platform %d:\n", i);
		printf("================================\n");
		btOpenCLUtils_printPlatformInfo(platform);

		context = btOpenCLUtils_createContextFromPlatform(platform,deviceType,&ciErrNum,0,0,-1,-1);
		
		numDevices = btOpenCLUtils_getNumDevices(context);
		printf("Num Devices = %d\n", numDevices);
		for (j=0;j<numDevices;j++)
		{
			cl_device_id dev = btOpenCLUtils_getDevice(context,j);
			printf("--------------------------------\n");
			printf("Device %d:\n", j);
			printf("--------------------------------\n");
			btOpenCLUtils_printDeviceInfo(dev);
		}

		clReleaseContext(context);
	}

	///Easier method to initialize OpenCL using createContextFromType for a GPU
	deviceType = CL_DEVICE_TYPE_GPU;
	

	printf("Initialize OpenCL using btOpenCLUtils_createContextFromType for CL_DEVICE_TYPE_GPU\n");
	g_cxMainContext = btOpenCLUtils_createContextFromType(deviceType, &ciErrNum, glCtx, glDC,-1,-1);
	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	numDev = btOpenCLUtils_getNumDevices(g_cxMainContext);

	for (i=0;i<numDev;i++)
	{
		cl_device_id		device;
		device = btOpenCLUtils_getDevice(g_cxMainContext,i);
		btOpenCLUtils_printDeviceInfo(device);
		// create a command-queue
		g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, device, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
		//normally you would create and execute kernels using this command queue

		clReleaseCommandQueue(g_cqCommandQue);
	}

	clReleaseContext(g_cxMainContext);
		
	return 0;
}
