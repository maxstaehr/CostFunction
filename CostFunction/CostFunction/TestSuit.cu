#include "TestSuit.h"
#include "mathcuda.h"
//#include "allKernelFct.cuh"
#include "CudaMem.h"



TestSuit::TestSuit() {
	// TODO Auto-generated destructor stub
}

TestSuit::~TestSuit() {
	// TODO Auto-generated destructor stub
}

void TestSuit::testCudaFunctions()
{
	float h1[16], h2[16], r1[16], r2[16];
	for(unsigned int i=0; i<16; i++)
	{
		h1[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
		h2[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	}

	mm16(h1, h2, r1);

	float *d_h1, *d_h2, *d_r2;
	CudaMem::cudaMemAllocReport((void**)&d_h1, 16*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&d_h2, 16*sizeof(float));
	CudaMem::cudaMemAllocReport((void**)&d_r2, 16*sizeof(float));

	CudaMem::cudaMemCpyReport(d_h1, h1, 16*sizeof(float), cudaMemcpyHostToDevice);
	CudaMem::cudaMemCpyReport(d_h2, h2, 16*sizeof(float), cudaMemcpyHostToDevice);

	cudaError_t cudaStatus;
	//cuda_calc::testMM16<<<1, 16>>>(d_h1, d_h2, d_r2);
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "test kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));
	}

	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "cudaDeviceSynchronize returned error code %d after launching test kernel!\n", cudaStatus);

	}

	CudaMem::cudaMemCpyReport(r2, d_r2, 16*sizeof(float), cudaMemcpyDeviceToHost);


	
	

}

void TestSuit::testCameraParameters(struct CAM* cam)
{


}