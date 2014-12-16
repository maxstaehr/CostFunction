#ifndef ALLKERNELFCT2_CUH_
#define ALLKERNELFCT2_CUH_

#include "global.h"

namespace cuda_calc2{

	__device__ void mm16_device(const float *a, const float *b, float *c)
	{
		c[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12];
		c[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13];
		c[2] = a[0]*b[2] + a[1]*b[6] + a[2]*b[10] + a[3]*b[14];
		c[3] = a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3]*b[15];

		c[4] = a[4]*b[0] + a[5]*b[4] + a[6]*b[8] + a[7]*b[12];
		c[5] = a[4]*b[1] + a[5]*b[5] + a[6]*b[9] + a[7]*b[13];
		c[6] = a[4]*b[2] + a[5]*b[6] + a[6]*b[10] + a[7]*b[14];
		c[7] = a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7]*b[15];

		c[8] = a[8]*b[0] + a[9]*b[4] + a[10]*b[8] + a[11]*b[12];
		c[9] = a[8]*b[1] + a[9]*b[5] + a[10]*b[9] + a[11]*b[13];
		c[10] = a[8]*b[2] + a[9]*b[6] + a[10]*b[10] + a[11]*b[14];
		c[11] = a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11]*b[15];

		c[12] = a[12]*b[0] + a[13]*b[4] + a[14]*b[8] + a[15]*b[12];
		c[13] = a[12]*b[1] + a[13]*b[5] + a[14]*b[9] + a[15]*b[13];
		c[14] = a[12]*b[2] + a[13]*b[6] + a[14]*b[10] + a[15]*b[14];
		c[15] = a[12]*b[3] + a[13]*b[7] + a[14]*b[11] + a[15]*b[15];

	}


	__device__ void invert4_device(const float const *mat, float *dst)
	{
		float tmp[12]; /* temp array for pairs */
		float src[16]; /* array of transpose source matrix */
		float det; /* determinant */
		/* transpose matrix */
		for (int i = 0; i < 4; i++) {
			src[i] = mat[i*4];
			src[i + 4] = mat[i*4 + 1];
			src[i + 8] = mat[i*4 + 2];
			src[i + 12] = mat[i*4 + 3];
		}
		/* calculate pairs for first 8 elements (cofactors) */
		tmp[0] = src[10] * src[15];
		tmp[1] = src[11] * src[14];
		tmp[2] = src[9] * src[15];
		tmp[3] = src[11] * src[13];
		tmp[4] = src[9] * src[14];
		tmp[5] = src[10] * src[13];
		tmp[6] = src[8] * src[15];
		tmp[7] = src[11] * src[12];
		tmp[8] = src[8] * src[14];
		tmp[9] = src[10] * src[12];
		tmp[10] = src[8] * src[13];
		tmp[11] = src[9] * src[12];
		/* calculate first 8 elements (cofactors) */
		dst[0] = tmp[0]*src[5] + tmp[3]*src[6] + tmp[4]*src[7];
		dst[0] -= tmp[1]*src[5] + tmp[2]*src[6] + tmp[5]*src[7];
		dst[1] = tmp[1]*src[4] + tmp[6]*src[6] + tmp[9]*src[7];
		dst[1] -= tmp[0]*src[4] + tmp[7]*src[6] + tmp[8]*src[7];
		dst[2] = tmp[2]*src[4] + tmp[7]*src[5] + tmp[10]*src[7];
		dst[2] -= tmp[3]*src[4] + tmp[6]*src[5] + tmp[11]*src[7];
		dst[3] = tmp[5]*src[4] + tmp[8]*src[5] + tmp[11]*src[6];
		dst[3] -= tmp[4]*src[4] + tmp[9]*src[5] + tmp[10]*src[6];
		dst[4] = tmp[1]*src[1] + tmp[2]*src[2] + tmp[5]*src[3];
		dst[4] -= tmp[0]*src[1] + tmp[3]*src[2] + tmp[4]*src[3];
		dst[5] = tmp[0]*src[0] + tmp[7]*src[2] + tmp[8]*src[3];
		dst[5] -= tmp[1]*src[0] + tmp[6]*src[2] + tmp[9]*src[3];
		dst[6] = tmp[3]*src[0] + tmp[6]*src[1] + tmp[11]*src[3];
		dst[6] -= tmp[2]*src[0] + tmp[7]*src[1] + tmp[10]*src[3];
		dst[7] = tmp[4]*src[0] + tmp[9]*src[1] + tmp[10]*src[2];
		dst[7] -= tmp[5]*src[0] + tmp[8]*src[1] + tmp[11]*src[2];
		/* calculate pairs for second 8 elements (cofactors) */
		tmp[0] = src[2]*src[7];
		tmp[1] = src[3]*src[6];
		tmp[2] = src[1]*src[7];
		tmp[3] = src[3]*src[5];
		tmp[4] = src[1]*src[6];
		tmp[5] = src[2]*src[5];

		tmp[6] = src[0]*src[7];
		tmp[7] = src[3]*src[4];
		tmp[8] = src[0]*src[6];
		tmp[9] = src[2]*src[4];
		tmp[10] = src[0]*src[5];
		tmp[11] = src[1]*src[4];
		/* calculate second 8 elements (cofactors) */
		dst[8] = tmp[0]*src[13] + tmp[3]*src[14] + tmp[4]*src[15];
		dst[8] -= tmp[1]*src[13] + tmp[2]*src[14] + tmp[5]*src[15];
		dst[9] = tmp[1]*src[12] + tmp[6]*src[14] + tmp[9]*src[15];
		dst[9] -= tmp[0]*src[12] + tmp[7]*src[14] + tmp[8]*src[15];
		dst[10] = tmp[2]*src[12] + tmp[7]*src[13] + tmp[10]*src[15];
		dst[10]-= tmp[3]*src[12] + tmp[6]*src[13] + tmp[11]*src[15];
		dst[11] = tmp[5]*src[12] + tmp[8]*src[13] + tmp[11]*src[14];
		dst[11]-= tmp[4]*src[12] + tmp[9]*src[13] + tmp[10]*src[14];
		dst[12] = tmp[2]*src[10] + tmp[5]*src[11] + tmp[1]*src[9];
		dst[12]-= tmp[4]*src[11] + tmp[0]*src[9] + tmp[3]*src[10];
		dst[13] = tmp[8]*src[11] + tmp[0]*src[8] + tmp[7]*src[10];
		dst[13]-= tmp[6]*src[10] + tmp[9]*src[11] + tmp[1]*src[8];
		dst[14] = tmp[6]*src[9] + tmp[11]*src[11] + tmp[3]*src[8];
		dst[14]-= tmp[10]*src[11] + tmp[2]*src[8] + tmp[7]*src[9];
		dst[15] = tmp[10]*src[10] + tmp[4]*src[8] + tmp[9]*src[9];
		dst[15]-= tmp[8]*src[9] + tmp[11]*src[10] + tmp[5]*src[8];
		/* calculate determinant */
		det=src[0]*dst[0]+src[1]*dst[1]+src[2]*dst[2]+src[3]*dst[3];
		/* calculate matrix inverse */
		det = 1/det;
		for (int j = 0; j < 16; j++)
			dst[j] *= det;
	}

	__global__ void transformVertexRobot(float* xi, float* yi, float* zi, int* ii,  float* t, float* xo, float* yo, float* zo)
	{
		int idx, id;
		float *h;
		float x, y, z;

		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		id = ii[idx];
		h = t + NUMELEM_H*id;

		x = xi[idx];
		y = yi[idx];
		z = zi[idx];

		xo[idx] = h[0]*x + h[1]*y + h[2]*z + h[3];
		yo[idx] = h[4]*x + h[5]*y + h[6]*z + h[7];
		zo[idx] = h[8]*x + h[9]*y + h[10]*z + h[11];

	}

	__global__ void transformVertexHumanOrEnvironment(float* xi, float* yi, float* zi, float* h, float* xo, float* yo, float* zo)
	{
		int idx, id;
		float x, y, z;

		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;


		x = xi[idx];
		y = yi[idx];
		z = zi[idx];

		xo[idx] = h[0]*x + h[1]*y + h[2]*z + h[3];
		yo[idx] = h[4]*x + h[5]*y + h[6]*z + h[7];
		zo[idx] = h[8]*x + h[9]*y + h[10]*z + h[11];

	}

	__global__ void transformBoundingBoxEnvironment(float* hi_robot, float* h_bb, float* h_inv)
	{
		int idx;
	

		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		float t[16];
		mm16_device(hi_robot+idx*NUMELEM_H, h_bb+idx*NUMELEM_H, t);
		invert4_device(t,h_inv+idx*NUMELEM_H);
	}

	__global__ void transformBoundingBoxRobot(float* t, int* ii, float* h_bb, float* h_inv)
	{
		int idx, id;
		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		float tmp[16];
		float* h;

		id = ii[idx];
		h = t + NUMELEM_H*id;

		mm16_device(h, h_bb+idx*NUMELEM_H, tmp);
		invert4_device(tmp,h_inv+idx*NUMELEM_H);
	}

	__device__ float raytraceRayFace(float *x, float* y, float* z)
	{
		//doing calculation here
		return 3.0;
	}

	__global__ void raytraceVertices(	float* xi, float* yi, float* zi,
														int* fx, int* fy, int* fz, int nF,
														float* bb_H, float* bb_D, int nBB, 
														float* D)
	{
		//defining vertex buffer
		__shared__ float vx[VERTEX_BUFFER_SIZE];
		__shared__ float vy[VERTEX_BUFFER_SIZE];
		__shared__ float vz[VERTEX_BUFFER_SIZE];

		int blockSize = blockDim.x * blockDim.y * blockDim.z;
		int threadIDinBlock = threadIdx.z * blockDim.y * blockDim.x + threadIdx.y * blockDim.x + threadIdx.x;
		int blockId = blockIdx.x + blockIdx.y * gridDim.x + gridDim.x * gridDim.y * blockIdx.z;

		float d = FLT_MAX;



		//determining number of vertices to be copied by each thread into buffer
		int nItePerThread = (int)((VERTEX_BUFFER_SIZE/blockSize)+1);


		//loading vertex data into local buffer
		int t,f;
		int numOfVerticesCopied = 0;
		int numberOfValidVertices;
		//keep loading vertices until all faces have been raytraced
		while(numOfVerticesCopied < nF)
		{
			if(nF - numOfVerticesCopied >= VERTEX_BUFFER_SIZE)
			{
				numberOfValidVertices = VERTEX_BUFFER_SIZE;
			}else{
				numberOfValidVertices = nF - numOfVerticesCopied;
			}
			

			for(int i=0; i<nItePerThread; i++)
			{
				t = threadIDinBlock*nItePerThread+i;
				//check if enough space in vertex buffer and enough remaing faces
				if(t < numberOfValidVertices)
				{
					//copying corresponding vertices
					f = fx[numOfVerticesCopied + t];
					vx[t] = xi[f];
					f = fy[numOfVerticesCopied + t];
					vy[t] = yi[f];
					f = fz[numOfVerticesCopied + t];
					vz[t] = zi[f];

				}
			}
			numOfVerticesCopied += numberOfValidVertices;
			__syncthreads();

			//raytrace and check minum distance
			for(int i=0; i<nItePerThread; i++)
			{
				t = threadIDinBlock*nItePerThread+i;
				d = fminf(d, raytraceRayFace(&(vx[t]), &(vy[t]), &(vz[t])));
				
			}
			D[blockId*blockSize+threadIDinBlock] = d;
		}
	}
}
#endif