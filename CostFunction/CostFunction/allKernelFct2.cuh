#ifndef ALLKERNELFCT2_CUH_
#define ALLKERNELFCT2_CUH_

#include "global.h"
#include "math_constants.h"
#include <curand_kernel.h>

namespace cuda_calc2{

	//#define EPSILON 0.000001
	//#define EPSILON 0.00000001
	#define EPSILON 0.00001
	//#define EPSILON (0.5f)


	__device__ void CROSS(float* r, const float *a, const float *b ) {
	  r[0] =   ( (a[1] * b[2]) - (a[2] * b[1]) );
	  r[1] =   ( (a[2] * b[0]) - (a[0] * b[2]) );
	  r[2] =   ( (a[0] * b[1]) - (a[1] * b[0]) );
	}

	__device__ void SUB(float* r, const float *a, const float *b ) {
	  r[0] =   a[0] - b[0];
	  r[1] =   a[1] - b[1];
	  r[2] =   a[2] - b[2];
	}

	__device__ float DOT(const float *a, const float *b) {
	  return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
	}

 
	__device__ int triangle_intersection( const float*   V1,  // Triangle vertices
								   const float*   V2,
								   const float*   V3,
								   const float*    O,  //Ray origin
								   const float*    D,  //Ray direction
										 float* out )
		{
		  float e1[3], e2[3];  //Edge1, Edge2
		  float P[3], Q[3], T[3];
		  float det, inv_det, u, v;
		  float t;
 
		  //Find vectors for two edges sharing V1
		  SUB(e1, V2, V1);
		  SUB(e2, V3, V1);
		  //Begin calculating determinant - also used to calculate u parameter
		  CROSS(P, D, e2);
		  //if determinant is near zero, ray lies in plane of triangle
		  det = DOT(e1, P);
		  //NOT CULLING
		  if(det > -EPSILON && det < EPSILON) return 0;
		  inv_det = 1.0f / det;
 
		  //calculate distance from V1 to ray origin
		  SUB(T, O, V1);
 
		  //Calculate u parameter and test bound
		  u = DOT(T, P) * inv_det;
		  //The intersection lies outside of the triangle
		  if(u < 0.f || u > 1.f) return 0;
 
		  //Prepare to test v parameter
		  CROSS(Q, T, e1);
 
		  //Calculate V parameter and test bound
		  v = DOT(D, Q) * inv_det;
		  //The intersection lies outside of the triangle
		  if(v < 0.0f || u + v  > 1.0f) return 0;
 
		  t = DOT(e2, Q) * inv_det;
 
		  if(t > EPSILON) { //ray intersection
			*out = t;
			return 1;
		  }
 
		  // No hit, no win
		  return 0;
	}



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


	__global__ void transformVertexCamera(float* xi, float* yi, float* zi, int nV, float* camPos_H, float* camRot_H, float* xo, float* yo, float* zo)
	{
		int idx, id;
		float x, y, z;

		float hr[16];

		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		
		//first calculate the resulting transformation
		//and inverse bb transformation

		mm16_device(camPos_H, camRot_H, hr);			
				
		if (idx < nV)
		{
			x = xi[idx];
			y = yi[idx];
			z = zi[idx];

			xo[idx] = hr[0]*x + hr[1]*y + hr[2]*z + hr[3];
			yo[idx] = hr[4]*x + hr[5]*y + hr[6]*z + hr[7];
			zo[idx] = hr[8]*x + hr[9]*y + hr[10]*z + hr[11];
		}

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

	__global__ void transformSamplePoint(float* t, int* ii, float* spi, float* spo)
	{
		int idx, id;
		float* h;

		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		id = ii[idx];
		h = t + NUMELEM_H*id;
		mm16_device(h, spi+idx*NUMELEM_H, spo+idx*NUMELEM_H);
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

	__device__ float calculateOrginAndDirection(float *h_samplePos, float *h_sampleRot, float xd, float yd, float zd, float*o, float* di)
	{
		//calculating resulting transformation
		float h[16];
		mm16_device(h_samplePos, h_sampleRot, h);

		o[0] = h[3];
		o[1] = h[7];
		o[2] = h[11];


		di[0] = h[0]*xd+h[1]*yd+h[2]*zd;
		di[1] = h[4]*xd+h[5]*yd+h[6]*zd;
		di[2] = h[8]*xd+h[9]*yd+h[10]*zd;

	}

	__device__ void calcNewAverage(float* average, int weight, float value)
	{
		float v = (*average)*weight;
		v += value;		
		(*average) = v/(weight+1);
	}



	__global__ void calcMiddlePoint(float* Dx, float* Dy, float* Dz, int* cis, int nP, int maxIndex,  float* a_x, float* a_y, float* a_z){
		//defining vertex buffer
		__shared__ float avg_x[AVG_BUFFER_SIZE];
		__shared__ float avg_y[AVG_BUFFER_SIZE];
		__shared__ float avg_z[AVG_BUFFER_SIZE];
		__shared__ int avg_w[AVG_BUFFER_SIZE];

	

		//init all arrays
		avg_x[threadIdx.x] = 0.0f;
		avg_y[threadIdx.x] = 0.0f;
		avg_z[threadIdx.x] = 0.0f;
		avg_w[threadIdx.x] = 0;

		__syncthreads();

		int nItePerThread = (int)((nP/AVG_BUFFER_SIZE)+1);
		int curIndex;
		int i;

		for(i=0; i<nItePerThread; i++)
		{
			curIndex = threadIdx.x*nItePerThread+i;
			
			//check if the iteration is still within limits
			if(curIndex < nP )
			{
			
				//check if the distance value is within limits
				//avg_w[threadIdx.x]=1;
				if(!isnan(Dx[curIndex]) && cis[curIndex] == maxIndex)
				{
					calcNewAverage(avg_x+threadIdx.x, avg_w[threadIdx.x], Dx[curIndex]);					
					calcNewAverage(avg_y+threadIdx.x, avg_w[threadIdx.x], Dy[curIndex]);					
					calcNewAverage(avg_z+threadIdx.x, avg_w[threadIdx.x], Dz[curIndex]);

					avg_w[threadIdx.x]++;
				}
			}
			
		}
		__syncthreads();
		//if(threadIdx.x == 0)
		//{
		//	int sum=0;
		//	for(i=0; i< AVG_BUFFER_SIZE; i++)
		//	{
		//		sum += avg_w[i];
		//	}
		//	printf("weight %d\n", sum);
		//}
		//
		//__syncthreads();
		
		int avg_weight = 0;
		float value = 0;
		////starting reduction
		////1024 threads turn out to 10 iterations
		for (i = AVG_BUFFER_SIZE / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{
				avg_weight = avg_w[threadIdx.x]+avg_w[threadIdx.x+i];
				if(avg_weight > 0)
				{
					value =  avg_x[threadIdx.x]*avg_w[threadIdx.x]+avg_x[threadIdx.x+i]*avg_w[threadIdx.x+i];
					avg_x[threadIdx.x] = value/avg_weight;
				
					value =  avg_y[threadIdx.x]*avg_w[threadIdx.x]+avg_y[threadIdx.x+i]*avg_w[threadIdx.x+i];
					avg_y[threadIdx.x] = value/avg_weight;

					value =  avg_z[threadIdx.x]*avg_w[threadIdx.x]+avg_z[threadIdx.x+i]*avg_w[threadIdx.x+i];
					avg_z[threadIdx.x] = value/avg_weight;

					//adding the weight to the average
					avg_w[threadIdx.x] = avg_weight;
				}
			}
		}
		__syncthreads();

		if (threadIdx.x == 0)
		{
			if (avg_w[0] > MINIMUM_POINTS_DETECTION)
			{
				a_x[0] = avg_x[0];
				a_y[0] = avg_y[0];
				a_z[0] = avg_z[0];
			}else
			{
				a_x[0] = nanf("");;
				a_y[0] = nanf("");;
				a_z[0] = nanf("");;
			}
			//printf("centroid at %.3f %.3f %.3f\n", a_x[0], a_y[0], a_z[0]);
		}


	}

	__device__ bool isInBB(float* h, float* dim, float* p)
	{
		float x = h[0]*p[0]+h[1]*p[1]+h[2]*p[2]+h[3];
		float y = h[4]*p[0]+h[5]*p[1]+h[6]*p[2]+h[7];
		float z = h[8]*p[0]+h[9]*p[1]+h[10]*p[2]+h[11];
		return x>0 && y>0 && z>0 && x<dim[0] && y<dim[1] && z<dim[2];
	}

	__device__ void transformPoint(float* h, float* p, float* pret)
	{
		pret[0] = h[0]*p[0]+h[1]*p[1]+h[2]*p[2]+h[3];
		pret[1] = h[4]*p[0]+h[5]*p[1]+h[6]*p[2]+h[7];
		pret[2] = h[8]*p[0]+h[9]*p[1]+h[10]*p[2]+h[11];		
	}

	__device__ void rotatePoint16(float* h, float* p, float* pret)
	{
		pret[0] = h[0]*p[0]+h[1]*p[1]+h[2]*p[2];
		pret[1] = h[4]*p[0]+h[5]*p[1]+h[6]*p[2];
		pret[2] = h[8]*p[0]+h[9]*p[1]+h[10]*p[2];		
	}

	__global__ void setup_kernel(curandState *state)
	{
		int id = threadIdx.x + blockIdx.x * 64;
		/* Each thread gets same seed, a different sequence 
		   number, no offset */
		curand_init(1234, id, 0, &state[id]);
	}

	__device__ float generateDepthValue(float* coeffs, float z, float sigmaFactor, curandState_t *state)
	{

		float rand = curand_normal (state);
		float sigma = 0.0f;		
		float exp;
		for(int i=0; i<N_COEFFS; i++)
		{	exp = N_COEFFS-i-1;
			sigma += coeffs[i]*powf(z, exp);
		}
		sigma *= sigmaFactor;
		return z;//+sigma*rand;
	}

	__global__ void raytraceVerticesCamera(				float* xi, float* yi, float* zi,
														int* fx, int* fy, int* fz, int nF,														
														float* camPos_H, float* camRot_H,
														float* camRayX, float* camRayY, float* camRayZ,
														float* Dx, float* Dy, float* Dz)
	{
		//defining vertex buffer
		__shared__ float v1[VERTEX_BUFFER_SIZE*3];
		__shared__ float v2[VERTEX_BUFFER_SIZE*3];
		__shared__ float v3[VERTEX_BUFFER_SIZE*3];
		
		

		int blockSize = blockDim.x * blockDim.y * blockDim.z;
		int threadIDinBlock = threadIdx.z * blockDim.y * blockDim.x + threadIdx.y * blockDim.x + threadIdx.x;
		int blockId = blockIdx.x + blockIdx.y * gridDim.x + gridDim.x * gridDim.y * blockIdx.z;
		int threadId = blockId * (blockDim.x * blockDim.y * blockDim.z)
			  + (threadIdx.z * (blockDim.x * blockDim.y))
			  + (threadIdx.y * blockDim.x)
			  + threadIdx.x;
		

		float dr;
		int hit = 0;
		bool validRay = !isnan(Dx[threadId]);

		


		//calculating current origin
		float o[3];
		float di[3];
		float* v1_d;
		float* v2_d;
		float* v3_d;
		calculateOrginAndDirection(camPos_H, camRot_H, camRayX[threadId], camRayY[threadId], camRayZ[threadId], o, di);
		
		

		//determining number of vertices to be copied by each thread into buffer
		int nItePerThread = (int)((VERTEX_BUFFER_SIZE/blockSize)+1);


		//loading vertex data into local buffer
		int t,f, fix, fiy, fiz;
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
			
			//calcNumberOfValidPoints(nF, int bufferSize, int numOfVerticesCopied)

			for(int i=0; i<nItePerThread; i++)
			{
				t = threadIDinBlock*nItePerThread+i;
				//check if enough space in vertex buffer and enough remaing faces
				if(t < numberOfValidVertices)
				{


					//copying corresponding vertices
					//dirty hack because the facesnumeration comes from
					//matlab which start 1 as first indice
					//therefore we have to substract each face indice by 1
					fix = fx[numOfVerticesCopied + t]-1;
					fiy = fy[numOfVerticesCopied + t]-1;
					fiz = fz[numOfVerticesCopied + t]-1;


					v1[3*t+0] = xi[fix];
					v1[3*t+1] = yi[fix];
					v1[3*t+2] = zi[fix];

					
					v2[3*t+0] = xi[fiy];
					v2[3*t+1] = yi[fiy];
					v2[3*t+2] = zi[fiy];

					
					v3[3*t+0] = xi[fiz];
					v3[3*t+1] = yi[fiz];
					v3[3*t+2] = zi[fiz];

				}
			}
			numOfVerticesCopied += numberOfValidVertices;
			__syncthreads();


			//we just need to do raytracing if it is a valid ray
			if(validRay)
			{
				//raytrace and check minum distance
				for(int i=0; i<numberOfValidVertices; i++)
				{

					//check if face is valid								
					v1_d = v1+3*i;
					v2_d = v2+3*i;
					v3_d = v3+3*i;
					hit =  triangle_intersection(	
											v1_d,  // Triangle vertices
											v2_d,
											v3_d,
											o,  //Ray origin
											di,  //Ray direction
											&dr);
					// we are just interested in a single hit
					if(hit > 0){						
						break;
					}
				
				}
			}
			__syncthreads();

		}

		if(hit > 0)
		{
			//setting to error value
			Dx[threadId] = nanf("");
			Dy[threadId] = nanf("");
			Dz[threadId] = nanf("");
		}


	}

	__global__ void raytraceVertices(					float* xi, float* yi, float* zi,
														int* fx, int* fy, int* fz, int nF,
														float dist_min, float dist_max,  
														float* camPos_H, float* camRot_H,
														float* camRayX, float* camRayY, float* camRayZ,
														float* Dx, float* Dy, float* Dz,
														int* fbbi, bool* bbHitGlobal, int nBB,
														int humanBB, int session)
	{
		//defining vertex buffer
		__shared__ float v1[VERTEX_BUFFER_SIZE*3];
		__shared__ float v2[VERTEX_BUFFER_SIZE*3];
		__shared__ float v3[VERTEX_BUFFER_SIZE*3];
		__shared__ bool isVerticeValid[VERTEX_BUFFER_SIZE];
		__shared__ bool bbHit[MAX_BB_HIT_BUFFER];

		int blockSize = blockDim.x * blockDim.y * blockDim.z;
		int threadIDinBlock = threadIdx.z * blockDim.y * blockDim.x + threadIdx.y * blockDim.x + threadIdx.x;
		int blockId = blockIdx.x + blockIdx.y * gridDim.x + gridDim.x * gridDim.y * blockIdx.z;
		int threadId = blockId * (blockDim.x * blockDim.y * blockDim.z)
			  + (threadIdx.z * (blockDim.x * blockDim.y))
			  + (threadIdx.y * blockDim.x)
			  + threadIdx.x;
		float d = dist_max;		
		float dr;
		int hit;

		//copying the global bbhit information into the shared buffer
		if(threadId < nBB)
		{
			bbHit[threadId] = bbHitGlobal[threadId];
		}
		__syncthreads();

		if(!bbHit[humanBB])
		{
			Dx[threadId] = nanf("");
			Dy[threadId] = nanf("");
			Dz[threadId] = nanf("");
			return;
		}

		//calculating current origin
		float o[3];
		float di[3];
		float p[3];
		float* v1_d;
		float* v2_d;
		float* v3_d;
		calculateOrginAndDirection(camPos_H, camRot_H, camRayX[threadId], camRayY[threadId], camRayZ[threadId], o, di);
		
		//if(threadId < 10)
		//{
		//	printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", o[0],o[1],o[2],di[0],di[1],di[2]); 
		//}

		//determining number of vertices to be copied by each thread into buffer
		int nItePerThread = (int)((VERTEX_BUFFER_SIZE/blockSize)+1);


		//loading vertex data into local buffer
		int t,f, fix, fiy, fiz;
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
			
			//calcNumberOfValidPoints(nF, int bufferSize, int numOfVerticesCopied)

			for(int i=0; i<nItePerThread; i++)
			{
				t = threadIDinBlock*nItePerThread+i;
				//check if enough space in vertex buffer and enough remaing faces
				if(t < numberOfValidVertices)
				{


					//copying corresponding vertices
					//dirty hack because the facesnumeration comes from
					//matlab which start 1 as first indice
					//therefore we have to substract each face indice by 1
					fix = fx[numOfVerticesCopied + t]-1;
					fiy = fy[numOfVerticesCopied + t]-1;
					fiz = fz[numOfVerticesCopied + t]-1;

					//check first if the face is valid
					//do not copy the rest
					//isVerticeValid = bbHit
					isVerticeValid[t] = bbHit[fbbi[fix]] && bbHit[fbbi[fiy]] && bbHit[fbbi[fiz]];
					if(!isVerticeValid[t])
						continue;

					v1[3*t+0] = xi[fix];
					v1[3*t+1] = yi[fix];
					v1[3*t+2] = zi[fix];

					
					v2[3*t+0] = xi[fiy];
					v2[3*t+1] = yi[fiy];
					v2[3*t+2] = zi[fiy];

					
					v3[3*t+0] = xi[fiz];
					v3[3*t+1] = yi[fiz];
					v3[3*t+2] = zi[fiz];

				}
			}
			numOfVerticesCopied += numberOfValidVertices;
			__syncthreads();

			
			//raytrace and check minum distance
			for(int i=0; i<numberOfValidVertices; i++)
			{

				//check if face is valid								
				v1_d = v1+3*i;
				v2_d = v2+3*i;
				v3_d = v3+3*i;
				hit =  triangle_intersection(	
										v1_d,  // Triangle vertices
										v2_d,
										v3_d,
										o,  //Ray origin
										di,  //Ray direction
										&dr);

				if(hit > 0){
					d = fminf(d, dr);
				}
				
			}
			__syncthreads();

		}

		
		
		p[0] = o[0]+d*di[0];
		p[1] = o[1]+d*di[1];
		p[2] = o[2]+d*di[2];

		if(d > dist_min && d < dist_max)
		{

			//setting real value and calculate weighted average
			Dx[threadId] = p[0];
			Dy[threadId] = p[1];
			Dz[threadId] = p[2];
		}else
		{
			//setting to error value
			Dx[threadId] = nanf("");
			Dy[threadId] = nanf("");
			Dz[threadId] = nanf("");
		}


	}

		//	float v0_2 = v[0]*v[0];
		//float v1_2 = v[1]*v[1];
		//float v2_2 = v[2]*v[2];
		//float t1 = v0_2/human_rx_2;
		//float t2 = v1_2/human_ry_2;
		//float t3 = v2_2/human_rz_2;		
		//float mg = sqrtf(t1+t2+t3);
		//float t = powf((1/mg)-1, 2.0);
		//return sqrtf(v0_2+v0_2+v0_2*t);

	__device__ bool intersectBox(float* origin, float* direction, float* vmin, float* vmax)
	{

	float tmin, tmax, tymin, tymax, tzmin, tzmax;
	bool flag;
    if (direction[0] >= 0) 
	{
    	tmin = (vmin[0] - origin[0]) / direction[0];
    	tmax = (vmax[0] - origin[0]) / direction[0];
	}
    else
	{
    	tmin = (vmax[0] - origin[0]) / direction[0];
    	tmax = (vmin[0] - origin[0]) / direction[0];
	}
  
    if (direction[1] >= 0) 
	{
        tymin = (vmin[1] - origin[1]) / direction[1];
        tymax = (vmax[1] - origin[1]) / direction[1];
	}
    else
	{
    	tymin = (vmax[1] - origin[1]) / direction[1];
    	tymax = (vmin[1] - origin[1]) / direction[1];
	}
    

    if ( (tmin > tymax) || (tymin > tmax) )
	{
    	return false;
	}
       
    if (tymin > tmin)
	{
        tmin = tymin;
	}
    
    
	if (tymax < tmax)
	{
        tmax = tymax;
	}
    
    
	if (direction[2] >= 0)
	{
       tzmin = (vmin[2] - origin[2]) / direction[2];
       tzmax = (vmax[2] - origin[2]) / direction[2];
	}
    else
	{
       tzmin = (vmax[2] - origin[2]) / direction[2];
       tzmax = (vmin[2] - origin[2]) / direction[2];
	}


    if ((tmin > tzmax) || (tzmin > tmax))
	{
       return false;
	}
    
	   return true;
	}
	__global__ void raytraceBox(float* camPos_H, float* camRot_H, float* camRayX, float* camRayY, float* camRayZ, int nRays,
						   float* bb_H, float* bb_D, int nBB, bool* bb_intersect)
	{
		__shared__ bool hhb [MAX_RAYTRACE_BOX];

		float o[3];
		float di[3];
		float o_t[3];
		float di_t[3];
		float dim_0[3];
		float* pH;
		dim_0[0] = 0.0f;
		dim_0[1] = 0.0f;
		dim_0[2] = 0.0f;
		int index = 0;

		//calculating number of rays to test per thread
		int nRaysPerThread = (nRays/blockDim.x)+1;
		for(int bbi=0; bbi<nBB; bbi++)
		{
			//setting buffer to false
			hhb[threadIdx.x] = false;
			__syncthreads();
		

			//starting to raytrace each single 
			bool iB = false;
			for(int ray=0; ray<nRaysPerThread; ray++)
			{
				index = threadIdx.x * nRaysPerThread+ray;
				if(index >= nRays)
					break;
				
				pH = bb_H+bbi*NUMELEM_H;

				calculateOrginAndDirection(camPos_H, camRot_H, camRayX[index], camRayY[index], camRayZ[index], o, di);
				transformPoint(pH, o, o_t);
				rotatePoint16(pH, di, di_t);
				iB |= intersectBox(o_t, di_t,dim_0, bb_D+bbi*3);

				//we just want to know if intersecting, not the number of intersections
				if(iB)
					break;

			}
			hhb[threadIdx.x] = iB;
			


			for (int i = MAX_RAYTRACE_BOX / 2; i > 0; i >>= 1)
			{
				__syncthreads();
				if(threadIdx.x < i)
				{	
					hhb[threadIdx.x] |= hhb[threadIdx.x+i];
				}
			}

			if(threadIdx.x == 0)
			{
				//bb_intersect[bbi] = true;
				bb_intersect[bbi] = hhb[0];
			}
			
			__syncthreads();
		}

		__syncthreads();
	}

	__device__ float distanceToEllipse(float* v)
	{
		/*float vp[3];*/
		float t1 = powf(v[0]/human_rx, 2.0);
		float t2 = powf(v[1]/human_ry, 2.0);
		float t3 = powf(v[2]/human_rz, 2.0);
		float mag = sqrtf(t1+t2+t3);
	/*	vp[0] = v[0]/mag;
		vp[1] = v[1]/mag;
		vp[2] = v[2]/mag;*/
		float inner = powf(v[0], 2.0f)+powf(v[1], 2.0f)+powf(v[2], 2.0f);
		float factor = powf((-1.0f + 1.0f/mag), 2.0f);
		return sqrtf(inner*factor);
		//return sqrtf(powf(vp[0]-v[0],2.0)+powf(vp[1]-v[1],2.0)+powf(vp[2]-v[2],2.0));
	}

	__device__ void distanceToEllipseCache(float* v, float* ws_l, int* ws_n, float* d, float* dist, int* w)
	{
		int x = floorf((v[0]-ws_l[0])/ws_l[6]);
		int y = floorf((v[1]-ws_l[2])/ws_l[7]);
		int z = floorf((v[2]-ws_l[4])/ws_l[8]);
		if(	x >= 0 && x < ws_n[0] &&
			y >= 0 && y < ws_n[1] &&
			z >= 0 && z < ws_n[2]){
			int index = x*ws_n[1]*ws_n[2] + y*ws_n[2] +z;
			//printf("%d\t%d\t%d\t%d\t%.4f\n",x,y,z,index, d[index]);
			(*dist) += d[index];
			(*w) ++;
		}

	}

	__device__ void rotatePoint(float* R, float* p)
	{
		float b[3];
		b[0] = R[0]*p[0]+R[1]*p[1]+R[2]*p[2];
		b[1] = R[3]*p[0]+R[4]*p[1]+R[5]*p[2];
		b[2] = R[6]*p[0]+R[7]*p[1]+R[8]*p[2];

		p[0] = b[0];
		p[1] = b[1];
		p[2] = b[2];
	}

	#define	tailor_a (2.055792709094123)
	#define	tailor_b (-13.314579265182624)
	#define tailor_min (0.054125744849443)

	#define tailor_weight_a (0.056234132519035)
	#define tailor_weight_b (0.028782313662426)
	#define tailor_weight_max 100


	__device__ double calculateProbabilityOfDetection(float meanDistance)
	{
		//meanDistance = 4.0*meanDistance;
		if(meanDistance < tailor_min){
			//printf("%.5f\n", meanDistance);
			return 1.0;
		}else{
			return tailor_a*exp(tailor_b*meanDistance);
		}
	}

	__device__ double calculateWeightOfDetection(int w)
	{
		//if(w > MINIMUM_POINTS_DETECTION)
		//	return 1.0f;
		//else
		//	return 0.0f;
		//meanDistance = 4.0*meanDistance;
		if(w > tailor_weight_max){
			//printf("%.5f\n", meanDistance);
			return 1.0;
		}else{
			return tailor_weight_a*exp(tailor_weight_b*w);
		}
	}

	__device__ int calcNumberOfValidPoints(int nP, int bufferSize, int numOfPointsCopied)
	{

			int remainingPoints = nP - numOfPointsCopied;
			if(remainingPoints >= POINT_BUFFER_SIZE)
			{
				//remaining points are larger as point buffer size
				return POINT_BUFFER_SIZE;
			}else{
				//remaining points are not larger as point buffer size
				return remainingPoints;
			}

	}

	__global__ void zeroProb(float* prob)
	{
		int threadId =  blockIdx.x *blockDim.x + threadIdx.x;
		prob[threadId] = 0.0;
	}

	__global__ void normalizeProb(float* prob, int n)
	{
		int threadId =  blockIdx.x *blockDim.x + threadIdx.x;
		prob[threadId] /= n;
	}


		

	__global__ void distanceToEllipseModel(float* Dx, float* Dy, float* Dz, int nP,
									  float* R, float* Fx, float* Fy,
									  float* cx, float* cy, float* cz,	
									  float* ws_l_params, int* ws_n_params, float* d_m,									 
									  double* prop, float* distance, int* weight)
	{
		__shared__ float vx[POINT_BUFFER_SIZE];
		__shared__ float vy[POINT_BUFFER_SIZE];
		__shared__ float vz[POINT_BUFFER_SIZE];

		__shared__ float ws_l[9];
		__shared__ int ws_n[3];
		

		//__shared__ float R_l[THREADS_MODEL_FITTING*9];
		//__shared__ float Fx_l[THREADS_MODEL_FITTING];
		//__shared__ float Fy_l[THREADS_MODEL_FITTING];


		int blockSize = blockDim.x * blockDim.y * blockDim.z;
		int threadIDinBlock = threadIdx.z * blockDim.y * blockDim.x + threadIdx.y * blockDim.x + threadIdx.x;
		int blockId = blockIdx.x + blockIdx.y * gridDim.x + gridDim.x * gridDim.y * blockIdx.z;
		int threadId = blockId*blockSize+threadIDinBlock;

		//checking if the centroid calculation was valid
		if(isnan(cx[0]))
		{
			//probability of detection zero and leave
			prop[threadId] = 0.0;
			distance[threadId] = FLT_MAX;
			weight[threadId] = 0;
			return;
		}

		int index,i;
		int numOfPointsCopied = 0;
		int numberOfValidPoints;

		float	dist = 0.0f;
		int		w = 0;
		float	p[3];


		//copying the rotation and offset in x and y first into shared memory		 


		//copying rotations
		//for(int j=0; j<9; j++)
		//{
		//	R_l[threadIDinBlock*9+j] =R[(blockId*blockSize+threadIDinBlock)*9+j];
		//}
		//Fx_l[threadIDinBlock] = Fx[blockId*blockSize+threadIDinBlock];
		//Fy_l[threadIDinBlock] = Fy[blockId*blockSize+threadIDinBlock];
		
	


		//determining number of vertices to be copied by each thread into buffer
		int nItePerThread = (int)((POINT_BUFFER_SIZE/blockSize)+1);

		//loading vertex data into local buffer
		if(threadIDinBlock < 9)
		{
			ws_l[threadIDinBlock] = ws_l_params[threadIDinBlock];
		}

		if(threadIDinBlock < 3)
		{
			ws_n[threadIDinBlock] = ws_n_params[threadIDinBlock];
		}


		

		//keep loading vertices until all faces have been raytraced
		while(numOfPointsCopied < nP)
		{

			numberOfValidPoints = calcNumberOfValidPoints(nP, POINT_BUFFER_SIZE,numOfPointsCopied);

			for(i=0; i<nItePerThread; i++)
			{
			
				index = threadIDinBlock*nItePerThread+i;
				//check if enough space in vertex buffer and enough remaing faces
				if(index < numberOfValidPoints)
				{
					vx[index] = Dx[numOfPointsCopied+index];
					vy[index] = Dy[numOfPointsCopied+index];
					vz[index] = Dz[numOfPointsCopied+index];
				}
			}
			numOfPointsCopied += numberOfValidPoints;
			__syncthreads();

			//points copied into the buffer
			//calculate actual distance to model
			//p_t4 = rotz(-SA(3,s))*(p'-(c'+shift));
			for(i=0; i<numberOfValidPoints; i++)
			{
				//checking if valid point is in buffer
				if(!isnan(vx[i]))
				{
					//p[0] = vx[i]-(cx[0]+Fx[threadId]);
					//p[1] = vy[i]-(cy[0]+Fy[threadId]);
					//p[2] = vz[i]-(cz[0]);

					//rotatePoint(R+9*threadId, p);
					//dist += distanceToEllipse(p);
					//w++;
					p[0] = vx[i]-(cx[0]+Fx[threadId]);
					p[1] = vy[i]-(cy[0]+Fy[threadId]);
					p[2] = vz[i];

					rotatePoint(R+9*threadId, p);					
					distanceToEllipseCache(p, ws_l, ws_n, d_m, &dist, &w);
					
				}
			}
			__syncthreads();
			
		}


		if(w > 0 )
		{
			dist /= w;
			
			double w_w = calculateWeightOfDetection(w);
			double w_p = calculateProbabilityOfDetection(dist);
			double ret = w_p*w_w;
			//prop[threadId] = ((WEIGHT_P*w_p)+(WEIGHT_W*w_w))/WEIGHT_SUM;
			prop[threadId] = fmin(ret, 1.0);
			distance[threadId] = dist;
			weight[threadId] = w;
			//if(w > MINIMUM_POINTS_DETECTION)
			//{
			//	prop[threadId] = calculateProbabilityOfDetection(dist);
			//	distance[threadId] = dist;
			//}
			//else
			//{
			//	prop[threadId] = 0.0f;
			//	distance[threadId] = FLT_MAX;
			//}

		}else
		{
				prop[threadId] = 0.0;
				distance[threadId] = FLT_MAX;
				weight[threadId] = 0;
		}
		//if(w > MINIMUM_POINTS_DETECTION)
		//{
		//	dist = dist/w;
		//	prop[threadId] = dist;// calculateProbabilityOfDetection(dist);
		//	//printf("%.5f\t%.5f\n", dist,prop[threadId]);
		//}
		//else{
		//	prop[threadId] = nanf("");
		//	//prop[threadId] = FLT_MAX;
		//}
		
	}
	__global__ void calculateMinWeight(int* weights, int n, float* dist, float* mindist, int* w)
	{
		__shared__ int max_buffer[MAX_BUFFER_SIZE];

		int nItePerThread = (int)((n/MAX_BUFFER_SIZE)+1);
		int curIndex;
		int i;

		max_buffer[threadIdx.x] = -1;
		__syncthreads();
		
		float localMax = FLT_MAX;
		for(i=0; i<nItePerThread; i++)
		{
			curIndex = threadIdx.x*nItePerThread+i;
			//check if the iteration is still within limits
			if(curIndex < n)
			{				
				if(dist[curIndex] == mindist[0])
				{
					max_buffer[threadIdx.x] = weights[curIndex];
				}
			}
		}
		
		__syncthreads();
		
		////starting reduction
		////1024 threads turn out to 10 iterations
		for (i = MAX_BUFFER_SIZE / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{	
				if (max_buffer[threadIdx.x+i] > 0)
				{
					max_buffer[threadIdx.x] = max_buffer[threadIdx.x+i];
				}
				//max_buffer[threadIdx.x] = fmin(max_buffer[threadIdx.x],  max_buffer[threadIdx.x+i]);
			}
		}

		__syncthreads();
		if(threadIdx.x == 0)
		{
			//maxp[0] = pr[0] * max_buffer[0];
			w[0] = max_buffer[0];
		}

	}


	__global__ void calculateMinDist(float* prob, int n, float* maxp, float* pr)
	{
		__shared__ float max_buffer[MAX_BUFFER_SIZE];

		int nItePerThread = (int)((n/MAX_BUFFER_SIZE)+1);
		int curIndex;
		int i;

		

		
		float localMax = FLT_MAX;
		for(i=0; i<nItePerThread; i++)
		{
			curIndex = threadIdx.x*nItePerThread+i;
			//check if the iteration is still within limits
			if(curIndex < n)
			{				
				localMax = fmin(prob[curIndex],  localMax);
			}
		}
		max_buffer[threadIdx.x] = localMax;

		__syncthreads();
		
		////starting reduction
		////1024 threads turn out to 10 iterations
		for (i = MAX_BUFFER_SIZE / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{				
				max_buffer[threadIdx.x] = fmin(max_buffer[threadIdx.x],  max_buffer[threadIdx.x+i]);
			}
		}

		__syncthreads();
		if(threadIdx.x == 0)
		{
			//maxp[0] = pr[0] * max_buffer[0];
			maxp[0] = max_buffer[0];
		}

	}

	__global__ void zeroMaxProb(double* prob, int n, double* maxp)
	{
		

		int nItePerThread = (int)((n/MAX_BUFFER_SIZE)+1);
		int curIndex;
		int i;

				
		for(i=0; i<nItePerThread; i++)
		{
			curIndex = threadIdx.x*nItePerThread+i;			
			if(curIndex < n)
			{
				prob[curIndex] = 0.0;				
			}
		}
		maxp[0] = 0.0;
	}


	__global__ void calculateMaxProb(double* prob, int n, double* maxp, float* pr)
	{
		__shared__ double max_buffer[MAX_BUFFER_SIZE];

		int nItePerThread = (int)((n/MAX_BUFFER_SIZE)+1);
		int curIndex;
		int i;

		

		double localMax = 0.0;
		//float localMax = FLT_MAX;
		for(i=0; i<nItePerThread; i++)
		{
			curIndex = threadIdx.x*nItePerThread+i;
			//check if the iteration is still within limits
			if(curIndex < n)
			{
				localMax = fmax(prob[curIndex],  localMax);
				//localMax = fmin(prob[curIndex],  localMax);
			}
		}
		max_buffer[threadIdx.x] = localMax;

		__syncthreads();
		
		////starting reduction
		////1024 threads turn out to 10 iterations
		for (i = MAX_BUFFER_SIZE / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{
				max_buffer[threadIdx.x] = fmax(max_buffer[threadIdx.x],  max_buffer[threadIdx.x+i]);
				//max_buffer[threadIdx.x] = fmin(max_buffer[threadIdx.x],  max_buffer[threadIdx.x+i]);
			}
		}

		__syncthreads();		
		if(threadIdx.x == 0)
		{
			
			//printf("res on cuda %.10f\n", res);
			//maxp[0] += ((double)pr[0]) * max_buffer[0];
			maxp[0] = max_buffer[0];
		}

	}

	__device__ float calculateSigmaFactor(int nx, int ny, int u, int v, int rmax)
	{
		int du = u-nx/2;
		int dv = v-ny/2;
		int r = du*du+dv*dv;
		return 1.0f + ((float)r)/((float)rmax);
	}

	__global__ void mergeSuperSampling(float* s_dx, float* s_dy, float* s_dz,
										int ss_x, int ss_y,
										int nx, int ny,
										int minW, int rmax,
										float* bb_H, float* bb_D, int nBB,
										float* camPos_H, float* camRot_H,
										float* camRayX, float* camRayY, float* camRayZ,
										float* c, curandState_t* devStates,
										float* dx, float* dy, float* dz,
										int humanID)
	{
		float o[3];
		float di[3];
		float p[3];
		float d;


		int blockId = blockIdx.x + blockIdx.y * gridDim.x + gridDim.x * gridDim.y * blockIdx.z;
		int threadId = blockId * (blockDim.x * blockDim.y * blockDim.z)
			  + (threadIdx.z * (blockDim.x * blockDim.y))
			  + (threadIdx.y * blockDim.x)
			  + threadIdx.x;

		int w = 0;
		float dist, vec[3];
		dist = 0.0f;		
		vec[0] = 0.0f;
		vec[1] = 0.0f;
		vec[2] = 0.0f;
		int ss_nx = ss_x*nx;

		int u = (int)fmod((float)threadId, (float)nx);
		int	v = (int)floor(((float)threadId)/((float)nx));

		float sigmaFactor = calculateSigmaFactor(nx, ny, u,v,rmax);

		int id;
		for(int x=0; x<ss_x; x++)
		{
			for(int y=0; y<ss_y; y++)
			{
				id = (ss_y*v+y)*ss_nx+(ss_x*u+x);
				//check if raytrace is valid
				if(!isnan(s_dx[id]))
				{
					w++;
					vec[0] += s_dx[id];
					vec[1] += s_dy[id];
					vec[2] += s_dz[id];
				}
			}
		}

		//check if the minimum number of rays have actually hit anything
		if(w < minW)
		{
			dx[threadId] = nan("");
			dy[threadId] = nan("");
			dz[threadId] = nan("");
			return;
		}
		//if not find the medium distance from model
		vec[0] /= (float)w;
		vec[1] /= (float)w;
		vec[2] /= (float)w;




		for(int x=0; x<ss_x; x++)
		{
			for(int y=0; y<ss_y; y++)
			{
				id = (ss_y*v+y)*ss_nx+(ss_x*u+x);
				//check if raytrace is valid
				if(!isnan(s_dx[id]))
				{					
					dist += powf(s_dx[id]-vec[0], 2.0f)+
							powf(s_dy[id]-vec[1], 2.0f)+
							powf(s_dz[id]-vec[2], 2.0f);
				}
			}
		}
		dist /= (float)w;
		if(dist > DIST_PIXEL)
		{
			//no valid pixel
			dx[threadId] = nan("");
			dy[threadId] = nan("");
			dz[threadId] = nan("");
		}else{
			//valid pixel

			calculateOrginAndDirection(camPos_H, camRot_H, camRayX[threadId], camRayY[threadId], camRayZ[threadId], o, di);
			//printf("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n", o[0], o[1], o[2], di[0], di[1],di[2]);
			float diff1 =vec[0]-o[0];
			float diff2 =vec[1]-o[1];
			float diff3 =vec[2]-o[2];
			float sum = diff1*diff1 + diff2*diff2+ diff3*diff3;
			d = sqrtf(sum);
			

			d = generateDepthValue(c,d,sigmaFactor,&devStates[threadId]);
		

			p[0] = o[0]+d*di[0];
			p[1] = o[1]+d*di[1];
			p[2] = o[2]+d*di[2];

			bool isInBox = false;
			for(int i=0; i<nBB; i++)
			{
				if(i==humanID)
					continue;
				isInBox |= isInBB(bb_H+i*16, bb_D+i*3, vec);
			}
			
			//if point is in bounding box, set nan and leave
			if(isInBox){
				dx[threadId] = nan("");
				dy[threadId] = nan("");
				dz[threadId] = nan("");
			}else{
				dx[threadId] = p[0];
				dy[threadId] = p[1];
				dz[threadId] = p[2];
			}
		}
	}
	

	__global__ void calcDistanceMatrix(float* x, float* y, float* z, int nsample, int nx, float* dist)
	{
		
		int threadIDX = blockIdx.x*blockDim.x+threadIdx.x;
		int threadIDY = blockIdx.y*blockDim.y+threadIdx.y;
		

		int xid, yid, id1, id2;
		float d;
		
		for(int xi=0; xi<nsample; xi++)
		{
			for(int yi=0; yi<nsample; yi++)
			{
				xid = threadIDX*nsample+xi;
				yid = threadIDY*nsample+yi;

				id1 = yid*nx+xid;						
				if(xid < nx && yid < nx && !isnan(x[xid]) && !isnan(x[yid]))
				{
					d = powf(x[xid]-x[yid], 2.0f)+ powf(y[xid]-y[yid], 2.0f)+ powf(z[xid]-z[yid], 2.0f);
					dist[id1] = d;					
				}else
				{
					dist[id1] = nan("");					
				}
				
			}
		}
	}


	__global__ void calcCluster(int* ci, float* dist,  int nsample, int nx, int* nofClusters)
	{
		

		__shared__ int maxclustersize[MAX_CLUSTER_BUFFER_SIZE];

		int xid1, xid2, did;
		float d;
		int clusterid = -1;
		
		for(int xi1=0; xi1<nsample; xi1++)
		{
			xid1 = threadIdx.x*nsample+xi1;			
			if(xid1 < nx/* && !isnan(dist[xid1])*/)
			{
				if(ci[xid1] <0)
				{
					clusterid++;
					ci[xid1] = clusterid;
					
				}
				//point does not belong to a cluster
				//assign new cluster and check if other points in the area belong to the cluster
				

				//check the distance to other points
				for(int xi2=0; xi2<nsample; xi2++)
				{
					xid2 = threadIdx.x*nsample+xi2;	
					did = xid1*nx+xid2;	
					if(xid2 < nx && ci[xid2] < 0 && !isnan(dist[did]) && dist[did] <= EUCLIDEAN_THRESHOLD)
					{
						ci[xid2] = clusterid;							
					}
				}
				
				// end of cluster			
			}
		}
		maxclustersize[threadIdx.x] = clusterid;


		int startIndex1, startIndex2;
		int currentClusterSize = nsample;

		for (int i = MAX_CLUSTER_BUFFER_SIZE / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{

				int mergedCluster = 0;
				//merging two clusters
				startIndex1 = (threadIdx.x+0)*nsample;
				startIndex2 = (threadIdx.x+i)*nsample;

				//first setting the indices of the second part higher
				
				//for(int i1=0; i1<currentClusterSize; i1++)
				//{
				//	if(ci[startIndex2+i1] >= 0)
				//	{
				//		ci[startIndex2+i1] += indexOffset;
				//	}
				//}

				//__syncthreads();

				for(int i1=0;  maxclustersize[threadIdx.x] > 0 && i1<maxclustersize[threadIdx.x]; i1++)
				{
					bool canMergeCluster = false;
					int  mergeIndice = -1;
					for(int i2=0; maxclustersize[threadIdx.x+i] > 0 && i2<maxclustersize[threadIdx.x+i] && !canMergeCluster; i2++)
					{

						//finding all the points of the first cluster and compare them to all the points of the second cluster
						//if at least one point is below the thresholds, both cluster can be merged
						for(int ip1=0; ip1<currentClusterSize && !canMergeCluster; ip1++)
						{
							if(ci[startIndex1+ip1] == i1)
							{
								for(int ip2=0; ip2<currentClusterSize && !canMergeCluster; ip2++)
								{
									did = (startIndex1+ip1)*nx+startIndex2+ip2;	
									if( ci[startIndex2+ip2] == i2 && !isnan(dist[did]) && dist[did] <= EUCLIDEAN_THRESHOLD)
									{
										canMergeCluster = true;		
										mergeIndice = i2;
									}
								}

							}
						}


					}
											//if i can merge cluster, i set all corresponding indices in the second half
					if(canMergeCluster){
						mergedCluster++;
						for(int ip2=0; ip2<currentClusterSize; ip2++)
						{	
							if(ci[startIndex2+ip2] == mergeIndice)
							{
								ci[startIndex2+ip2] = i1;
							}
						}
					}
					else{
						//cannot merge-> setting the first half high
						for(int ip2=0; ip2<currentClusterSize; ip2++)
						{	
							if(ci[startIndex1+ip2] == i1)
							{
								ci[startIndex1+ip2] += maxclustersize[threadIdx.x+i]+1;
							}
						}
					}
				}
				maxclustersize[threadIdx.x] = maxclustersize[threadIdx.x] + maxclustersize[threadIdx.x + i] - mergedCluster+1;
				//max_buffer[threadIdx.x] = fmaxf(max_buffer[threadIdx.x],  max_buffer[threadIdx.x+i]);
				//max_buffer[threadIdx.x] = fmin(max_buffer[threadIdx.x],  max_buffer[threadIdx.x+i]);
			}
			
			currentClusterSize *= 2;
		}

		__syncthreads();

		if(threadIdx.x == 0)
		{
			nofClusters[0] = maxclustersize[0];
		}


	}

	__global__ void calcNmembersOfCluster(int* ci, int n, int nsample, int* NmembersOfCluster, int* indiceMaxCluster)
	{
		__shared__ int maxclustersize[MAX_CLUSTER_BUFFER_SIZE*MAX_CLUSTER_N_SIZE];
		//setting shared memory to zero 
		for(int i=0; i<MAX_CLUSTER_N_SIZE; i++)
		{
			maxclustersize[threadIdx.x*MAX_CLUSTER_BUFFER_SIZE+i] = 0;
		}

		int xid1, clusteri;
		for(int xi1=0; xi1<nsample; xi1++)
		{
			xid1 = threadIdx.x*nsample+xi1;
			if(xid1<n)
			{
				clusteri = ci[xid1];
				if(clusteri >= 0 && clusteri < MAX_CLUSTER_N_SIZE)
				{
					maxclustersize[threadIdx.x*MAX_CLUSTER_N_SIZE+clusteri]++;
				}
			}
		}

		//reduction
		for (int i = MAX_CLUSTER_BUFFER_SIZE / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{
				for(int j=0; j<MAX_CLUSTER_N_SIZE; j++)
				{
					maxclustersize[(threadIdx.x + 0)*MAX_CLUSTER_N_SIZE+j] += maxclustersize[(threadIdx.x + i)*MAX_CLUSTER_N_SIZE+j];
				}
			}
		}

		__syncthreads();
		if(threadIdx.x ==0)
		{
			int maxClusterIndice = -1;
			int maxN = 0;
			for(int j=0; j<MAX_CLUSTER_N_SIZE; j++)
			{
				if(NmembersOfCluster[j] > maxN)
				{
					maxClusterIndice = j;
					maxN = NmembersOfCluster[j];
				}
				NmembersOfCluster[j] = maxclustersize[j];
			}
			indiceMaxCluster[0] = maxClusterIndice;
		}

	}
	
	__device__ void initArray(bool* a, bool value, int nsample, int N)
	{
		int index;
		for(int i=0; i<nsample; i++)
		{
			index = threadIdx.x*nsample+i;
			if(index < N)
			{
				a[index] = value;
			}
		}
		
	}

	__device__ void hasAllProcessedP(float* dx, bool* a, bool *b, int nsample, int N)
	{
		int index;
		bool res = true;
		for(int i=0; i<nsample; i++)
		{
			index = threadIdx.x*nsample+i;
			if(index < N && !isnan(dx[index]))
			{
				res &= a[index];
			}
		}
		b[threadIdx.x] = res;

		for (int i = MAX_CLUSTER_BUFFER_SIZE / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{
				b[threadIdx.x] &= b[threadIdx.x+i];
			}
		}
	
		__syncthreads();
	}

	__device__ void hasAllProcessedQ(float* dx, bool* a, bool* s, bool *b, int nsample, int N)
	{
		int index;
		bool res = true;
		for(int i=0; i<nsample; i++)
		{
			index = threadIdx.x*nsample+i;
			if(index < N && !isnan(dx[index]) && s[index])
			{
				res &= a[index];
			}
		}
		b[threadIdx.x] = res;

		for (int i = MAX_CLUSTER_BUFFER_SIZE / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{
				b[threadIdx.x] &= b[threadIdx.x+i];
			}
		}
	
		__syncthreads();
	}

	__device__ void process(float* dx, bool* isProcessed, bool* inQ, float* dm, int nsample, int N)
	{
		int index_p, index_dd;
		for(int i_q=0; i_q<N; i_q++)
		{
			
			if( isnan(dx[i_q]) || isProcessed[i_q] || !inQ[i_q])
				continue;
			
			//check all points in P is there is a s
			for(int i_p=0; i_p<nsample; i_p++)
			{
				index_p = threadIdx.x*N+i_p;
				if(index_p >= N || isProcessed[index_p])
					continue;

				index_dd = i_q*N+index_p;
				if(!isnan(dm[index_dd]) && dm[index_dd] < EUCLIDEAN_THRESHOLD)
				{
					inQ[index_p] = true;
				}
			}
			
			if(threadIdx.x == 0)
			{
				isProcessed[i_q] = true;
			}
			__syncthreads();
		}

	}

	__device__ void setFirstNotProcessedIndice(float* dx, bool* isProcessed, bool* inQ, int N)
	{
		//only one thread is doing the work
		if(threadIdx.x == 0)
		{
			for(int i=0;i<N;i++)
			{
				if(!isnan(dx[i]) && !isProcessed[i])
				{
					inQ[i] = true;
					break;
				}
			}
		}
		__syncthreads();
	}


	__device__ int setIndices(int ci, int* cis, bool* inQ, int nsample, int N)
	{
		int index;
		for(int i=0; i<nsample; i++)
		{
			index = threadIdx.x*nsample+i;
			if(index < N && inQ[index])
			{
				cis[index] = ci;
			}
		}
		__syncthreads();
	}

	__device__ int printNofIndices(int cc, bool* inQ, int N)
	{
		if(threadIdx.x == 0)
		{
			int n=0;
			for(int i=0; i<N; i++)
			{
				if(inQ[i])
					n++;
			}
			printf("cluster: %d\t size: %d\n", cc, n);
		}
		__syncthreads();
	}

	__global__ void cluster(bool* isProcessed, bool* inQ, float* dx, float*dm, int* cis, int nsample, int N)
	{
		//initialisation of arrays
		__shared__ bool b_isProcessed[MAX_CLUSTER_BUFFER_SIZE];
		b_isProcessed[threadIdx.x] = false;
		initArray(isProcessed, false, nsample, N);
		initArray(inQ, false, nsample, N);
		int currentCluster = 0;

		__syncthreads();
		while(!b_isProcessed[0])
		{
			setFirstNotProcessedIndice(dx, isProcessed, inQ, N);
			hasAllProcessedQ(dx, isProcessed, inQ, b_isProcessed, nsample, N);
			while(!b_isProcessed[0])
			{
				process(dx, isProcessed, inQ, dm, nsample, N);
				hasAllProcessedQ(dx, isProcessed, inQ, b_isProcessed, nsample, N);
			}
			setIndices(currentCluster, cis, inQ, nsample, N);
			printNofIndices(currentCluster,inQ,N);
			initArray(inQ, false, nsample, N);
			hasAllProcessedP(dx, isProcessed, b_isProcessed, nsample, N);
			currentCluster++;
		}

	}


}
#endif