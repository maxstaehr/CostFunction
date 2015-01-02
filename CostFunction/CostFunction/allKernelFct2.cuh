#ifndef ALLKERNELFCT2_CUH_
#define ALLKERNELFCT2_CUH_

#include "global.h"
#include "math_constants.h"
#include <curand_kernel.h>

namespace cuda_calc2{

	#define EPSILON 0.000001


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
		  inv_det = 1.f / det;
 
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
		  if(v < 0.f || u + v  > 1.f) return 0;
 
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



	__global__ void calcMiddlePoint(float* Dx, float* Dy, float* Dz, int nP, float* a_x, float* a_y, float* a_z){
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
			if(curIndex < nP)
			{
				//check if the distance value is within limits
				
				if(Dx[curIndex] < 7.0)
				{
					calcNewAverage(avg_x+threadIdx.x, avg_w[threadIdx.x], Dx[curIndex]);					
					calcNewAverage(avg_y+threadIdx.x, avg_w[threadIdx.x], Dy[curIndex]);					
					calcNewAverage(avg_z+threadIdx.x, avg_w[threadIdx.x], Dz[curIndex]);

					avg_w[threadIdx.x]++;
				}
			}
			
		}
		__syncthreads();
		
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
			a_x[0] = avg_x[0];
			a_y[0] = avg_y[0];
			a_z[0] = avg_z[0];
		}


	}

	__device__ bool isInBB(float* h, float* dim, float* p)
	{
		float x = h[0]*p[0]+h[1]*p[1]+h[2]*p[2]+h[3];
		float y = h[4]*p[0]+h[5]*p[1]+h[6]*p[2]+h[7];
		float z = h[8]*p[0]+h[9]*p[1]+h[10]*p[2]+h[11];
		return x>0 && y>0 && z>0 && x<dim[0] && y<dim[1] && z<dim[2];
	}

	__global__ void raytraceVertices(					float* xi, float* yi, float* zi,
														int* fx, int* fy, int* fz, int nF,
														float* bb_H, float* bb_D, int nBB, 
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
		int threadId = blockId*blockSize+threadIDinBlock;
		float d = FLT_MAX;		
		float dr;
		int hit;



		//calculating current origin
		float o[3];
		float di[3];
		calculateOrginAndDirection(camPos_H, camRot_H, camRayX[threadId], camRayY[threadId], camRayZ[threadId], o, di);

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
					//dirty hack because the facesnumeration comes from
					//matlab which start 1 as first indice
					//therefore we have to substract each face indice by 1
					f = fx[numOfVerticesCopied + t]-1;
					v1[3*t+0] = xi[f];
					v1[3*t+1] = yi[f];
					v1[3*t+2] = zi[f];

					f = fy[numOfVerticesCopied + t]-1;
					v2[3*t+0] = xi[f];
					v2[3*t+1] = yi[f];
					v2[3*t+2] = zi[f];

					f = fz[numOfVerticesCopied + t]-1;
					v3[3*t+0] = xi[f];
					v3[3*t+1] = yi[f];
					v3[3*t+2] = zi[f];

				}
			}
			numOfVerticesCopied += numberOfValidVertices;
			__syncthreads();

			//raytrace and check minum distance
			for(int i=0; i<numberOfValidVertices; i++)
			{				
				hit = triangle_intersection(	
										v1+3*i,  // Triangle vertices
										v2+3*i,
										v3+3*i,
										o,  //Ray origin
										di,  //Ray direction
										&dr);

				if(hit){
					d = fminf(d, dr);
				}
				
			}

		}

		float p[3];
		p[0] = o[0]+d*di[0];
		p[1] = o[1]+d*di[1];
		p[2] = o[2]+d*di[2];

		bool isInBox = false;
		for(int i=0; i<nBB; i++)
		{
			isInBox |= isInBB(bb_H+i*16, bb_D+i*3, p);
		}

		if(d < 7.0 && !isInBox)
		{
			//setting real value and calculate weighted average
			Dx[blockId*blockSize+threadIDinBlock] = p[0];
			Dy[blockId*blockSize+threadIDinBlock] = p[1];
			Dz[blockId*blockSize+threadIDinBlock] = p[2];
		}else
		{
			//setting to error value
			Dx[blockId*blockSize+threadIDinBlock] = INF_DIST;
			Dy[blockId*blockSize+threadIDinBlock] = INF_DIST;
			Dz[blockId*blockSize+threadIDinBlock] = INF_DIST;
		}


	}

	__device__ float distanceToEllipse(float* v)
	{
		float vp[3];
		float t1 = powf(v[0]/human_rx, 2.0);
		float t2 = powf(v[1]/human_ry, 2.0);
		float t3 = powf(v[2]/human_rz, 2.0);
		float mag = sqrtf(t1+t2+t3);
		vp[0] = v[0]/mag;
		vp[1] = v[1]/mag;
		vp[2] = v[2]/mag;
		return sqrtf(powf(vp[0]-v[0],2.0)+powf(vp[1]-v[1],2.0)+powf(vp[2]-v[2],2.0));
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

	#define	tailor_a (1.927683761067405f)
	#define	tailor_b (-12.914812798334909f)
	#define tailor_min (0.050819128176881f)

	__device__ float calculateProbabilityOfDetection(float meanDistance)
	{
		if(meanDistance < tailor_min){
			return 1.0f;
		}else{
			return tailor_a*expf(tailor_b*meanDistance);
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

	__device__ float generateDepthValue(float* coeffs, curandState_t *state)
	{
		float rand = curand_normal (state);
	}
		

	__global__ void distanceToEllipseModel(float* Dx, float* Dy, float* Dz, int nP,
									  float* R, float* Fx, float* Fy,
									  float* cx, float* cy, float* cz,									  
									  float* prop)
	{
		__shared__ float vx[POINT_BUFFER_SIZE];
		__shared__ float vy[POINT_BUFFER_SIZE];
		__shared__ float vz[POINT_BUFFER_SIZE];

		//int threadId =  blockIdx.x *blockDim.x + threadIdx.x;
		//int threadId =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
		//	+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
		//	+ threadIdx.z * blockDim.y * blockDim.x
		//	+ threadIdx.y * blockDim.x
		//	+ threadIdx.x;

		int blockSize = blockDim.x * blockDim.y * blockDim.z;
		int threadIDinBlock = threadIdx.z * blockDim.y * blockDim.x + threadIdx.y * blockDim.x + threadIdx.x;
		int blockId = blockIdx.x + blockIdx.y * gridDim.x + gridDim.x * gridDim.y * blockIdx.z;
		int threadId = blockId*blockSize+threadIDinBlock;


		//determining number of vertices to be copied by each thread into buffer
		int nItePerThread = (int)((POINT_BUFFER_SIZE/blockSize)+1);

		//loading vertex data into local buffer
		int index,i;
		int numOfPointsCopied = 0;
		int numberOfValidPoints;

		float	dist = 0.0f;
		int		w = 0;
		float	p[3];


		

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
				if(vx[i] < INF_DIST)
				{
					p[0] = vx[i]-(cx[0]+Fx[threadId]);
					p[1] = vy[i]-(cy[0]+Fy[threadId]);
					p[2] = vz[i]-(cz[0]);

					rotatePoint(R+9*threadId, p);
					dist += distanceToEllipse(p);
					w++;
				}
			}
			__syncthreads();
			
		}
		dist = dist/w;
		prop[threadId] = calculateProbabilityOfDetection(dist);
		
	}

	__global__ void calculateMaxProb(float* prob, int n, float* maxp)
	{
		__shared__ float max_buffer[MAX_BUFFER_SIZE];

		int nItePerThread = (int)((n/MAX_BUFFER_SIZE)+1);
		int curIndex;
		int i;

		

		float localMax = 0.0f;
		for(i=0; i<nItePerThread; i++)
		{
			curIndex = threadIdx.x*nItePerThread+i;
			//check if the iteration is still within limits
			if(curIndex < n)
			{
				localMax = fmaxf(prob[curIndex],  localMax);
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
				max_buffer[threadIdx.x] = fmaxf(max_buffer[threadIdx.x],  max_buffer[threadIdx.x+i]);
			}
		}
		maxp[0] = max_buffer[0];

	}


}
#endif