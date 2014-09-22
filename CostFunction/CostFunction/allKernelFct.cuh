/*
 * allKernelFct.cuh
 *
 *  Created on: 19.06.2014
 *      Author: tsdf
 */

#ifndef ALLKERNELFCT_CUH_
#define ALLKERNELFCT_CUH_

#include "global.h"






namespace cuda_calc{




	/*__forceinline__ */__device__ bool isBitSet(const unsigned int* const a, unsigned int index)
	{
		unsigned int mask = 0x80000000;
		unsigned char ai =  (unsigned char)index&(0x0000001f);
		mask >>= ai;
		unsigned int access = index>>5;
		unsigned int val = a[access];
		return  val & mask ? true:false;

	}

	bool isBitSetHost(const unsigned int* const a, unsigned int index)
	{
		unsigned int mask = 0x80000000;
		unsigned char ai =  (unsigned char)index&(0x0000001f);
		mask >>= ai;
		return  a[index>>5] & mask ? true:false;

	}


	__forceinline__ __device__ void setBit(unsigned int* const a, unsigned int index)
	{
		unsigned int mask = 0x80000000;
		unsigned char ai =  (unsigned char)index&(0x0000001f);
		mask >>= ai;
		unsigned int *adr = &a[index>>5];
		atomicOr(adr,mask);

	}

	__forceinline__ __device__ void setBitAnd(unsigned int* const a, unsigned int index)
	{
		unsigned int mask = 0x80000000;
		unsigned char ai =  (unsigned char)index&(0x0000001f);
		mask >>= ai;
		unsigned int *adr = &a[index>>5];
		atomicAnd(adr,mask);

	}

	__forceinline__ __device__ void clearBitAnd(unsigned int* const a, unsigned int index)
	{
		unsigned int mask = 0x80000000;
		unsigned char ai =  (unsigned char)index&(0x0000001f);
		mask >>= ai;
		unsigned int *adr = &a[index>>5];
		atomicAnd(adr,mask);

	}


	void setBitHost(unsigned int* const a, unsigned int index)
	{
		unsigned int mask = 0x80000000;
		unsigned char ai =  (unsigned char)index&(0x0000001f);
		mask >>= ai;
		unsigned int *adr = &a[index>>5];
		unsigned int b =   *adr;
		b |= mask;
		(*adr) = b;
	}





	__forceinline__ __device__ void ray_trace_ray(float rv1, float rv2, float rv3,
								   float xpos, float ypos, float zpos,
								   const unsigned int* const H, const unsigned int* const E,
								   float* HDM, float* EDM)
	{
			//**method
		int x = (int)round(((xpos-WS_X_MIN)/(WS_X_MAX-WS_X_MIN))*(WS_NX-1)+1);
		int y = (int)round(((ypos-WS_Y_MIN)/(WS_Y_MAX-WS_Y_MIN))*(WS_NY-1)+1);
		int z = (int)round(((zpos-WS_Z_MIN)/(WS_Z_MAX-WS_Z_MIN))*(WS_NZ-1)+1);


		float xdiff = (x-1)*V_DX + WS_X_MIN;
		float ydiff = (y-1)*V_DY + WS_Y_MIN;
		float zdiff = (z-1)*V_DZ + WS_Z_MIN;

		bool isHDMset = false, isEDMset=false;

		float tMaxX, tMaxY, tMaxZ;
		int stepX, stepY, stepZ;
		if(rv1 < 0 )
		{
			stepX = -1;
			tMaxX = fabsf((xdiff - 0.5*V_DX - xpos)/rv1);
		}
		else
		{
			stepX = 1;
			tMaxX = fabsf((xdiff + 0.5*V_DX - xpos)/rv1);
		}


		if(rv2 < 0 )
		{
			stepY = -1;
			tMaxY = fabsf((ydiff - 0.5*V_DY - ypos)/rv2);
		}
		else
		{
			stepY = 1;
			tMaxY = fabsf((ydiff + 0.5*V_DY - ypos)/rv2);
		}


		if(rv3 < 0 )
		{
			stepZ = -1;
			tMaxZ = fabsf((zdiff - 0.5*V_DZ - zpos)/rv3);
		}
		else
		{
			stepZ = 1;
			tMaxZ = fabsf((zdiff + 0.5*V_DZ - zpos)/rv3);
		}

		float  tDeltaX, tDeltaY, tDeltaZ;
		tDeltaX = fabsf(V_DX/rv1);
		tDeltaY = fabsf(V_DY/rv2);
		tDeltaZ = fabsf(V_DZ/rv3);

		if(	x < 1 || x > WS_NX ||
			y < 1 || y > WS_NY ||
			z < 1 || z > WS_NZ )
		{
			*HDM = MAX_DEPTH;
			*EDM = MAX_DEPTH;
			return;
		}


		while(1)
		{
			if(tMaxX < tMaxY)
			{
				if(tMaxX < tMaxZ)
				{
					x = x +stepX;
					if(x < 1 || x > WS_NX)
					{
						//HDM[idx] = MAX_DEPTH;
						if(!isHDMset)
						{
							*HDM = MAX_DEPTH;
							isHDMset = true;
						}

						if(!isEDMset)
						{
							*EDM = MAX_DEPTH;
							isEDMset = true;
						}
						if(isEDMset && isHDMset)
						{
							return;
						}
						//return MAX_DEPTH*MAX_DEPTH;
					}
					tMaxX = tMaxX + tDeltaX;
				}
				else
				{
					z = z + stepZ;
					if(z < 1 || z > WS_NZ)
					{
						//HDM[idx] = MAX_DEPTH;
						if(!isHDMset)
						{
							*HDM = MAX_DEPTH;
							isHDMset = true;
						}

						if(!isEDMset)
						{
							*EDM = MAX_DEPTH;
							isEDMset = true;
						}
						if(isEDMset && isHDMset)
						{
							return;
						}
						//return MAX_DEPTH*MAX_DEPTH;
					}
					tMaxZ = tMaxZ + tDeltaZ;
				}
			}
			else
			{
				if(tMaxY < tMaxZ)
				{
					y = y + stepY;
					if(y < 1 || y > WS_NY)
					{
						//HDM[idx] = MAX_DEPTH;
						if(!isHDMset)
						{
							*HDM = MAX_DEPTH;
							isHDMset = true;
						}

						if(!isEDMset)
						{
							*EDM = MAX_DEPTH;
							isEDMset = true;
						}
						if(isEDMset && isHDMset)
						{
							return;
						}
						//return MAX_DEPTH*MAX_DEPTH;
					}
					tMaxY = tMaxY + tDeltaY;
				}
				else
				{
					z = z + stepZ;
					if(z < 1 || z > WS_NZ)
					{
						//HDM[idx] = MAX_DEPTH;
						if(!isHDMset)
						{
							*HDM = MAX_DEPTH;
							isHDMset = true;
						}

						if(!isEDMset)
						{
							*EDM = MAX_DEPTH;
							isEDMset = true;
						}
						if(isEDMset && isHDMset)
						{
							return;
						}
						//return MAX_DEPTH*MAX_DEPTH;
					}
					tMaxZ = tMaxZ + tDeltaZ;
				}
			}


			int id = int((z-1)*WS_NY*WS_NX + (x-1)*WS_NY + (y-1));

			float dx, dy, dz, d;
			if(isBitSet(H, id))
			{
				xdiff = (x-1)*V_DX + WS_X_MIN;
				ydiff = (y-1)*V_DY + WS_Y_MIN;
				zdiff = (z-1)*V_DZ + WS_Z_MIN;
				dx = xdiff - xpos;
				dy = ydiff - ypos;
				dz = zdiff - zpos;
				d = sqrtf(dx*dx + dy*dy + dz*dz);
				//d = dx*dx + dy*dy + dz*dz;

				if(d > MIN_DEPTH)
				{
					if(!isHDMset)
					{
						*HDM = d;
						isHDMset = true;
					}
					//HDM[idx] = d;
					//return d;
				}
			}


			if(isBitSet(E, id))
			{
				xdiff = (x-1)*V_DX + WS_X_MIN;
				ydiff = (y-1)*V_DY + WS_Y_MIN;
				zdiff = (z-1)*V_DZ + WS_Z_MIN;
				dx = xdiff - xpos;
				dy = ydiff - ypos;
				dz = zdiff - zpos;
				d = sqrtf(dx*dx + dy*dy + dz*dz);
				//d = dx*dx + dy*dy + dz*dz;
				if(d > MIN_DEPTH)
				{
					if(!isEDMset)
					{
						*EDM = d;
						isEDMset = true;
					}
					//HDM[idx] = d;
					//return d;
				}
			}


			if(isEDMset && isHDMset)
			{
				return;
			}


		}

	}


	__global__ void raytrace_shared(const unsigned int* const E, const unsigned int* const H, const float* Mia, const float* CP,
		float* EDM, float* HDM)
	{
		__shared__ float MI_shared[9*CAM_ITE];
		__shared__ float CP_shared[3*CAM_ITE];


		unsigned int idx;
		int u,v;
		float rv1, rv2, rv3, xpos, ypos, zpos;



		/////********************************************************************************/



		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		//we loop over robot and human position to avoid
		//unnecessary load to shared mem



		for(unsigned int i=0; i<8; i++)
		{
			if(threadIdx.x*8+i < CAM_ITE * 9)
			{
				MI_shared[threadIdx.x*8+i] = Mia[threadIdx.x*8+i];
			}
		}

		for(unsigned int i=0; i<3; i++)
		{
			if(threadIdx.x*3+i < CAM_ITE*3)
			{
				CP_shared[threadIdx.x*3+i] = CP[threadIdx.x*3+i];
			}
		}




		__syncthreads();

		for(unsigned int i=0; i<CAM_ITE; i++)
		{
			u = (int)fmod((float)idx, (float)RES_X);
			v = (int)floor(((float)idx)/((float)RES_X));

			rv1 = MI_shared[i*9+0]*u + MI_shared[i*9+1]*v + MI_shared[i*9+2];
			rv2 = MI_shared[i*9+3]*u + MI_shared[i*9+4]*v + MI_shared[i*9+5];
			rv3 = MI_shared[i*9+6]*u + MI_shared[i*9+7]*v + MI_shared[i*9+8];

			xpos = CP[i*3+0];
			ypos = CP[i*3+1];
			zpos = CP[i*3+2];


			ray_trace_ray(rv1, rv2, rv3, xpos, ypos, zpos, H, E, &HDM[i*IMG_SIZE+idx], &EDM[i*IMG_SIZE+idx]);
			//HDM[i*IMG_SIZE+idx] = ray_trace_ray(rv1, rv2, rv3, xpos, ypos, zpos, H, E, &HDM[i*IMG_SIZE+idx], &EDM[i*IMG_SIZE+idx]);
			//EDM[i*IMG_SIZE+idx] = ray_trace_ray(rv1, rv2, rv3, xpos, ypos, zpos, E);
		}

	}

	__global__ void project_voxel_into_ws(const unsigned int* const H,
							   const float* const EDM, const float* const HDM,
							   const float* const C, const float* const CP,
							   unsigned int rp_N, unsigned int hp_N,
							   unsigned int* const FA, unsigned int* const HS)
	{
		__shared__ float CP_shared[3*CAM_ITE];
		__shared__ float C_shared[12*CAM_ITE];


		unsigned int threads_per_block = blockDim.x * blockDim.y * blockDim.z;
		unsigned int threadIDinBlock = threadIdx.z * blockDim.y * blockDim.x + threadIdx.y * blockDim.x + threadIdx.x;





		unsigned int hprp_index, ws_offset;
		unsigned int depthImageOffset;
		unsigned int id, idx, idy, idz;
		float x,y,z,xi,yi,zi,d, dx, dy, dz, xcam, ycam, zcam;
		int u,v,ic;

		idx = blockIdx.x *blockDim.x + threadIdx.x;
		idy = blockIdx.y *blockDim.y + threadIdx.y;
		idz = blockIdx.z *blockDim.z + threadIdx.z;

		id = idz*WS_NY*WS_NX + idx*WS_NY + idy;

		x = V_DX*idx + WS_X_MIN;
		y = V_DY*idy + WS_Y_MIN;
		z = V_DZ*idz + WS_Z_MIN;



		for(unsigned int i=0; i<6; i++)
		{
			C_shared[threadIDinBlock*6+i] = C[threadIDinBlock*6+i];
		}

		for(unsigned int i=0; i<2; i++)
		{
			if(threadIDinBlock*2+i < 3*CAM_ITE)
			{
				CP_shared[threadIDinBlock*2+i] = CP[threadIDinBlock*2+i];
			}
		}

		__syncthreads();






		//syncthreads before calculating
		for(unsigned int i=0; i<CAM_ITE; i++)
		{
			xi = C_shared[12*i+0]*x + C_shared[12*i+1]*y + C_shared[12*i+2]*z +C_shared[12*i+3];
			yi = C_shared[12*i+4]*x + C_shared[12*i+5]*y + C_shared[12*i+6]*z +C_shared[12*i+7];
			zi = C_shared[12*i+8]*x + C_shared[12*i+9]*y + C_shared[12*i+10]*z +C_shared[12*i+11];

			xcam = CP[3*i+0];
			ycam = CP[3*i+1];
			zcam = CP[3*i+2];





			u = floor(xi/zi);
			v = floor(yi/zi);
			ic = v*RES_X + u;

			bool isInHuman =  isBitSet(H, id);
			if( u >= 0 &&  u < RES_X && v >= 0 && v < RES_Y && zi > 0)
			{
				dx = x - xcam;
				dy = y - ycam;
				dz = z - zcam;


				d = sqrtf(dx*dx + dy*dy + dz*dz);
				//d = dx*dx + dy*dy + dz*dz;
				ic = v*RES_X + u;


				//the projection of the voxel into to image plane
				//just needs to be done for a single image
				//that it is just a comparision of values
				if(isInHuman && EDM[i*IMG_SIZE+ic] <= HDM[i*IMG_SIZE+ic] )
				{
					setBit(&HS[i*SHARED_MEM_WS_INT], id);
					//setBitAnd(&HS[i*SHARED_MEM_WS_INT], id);
				}

				if(d < HDM[i*IMG_SIZE+ic] && d < EDM[i*IMG_SIZE+ic])
				{
					setBit(&FA[i*SHARED_MEM_WS_INT], id);
				}
			}else
			{
				//not in the field of view
				if(isInHuman)
				{
					setBit(&HS[i*SHARED_MEM_WS_INT], id);
					//setBitAnd(&HS[i*SHARED_MEM_WS_INT], id);					
				}
			}
		}





	}




	__global__ void fuseGrids_shared(const unsigned int* const HSinput, const unsigned int* const FAinput, const float* const KSDF, float* const HS_costs, float* const FA_costs)
	{

		__shared__ double shared_costs_hs[ACCUM_N_WS];
		__shared__ double shared_costs_fa[ACCUM_N_WS];




		unsigned int b0, b1, b2, b3, i;
		float c0, c1, c2, c3;
		float costs;

		int blockId = blockIdx.x
					+ blockIdx.y * gridDim.x
					+ gridDim.x * gridDim.y * blockIdx.z;



		b0 = HSinput[blockId*SHARED_MEM_WS_INT+threadIdx.x*4+0];
		b1 = HSinput[blockId*SHARED_MEM_WS_INT+threadIdx.x*4+1];
		b2 = HSinput[blockId*SHARED_MEM_WS_INT+threadIdx.x*4+2];
		b3 = HSinput[blockId*SHARED_MEM_WS_INT+threadIdx.x*4+3];
	

		c0 = 0.0f;
		c1 = 0.0f;
		c2 = 0.0f;
		c3 = 0.0f;
		for(i=0; i<32; i++)
		{
			if(isBitSet(&b0,i))
			{
				c0 += KSDF[threadIdx.x*128+0+i];
			}

			if(isBitSet(&b1,i))
			{
				c1 += KSDF[threadIdx.x*128+32+i];
			}

			if(isBitSet(&b2,i))
			{
				c2 += KSDF[threadIdx.x*128+64+i];
			}

			if(isBitSet(&b3,i))
			{
				c3 += KSDF[threadIdx.x*128+96+i];
			}
		}
		shared_costs_hs[threadIdx.x] = c0+c1+c2+c3;




		b0 = FAinput[blockId*SHARED_MEM_WS_INT+threadIdx.x*4+0];
		b1 = FAinput[blockId*SHARED_MEM_WS_INT+threadIdx.x*4+1];
		b2 = FAinput[blockId*SHARED_MEM_WS_INT+threadIdx.x*4+2];
		b3 = FAinput[blockId*SHARED_MEM_WS_INT+threadIdx.x*4+3];

		c0 = 0.0;
		c1 = 0.0;
		c2 = 0.0;
		c3 = 0.0;
		for(i=0; i<32; i++)
		{
			if(isBitSet(&b0,i))
			{
				c0 += KSDF[threadIdx.x*128+0+i];
			}

			if(isBitSet(&b1,i))
			{
				c1 += KSDF[threadIdx.x*128+32+i];
			}

			if(isBitSet(&b2,i))
			{
				c2 += KSDF[threadIdx.x*128+64+i];
			}

			if(isBitSet(&b3,i))
			{
				c3 += KSDF[threadIdx.x*128+96+i];
			}
		}
		shared_costs_fa[threadIdx.x] = c0+c1+c2+c3;



		////starting reduction
		////1024 threads turn out to 10 iterations
		for (i = ACCUM_N_WS / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{
				shared_costs_hs[threadIdx.x] += shared_costs_hs[threadIdx.x + i];
				shared_costs_fa[threadIdx.x] += shared_costs_fa[threadIdx.x + i];
			}
		}
		__syncthreads();
		if (threadIdx.x == 0)
		{
			HS_costs[blockId] = shared_costs_hs[0];
			FA_costs[blockId] = shared_costs_fa[0];
		}

	}

	__global__ void pclintows_shared(unsigned int * const O,
		const float* const X, const float* const Y, const float* const Z, int numEl,
		const float* H)
	{
		unsigned int idx;
		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;


		int indexPoint;
		int id;
		float xt;
		float yt;
		float zt;
		int xid;
		int yid;
		int zid;

		__shared__ float X_shared[MAX_PCL_SIZE];
		__shared__ float Y_shared[MAX_PCL_SIZE];
		__shared__ float Z_shared[MAX_PCL_SIZE];
		__shared__ float H_shared[16];


		for(unsigned int i=0; i<6; i++)
		{

			if(6*threadIdx.x+i < numEl)
			{
				X_shared[6*threadIdx.x+i] = X[6*threadIdx.x+i];
				Y_shared[6*threadIdx.x+i] = Y[6*threadIdx.x+i];
				Z_shared[6*threadIdx.x+i] = Z[6*threadIdx.x+i];
			}
		}

		if(threadIdx.x < 16)
		{
			H_shared[threadIdx.x] = H[threadIdx.x];
		}


		__syncthreads();

		for (indexPoint=0; indexPoint<numEl; indexPoint++)
		{
			xt = H_shared[0]*X_shared[indexPoint] + H_shared[1]*Y_shared[indexPoint] + H_shared[2]*Z_shared[indexPoint] + H_shared[3];
			yt = H_shared[4]*X_shared[indexPoint] + H_shared[5]*Y_shared[indexPoint] + H_shared[6]*Z_shared[indexPoint] + H_shared[7];
			zt = H_shared[8]*X_shared[indexPoint] + H_shared[9]*Y_shared[indexPoint] + H_shared[10]*Z_shared[indexPoint] + H_shared[11];

			if(	xt < WS_X_MIN || xt > WS_X_MAX ||
				yt < WS_Y_MIN || yt > WS_Y_MAX ||
				zt < WS_Z_MIN || zt > WS_Z_MAX)
			{
				continue;
			}

			xid = (int)round(((xt-WS_X_MIN)/WS_X_DIM)*(WS_NX-1)+1);
			yid = (int)round(((yt-WS_Y_MIN)/WS_Y_DIM)*(WS_NY-1)+1);
			zid = (int)round(((zt-WS_Z_MIN)/WS_Z_DIM)*(WS_NZ-1)+1);
			id = (unsigned int)((zid-1)*WS_NY*WS_NX + (xid-1)*WS_NY + (yid-1));

			if(idx == id)
			{
				setBit(O, idx);
				return;
			}
		}

	}

	__global__ void calcCalcCosts(const float* const d_hs_costs, const float* const d_fa_costs, double* const d_costs)
	{
		unsigned int idx;
		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

	//	d_costs[idx] =  (d_hs_costs[idx] - (d_fa_costs[idx]/WS_ID_SIZE))/WS_ID_SIZE;
		d_costs[idx] =  d_hs_costs[idx];///WS_ID_SIZE;// - (d_fa_costs[idx]/WS_ID_SIZE))/WS_ID_SIZE;

	}



	__global__ void visibilty_costs(const unsigned int* const E, const unsigned int* const H, const float* C, const float* CP,
		unsigned int* const HS,  unsigned int* const FA)
	{
		__shared__ float CP_shared[256*3];
		__shared__ float C_shared[256*12];
		__shared__ int HDM[IMG_SIZE];
		__shared__ int EDM[IMG_SIZE];

		int id, u, v, ic, d_int, idx, idy, idz, xindex, yindex, zindex;;
		float xi, yi, zi, d, x, y, z;

		//block id determines the camera
		int blockId = blockIdx.x + blockIdx.y * gridDim.x + gridDim.x * gridDim.y * blockIdx.z;

		//loading into shared memory
		for(unsigned int i=0; i<2; i++)
		{
			if(threadIdx.x*2+i < 256*3)
			{
				CP_shared[threadIdx.x*2+i] = CP[threadIdx.x*2+i];
			}
		}

		for(unsigned int i=0; i<6; i++)
		{
			C_shared[6*threadIdx.x+i] = C[12*blockId+6*threadIdx.x+i];
		}


		d_int =(int)( MAX_DEPTH * INT_FACTOR);
		id =  (threadIdx.z * (blockDim.x * blockDim.y))
			 + (threadIdx.y * blockDim.x)
			 + threadIdx.x;

		if(id < IMG_SIZE)
		{
			HDM[id] = d_int;
			EDM[id] = d_int;
		}

		__syncthreads();



		for(unsigned int ite = 0; ite < 256; ite++)
		{

			//each thread works on a 8x8x8 grid

			for(int xite=0; xite<8; xite++)
			{
				for(int yite=0; yite<8; yite++)
				{
					for(int zite=0; zite<8; zite++)
					{


						/*************************************************************/
						//checking if on the image plane

						xindex = threadIdx.x*8+xite;
						yindex = threadIdx.y*8+yite;
						zindex = threadIdx.z*8+zite;

						x = V_DX*xindex + WS_X_MIN;
						y = V_DY*yindex + WS_Y_MIN;
						z = V_DZ*zindex + WS_Z_MIN;

						xi = C_shared[0]*x + C_shared[1]*y + C_shared[2]*z +C_shared[3];
						yi = C_shared[4]*x + C_shared[5]*y + C_shared[6]*z +C_shared[7];
						zi = C_shared[8]*x + C_shared[9]*y + C_shared[10]*z +C_shared[11];


						u = floor(xi/zi);
						v = floor(yi/zi);
						ic = v*RES_X + u;


						if( u >= 0 &&  u < RES_X && v >= 0 && v < RES_Y && zi > 0)
						{

						/*************************************************************/
							id = zindex*WS_NY*WS_NX + xindex*WS_NY + yindex;

							xi = x - CP_shared[0];
							yi = y - CP_shared[1];
							zi = z - CP_shared[2];
							d = xi*xi + yi*yi + zi*zi;
							d_int = (int)(d*INT_FACTOR);
							ic = v*RES_X + u;

							if(isBitSet(H, id))
							{
								//atomicMin(&HDM[ic], d_int);
							}

							if(isBitSet(E, id))
							{
								//atomicMin(&EDM[ic], d_int);
							}
						}
					}
				}
			}

			__syncthreads();

			for(int xite=0; xite<8; xite++)
			{
				for(int yite=0; yite<8; yite++)
				{
					for(int zite=0; zite<8; zite++)
					{


						/*************************************************************/
						//checking if on the image plane

						xindex = threadIdx.x*8+xite;
						yindex = threadIdx.y*8+yite;
						zindex = threadIdx.z*8+zite;

						x = V_DX*xindex + WS_X_MIN;
						y = V_DY*yindex + WS_Y_MIN;
						z = V_DZ*zindex + WS_Z_MIN;

						xi = C_shared[0]*x + C_shared[1]*y + C_shared[2]*z +C_shared[3];
						yi = C_shared[4]*x + C_shared[5]*y + C_shared[6]*z +C_shared[7];
						zi = C_shared[8]*x + C_shared[9]*y + C_shared[10]*z +C_shared[11];


						u = floor(xi/zi);
						v = floor(yi/zi);
						ic = v*RES_X + u;


						if( u >= 0 &&  u < RES_X && v >= 0 && v < RES_Y && zi > 0)
						{

						/*************************************************************/
							id = zindex*WS_NY*WS_NX + xindex*WS_NY + yindex;

							xi = x - CP_shared[0];
							yi = y - CP_shared[1];
							zi = z - CP_shared[2];
							d = xi*xi + yi*yi + zi*zi;
							d_int = (int)(d*INT_FACTOR);
							ic = v*RES_X + u;

							if(d_int >= HDM[ic] && EDM[ic] > HDM[ic] && isBitSet(H, id))
							{
								setBit(HS, id);
							}

							if(d_int < HDM[ic] && d_int < EDM[ic])
							{
								setBit(FA, id);
							}
						}
					}
				}
			}
		}
	}




	__global__ void pclintows(unsigned char * O,
		const float* X, const float* Y, const float* Z, int numEl,
		const float* H,
		float xmin, float xmax, float ymin, float ymax, float zmin, float zmax,
		int nx, int ny, int nz)
	{
		unsigned int idx;
		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;


		int indexPoint;
		int id;
		float xt;
		float yt;
		float zt;
		int xid;
		int yid;
		int zid;


		for (indexPoint=0; indexPoint<numEl; indexPoint++)
		{
			xt = H[0]*X[indexPoint] + H[1]*Y[indexPoint] + H[2]*Z[indexPoint] + H[3];
			yt = H[4]*X[indexPoint] + H[5]*Y[indexPoint] + H[6]*Z[indexPoint] + H[7];
			zt = H[8]*X[indexPoint] + H[9]*Y[indexPoint] + H[10]*Z[indexPoint] + H[11];

			xid = (int)round(((xt-xmin)/(xmax-xmin))*(nx-1))+1;
			yid = (int)round(((yt-ymin)/(ymax-ymin))*(ny-1))+1;
			zid = (int)round(((zt-zmin)/(zmax-zmin))*(nz-1))+1;
			id = (unsigned int)((zid-1)*ny*nx + (xid-1)*ny + (yid-1));

			if(idx != id || xt < xmin || xt > xmax || yt < ymin || yt > ymax || zt < zmin || zt > zmax)
				continue;

			O[idx] = ((unsigned char)1);
			return;
		}

	}


	__global__ void raytrace( float * DM, /*unsigned char* O,*/ const float* ws_x, const float* ws_y, const float* ws_z,
		const unsigned char* OG, const float* Mia, int resx, int resy,
		float xmin, float xmax, float ymin, float ymax, float zmin, float zmax,
		int nx, int ny, int nz,
		float xpos, float ypos, float zpos,
		float minimumDist)
	{
		unsigned int idx;
		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;


		float xvoxelwidth = (xmax -xmin)/(nx-1);
		float yvoxelwidth = (ymax -ymin)/(ny-1);
		float zvoxelwidth = (zmax -zmin)/(nz-1);

		int xid = (int)round(((xpos-xmin)/(xmax-xmin))*(nx-1)+1);
		int yid = (int)round(((ypos-ymin)/(ymax-ymin))*(ny-1)+1);
		int zid = (int)round(((zpos-zmin)/(zmax-zmin))*(nz-1)+1);


		int x;
		int y;
		int z;

		float rv1;
		float rv2;
		float rv3;

		float tMaxX;
		float tMaxY;
		float tMaxZ;

		float tDeltaX;
		float tDeltaY;
		float tDeltaZ;

		int stepX;
		int stepY;
		int stepZ;

		int id;

		float d;
		float dx;
		float dy;
		float dz;



		int u;
		int v;




		u = (int)fmod((float)idx, (float)resx);;
		v = (int)floor(((float)idx)/((float)resx));

		x = xid;
		y = yid;
		z = zid;


		id = int((z-1)*ny*nx + (x-1)*ny + (y-1));
		rv1 = Mia[0]*u + Mia[1]*v + Mia[2];
		rv2 = Mia[3]*u + Mia[4]*v + Mia[5];
		rv3 = Mia[6]*u + Mia[7]*v + Mia[8];


		DM[idx] = MAX_DEPTH;

		if(rv1 < 0 )
		{
			stepX = -1;
			tMaxX = fabsf((ws_x[id] - 0.5*xvoxelwidth - xpos)/rv1);
		}
		else
		{
			stepX = 1;
			tMaxX = fabsf((ws_x[id] + 0.5*xvoxelwidth - xpos)/rv1);
		}


		if(rv2 < 0 )
		{
			stepY = -1;
			tMaxY = fabsf((ws_y[id] - 0.5*yvoxelwidth - ypos)/rv2);
		}
		else
		{
			stepY = 1;
			tMaxY = fabsf((ws_y[id] + 0.5*yvoxelwidth - ypos)/rv2);
		}


		if(rv3 < 0 )
		{
			stepZ = -1;
			tMaxZ = fabsf((ws_z[id] - 0.5*zvoxelwidth - zpos)/rv3);
		}
		else
		{
			stepZ = 1;
			tMaxZ = fabsf((ws_z[id] + 0.5*zvoxelwidth - zpos)/rv3);
		}


		tDeltaX = fabsf(xvoxelwidth/rv1);
		tDeltaY = fabsf(yvoxelwidth/rv2);
		tDeltaZ = fabsf(zvoxelwidth/rv3);

		if(	x < 1 || x > nx ||
		y < 1 || y > ny ||
		z < 1 || z > nz )
		{
			DM[idx] = MAX_DEPTH;
			return;
		}


		while(1)
		{
			if(tMaxX < tMaxY)
			{
				if(tMaxX < tMaxZ)
				{
					x = x +stepX;
					if(x < 1 || x > nx)
					{
						DM[idx] = MAX_DEPTH;
						return;
					}
					tMaxX = tMaxX + tDeltaX;
				}
				else
				{
					z = z + stepZ;
					if(z < 1 || z > nz)
					{
						DM[idx] = MAX_DEPTH;
						return;
					}
					tMaxZ = tMaxZ + tDeltaZ;
				}
			}
			else
			{
				if(tMaxY < tMaxZ)
				{
					y = y + stepY;
					if(y < 1 || y > ny)
					{
						DM[idx] = MAX_DEPTH;
						return;
					}
					tMaxY = tMaxY + tDeltaY;
				}
				else
				{
					z = z + stepZ;
					if(z < 1 || z > nz)
					{
						DM[idx] = MAX_DEPTH;
						return;
					}
					tMaxZ = tMaxZ + tDeltaZ;
				}
			}

			id = int((z-1)*ny*nx + (x-1)*ny + (y-1));
			if(OG[id] > 0)
			{
				/*O[id] = 1;*/
				dx = ws_x[id] - xpos;
				dy = ws_y[id] - ypos;
				dz = ws_z[id] - zpos;
				d = sqrtf(dx*dx + dy*dy + dz*dz);
				if(d > MIN_DEPTH2)
				{
					DM[idx] = d;
					return;
				}
			}

		}


	}


	__device__  void cuda_cross(const float *a, const float *b , float* r) {
		r[0] =   ( (a[1] * b[2]) - (a[2] * b[1]) );
		r[1] =   ( (a[2] * b[0]) - (a[0] * b[2]) );
		r[2] =   ( (a[0] * b[1]) - (a[1] * b[0]) );
	}

	__global__ void ksdf_memory(float* ksdf,
		const float* sample_x, const float* sample_y, const float* sample_z, int numElem,
		const float* gv_x, const float* gv_y, const float* gv_z)
	{
		int idx, id, idx_x, idx_y, idx_z;
		float k1(0.02), k2(40.0), gamma(1.0),
			x,y,z,sx,sy,sz,v_x,v_y,v_z, norm_vek,
			norm_v, ksdf_static, ksdf_dynamic,
			resCos, vek_x, vek_y, vek_z, ksdf_temp(0.0f);



		idx_x = blockIdx.x *blockDim.x + threadIdx.x;
		idx_y = blockIdx.y *blockDim.y + threadIdx.y;
		idx_z = blockIdx.z *blockDim.z + threadIdx.z;

		x = V_DX*idx_x + WS_X_MIN;
		y = V_DY*idx_y + WS_Y_MIN;
		z = V_DZ*idx_z + WS_Z_MIN;


		idx = idx_z*WS_NY*WS_NX + idx_x*WS_NY + idx_y;





		for(id = 0; id< numElem; id++)
		{
			sx  =  sample_x[id];
			sy  =  sample_y[id];
			sz  =  sample_z[id];

			v_x = gv_x[id];
			v_y = gv_y[id];
			v_z = gv_z[id];

			vek_x = x - sx;
			vek_y = y - sy;
			vek_z = z - sz;

			norm_vek = sqrtf(vek_x * vek_x + vek_y*vek_y + vek_z * vek_z);
			norm_v = sqrtf(v_x*v_x + v_y*v_y + v_z*v_z);

			if(norm_vek == 0.0f)
				continue;

			ksdf_static = k1/norm_vek;

			if(norm_v > 0)
			{
				resCos = (v_x*vek_x + v_y*vek_y +v_z*vek_z )/(norm_v * norm_vek);
				ksdf_dynamic = ((k2*norm_v)*(gamma+resCos))/(norm_vek*norm_vek);
			}
			else
			{
				ksdf_dynamic = 0.0;
			}
			ksdf_temp =  ksdf_temp + ksdf_static + ksdf_dynamic;
		}
		ksdf[idx] = ksdf_temp;
	}


	__global__ void ksdf(float* ksdf,
		const float* sample_x, const float* sample_y, const float* sample_z, int numElem,
		const float* gv_x, const float* gv_y, const float* gv_z,
		const float* ws_x, const float* ws_y, const float* ws_z)
	{
		int idx, id;
		float k1(0.02), k2(40.0), gamma(1.0),
			x,y,z,sx,sy,sz,v_x,v_y,v_z, norm_vek,
			norm_v, ksdf_static, ksdf_dynamic,
			resCos, vek_x, vek_y, vek_z, ksdf_temp(0.0f);


		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		x   = ws_x[idx];
		y   = ws_y[idx];
		z   = ws_z[idx];

		for(id = 0; id< numElem; id++)
		{
			sx  =  sample_x[id];
			sy  =  sample_y[id];
			sz  =  sample_z[id];

			v_x = gv_x[id];
			v_y = gv_y[id];
			v_z = gv_z[id];

			vek_x = x - sx;
			vek_y = y - sy;
			vek_z = z - sz;

			norm_vek = sqrtf(vek_x * vek_x + vek_y*vek_y + vek_z * vek_z);
			norm_v = sqrtf(v_x*v_x + v_y*v_y + v_z*v_z);

			if(norm_vek == 0.0f)
				continue;

			ksdf_static = k1/norm_vek;

			if(norm_v > 0)
			{
				resCos = (v_x*vek_x + v_y*vek_y +v_z*vek_z )/(norm_v * norm_vek);
				ksdf_dynamic = ((k2*norm_v)*(gamma+resCos))/(norm_vek*norm_vek);
			}
			else
			{
				ksdf_dynamic = 0.0;
			}
			ksdf_temp =  ksdf_temp + ksdf_static + ksdf_dynamic;
		}
		ksdf[idx] = ksdf_temp;
	}

	__global__ void transformPCL(float* xi, float* yi, float* zi, int* ii,  float* t)
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

		xi[idx] = h[0]*x + h[1]*y + h[2]*z + h[3];
		yi[idx] = h[4]*x + h[5]*y + h[6]*z + h[7];
		zi[idx] = h[8]*x + h[9]*y + h[10]*z + h[11];

	}

	__global__ void calcShadow( unsigned char* O,
		unsigned char* FA,
		const float* EDM, const float* HDM,
		const float* d_C, short resx, short resy,
		float xpos, float ypos, float zpos,
		const float* ws_x, const float* ws_y, const float* ws_z)
	{
		unsigned int idx;
		float x,y,z,xi,yi,zi,d, dx, dy, dz;
		int u,v,ic;

		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		x = ws_x[idx];
		y = ws_y[idx];
		z = ws_z[idx];

		xi = d_C[0]*x + d_C[1]*y + d_C[2]*z +d_C[3];
		yi = d_C[4]*x + d_C[5]*y + d_C[6]*z +d_C[7];
		zi = d_C[8]*x + d_C[9]*y + d_C[10]*z +d_C[11];



		u = floor(xi/zi);
		v = floor(yi/zi);
		ic = v*resx + u;


		if( u >= 0 &&  u < resx && v >= 0 && v < resy && zi > 0)
		{

			dx = x - xpos;
			dy = y - ypos;
			dz = z - zpos;
			d = sqrtf(dx*dx + dy*dy + dz*dz);
			ic = v*resx + u;

			if(d >= HDM[ic] && EDM[ic] > HDM[ic])
			{
				O[idx] = 1;
			}else
			{
				O[idx] = 0;
			}
			if(d < HDM[ic] && d < EDM[ic])
			{
				FA[idx] = 1;
			}else{
				FA[idx] = 0;
			}

		}else
		{
			O[idx] = 0;
			FA[idx] = 0;
		}
	}

	__global__ void calcHumanNotVisible(unsigned char* HS, unsigned char* HV, unsigned char* H)
	{
		int idx;

		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		if(H[idx] != 0 && HV[idx] == 0)
			HS[idx] = 1;
		else
			HS[idx] = 0;


	}


	__global__ void fuse2HSGrids(unsigned int* HS1, unsigned int* HS2, unsigned int* FA1, unsigned int* FA2)
	{
		int idx;

		idx =  blockIdx.y  * gridDim.x  * blockDim.z * blockDim.y * blockDim.x
			+ blockIdx.x  * blockDim.z * blockDim.y * blockDim.x
			+ threadIdx.z * blockDim.y * blockDim.x
			+ threadIdx.y * blockDim.x
			+ threadIdx.x;

		unsigned int hs1 = HS1[idx];
		unsigned int hs2 = HS2[idx];
		unsigned int fa1 = FA1[idx];
		unsigned int fa2 = FA2[idx];


		HS1[idx] = hs1 & hs2;
		FA1[idx] = fa1 | fa2;

	}


	__global__ void transformPCLandVelocity(float* xi, float* yi, float* zi, float* vs_x, float* vs_y, float* vs_z, int* ii,  float* t,
		float* v_x, float* v_y, float* v_z, float* w_x, float* w_y, float* w_z, float vxp, float vyp)
	{
		int idx, id;
		float *h;
		float x, y, z;
		float p[3];
		float pr[3];
		float ps[3];
		float w[3];

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

		p[0] = h[0]*x + h[1]*y + h[2]*z + h[3];
		p[1] = h[4]*x + h[5]*y + h[6]*z + h[7];
		p[2] = h[8]*x + h[9]*y + h[10]*z + h[11];


		ps[0] = p[0] - h[3];
		ps[1] = p[1] - h[7];
		ps[2] = p[2] - h[11];

		w[0] = w_x[id-1];
		w[1] = w_y[id-1];
		w[2] = w_z[id-1];

		cuda_cross(w, ps , pr);


		xi[idx] = p[0];
		yi[idx] = p[1];
		zi[idx] = p[2];

		vs_x[idx] = v_x[id-1] + pr[0] + vxp;
		vs_y[idx] = v_y[id-1] + pr[1] + vyp;
		vs_z[idx] = v_z[id-1] + pr[2];

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


	__device__ void rotx_device(const float t, float *r)
	{
		float ct = cos(t);
		float st = sin(t);
		r[0] = 1.0f;
		r[1] = 0.0f;
		r[2] = 0.0f;
		r[3] = 0.0f;

		r[4] = 0.0f;
		r[5] = ct;
		r[6] = -st;
		r[7] = 0.0f;

		r[8] = 0.0f;
		r[9] = st;
		r[10] = ct;
		r[11] = 0.0f;

		r[12] = 0.0f;
		r[13] = 0.0f;
		r[14] = 0.0f;
		r[15] = 1.0f;

	}

	__device__ void roty_device(const float t, float *r)
	{
		float ct = cos(t);
		float st = sin(t);
		r[0] = ct;
		r[1] = 0.0f;
		r[2] = st;
		r[3] = 0.0f;

		r[4] = 0.0f;
		r[5] = 1.0f;
		r[6] = 0.0f;
		r[7] = 0.0f;

		r[8] = -st;
		r[9] = 0.0f;
		r[10] = ct;
		r[11] = 0.0f;

		r[12] = 0.0f;
		r[13] = 0.0f;
		r[14] = 0.0f;
		r[15] = 1.0f;

	}

	__device__ void rotz_device(const float t, float *r)
	{
		float ct = cos(t);
		float st = sin(t);
		r[0] = ct;
		r[1] = -st;
		r[2] = 0.0f;
		r[3] = 0.0f;

		r[4] = st;
		r[5] = ct;
		r[6] = 0.0f;
		r[7] = 0.0f;

		r[8] = 0.0;
		r[9] = 0.0f;
		r[10] = 1.0f;
		r[11] = 0.0f;

		r[12] = 0.0f;
		r[13] = 0.0f;
		r[14] = 0.0f;
		r[15] = 1.0f;

	}

	__device__ void rpy2r_device(float roll, float pitch, float yaw, float *r)
	{
		float rx[16];
		float ry[16];
		float rz[16];
		float t[16];

		rotx_device(roll,rx);
		roty_device(pitch,ry);
		rotz_device(yaw,rz);

		mm16_device(rx,ry,t);
		mm16_device(t,rz,r);

	}

	__device__ void getRollPitchYawFromAngleID(int a_i, float* roll, float* pitch, float* yaw)
	{
		 
		int ri = a_i/N_OF_A_SQ;
		int pi = (a_i - ri*N_OF_A_SQ)/N_OF_A;
		int yi = a_i-ri*N_OF_A_SQ-pi*N_OF_A;

		*roll = DA * ri - MATH_PI;
		*pitch = DA * pi -MATH_PI;
		*yaw =  DA * yi -MATH_PI;


	}

	__global__ void updateCameraPositions(int* pcl_id, int* angle_id,
			const float* const d_r_x,
			const float* const d_r_y,
			const float* const d_r_z,
			const int* const d_r_i,
			const float* const d_T0,
			const float* const d_T,
			float* h)
	{

		int id =  blockIdx.x *blockDim.x + threadIdx.x;

		int pcl_index = pcl_id[id];
		int index = d_r_i[pcl_index];
		int angle_index = angle_id[id];

		float roll, pitch, yaw;
		//	= d_roll[angle_index];
		//float pitch = d_pitch[angle_index];
		//float yaw = d_yaw[angle_index];

		getRollPitchYawFromAngleID(angle_index, &roll, &pitch, &yaw);

		float r[16];
		float cii[16];
		float cil[16];


		rpy2r_device(roll,pitch, yaw, r);
		r[3] = d_r_x[pcl_index];
		r[7] = d_r_y[pcl_index];
		r[11] = d_r_z[pcl_index];

		invert4_device(&(d_T0[16*index]), cii);
		mm16_device(cii,r,cil);
		mm16_device(&(d_T[16*index]), cil, &(h[16*id]));

	}

	__device__ void mm12x16_device(const float *a, const float *b, float *c)
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

	}

	__device__ void invert3_device(float *mat, float *dst)
	{
		float det = mat[0]*mat[4]*mat[8]+
					mat[1]*mat[5]*mat[6]+
					mat[2]*mat[3]*mat[7]-
					mat[1]*mat[3]*mat[8]-
					mat[2]*mat[4]*mat[6]-
					mat[0]*mat[5]*mat[7];

		dst[0] = mat[4]*mat[8]  - mat[5]*mat[7];
		dst[1] = mat[2]*mat[7]  - mat[1]*mat[8];
		dst[2] = mat[1]*mat[5]  - mat[2]*mat[4];
		dst[3] = mat[5]*mat[6]  - mat[3]*mat[8];
		dst[4] = mat[0]*mat[8]  - mat[2]*mat[6];
		dst[5] = mat[2]*mat[3]  - mat[0]*mat[5];
		dst[6] = mat[3]*mat[7]  - mat[4]*mat[6];
		dst[7] = mat[1]*mat[6]  - mat[0]*mat[7];
		dst[8] = mat[0]*mat[4]  - mat[1]*mat[3];
		det = 1/det;
		for (int j = 0; j < 9; j++)
			dst[j] *= det;
	}



	__global__ void updateCameraParameters(
			//struct CAM* cam,
			float* C0,
			float* d_h,
			float* d_C,
			float* d_CP,
			float* d_Mi)
	{

		int id =  blockIdx.x *blockDim.x + threadIdx.x;

		float i[16];
		float iMi[9];

		float* C = d_C+id*NUMELEM_C;
		float* Mi = d_Mi+id*NUMELEM_Mi;
		float* CP = d_CP+id*NUMELEM_Cp;
		invert4_device(d_h+id*16,i);
		mm12x16_device(C0, i,C);

		iMi[0] = C[0];
		iMi[1] = C[1];
		iMi[2] = C[2];

		iMi[3] = C[4];
		iMi[4] = C[5];
		iMi[5] = C[6];

		iMi[6] = C[8];
		iMi[7] = C[9];
		iMi[8] = C[10];

		invert3_device(iMi, Mi);

		CP[0] = d_h[id*16+3];
		CP[1] = d_h[id*16+7];
		CP[2] = d_h[id*16+11];

	}

	__global__ void testMM16(float* h1, float* h2, float* r1)
	{
		mm16_device(h1, h2,r1); 
	}

	__global__ void testInvert4(float* h1, float* r1)
	{
		invert4_device(h1,r1); 
	}

	__global__ void gaussjordan(float *A,  float *I,int n, int i)
	{
		int x = blockIdx.x * blockDim.x + threadIdx.x;
		int y = blockIdx.y * blockDim.y + threadIdx.y;
		float P;

		if(x<n && y<n)
			if(x>i){ // this limits operation to rows below the pivot point
				P=A[x*n+i]/A[i*n+i];
				I[x*n+y] -= I[i*n+y]*P;  // apply for every row member
				if(y>=i){ //limits  to row members to the right of the pivot
					A[x*n+y] -= A[i*n+y]*P;  // apply only to members right of pivot
				}
			}
	 }

	__global__ void checkCollisionEnvironmentHuman(unsigned int* h,unsigned int* r, unsigned int* result)
	{
		__shared__ unsigned int t[ACCUM_N_WS];
		int id =  blockIdx.x *blockDim.x + threadIdx.x;
		unsigned int t2 = h[4*id] & r[4*id];
		unsigned int t1;
		for(int i=1; i<4; i++)
		{
			t1 = h[4*id+i] & r[4*id+i];
			t2 |= t1;
		}
		t[id] = t2;

		
		////starting reduction
		////1024 threads turn out to 10 iterations
		for (int i = ACCUM_N_WS / 2; i > 0; i >>= 1)
		{
			__syncthreads();
			if(threadIdx.x < i)
			{
				t[threadIdx.x] |= t[threadIdx.x + i];
			}
		}
		__syncthreads();
		result[0] = t[0];


	}

}


#endif /* ALLKERNELFCT_CUH_ */
