#ifndef MATHCUDA_H
#define MATHCUDA_H

#include "struct_definitions.h"
#include "_generate.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

static void convertAItoRPY(int a_i, SAMPLE_ROTATIONS* sr, int* ri, int *pi, int* yi)
{
	int ntheta = sr->nPitch;
	int nZ = sr->nYaw;

	*ri = a_i/(ntheta*nZ);
	*pi = (a_i-(*ri)*ntheta*nZ)/nZ;
	*yi = a_i-(*ri)*ntheta*nZ - (*pi)*nZ;
}


static void convertRPYtoAI(int ri, int pi, int yi, SAMPLE_ROTATIONS* sr, int* a_i)
{
	int ntheta = sr->nPitch;
	int nZ = sr->nYaw;

	*a_i = ri*ntheta*nZ + pi*nZ + yi;
	assert(*a_i >=0 && *a_i < sr->nRotations);
}


static inline void mm16(const float *a, const float *b, float *c)
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

static inline void mm12x16(const float *a, const float *b, float *c)
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




static inline void rotx(const float t, float *r)
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

static inline void roty(const float t, float *r)
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

static inline void rotz(const float t, float *r)
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

static void rpy2r(float roll, float pitch, float yaw, float *r)
{
	float rx[16];
	float ry[16];
	float rz[16];
	float t[16];

	rotx(roll,rx);
	roty(pitch,ry);
	rotz(yaw,rz);

	mm16(rx,ry,t);
	mm16(t,rz,r);

}


static inline void cross(const float *a, const float *b , float* r) {
  r[0] =   ( (a[1] * b[2]) - (a[2] * b[1]) );
  r[1] =   ( (a[2] * b[0]) - (a[0] * b[2]) );
  r[2] =   ( (a[0] * b[1]) - (a[1] * b[0]) );
}



static inline void invert4(float *mat, float *dst)
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

static inline void invert3(float *mat, float *dst)
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



static unsigned long long numberOfCases(unsigned long long n, unsigned long long k)
{


	unsigned long long *vector     = NULL; //where the current figure is stored
	int           gen_result;         //return value of generation functions
	unsigned long long nC = 0;
	vector = new unsigned long long[k];
	gen_result = gen_comb_norep_lex_init(vector, n, k);
	do{		
		gen_result = gen_comb_norep_lex_next(vector, n,k);
		if(nC%100000000 == 0)
			printf("calculating nc.....\n");
		nC++;
	}while(gen_result == GEN_NEXT);
	return nC;

}




#endif
