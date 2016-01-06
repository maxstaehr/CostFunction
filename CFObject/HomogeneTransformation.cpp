#include "HomogeneTransformation.h"
#include "Rotation.h"
#include <string>
#include <algorithm>

#define eps_div (1e-7)

HomogeneTransformation::HomogeneTransformation(void)
{
	float temp[16] = EYE_16;
	memcpy(H, temp, sizeof(float)*16);
}

HomogeneTransformation::HomogeneTransformation(const float const* temp)
{
	memcpy(H, temp, sizeof(float)*16);
}

void HomogeneTransformation::operator=(HomogeneTransformation& rhs )
{
	
	memcpy(H, rhs.getH(), sizeof(float)*16);
}
float HomogeneTransformation::getDist(HomogeneTransformation& rhs)
{
	float a_rpy[3];
	float b_rpy[3];
	tr2rpy(a_rpy);
	rhs.tr2rpy(b_rpy);
	float ret = 0.0f;
	ret +=  abs(rhs.getH()[3] - H[3]);
	ret +=  abs(rhs.getH()[7] - H[7]);
	ret +=  abs(rhs.getH()[11] - H[11]);
	ret +=  abs(b_rpy[0] - a_rpy[0]);
	ret +=  abs(b_rpy[1] - a_rpy[1]);
	ret +=  abs(b_rpy[2] - a_rpy[2]);
	return ret;
}

float HomogeneTransformation::getDist(HomogeneTransformation& rhs, DIM_DIR dim)
{
	float ret = FLT_MAX;
	float dist;
	float a_rpy[3];
	float b_rpy[3];
	tr2rpy(a_rpy);
	rhs.tr2rpy(b_rpy);
	using namespace std;
	switch (dim)
	{
		case DIM_DIR::XP:
			ret = rhs.getH()[3] - H[3];
			if(ret < 0)
			{
				ret = FLT_MAX;
			}			
			break;
		case DIM_DIR::XM:
			ret = rhs.getH()[3] - H[3];
			if(ret > 0)
			{
				ret = FLT_MAX;
			}	
			break;
		case DIM_DIR::YP:
			ret = rhs.getH()[7] - H[7];
			if(ret < 0)
			{
				ret = FLT_MAX;
			}			
			break;			
		case DIM_DIR::YM:
			ret = rhs.getH()[7] - H[7];
			if(ret > 0)
			{
				ret = FLT_MAX;
			}				
			break;
		case DIM_DIR::ZP:
			ret = rhs.getH()[11] - H[11];
			if(ret < 0)
			{
				ret = FLT_MAX;
			}			
			break;	
		case DIM_DIR::ZM:
			ret = rhs.getH()[11] - H[11];
			if(ret > 0)
			{
				ret = FLT_MAX;
			}				
			break;
		case DIM_DIR::ROLLP:
			ret = b_rpy[0] - a_rpy[0];
			if(ret < 0)
			{
				ret = FLT_MAX;
			}			
			break;			
		case DIM_DIR::ROLLM:
			ret = b_rpy[0] - a_rpy[0];
			if(ret > 0)
			{
				ret = FLT_MAX;
			}	
			break;
		case DIM_DIR::PITCHP:
			ret = b_rpy[1] - a_rpy[1];
			if(ret < 0)
			{
				ret = FLT_MAX;
			}			
			break;			
		case DIM_DIR::PITCHM:
			ret = b_rpy[1] - a_rpy[1];
			if(ret > 0)
			{
				ret = FLT_MAX;
			}	
			break;
		case DIM_DIR::YAWP:
			ret = b_rpy[2] - a_rpy[2];
			if(ret < 0)
			{
				ret = FLT_MAX;
			}			
			break;			
		case DIM_DIR::YAWM:
			ret = b_rpy[2] - a_rpy[2];
			if(ret > 0)
			{
				ret = FLT_MAX;
			}	
			break;
	};
	return abs(ret);

}

void HomogeneTransformation::tr2rpy(float* rpy)
{

	if (abs(H[10]) < eps_div && abs(H[6]) < eps_div)
	{
		rpy[0] = 0;
		rpy[1] = atan2(H[2], H[10]);
		rpy[2] = atan2(H[4], H[5]);
	}
	else
	{
		rpy[0] = atan2(-H[6], H[10]);
	    float sr = sin(rpy[0]);
		float cr = cos(rpy[0]);
		rpy[1] = atan2(H[2], cr * H[10] - sr * H[6]);
		rpy[2] = atan2(-H[1], H[0]);  
	}
}

bool HomogeneTransformation::isEqual(HomogeneTransformation& rhs)
{
	bool ret  = true;
	for(int i=0; i<16; i++)
	{
		ret &= abs(H[i] - rhs.getH()[i]) < 1e-5f;
	}
	return ret;
}


HomogeneTransformation::HomogeneTransformation(const HomogeneTransformation& inst)
{
	memcpy(H, inst.H, sizeof(float)*16);
}


HomogeneTransformation::~HomogeneTransformation(void)
{
}


void HomogeneTransformation::setH(const float const* res)
{
	memcpy(H, res, sizeof(float)*16);
}



HomogeneTransformation HomogeneTransformation::inv(void)
{
	HomogeneTransformation ret;
	float mat[16];
	float dst[16];
	float tmp[12]; /* temp array for pairs */
	float src[16]; /* array of transpose source matrix */
	float det; /* determinant */
	/* transpose matrix */
	memcpy(mat, H, sizeof(float)*16);


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

	ret.setH(dst);
	return ret;
}
HomogeneTransformation HomogeneTransformation::mul(HomogeneTransformation in)
{
	HomogeneTransformation ret;	
	float const * const a = H;
	float const * const b = in.H;
	ret.H[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12];
	ret.H[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13];
	ret.H[2] = a[0]*b[2] + a[1]*b[6] + a[2]*b[10] + a[3]*b[14];
	ret.H[3] = a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3]*b[15];

	ret.H[4] = a[4]*b[0] + a[5]*b[4] + a[6]*b[8] + a[7]*b[12];
	ret.H[5] = a[4]*b[1] + a[5]*b[5] + a[6]*b[9] + a[7]*b[13];
	ret.H[6] = a[4]*b[2] + a[5]*b[6] + a[6]*b[10] + a[7]*b[14];
	ret.H[7] = a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7]*b[15];

	ret.H[8] = a[8]*b[0] + a[9]*b[4] + a[10]*b[8] + a[11]*b[12];
	ret.H[9] = a[8]*b[1] + a[9]*b[5] + a[10]*b[9] + a[11]*b[13];
	ret.H[10] = a[8]*b[2] + a[9]*b[6] + a[10]*b[10] + a[11]*b[14];
	ret.H[11] = a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11]*b[15];
	
	ret.H[12] = a[12]*b[0] + a[13]*b[4] + a[14]*b[8] + a[15]*b[12];
	ret.H[13] = a[12]*b[1] + a[13]*b[5] + a[14]*b[9] + a[15]*b[13];
	ret.H[14] = a[12]*b[2] + a[13]*b[6] + a[14]*b[10] + a[15]*b[14];
	ret.H[15] = a[12]*b[3] + a[13]*b[7] + a[14]*b[11] + a[15]*b[15];	
	return ret;

}


void HomogeneTransformation::init(float roll, float pitch, float yaw, float x, float y, float z)
{
	Rotation r1;
	r1.initRoll(roll);

	Rotation r2;
	r2.initPitch(pitch);

	Rotation r3;
	r3.initYaw(yaw);

	Rotation r4 = r1.mul(r2);
	Rotation r5 = r4.mul(r3);

	H[0] = r5.getH()[0];
	H[1] = r5.getH()[1];
	H[2] = r5.getH()[2];
	H[3] = x;

	H[4] = r5.getH()[3];
	H[5] = r5.getH()[4];
	H[6] = r5.getH()[5];
	H[7] = y;

	H[8] = r5.getH()[6];
	H[9] = r5.getH()[7];
	H[10] = r5.getH()[8];
	H[11] = z;

	H[12] = 0.0f;
	H[13] = 0.0f;
	H[14] = 0.0f;
	H[15] = 1.0f;

}
