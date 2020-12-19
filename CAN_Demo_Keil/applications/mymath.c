#include "math.h"
#include "mymath.h"

float sindw(float in){
    return sinf(in/57.3);
}

float cosdw(float in){
    return cosf(in/57.3);
}

float To_180_degreesw(float x)
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}

float To_360_degreesw(float x)
{
	if(x<0)
		return 360+x;
	else
		return x;
}


float limitw(float x,float min,float max)
{
  if(x>max)return max;
	if(x<min)return min;
	return x;
}

float deadw(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}


static void inv22w(const float x[4], float y[4])
{
  float r;
  float t;
  if ((float)fabs(x[1]) > (float)fabs(x[0])) {
    r = x[0] / x[1];
    t = 1.0F / (r * x[3] - x[2]);
    y[0] = x[3] / x[1] * t;
    y[1] = -t;
    y[2] = -x[2] / x[1] * t;
    y[3] = r * t;
  } else {
    r = x[1] / x[0];
    t = 1.0F / (x[3] - r * x[2]);
    y[0] = x[3] / x[0] * t;
    y[1] = -r * t;
    y[2] = -x[2] / x[0] * t;
    y[3] = t;
  }
}
void invet22w(const float A[4], float *dA, float inA[4])
{
  int ix;
  float x[4];
  signed char ipiv[2];
  int iy;
  char isodd;
  int k;
  float temp;
  float b_A[4];
  for (ix = 0; ix < 4; ix++) {
    x[ix] = A[ix];
  }

  for (ix = 0; ix < 2; ix++) {
    ipiv[ix] = (signed char)(1 + ix);
  }

  ix = 0;
  if ((float)fabs(A[1]) > (float)fabs(A[0])) {
    ix = 1;
  }

  if (A[ix] != 0.0F) {
    if (ix != 0) {
      ipiv[0] = 2;
      ix = 0;
      iy = 1;
      for (k = 0; k < 2; k++) {
        temp = x[ix];
        x[ix] = x[iy];
        x[iy] = temp;
        ix += 2;
        iy += 2;
      }
    }

    x[1] /= x[0];
  }

  if (x[2] != 0.0F) {
    x[3] += x[1] * -x[2];
  }

  *dA = x[0] * x[3];
  isodd = 0;
  if (ipiv[0] > 1) {
    isodd = 1;
  }

  if (isodd) {
    *dA = -*dA;
  }

  if (*dA == 0.0F) {
    for (ix = 0; ix < 2; ix++) {
      for (iy = 0; iy < 2; iy++) {
        b_A[ix + (iy << 1)] = 0.0F;
        for (k = 0; k < 2; k++) {
          b_A[ix + (iy << 1)] += A[k + (ix << 1)] * A[k + (iy << 1)];
        }
      }
    }

    inv22w(b_A, x);
    for (ix = 0; ix < 2; ix++) {
      for (iy = 0; iy < 2; iy++) {
        inA[ix + (iy << 1)] = 0.0F;
        for (k = 0; k < 2; k++) {
          inA[ix + (iy << 1)] += x[ix + (k << 1)] * A[iy + (k << 1)];
        }
      }
    }
  } else {
    inv22w(A, inA);
  }
}

void DigitalLPFw(float in, float* out, float cutoff_freq, float dt) {
	  float input_reg=in;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        *out = input_reg;
    }
    float rc = 1.0f/(2*3.1415926*cutoff_freq);
    float alpha = limitw(dt/(dt+rc), 0.0f, 1.0f);
    *out += (input_reg - *out) * alpha;
}

void force_disw(float pos_force[3],float att_torque[3],float dt)
{




}
