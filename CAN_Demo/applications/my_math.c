
#include "my_math.h"

ESO_AngularRate leg_td2[4][3];
_TD4 leg_td4[4][3];
_TD4 odom_td[3];

void TD4_reset( _TD4* filter )
{
	filter->x1 = filter->x2 = filter->x3 = filter->x4 = 0;
}

//void TD4_setP( TD4* filter , float P )
//{
//	filter->P1 = filter->P2 = filter->P3 = filter->P4 = P;
//}

void TD4_init( _TD4* filter , float P1 , float P2 , float P3 , float P4 )
{
	filter->P1 = P1;
	filter->P2 = P2;
	filter->P3 = P3;
	filter->P4 = P4;
	filter->r2p = filter->r2n = filter->r3p = filter->r3n = filter->r4p = filter->r4n = 1e12;
	TD4_reset( filter );
}

float TD4_track4( _TD4* filter , const float expect , const float h )
{
	filter->tracking_mode = 4;
	
	float e1 = expect - filter->x1;
	float e1_1 = -filter->x2;
	float e1_2 = -filter->x3;
	float e1_3 = -filter->x4;
	float T2 = filter->P1 * e1;
	float P1 = 0;
	if( T2 > filter->r2p )
		T2 = filter->r2p;
	else if( T2 < -filter->r2n )
		T2 = -filter->r2n;
	else
		P1 = filter->P1;
	float T2_1 = P1 * e1_1;
	float T2_2 = P1 * e1_2;
	float T2_3 = P1 * e1_3;
	
	float e2 = T2 - filter->x2;
	float e2_1 = T2_1-filter->x3;
	float e2_2 = T2_2-filter->x4;
	float T3 = filter->P2 * e2;
	float P2 = 0;
	if( T3 > filter->r3p )
		T3 = filter->r3p;
	else if( T3 < -filter->r3n )
		T3 = -filter->r3n;
	else
		P2 = filter->P2;
	T3 += T2_1;
	float T3_1 = P2 * e2_1 + T2_2;
	float T3_2 = P2 * e2_2 + T2_3;
	
	float e3 = T3 - filter->x3;
	float e3_1 = T3_1-filter->x4;
	float T4 = filter->P3 * e3;
	float P3 = 0;
	if( T4 > filter->r4p )
		T4 = filter->r4p;
	else if( T4 < -filter->r4n )
		T4 = -filter->r4n;
	else
		P3 = filter->P3;
	T4 += T3_1;
	float T4_1 = P3 * e3_1 + T3_2;
	
	float e4 = T4 - filter->x4;
	float T5 = filter->P4 * e4 + T4_1;
	
	filter->x1 += h*filter->x2;
	filter->x2 += h*filter->x3;
	filter->x3 += h*filter->x4;
	filter->x4 += h*T5;
	
	return filter->x1;
	
	
//	float Tv1 = filter->P1*(-filter->x2);
//	float Tv2 = filter->P1*(-filter->x3);
//	float Tv3 = filter->P1*(-filter->x4);
//	float Ta1 = filter->P2*( Tv1 - filter->x3 ) + Tv2;
//	float Ta2 = filter->P2*( Tv2 - filter->x4 ) + Tv3;
//	float Tj1 = filter->P3*( Ta1 - filter->x4 ) + Ta2;
//	
//	float Tx2 = filter->P1*( expect - filter->x1 );
//	float Tx3 = filter->P2*( Tx2 - filter->x2 ) + Tv1;
//	float Tx4 = filter->P3*( Tx3 - filter->x3 ) + Ta1;
//	float Tx5 = filter->P4*( Tx4 - filter->x4 ) + Tj1;
//	
//	filter->x1 += h*filter->x2;
//	filter->x2 += h*filter->x3;
//	filter->x3 += h*filter->x4;
//	filter->x4 += h*Tx5;
//	
//	return filter->x1;
}

float TD4_track3( _TD4* filter , const float expect , const float h )
{
	filter->tracking_mode = 3;
	
	float e2 = expect - filter->x2;
	float e2_1 = -filter->x3;
	float e2_2 = -filter->x4;
	float T3 = filter->P2 * e2;
	float P2 = 0;
	if( T3 > filter->r3p )
		T3 = filter->r3p;
	else if( T3 < -filter->r3n )
		T3 = -filter->r3n;
	else
		P2 = filter->P2;
	float T3_1 = P2 * e2_1;
	float T3_2 = P2 * e2_2;
	
	float e3 = T3 - filter->x3;
	float e3_1 = T3_1-filter->x4;
	float T4 = filter->P3 * e3;
	float P3 = 0;
	if( T4 > filter->r4p )
		T4 = filter->r4p;
	else if( T4 < -filter->r4n )
		T4 = -filter->r4n;
	else
		P3 = filter->P3;
	T4 += T3_1;
	float T4_1 = P3 * e3_1 + T3_2;
	
	float e4 = T4 - filter->x4;
	float T5 = filter->P4 * e4 + T4_1;
	
	filter->x2 += h*filter->x3;
	filter->x3 += h*filter->x4;
	filter->x4 += h*T5;
	
	return filter->x2;
	
	
	
	
//	float Ta1 = filter->P2*( - filter->x3 );
//	float Ta2 = filter->P2*( - filter->x4 );
//	float Tj1 = filter->P3*( Ta1 - filter->x4 ) + Ta2;
//	
//	float Tx3 = filter->P2*( expect - filter->x2 );
//	float Tx4 = filter->P3*( Tx3 - filter->x3 ) + Ta1;
//	float Tx5 = filter->P4*( Tx4 - filter->x4 ) + Tj1;
//	
//	filter->x2 += h*filter->x3;
//	filter->x3 += h*filter->x4;
//	filter->x4 += h*Tx5;
//	
//	return filter->x2;
}


void init_ESO_AngularRate( ESO_AngularRate* eso , float T , float b , float beta1 , float beta2 )
{
	eso->beta1 = beta1;
	eso->beta2 = beta2;
	
	eso->z1 = eso->z2 = eso->z_inertia = 0;
	eso->err_continues_time = 0;	
	
	eso->T = T;	eso->invT = 1.0f / T;
	eso->b = b;
	for( unsigned char i = 0 ; i < ESO_AngularRate_his_length ; ++i )
		eso->his_z1[i] = 0;
}

void ESO_AngularRate_update_u( ESO_AngularRate* eso , float u )
{
	eso->u = u;
	eso->z_inertia += eso->h * eso->invT * ( eso->b*eso->u - eso->z_inertia );
	eso->z1 += eso->h * ( eso->z_inertia + eso->z2 );
}

float ESO_AngularRate_run( ESO_AngularRate* eso , const float v , const float h )
{
	float err = v - eso->his_z1[0];
	
	//¸üÐÂÎó²î³ÖÐøÊ±¼ä
	if( (err > 0) ^ eso->err_sign )
	{
		eso->err_continues_time = 0;
		eso->err_sign = err > 0;
	}
	else
		eso->err_continues_time += h;
	
	float max_beta1_scale = 0.9f / eso->beta1;
	float err_continues_time3 = eso->err_continues_time*eso->err_continues_time*eso->err_continues_time;
	float beta1_scale = 1 + 500 * err_continues_time3;	
	float beta2_scale = 1 + 50 * err_continues_time3;		
	if( beta1_scale > 15 )
		beta1_scale = 15;
	if( beta2_scale > 15 )
		beta2_scale = 15;
	if( beta1_scale > max_beta1_scale )
		beta1_scale = max_beta1_scale;
	
	//ÐÞÕý
	float z1_correction = beta1_scale*eso->beta1*err;
	float z2_correction = beta2_scale*eso->beta2*err;
	float filter_dt = h;
	for( unsigned char k = 0 ; k < ESO_AngularRate_his_length - 1 ; ++k )
	{
		eso->his_z1[ k ] = eso->his_z1[ k + 1 ] + z1_correction + filter_dt*z2_correction;
		filter_dt += h;
	}
	eso->z2 += z2_correction;
	eso->z1 += z1_correction + filter_dt*z2_correction;
	eso->his_z1[ ESO_AngularRate_his_length - 1 ] = eso->z1;
	
	eso->h = h;
	return eso->z2;
}

static float sign(float x)
{
  if(x>0)
      return(1);
  if(x<0)
      return(-1);
}

static float fst2(float x1,float x2,float w, float h)
{
	float td_y=0;
	float a0=0,a1,a2;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;//dead
	float sy,sa;
	
	d=w*h*h;
	a0=h*x2;
	td_y=x1+a0;
	a1=my_sqrt(d*(d+8*ABS(td_y)));
	a2=a0+sign(td_y)*(a1-d)/2;
	sy=(sign(td_y+d)-sign(td_y-d))/2;
	a=(a0+td_y-a2)*sy+a2;
	sa=(sign(a+d)-sign(a-d))/2;
	fhan=-w*(a/d-sign(a))*sa-w*sign(a);
	return(fhan);
}

static float fst(float x1,float x2,float w, float h)
{
	float td_y=0;
	float a0=0;
	float a=0;
	float fhan=0;
	float d=0;
	float d0=0;//dead
	
	d=w*h;
	d0=h*d;
	td_y=x1+h*x2;
	a0=my_sqrt(d*d+8*w*ABS(td_y));
	
	if(ABS(td_y)>d0)
		a=x2+0.5*(a0-d)*sign(td_y);
	else
		a=x2+td_y/h;
		
	if (ABS(a)>d)
		fhan=-w*sign(a);
	else
		fhan=-w*a/d;
	return(fhan);
}

void OLDX_SMOOTH_IN_ESOX(ESO_X *eso_in,float in)
{
eso_in->v1+=eso_in->h0*eso_in->v2;                        //td_x1=v1;
eso_in->v2+=eso_in->h0*fst(eso_in->v1-in,eso_in->v2,eso_in->r0,eso_in->h0);           //td_x2=v2;
}


float fast_atan_table[257] = 
{
	0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02,
	1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
	3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02,
	4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
	6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02,
	7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
	9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01,
	1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
	1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01,
	1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
	1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01,
	1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
	1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01,
	2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
	2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01,
	2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
	2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01,
	2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
	2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01,
	2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
	3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01,
	3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
	3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01,
	3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
	3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01,
	3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
	3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01,
	4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
	4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01,
	4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
	4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01,
	4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
	4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01,
	4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
	4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01,
	5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
	5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01,
	5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
	5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01,
	5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
	5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01,
	5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
	5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01,
	5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
	6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01,
	6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
	6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01,
	6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
	6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01,
	6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
	6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01,
	6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
	6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01,
	6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
	7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01,
	7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
	7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01,
	7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
	7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01,
	7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
	7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01,
	7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
	7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01,
	7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
	7.853983e-01
};

float my_abs(float f)
{
	if (f >= 0.0f)
	{
		return f;
	}

	return -f;
}

REAL fast_atan2(REAL y, REAL x) 
{
	REAL x_abs, y_abs, z;
	REAL alpha, angle, base_angle;
	int index;

	/* don't divide by zero! */
	if ((y == 0.0f) && (x == 0.0f))
		angle = 0.0f;
	else 
	{
		/* normalize to +/- 45 degree range */
		y_abs = my_abs(y);
		x_abs = my_abs(x);
		//z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
		if (y_abs < x_abs)
			z = y_abs / x_abs;
		else
			z = x_abs / y_abs;
		/* when ratio approaches the table resolution, the angle is */
		/*      best approximated with the argument itself...       */
		if (z < TAN_MAP_RES)
			base_angle = z;
		else 
		{
			/* find index and interpolation value */
			alpha = z * (REAL) TAN_MAP_SIZE - .5f;
			index = (int) alpha;
			alpha -= (REAL) index;
			/* determine base angle based on quadrant and */
			/* add or subtract table value from base angle based on quadrant */
			base_angle = fast_atan_table[index];
			base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
		}

		if (x_abs > y_abs) 
		{        /* -45 -> 45 or 135 -> 225 */
			if (x >= 0.0f) 
			{           /* -45 -> 45 */
				if (y >= 0.0f)
					angle = base_angle;   /* 0 -> 45, angle OK */
				else
					angle = -base_angle;  /* -45 -> 0, angle = -angle */
			} 
			else
			{                  /* 135 -> 180 or 180 -> -135 */
				angle = 3.14159265358979323846;

				if (y >= 0.0f)
					angle -= base_angle;  /* 135 -> 180, angle = 180 - angle */
				else
					angle = base_angle - angle;   /* 180 -> -135, angle = angle - 180 */
			}
		} 
		else 
		{                    /* 45 -> 135 or -135 -> -45 */
			if (y >= 0.0f) 
			{           /* 45 -> 135 */
				angle = 1.57079632679489661923;

				if (x >= 0.0f)
					angle -= base_angle;  /* 45 -> 90, angle = 90 - angle */
				else
					angle += base_angle;  /* 90 -> 135, angle = 90 + angle */
			} 
			else
			{                  /* -135 -> -45 */
				angle = -1.57079632679489661923;

				if (x >= 0.0f)
					angle += base_angle;  /* -90 -> -45, angle = -90 + angle */
				else
					angle -= base_angle;  /* -135 -> -90, angle = -90 - angle */
			}
		}
	}


#ifdef ZERO_TO_TWOPI
	if (angle < 0)
		return (angle + TWOPI);
	else
		return (angle);
#else
	return (angle);
#endif
}

float my_atan(float x, float y)
{
	return fast_atan2(y, x);
}

//���㸡����ƽ��
float my_pow(float a)
{
	return a*a;
}

//����ƽ�����㷨
float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}

#define ONE_PI   (3.14159265)
#define TWO_PI   (2.0 * 3.14159265)
#define ANGLE_UNIT (TWO_PI/10.0)

double mx_sin(double rad)
{   
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

double my_sin(double rad)
{
	s8 flag = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag = -1;
	}

	return mx_sin(rad) * flag;
}

float my_cos(double rad)
{
	s8 flag = 1;
	rad += ONE_PI/2.0;

	if (rad >= ONE_PI)
	{
		flag = -1;
		rad -= ONE_PI;
	}

	return my_sin(rad)*flag;
}

float my_deathzoom(float x,float zoom)
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



float my_deathzoom_2(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}

  return (t);
}


float my_deathzoom_21(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}

  return (t);
}

float my_deathzoom_rc(float x,float zoom)
{
	float t;
	if(x>1500)
			t=LIMIT(my_deathzoom_21(x,zoom)-zoom,1500,2000);
		else
			t=LIMIT(my_deathzoom_21(x,zoom)+zoom,1000,1500);
  return (t);
}

float limit_mine(float x,float zoom)
{
	float t;
	
	if( x< -zoom)
	{
		t = -zoom;
	}
	else if( x>zoom)
	{
		t = zoom;
	}

  return (t);
}


float limit_mine2(float x,float min,float max)
{
	float t;
	
	if( x<min)
	{
		t = min;
	}
	else if( x>max)
	{
		t = max;
	}

  return (t);
}

float To_180_degrees(float x)
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}

float my_pow_2_curve(float in,float a,float max)
{
	if( a > 1 || a < 0 )
	{
		return 0;
	}
	return( (1.0f - a) + a *ABS(in / max) * in );

}

//jerk trajectory planner

_TRA traj[10];
void GenerateTrajectory(float  p0,float v0,float a0,float pf,float vf,float af,float Tf,
	 char defined[3],float*a,float*b,float*g,float *cost){
char accGoalDefined=defined[0];
char posGoalDefined=defined[1];
char velGoalDefined=defined[2];
 //define starting position:
 float  delta_a = af - a0;
 float  delta_v = vf - v0 - a0*Tf;
 float  delta_p = pf - p0 - v0*Tf - 0.5*a0*Tf*Tf;

 // %powers of the end time:
  float  T2 = Tf*Tf;
  float  T3 = T2*Tf;
  float  T4 = T3*Tf;
  float  T5 = T4*Tf;
  
  //%solve the trajectories, depending on what's constrained:
  if(posGoalDefined && velGoalDefined && accGoalDefined)
  {
    *a = ( 60*T2*delta_a - 360*Tf*delta_v + 720* 1*delta_p)/T5;
    *b = (-24*T3*delta_a + 168*T2*delta_v - 360*Tf*delta_p)/T5;
    *g = (  3*T4*delta_a -  24*T3*delta_v +  60*T2*delta_p)/T5;
}
  else if(posGoalDefined && velGoalDefined)
  {
    *a = (-120*Tf*delta_v + 320*   delta_p)/T5;
    *b = (  72*T2*delta_v - 200*Tf*delta_p)/T5;
    *g = ( -12*T3*delta_v +  40*T2*delta_p)/T5;
	}
      else if(posGoalDefined && accGoalDefined)
			{
    *a = (-15*T2*delta_a + 90*   delta_p)/(2*T5);
    *b = ( 15*T3*delta_a - 90*Tf*delta_p)/(2*T5);
    *g = (- 3*T4*delta_a + 30*T2*delta_p)/(2*T5);
		}
      else if(velGoalDefined && accGoalDefined)
			{
    *a = 0;
    *b = ( 6*Tf*delta_a - 12*   delta_v)/T3;
    *g = (-2*T2*delta_a +  6*Tf*delta_v)/T3;
		}
      else if(posGoalDefined)
  
  {  *a =  20*delta_p/T5;
    *b = -20*delta_p/T4;
    *g =  10*delta_p/T3;
		}
      else if(velGoalDefined)
			{
    *a = 0;
    *b =-3*delta_v/T3;
    *g = 3*delta_v/T2;
		}
      else if(accGoalDefined)
			{
    *a = 0;
    *b = 0;
    *g = delta_a/Tf;
		}
  else{
 //%Nothing to do!
    *a = 0;
    *b = 0;
    *g = 0;
}

 //%Calculate the cost:
  *cost =  *g* *g + *b* *g*Tf + *b* *b*T2/3.0 + *a* *g*T2/3.0 + *a* *b*T3/4.0 + *a* *a*T4/20.0;
}

void get_trajecotry(float p0,float v0,float a0,float a,float b,float g,float t,float *pos,float *spd,float *acc,float *jerk){
*pos=p0 + v0*t + (1/2.0)*a0*t*t + (1/6.0)*g*t*t*t + (1/24.0)*b*t*t*t*t + (1/120.0)*a*t*t*t*t*t;
*jerk=g  + b*t  + (1/2.0)*a*t*t;
*acc=a0 + g*t  + (1/2.0)*b*t*t  + (1/6.0)*a*t*t*t;
*spd=v0 + a0*t + (1/2.0)*g*t*t  + (1/6.0)*b*t*t*t + (1/24.0)*a*t*t*t*t;
}


void plan_tra(_TRA *tra)
{	
 char i,j;
 float cost[3];
	for(i=0;i<3;i++)
	{
	  GenerateTrajectory(tra->ps[i],tra->vs[i],tra->as[i], tra->pe[i],tra->ve[i],tra->ae[i], tra->Time,
	  tra->defined,&tra->param[i*3+0],&tra->param[i*3+1],&tra->param[i*3+2],&cost[i]);
	}
	tra->cost_all=cost[0]+cost[1]+cost[2];
}

void get_tra(_TRA *tra,float t)
{
char i;
float acc,jerk;
for(i=0;i<3;i++)
 get_trajecotry( tra->ps[i],tra->vs[i],tra->as[i],
	tra->param[i*3+0],tra->param[i*3+1],tra->param[i*3+2]
	, t,
	&tra->pt[i], &tra->vt[i], &acc, &jerk);
}	
