//#include "stm32f4xx.h"

#ifndef __MYMATH_H__
#define __MYMATH_H__
typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	ÎÞ·ûºÅ8Î»ÕûÐÍ±äÁ¿  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		ÓÐ·ûºÅ8Î»ÕûÐÍ±äÁ¿  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	ÎÞ·ûºÅ16Î»ÕûÐÍ±äÁ¿ */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		ÓÐ·ûºÅ16Î»ÕûÐÍ±äÁ¿ */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	ÎÞ·ûºÅ32Î»ÕûÐÍ±äÁ¿ */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		ÓÐ·ûºÅ32Î»ÕûÐÍ±äÁ¿ */
typedef float          fp32;                    /* single precision floating point variable (32bits) µ¥¾«¶È¸¡µãÊý£¨32Î»³¤¶È£© */
typedef double         fp64;                    /* double precision floating point variable (64bits) Ë«¾«¶È¸¡µãÊý£¨64Î»³¤¶È£© */
typedef uint16_t u16;
typedef uint8_t  u8;
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;
#define REAL   float
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII   3.14159f
#define MY_PPPIII_HALF   1.570796f

#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
float my_atan(float x, float y);
float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);
float my_pow(float a);
float my_sqrt(float number);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float x,float zoom);
float my_deathzoom_2(float x,float zoom);
float To_180_degrees(float x);
float my_pow_2_curve(float in,float a,float max);
float limit_mine(float x,float zoom);
float limit_mine2(float x,float min,float max);
float my_deathzoom_21(float x,float zoom);
float my_deathzoom_rc(float x,float zoom);


typedef struct
{ //control parameter
	float h0;
	float v1,v2,r0;
}ESO_X;
void OLDX_SMOOTH_IN_ESOX(ESO_X *eso_in,float in);


typedef struct
{
 float pt[3];
 float vt[3];
 float at[3];
 float ps[3];
 float vs[3];
 float as[3];
 float pe[3];
 float ve[3];
 float ae[3];
 float param[10];
 float Time,time_now,Dis;
 float cost,cost_all;
 float traj_pre_d;
 char defined[3];
	
}_TRA;

extern _TRA traj[10];

void plan_tra(_TRA *tra);
void get_tra(_TRA *tra,float t);

#define ESO_AngularRate_his_length 4

typedef struct
{
	float beta1;
	float beta2;
	
	float T;
	float invT;
	float z_inertia;
	float z1;
	float z2;
	float u;
	float his_z1[ ESO_AngularRate_his_length ];

	float h;

	char err_sign;
	float err_continues_time;
	
	float b;
}ESO_AngularRate;

extern ESO_AngularRate leg_td2[4][3];

typedef struct
{
	unsigned char tracking_mode;
	
	float x1;
	float x2;
	float x3;
	float x4;
	
	float P1;
	float P2;
	float P3;
	float P4;
	
	float r2p , r2n , r3p , r3n , r4p , r4n;
}_TD4;

extern _TD4 leg_td4[4][3];
extern _TD4 bldc_td4[4][3];
extern _TD4 odom_td[3];
//void TD4_setP( TD4* filter , float P);
void TD4_init( _TD4* filter , float P1 , float P2 , float P3 , float P4);
float TD4_track4( _TD4* filter , const float expect , const float h);
float TD4_track3( _TD4* filter , const float expect , const float h );
#endif

