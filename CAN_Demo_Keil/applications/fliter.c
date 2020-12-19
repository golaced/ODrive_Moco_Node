#include "include.h"
#include "filter.h"
#include "my_math.h"

#define PI 3.1415926
#define LPF_COF_05Hz  1.0f/(2*PI*0.5)
#define LPF_COF_1t5Hz  1.0f/(2*PI*3)
#define LPF_COF_5t10Hz  1.0f/(2*PI*7)
#define LPF_COF_10t15Hz  1.0f/(2*PI*12)
#define LPF_COF_15t20Hz  1.0f/(2*PI*17)
#define LPF_COF_20t25Hz  1.0f/(2*PI*22)
#define LPF_COF_25t30Hz  1.0f/(2*PI*27)
#define LPF_COF_30t50Hz  1.0f/(2*PI*40)
#define LPF_COF_50t70Hz  1.0f/(2*PI*60)
#define LPF_COF_70t100Hz  1.0f/(2*PI*80)
#define LPF_COF_100tHz  1.0f/(2*PI*100)

void Low_Fass_Filter(float in, float* out, float cutoff_freq, float dt) {
	  float input_reg=in;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) {
        *out = input_reg;
    }
		float lpf_cof;
		int frep=cutoff_freq;
		
		if(cutoff_freq<1)
			lpf_cof=LPF_COF_05Hz;
		else if(cutoff_freq>=1&&cutoff_freq<5)
			lpf_cof=LPF_COF_1t5Hz;
		else if(cutoff_freq>=5&&cutoff_freq<10)
			lpf_cof=LPF_COF_5t10Hz;	
		else if(cutoff_freq>=10&&cutoff_freq<15)
			lpf_cof=LPF_COF_10t15Hz;
		else if(cutoff_freq>=15&&cutoff_freq<20)
			lpf_cof=LPF_COF_15t20Hz;
		else if(cutoff_freq>=20&&cutoff_freq<25)
			lpf_cof=LPF_COF_20t25Hz;
		else if(cutoff_freq>=25&&cutoff_freq<30)
			lpf_cof=LPF_COF_25t30Hz;
		else if(cutoff_freq>=30&&cutoff_freq<50)
			lpf_cof=LPF_COF_30t50Hz;
		else if(cutoff_freq>=50&&cutoff_freq<70)
			lpf_cof=LPF_COF_50t70Hz;
		else if(cutoff_freq>=70&&cutoff_freq<100)
			lpf_cof=LPF_COF_70t100Hz;		
		else
			lpf_cof=LPF_COF_100tHz;
		float rc = lpf_cof;
    float alpha = LIMIT(dt/(dt+rc), 0.0f, 1.0f);
    *out += (input_reg - *out) * alpha;
}


 void Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}

#define MED_WIDTH_NUM 20
#define MED_FIL_ITEM  30

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

u8 med_fil_cnt[MED_FIL_ITEM];
// 1  2  3                                9
float Moving_Median(u8 item,u8 width_num,float in)
{
	u8 i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(width_num==0)
		return in;
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}

		
		return ( tmp[(u16)width_num/2] );
	}
}



/*
 * @brief   
 *   Init fields of structure @kalman1_state.
 *   I make some defaults in this init function:
 *     A = {{1, 0.1}, {0, 1}};
 *     H = {1,0}; 
 *   and @q,@r are valued after prior tests. 
 *
 *   NOTES: Please change A,H,q,r according to your application.
 *
 * @inputs  
 * @outputs 
 * @retval  
 */
void kalman2_init(kalman2_state *state, float *init_x, float (*init_p)[2])
{
    state->x[0]    = init_x[0];
    state->x[1]    = init_x[1];
    state->p[0][0] = init_p[0][0];
    state->p[0][1] = init_p[0][1];
    state->p[1][0] = init_p[1][0];
    state->p[1][1] = init_p[1][1];
    //state->A       = {{1, 0.1}, {0, 1}};
    state->A[0][0] = 1;
    state->A[0][1] = 0.1;
    state->A[1][0] = 0;
    state->A[1][1] = 1;
    //state->H       = {1,0};
    state->H[0]    = 1;
    state->H[1]    = 0;
    //state->q       = {{10e-6,0}, {0,10e-6}};  /* measure noise convariance */
    state->q[0]    = 10e-7;
    state->q[1]    = 10e-7;
    state->r       = 10e-7;  /* estimated error convariance */
}

/*
 * @brief   
 *   2 Dimension kalman filter
 * @inputs  
 *   state - Klaman filter structure
 *   z_measure - Measure value
 * @outputs 
 *   state->x[0] - Updated state value, Such as angle,velocity
 *   state->x[1] - Updated state value, Such as diffrence angle, acceleration
 *   state->p    - Updated estimated error convatiance matrix
 * @retval  
 *   Return value is equals to state->x[0], so maybe angle or velocity.
 */
float kalman2_filter(kalman2_state *state, float z_measure)
{
    float temp0 = 0.0f;
    float temp1 = 0.0f;
    float temp = 0.0f;

    /* Step1: Predict */
    state->x[0] = state->A[0][0] * state->x[0] + state->A[0][1] * state->x[1];
    state->x[1] = state->A[1][0] * state->x[0] + state->A[1][1] * state->x[1];
    /* p(n|n-1)=A^2*p(n-1|n-1)+q */
    state->p[0][0] = state->A[0][0] * state->p[0][0] + state->A[0][1] * state->p[1][0] + state->q[0];
    state->p[0][1] = state->A[0][0] * state->p[0][1] + state->A[1][1] * state->p[1][1];
    state->p[1][0] = state->A[1][0] * state->p[0][0] + state->A[0][1] * state->p[1][0];
    state->p[1][1] = state->A[1][0] * state->p[0][1] + state->A[1][1] * state->p[1][1] + state->q[1];

    /* Step2: Measurement */
    /* gain = p * H^T * [r + H * p * H^T]^(-1), H^T means transpose. */
    temp0 = state->p[0][0] * state->H[0] + state->p[0][1] * state->H[1];
    temp1 = state->p[1][0] * state->H[0] + state->p[1][1] * state->H[1];
    temp  = state->r + state->H[0] * temp0 + state->H[1] * temp1;
    state->gain[0] = temp0 / temp;
    state->gain[1] = temp1 / temp;
    /* x(n|n) = x(n|n-1) + gain(n) * [z_measure - H(n)*x(n|n-1)]*/
    temp = state->H[0] * state->x[0] + state->H[1] * state->x[1];
    state->x[0] = state->x[0] + state->gain[0] * (z_measure - temp); 
    state->x[1] = state->x[1] + state->gain[1] * (z_measure - temp);

    /* Update @p: p(n|n) = [I - gain * H] * p(n|n-1) */
    state->p[0][0] = (1 - state->gain[0] * state->H[0]) * state->p[0][0];
    state->p[0][1] = (1 - state->gain[0] * state->H[1]) * state->p[0][1];
    state->p[1][0] = (1 - state->gain[1] * state->H[0]) * state->p[1][0];
    state->p[1][1] = (1 - state->gain[1] * state->H[1]) * state->p[1][1];

    return state->x[0];
}