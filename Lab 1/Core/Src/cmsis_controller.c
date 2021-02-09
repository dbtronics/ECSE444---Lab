#include "main.h"
#include "utility.h"
#include "arm_math.h"
#include "math.h"

//BONUS
void cmsis_kalmanfilter(self *state, statistics *cmsis_stats, uint32_t i){
	float n = 0.0; //self.p + self.r
	float y = 0.0; //measurement - self.x
	float z = 0.0; //self.k *(measurement -self.x)
	float m = 0.0; //(1-self.k)
	float constant = 1;

	arm_add_f32(&state->p,&state->q, &state->p, 1); //self.p = self.p + self.q
	arm_add_f32(&state->p,&state->r, &n, 1); //n = self.p + self.r
	(state->k) = (state->p)/n; //self.k = self.p / n
	arm_sub_f32(&cmsis_stats->inputArray[i],&state->x, &y, 1); //y = measurement - self.x
	arm_mult_f32(&state->k,&y, &z, 1); //z = self.k * y
	arm_add_f32(&state->x,&z, &state->x, 1);//self.x = self.x + z
	arm_sub_f32(&constant,&state->k, &m, 1);//m = (1-self.k)
	arm_mult_f32(&m,&state->q, &state->p, 1); //self.p = m*self.p
}


void cmsis_analysis(statistics *cmsis_stats, uint32_t size){
	arm_sub_f32(&cmsis_stats->inputArray,&cmsis_stats->outputArray,&cmsis_stats->difference,size); //calculate (Input stream - Output stream)
	arm_std_f32(&cmsis_stats->difference,size,&cmsis_stats->stdDeviation); //calculate standard deviation of diff
	arm_mean_f32(&cmsis_stats->difference,size,&cmsis_stats->avgDifference); //calculate average of diff
	arm_correlate_f32(&cmsis_stats->inputArray, size, &cmsis_stats->outputArray, size,&cmsis_stats->correlation); //calculate correlation of Input & Output stream
	arm_conv_f32(&cmsis_stats->inputArray, size, &cmsis_stats->outputArray, size, &cmsis_stats->convolution); //calculate convolution of Input & Output stream
}
