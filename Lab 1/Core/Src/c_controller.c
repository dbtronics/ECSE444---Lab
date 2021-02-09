#include "utility.h"
#include "main.h"
#include "math.h"


float c_kalmanfilter(self *filter, float *measurement){
	filter->p = filter->p + filter->q;
	filter->k = filter->p/(filter->p + filter->r);
	filter->x = filter->x + filter->k*(*measurement - (filter->x));
	filter->p = (1-filter->k)*filter->p;
	return filter->x;
}

void c_analysis(statistics *statistics, float *measurement, float *x){
//		sample size and initialization
		int size = SIZE;
		float difference[size];
		float sum;
		int i;
//		difference values of state estimate with measured estimate
		for (i = 0; i<size; i++){
			difference[i] = measurement[i] - x[i];
			sum = sum+difference[i];
			statistics->difference[i] = difference[i];
		}
//		compute average and store it in a struct
		statistics->avgDifference = sum/(float)size;

//		Needed to calculate stdDeviation for struct
		float variance;
		float varianceSum;
		for (i=0; i<size; i++){
			varianceSum = varianceSum + pow(difference[i] - (statistics->avgDifference), 2);
		}
		variance = varianceSum/(float)size;
		statistics->stdDeviation = sqrt(variance);

//		correlation array calculation
		uint32_t k;
		for (uint32_t i = 0; i< (2*size-1); i++){
			for (k=0;k<size; k++){
				if((i-k) < 0 || (i-k)>size-1){
					continue;
				}else{
					statistics->correlation[i] += statistics->inputArray[k-i] * statistics->outputArray[k];
				}
			}
		}

//		convolution array calculation
		for (uint32_t i = 0; i< (2*size-1); i++){
			for (k=0;k<size; k++){
				if((i-k) < 0 || (i-k)>size-1){
					continue;
				}else{
					statistics->convolution[i] += statistics->inputArray[i-k] * statistics->outputArray[k];
				}
			}
		}

//		calculation for correlation coeff
		float sumX;
		float sumMeasurement;
		float sumXMeasurement;
		float squareSumX;
		float squareSumMeasurement;
		for (i=0; i<size; i++){
			sumX = sumX + x[i];
			sumMeasurement = sumMeasurement + measurement[i];
			sumXMeasurement = sumXMeasurement + x[i]*measurement[i];
			squareSumX = squareSumX + pow(x[i], 2);
			squareSumMeasurement = squareSumMeasurement + pow(measurement[i], 2);
		}
		statistics->correlation_coeff = size*sumXMeasurement - (sumX)*(sumMeasurement);
		statistics->correlation_coeff = statistics->correlation_coeff/sqrt(
				(size*squareSumX - pow(sumX, 2))
				*(size*squareSumMeasurement - pow(sumMeasurement, 2))
				);
}
