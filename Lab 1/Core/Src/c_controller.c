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
	//	sample size and initialization
	//	int size = sizeof(TEST_ARR)/sizeof(float);
		int size = SIZE;
		float difference[size];
		float sum;
		int i;
	//	difference values of state estimate with measured estimate
		for (i = 0; i<size; i++){
			difference[i] = measurement[i] - x[i];
			sum = sum+difference[i];
			statistics->difference[i] = difference[i];
		}
	//	compute average and store it in a struct
		statistics->avgDifference = sum/(float)size;

	//	Needed to calculate stdDeviation for struct
		float variance;
		float varianceSum;


		for (i=0; i<size; i++){
			varianceSum = varianceSum + pow(difference[i] - (statistics->avgDifference), 2);
		}
		variance = varianceSum/(float)size;
		statistics->stdDeviation = sqrt(variance);



	//	calculation for correlation
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

		statistics->correlation = size*sumXMeasurement - (sumX)*(sumMeasurement);
		statistics->correlation = statistics->correlation/sqrt(
				(size*squareSumX - pow(sumX, 2))
				*(size*squareSumMeasurement - pow(sumMeasurement, 2))
				);

	//	calculation for convolution
		uint32_t j;
		float h[size]; //convolution array
		for (uint32_t i = 0; i<size; i++){
			for (j=0;j<size; j++){
				if(measurement[j]-x[i]>0){
					h[i]+= x[i]*(measurement[j]-x[i]);
				}
			}
		}

		for (i = 0; i< size; i++){
			statistics->convolution[i] = h[i];
		}
}
