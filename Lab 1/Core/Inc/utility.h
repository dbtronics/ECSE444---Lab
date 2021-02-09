#ifndef __UTILITY_H
#define __UTILITY_H


#define SIZE 101 //need to modify this

typedef struct self {
			  float q;
			  float r;
			  float x;
			  float p;
			  float k;
}self;

typedef struct statistics {
	float difference[SIZE];
	float avgDifference;
	float stdDeviation;
	float correlation_coeff;
	float correlation[2*SIZE-1];
	float convolution[2*SIZE -1];
	float outputArray[SIZE];
	float inputArray[SIZE];

}statistics;


extern float c_kalmanfilter(self *filter, float *measurement);

extern void c_analysis(statistics *statistics, float *measurement, float *x);





#endif
