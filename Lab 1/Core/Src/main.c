/* Includes ------------------------------------------------------------------*/
#include "main.h" //allows us to specify data types such as uint32

/* Private includes ----------------------------------------------------------*/
#include "kalmanfilter_asm.h"
#include "stdio.h"


typedef struct self {
			  float q;
			  float r;
			  float x;
			  float p;
			  float k;
	  };

float TEST_ARRAY[] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706, 10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493, 9.82154078273};

int main(void){

	uint32_t size = sizeof(TEST_ARRAY)/sizeof(float);
	struct self teststruct = {0.1,0.1,0.1,5,1};

	float result = 0;
	uint32_t isValid = 0;

	  for(uint32_t i=0; i<size; i++){
		  kalmanfilter(&teststruct, &TEST_ARRAY[i], &isValid);
		  if(isValid){
			  result = teststruct.x;
			  printf("Result (self.x) = %f\n",result);
		  }else{
			  printf("Invalid = %ld\n",isValid);
		  }
	  }
	  result = teststruct.x;

}
