#include <avr/io.h>
#include "SoftwareController.h"


__attribute__((noreturn)) void main(void)
{
	PIDControllerConstants_t testPIDControllerConstants = {.KP = 2.f, .KI = 1.f, .KD=0.05f};
	GenericControlLoopStateValues_t initialValuesOfTestPIDController={.isValue=0,.targetValue=100};
	PIDControlLoopInformation_t * const testPIDController = createAndInitPIDRegulator(&testPIDControllerConstants,&initialValuesOfTestPIDController);
	
    while (1) 
    {

    }
}

