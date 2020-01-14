#include <avr/io.h>
#include "SoftwareController.h"

#pragma GCC diagnostic ignored "-Wmain"
__attribute__((noreturn)) void main(void)
{
	PIDControllerConstants_t testPIDControllerConstants = {.KP = 2.f, .KI = 1.f, .KD = 0.05f, .D_InFeedbackloop = false};
	GenericControlLoopStateValues_t initialValuesOfTestPIDController = {.isValue=0.f , .targetValue=100.f};
	PIDControlLoopInformation_t * const testPIDController = createAndInitPIDRegulator(&testPIDControllerConstants,&initialValuesOfTestPIDController);
	
    while (1) 
    {
		float regulatingVariable = regulateWithPIDCharacteristic(testPIDController);
		regulatingVariable *= 1.f;
    }
}
