#include "SoftwareController.h"

typedef struct
{
	float previousRegulationDifference;
	float sumOfRegulationDifferences;
}PIDControllerInternalConstants_t;

typedef struct PIDControlLoopInformation_s
{
	GenericControlLoopStateValues_t * stateValues;
	PIDControllerConstants_t * controllerConstants;
	PIDControllerInternalConstants_t * internalConstants;
} PIDControlLoopInformation_t;


__attribute__((optimize("O3"))) float regulateWithPIDCharacteristic(IN_PAR const PIDControlLoopInformation_t * const pidController)
{
	// uses the approach of three parallel regulators whose outputs are added together for the final output value since that is easier to implement from my perspective
	register float regulatingVariable = 0;

	register const PIDControllerConstants_t *  const controllerConstants = pidController->controllerConstants;
	register PIDControllerInternalConstants_t * const internalConstants = pidController->internalConstants;

	register const float regulationDifference = pidController->stateValues->targetValue - pidController->stateValues->isValue;

	// output=0 + KP*deltaX
	regulatingVariable += controllerConstants->KP * regulationDifference;
	// output=0 + KP*deltaX + KI*sumOfDeltaXs
	regulatingVariable += controllerConstants->KI * internalConstants->sumOfRegulationDifferences;
	// update sumOfRegulationDifferences to incorporate the current difference as well since that is crucial to make this an actual discrete approximation of the integral in the continuous world
	internalConstants->sumOfRegulationDifferences += regulationDifference;
	// output=0 + KP*deltaX + KI*sumOfDeltaXs + KD*deltaXNeu-deltaXAlt
	regulatingVariable += controllerConstants->KD * (regulationDifference - internalConstants->previousRegulationDifference);
	// update previousRegulationDifference to represent the new regulationDifference (which will be the previous in the next run...)
	internalConstants->previousRegulationDifference = regulationDifference;
	
	return regulatingVariable;
}

PIDControlLoopInformation_t * createAndInitPIDRegulator(IN_PAR PIDControllerConstants_t * const controllerConstants, IN_PAR const GenericControlLoopStateValues_t * const initialValues)
{
	PIDControlLoopInformation_t * newRegulator = malloc(sizeof(PIDControlLoopInformation_t));
	if(newRegulator == NULL) // no space left on device
		deathTrap();
	
	newRegulator->stateValues = malloc(sizeof(GenericControlLoopStateValues_t));
	if(newRegulator->stateValues == NULL)
		deathTrap();

	memcpy(newRegulator->stateValues, initialValues, sizeof(GenericControlLoopStateValues_t));
	
	newRegulator->controllerConstants = controllerConstants;
	
	newRegulator->internalConstants = malloc(sizeof(PIDControllerInternalConstants_t));
	if(newRegulator->internalConstants == NULL)
		deathTrap();
	
	newRegulator->internalConstants->previousRegulationDifference = 0;
	newRegulator->internalConstants->sumOfRegulationDifferences = 0;

	return newRegulator;
}

inline __attribute__((always_inline)) void destroyPIDRegulator(IN_PAR PIDControlLoopInformation_t * const controllerToBeFreed)
{
	free(controllerToBeFreed);
}

inline __attribute__((always_inline)) void updateIsValue(IN_PAR const float * const newIsValue, OUT_PAR const PIDControlLoopInformation_t * const pidController)
{
	pidController->stateValues->isValue = *newIsValue;
}

inline __attribute__((always_inline)) void updateTargetValue(IN_PAR const float * const newTargetValue, OUT_PAR const PIDControlLoopInformation_t * const pidController)
{
	pidController->stateValues->targetValue = *newTargetValue;
}
