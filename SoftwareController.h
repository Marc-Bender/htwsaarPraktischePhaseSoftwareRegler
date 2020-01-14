#ifndef SOFTWARECONTROLLOOP_H 
	#define SOFTWARECONTROLLER_H 
	
	#include "marcsLanguageExtension.h"
	#include <string.h> // for memcpy
	#include <stdlib.h> // for malloc & free
	#include <stdbool.h>

	typedef struct
	{
		float KP;
		float KI;
		float KD;
		bool D_InFeedbackloop;
	}PIDControllerConstants_t;
	
	typedef struct
	{
		float targetValue;
		float isValue;
	}GenericControlLoopStateValues_t;
	
	typedef struct PIDControlLoopInformation_s PIDControlLoopInformation_t;	
	
	PIDControlLoopInformation_t * createAndInitPIDRegulator(IN_PAR PIDControllerConstants_t * const controllerConstants, IN_PAR const GenericControlLoopStateValues_t * const initialValues);
	void destroyPIDRegulator(IN_PAR PIDControlLoopInformation_t * const controllerToBeFreed);
	
	void updateTargetValue(IN_PAR const float * const newTargetValue, OUT_PAR const PIDControlLoopInformation_t * const pidController);
	void updateIsValue(IN_PAR const float * const newIsValue, OUT_PAR const PIDControlLoopInformation_t * const pidController);
	float regulateWithPIDCharacteristic(IN_PAR const PIDControlLoopInformation_t * const pidController);
	
#endif /*SOFTWARECONTROLLER_H */