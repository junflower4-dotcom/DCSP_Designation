
/*----------------------------------------------------------------\
@ HGU_MCU_DCSP

Title			 : mygimbal_designation_step.c
Author           : Yejun_Lee
Created          : 06-06-2026
Modified         : 07-13-2026

----------------------------------------------------------------*/

#include "mygimbal_designation_step.h"
#include <math.h>

// Generate_Command
double Generate_Command(double time)
{
	return Step_Command(time);
}

// Step_Command
double Step_Command(double time)
{
	if (time < INIT_HOLD_TIME_EXP)
	{
		return 0.0;
	}

	return STEP_AMP;
}

// Exp_FinalTime
double GetExperimentFinalTime(void)
{
	return FINAL_TIME_EXP;
}

// ExpFile_Name
const char* GetOutputFileName(void)
{
	return OUT_FILE_NAME;
}
// Exp_Name
const char* GetExperimentName(void)
{
	return "Step";
}
