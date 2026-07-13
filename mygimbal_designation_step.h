
/*----------------------------------------------------------------\
@ DCSP

Title			 : mygimbal_designation_stepinput.h
Author           : Yejun_Lee
Created          : 06-06-2026
Modified         : 06-06-2026

Description		 : step input
----------------------------------------------------------------*/

#ifndef MYGIMBAL_DESIGNATION_STEP
#define MYGIMBAL_DESIGNATION_STEP

/* ===========================
		Constants
============================= */

#define UNIT_PI             (double)(3.14159265358979)

#define SAMPLING_FREQ       (double)(200.0)
#define SAMPLING_TIME       (double)(1.0 / SAMPLING_FREQ)

#define INIT_HOLD_TIME_EXP	(double)(1.0)

#define STEP_AMP			(double)(0.5)
#define STEP_PERIOD			(double)(5.0)

#define FINAL_TIME_EXP		(double)(INIT_HOLD_TIME_EXP + STEP_PERIOD)
#define N_STEP_EXP			(int)(FINAL_TIME_EXP * SAMPLING_FREQ)

#define OUT_FILE_NAME		(const char *)("Step_DesignationLoop.txt")
#define EXP_NAME			(const char *)("Step of Designation Loop")

/* ================================
		Funtion Declaration
================================== */

double Generate_Command(double time);
double Step_Command(double time);

double GetExperimentFinalTime(void);
const char* GetOutputFileName(void);
const char* GetExperimentName(void);


#endif


