/*----------------------------------------------------------------\
@ DCSP

Title            : mygimbal_runexp.h
Author           : Yejun_Lee
Created          : 06-06-2026
Modified         : 06-06-2026

Description      : Common experiment runner
----------------------------------------------------------------*/

#ifndef MYGIMBAL_RUNEXP_H
#define MYGIMBAL_RUNEXP_H

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include <conio.h>
#include "NIDAQmx.h"

/* ===========================
		Input File Include
============================= */

#include "mygimbal_designation_step.h"

/* ===========================
		Constants
============================= */


#define UNIT_PI             (double)(3.14159265358979)


// GYRO DATA
#define GYRO_K              (double)((UNIT_PI / 180.0) / (0.67 * 0.001)) // rad/s로 omega가 계산됨
#define GYRO_OFFSET_DATASET (double)(1.35)
#define N_OFFSET            (int)(1000)

// POTENTIO DATA
#define POTEN_K             (double)(0.7677887752) // 1 / 0.01335006 // deg로 계산.
#define POTEN_OFFSET        (double)(2.4967727273) // 2.5 V

// SWITCH
#define SWITCH_ON           (double)(5.0)
#define SWITCH_OFF          (double)(0.0)

// STANDARD VOLTAGE
#define STANDARD_VOLTAGE    (double)(2.5)

// STOP VOLTAGE
#define STOP_VOLTAGE		(double)(2.5)

// POTEN_VOLTAGE_EPSILON
#define EPS_POTEN_VOlT		(double)(0.01)


/*---------------------------
	Linearization const
---------------------------*/
// Gyro offset from dataset
#define GYRO_OFFSET_EXP     (double)(1.3589369749)

// Positive fit: Vgyro_rel = A*x^2 + B*x + C 
#define P_POS_A             (double)(-0.1883522768)
#define P_POS_B             (double)(0.8770818759)
#define P_POS_C             (double)(-0.2016391143)

// Negative fit: Vgyro_rel = A*x^2 + B*x + C 
#define P_NEG_A             (double)(0.1788930309)
#define P_NEG_B             (double)(0.8243849134)
#define P_NEG_C             (double)(0.1242926757)

// Target range based on Vgyro_rel
#define VGYRO_REL_CUT_POS   (double)(0.0136292264)
#define VGYRO_REL_SAT_POS   (double)(0.8163159930)
#define VGYRO_REL_CUT_NEG   (double)(-0.0131707735)
#define VGYRO_REL_SAT_NEG   (double)(-0.8235118642)

// Inverse solving range
#define VCMD_REAL_CUT_POS   (double)(0.2599482344)
#define VCMD_REAL_SAT_POS   (double)(2.2000000000)
#define VCMD_REAL_CUT_NEG   (double)(-0.1732609365)
#define VCMD_REAL_SAT_NEG   (double)(-2.2000000000)

/*---------------------------
	Designation Loop const
---------------------------*/
// Target_angle_deg
#define	TARGET_ANG_DEG		(double)(20.0)

// zeta_c = 0.707, Wc = 16
// P controller const Kp
#define DESIGNATION_KP		(double)(1.2866)  //
// D controller const Kd
#define DESIGNATION_KD		(double)(0.0740) //

// LPF const
#define GYRO_LPF_CUTOFF_FREQ (double)(10)



/* ===========================
		Main Function Declare
============================= */
void Read_AI(void);
void Write_AO(double Vswitch, double Vmotor);

void Program_Initialization(void);

void Import_Time(void);
void Digital_Controller(void);
void Export_Data(void);
void Import_Data(void);
void Time_Management(void);

void Store_Data_To_Buffer(void);

int Check_Stop_Condition(void);

void Data_Recording(void);
void Program_Termination(void);

/* ===============================
	Utility Function Declare
================================ */

double GetWindowTime(void);

double Calculate_GyroBias(TaskHandle taskAI_gyro, int N);

double GyroOutput(double Vgyro, double Vgyro_offset);
double PotentioOutput(double Vpoten);

double LimitVoltage(double x, double xmin, double xmax);

void Emergency_Stop_Check(void);

void SaveFile(const char* filename, int count, double* OutTime, double* OutVcmd, double* OutVc, double* OutVgyro, double* OutOmega_h, double* OutVpoten, double* OutPsi_h);

// Linearization Funtion Declare
double Poly2(double a, double b, double c, double x);

double SolveQuadraticInRange(double a, double b, double c, double Vout_motor, double xmin, double xmax, double reference);

double LinearReferenceMap(double u, double u_start, double u_end, double y_start, double y_end);

double LinearmappingFunction(double Vcmd_output_controller);

// Controller Funtion Declare
double LPF_1D(double x, double y_prev, double freq_cut, double Ts);


#endif