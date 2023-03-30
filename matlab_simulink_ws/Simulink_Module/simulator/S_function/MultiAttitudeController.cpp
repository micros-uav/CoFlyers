/*  File    : csfunc.c
 *  Abstract:
 *
 *      Example C-file S-function for defining a continuous system.  
 *
 *      x' = Ax + Bu
 *      y  = Cx + Du
 *
 *  Copyright 1990-2013 The MathWorks, Inc.
 */


#define S_FUNCTION_NAME MultiAttitudeController
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include<iostream>
#define NUMBER_PARAM(S) ssGetSFcnParam(S,0)
#define NUMBER_PTR(S) mxGetPr(NUMBER_PARAM(S))

#define INERTIA_PARAM(S) ssGetSFcnParam(S,1)
#define INERTIA_PTR(S) mxGetPr(INERTIA_PARAM(S))
#define N_INERTIA   mxGetN(INERTIA_PARAM(S))

#define PROPORTION_PARAM(S)   ssGetSFcnParam(S,2)
#define DERIVATIVE_PARAM(S)   ssGetSFcnParam(S,3)
#define N_P   mxGetN(PROPORTION_PARAM(S)) //Col
#define N_D   mxGetN(DERIVATIVE_PARAM(S)) //Col
#define PROPORTION_PTR(S) mxGetPr(PROPORTION_PARAM(S))
#define DERIVATIVE_PTR(S) mxGetPr(DERIVATIVE_PARAM(S))


#define LEN_ARM_PARAM(S) ssGetSFcnParam(S,4)
#define LEN_ARM_PTR(S) mxGetPr(LEN_ARM_PARAM(S))

#define CT_PARAM(S) ssGetSFcnParam(S,5)
#define CT_PTR(S) mxGetPr(CT_PARAM(S))

#define CM_PARAM(S) ssGetSFcnParam(S,6)
#define CM_PTR(S) mxGetPr(CM_PARAM(S))

#define N_INPUT 10
#define N_OUTPUT 4

#if defined(__WIN32__)||defined(WIN32)||defined(_WIN32)

#else
#include<math.h>
#endif
/*====================*
 * S-function methods *
 *====================*/

//#define MDL_CHECK_PARAMETERS
//#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify they are okay.
   */
  static void mdlCheckParameters(SimStruct *S)
  {
    real_T *NPtr = NUMBER_PTR(S);
    real_T *I = INERTIA_PTR(S);

    int_T  N = (int_T)floor(*NPtr + 0.5);
    
    real_T *lenArmPtr = LEN_ARM_PTR(S);
    real_T *ctPtr = CT_PTR(S);
    real_T *cmPtr = CM_PTR(S);
    real_T lenArm = *lenArmPtr;
    real_T ct = *ctPtr;
    real_T cm = *cmPtr;

    if (N <= 0)
        {ssSetErrorStatus(S,"The number of quadcopters must be positive!"); return;}
    if (N_P != 3)
        {ssSetErrorStatus(S,"The dimension of  proportion must be 3!");return;}
    if (N_D != 3)
        {ssSetErrorStatus(S,"The dimension of  derivative must be 3!");return;}

    {
        if (N_INERTIA != 3){
            ssSetErrorStatus(S,"The dimension of inertia must be 3!");
            return;
        }
        if (I[0]<=0 || I[1]<=0 || I[2]<=0 ){
            ssSetErrorStatus(S,"The inertia must be positive!");
            return;
        }
    }
    if (lenArm <= 0)
        {ssSetErrorStatus(S,"The length of arm must be positive!"); return;}
    if (ct <= 0)
        {ssSetErrorStatus(S,"The thrust coefficient of propeller must be positive!"); return;}
    if (cm <= 0)
        {ssSetErrorStatus(S,"The moment coefficient of propeller must be positive!"); return;}
  }
//#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 7);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink */
    }
#endif
    real_T *NPtr = NUMBER_PTR(S);
    int_T  N = (int_T)floor(*NPtr + 0.5);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    
    if (!ssSetNumInputPorts(S, 1)) return;
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, N_INPUT*N);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetOutputPortWidth(S, 0, N_OUTPUT*N);
    
    //ssSetInputPortWidth(S, 0, 4);
    //ssSetInputPortDirectFeedThrough(S, 0, 1);


    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    /* Set this S-function as runtime thread-safe for multicore execution */
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
    
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we have a continuous sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);         
}




/* Function: mdlOutputs =======================================================
 * Abstract:
 *      y = Cx + Du 
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // real_T            *y    = ssGetOutputPortRealSignal(S,0);
    // real_T            *x    = ssGetContStates(S);
    // InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
 
    // UNUSED_ARG(tid); /* not used in single tasking mode */

    // /* y=Cx+Du */
    // //y[0]=C[0][0]*x[0]+C[0][1]*x[1]+D[0][0]*U(0)+D[0][1]*U(1);
    // //y[1]=C[1][0]*x[0]+C[1][1]*x[1]+D[1][0]*U(0)+D[1][1]*U(1);
    // real_T *NPtr = NUMBER_PTR(S);
    // int_T  N = (int_T)floor(*NPtr + 0.5);
    // for (int id = 0; id < N; id++)
    // {
    //     real_T *y    = ssGetOutputPortRealSignal(S,id);
    //     for (int j = 0; j < N_STATE; j++){
    //         *y++ = *x++;
    //     }
    // }
    real_T *outputs = (real_T*)ssGetOutputPortRealSignal(S,0);
    real_T **inputs = (real_T**) ssGetInputPortSignalPtrs(S,0);
    real_T *NPtr = NUMBER_PTR(S);
    // Get parameters
    int_T  N = (int_T)floor(*NPtr + 0.5);
    real_T *kp = PROPORTION_PTR(S);
    real_T *kd = DERIVATIVE_PTR(S);
    real_T *I = INERTIA_PTR(S);
    real_T Ix = I[0], Iy = I[1], Iz = I[2];
    // Make the tuning independent of the inertia matrix.
    real_T kpx = kp[0]/Ix, kpy = kp[1]/Iy, kpz = kp[2]/Iz,
           kdx = kd[0]/Ix, kdy = kd[1]/Iy, kdz = kd[2]/Iz;
    real_T *ctPtr = CT_PTR(S);
    real_T *cmPtr = CM_PTR(S);
    real_T ct = *ctPtr; // Coefficient of thrust
    real_T cm = *cmPtr; // Coefficient of moment
    real_T *lenArmPtr = LEN_ARM_PTR(S);
    real_T lenArm = *lenArmPtr; //Length of arm

    real_T dxct = lenArm*0.7071*ct;
    // Per input
    real_T yaw_rate_d = 0.0, roll_d = 0.0, pitch_d = 0.0, thrust_d = 0.0,
    yaw = 0.0, roll = 0.0, pitch = 0.0, p = 0.0, q = 0.0, r = 0.0;
    // Desired moment
    real_T Mx_d = 0.0, My_d = 0.0, Mz_d = 0.0;
    // Desired rotor velocity
    real_T w0_d_2 = 0.0, w1_d_2 = 0.0, w2_d_2 = 0.0, w3_d_2 = 0.0;
    real_T w0_d = 0.0, w1_d = 0.0, w2_d = 0.0, w3_d = 0.0;
    for (int_T id = 0; id < N; id++)
    {
        // Get inputs
        yaw_rate_d = **inputs++;
        roll_d     = **inputs++;
        pitch_d    = **inputs++;
        thrust_d   = **inputs++;
        yaw        = **inputs++;
        roll       = **inputs++;
        pitch      = **inputs++;
        p          = **inputs++;
        q          = **inputs++;
        r          = **inputs++;
        
        // Attitude controller, need I
        Mx_d = (kpx*(roll_d - roll) + kdx*(0 - p)) * Ix;
        My_d = (kpy*(pitch_d - pitch) + kdy*(0 - q)) * Iy;
        Mz_d = (kdz*(yaw_rate_d - r)) * Iz;
        // thrust_d

        // Calculate rotor velocity
        w0_d_2 = (thrust_d/ct - Mx_d/dxct - My_d/dxct - Mz_d/cm)/4;
        w1_d_2 = (Mz_d/cm + Mx_d/dxct - My_d/dxct + thrust_d/ct)/4;
        w2_d_2 = (Mx_d/dxct - Mz_d/cm + My_d/dxct + thrust_d/ct)/4;
        w3_d_2 = (Mz_d/cm - Mx_d/dxct + My_d/dxct + thrust_d/ct)/4;
        //printf("%f,%f,%f,%f\n",w0_d_2, w1_d_2, w2_d_2, w3_d_2);
        if (w0_d_2<0)
            w0_d_2 = 0.0;
        if (w1_d_2<0)
            w1_d_2 = 0.0;
        if (w2_d_2<0)
            w2_d_2 = 0.0;
        if (w3_d_2<0)
            w3_d_2 = 0.0;
        w0_d = sqrt(w0_d_2);
        w1_d = sqrt(w1_d_2);
        w2_d = sqrt(w2_d_2);
        w3_d = sqrt(w3_d_2);

        //
        *outputs++ = w0_d;
        *outputs++ = w1_d;
        *outputs++ = w2_d;
        *outputs++ = w3_d;
    }
}




/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
