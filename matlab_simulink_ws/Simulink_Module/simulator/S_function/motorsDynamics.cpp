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


#define S_FUNCTION_NAME motorsDynamics
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include<iostream>
#define NUMBER_PARAM(S) ssGetSFcnParam(S,0)
#define NUMBER_PTR(S) mxGetPr(NUMBER_PARAM(S))

#define K_PARAM(S) ssGetSFcnParam(S,1)
#define K_PTR(S) mxGetPr(K_PARAM(S))

#define X0_PARAM(S) ssGetSFcnParam(S,2)
#define X0_PTR(S) mxGetPr(X0_PARAM(S))
#define N_X0   mxGetN(X0_PARAM(S)) //Col
#define M_X0   mxGetM(X0_PARAM(S)) //Row

#define N_INPUT 4
#define N_STATE 4


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
    real_T *KPtr = K_PTR(S);

    int_T  N = (int_T)floor(*NPtr + 0.5);
    real_T K = *KPtr;

    
    if (N <= 0)
        {ssSetErrorStatus(S,"The number of quadcopters must be positive!"); return;}
    if (K <= 0)
        {ssSetErrorStatus(S,"The reciprocal of time constant of motor must be positive!"); return;}
    
    if ((int_T)N_X0 != N)
        {ssSetErrorStatus(S,"The number of columns for the initial condition must be equal to the number of quadcopters!"); return;}
    if ((int_T)M_X0 != N_STATE)
        {ssSetErrorStatus(S,"The number of rows of the initial condition must be equal to the dimension of the state!"); return;}
  }
//#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 3);  /* Number of expected parameters */
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

    ssSetNumContStates(S, N*N_STATE);
    ssSetNumDiscStates(S, 0);

    
    if (!ssSetNumInputPorts(S, 1)) return;
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, N_INPUT*N);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetOutputPortWidth(S, 0, N_STATE*N);
    
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

#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    Initialize both continuous states to zero.
 */
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *NPtr = NUMBER_PTR(S);
    int_T  N = (int_T)floor(*NPtr + 0.5);
    real_T *x0 = ssGetContStates(S);
    real_T *X0 = X0_PTR(S);
    int_T  lp;
    for (lp=0;lp<N_STATE*N;lp++) { 
        *x0++=*X0++; 
    }
}



/* Function: mdlOutputs =======================================================
 * Abstract:
 *      y = Cx + Du 
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *y    = ssGetOutputPortRealSignal(S,0);
    real_T            *x    = ssGetContStates(S);
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
 
    UNUSED_ARG(tid); /* not used in single tasking mode */

    /* y=Cx+Du */
    //y[0]=C[0][0]*x[0]+C[0][1]*x[1]+D[0][0]*U(0)+D[0][1]*U(1);
    //y[1]=C[1][0]*x[0]+C[1][1]*x[1]+D[1][0]*U(0)+D[1][1]*U(1);
    real_T *NPtr = NUMBER_PTR(S);
    int_T  N = (int_T)floor(*NPtr + 0.5);
    for (int id = 0; id < N; id++)
    {
        // real_T *y    = ssGetOutputPortRealSignal(S,id);
        for (int j = 0; j < N_STATE; j++){
            *y++ = *x++;
        }
    }
}



#define MDL_DERIVATIVES
/* Function: mdlDerivatives =================================================
 * Abstract:
 *      xdot = Ax + Bu
 */
static void mdlDerivatives(SimStruct *S)
{
    real_T            *dstates   = ssGetdX(S);
    real_T            *states    = ssGetContStates(S);

    real_T *NPtr = NUMBER_PTR(S);
    int_T  N = (int_T)floor(*NPtr + 0.5);

    real_T *KPtr = K_PTR(S);
    real_T K = *KPtr;
    // Poiter to input
    real_T **u = (real_T**) ssGetInputPortSignalPtrs(S,0);

    for (int i = 0; i < N_STATE*N; i++)
    {
        *dstates++ = K*((**u++) - (*states++));
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
