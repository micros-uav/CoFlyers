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


#define S_FUNCTION_NAME quadcopterDynamics
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include<iostream>
#define NUMBER_PARAM(S) ssGetSFcnParam(S,0)
#define NUMBER_PTR(S) mxGetPr(NUMBER_PARAM(S))

#define MASS_PARAM(S) ssGetSFcnParam(S,1)
#define MASS_PTR(S) mxGetPr(MASS_PARAM(S))

#define INERTIA_PARAM(S) ssGetSFcnParam(S,2)
#define INERTIA_PTR(S) mxGetPr(INERTIA_PARAM(S))
#define N_INERTIA   mxGetN(INERTIA_PARAM(S))

#define LEN_ARM_PARAM(S) ssGetSFcnParam(S,3)
#define LEN_ARM_PTR(S) mxGetPr(LEN_ARM_PARAM(S))

#define CT_PARAM(S) ssGetSFcnParam(S,4)
#define CT_PTR(S) mxGetPr(CT_PARAM(S))

#define CM_PARAM(S) ssGetSFcnParam(S,5)
#define CM_PTR(S) mxGetPr(CM_PARAM(S))

#define X0_PARAM(S) ssGetSFcnParam(S,6)
#define X0_PTR(S) mxGetPr(X0_PARAM(S))
#define N_X0   mxGetN(X0_PARAM(S)) //Col
#define M_X0   mxGetM(X0_PARAM(S)) //Row


#define N_INPUT 4
#define N_STATE 13

#if defined(__WIN32__)||defined(WIN32)||defined(_WIN32)

#else
#include<math.h>
#endif
void clamp_min_max(real_T &val,const real_T &minimum,const real_T &maximum){
    if (val < minimum)
    {
        val = minimum;
    }
    else if(val > maximum)
    {
        val = maximum;
    }
    
}
real_T * axyz_s = nullptr;
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
    real_T *massPtr = MASS_PTR(S);
    real_T *lenArmPtr = LEN_ARM_PTR(S);
    real_T *ctPtr = CT_PTR(S);
    real_T *cmPtr = CM_PTR(S);
    real_T *I = INERTIA_PTR(S);

    int_T  N = (int_T)floor(*NPtr + 0.5);
    real_T mass = *massPtr;
    real_T lenArm = *lenArmPtr;
    real_T ct = *ctPtr;
    real_T cm = *cmPtr;
    
    
    if (N <= 0)
        {ssSetErrorStatus(S,"The number of quadcopters must be positive!"); return;}
    if (mass <= 0)
        {ssSetErrorStatus(S,"The mass must be positive!"); return;}
    if (lenArm <= 0)
        {ssSetErrorStatus(S,"The length of arm must be positive!"); return;}
    if (ct <= 0)
        {ssSetErrorStatus(S,"The thrust coefficient of propeller must be positive!"); return;}
    if (cm <= 0)
        {ssSetErrorStatus(S,"The moment coefficient of propeller must be positive!"); return;}
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
    if ((int_T)N_X0 != N)
        {ssSetErrorStatus(S,"The number of columns for the initial condition must be equal to the number of quadcopters!"); return;}
    if ((int_T)M_X0 != N_STATE)
        {ssSetErrorStatus(S,"The number of rows of the initial condition must be equal to the dimension of the state!"); return;}
    // printf("%d,%d.\n",N_X0,M_X0);
    // for (int i = 0; i < 26; i++)
    // {
    //     printf("%f,",X0_PTR(S)[i]);
    // }
    // printf("\n");
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

    ssSetNumContStates(S, N_STATE*N);
    ssSetNumDiscStates(S, 0);

    
    if (!ssSetNumInputPorts(S, 1)) return;
    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, N_INPUT*N);
    ssSetInputPortDirectFeedThrough(S, 0, 0);
    ssSetOutputPortWidth(S, 0, N_STATE*N);
    ssSetOutputPortWidth(S, 1, 3*N);

    axyz_s = new real_T[3*N];
    for (int i = 0; i < 3*N; i++)
    {
        axyz_s[i] = 0.0;
    }
    
    // for (int i = 0; i < N; i++){
    //     ssSetInputPortWidth(S, i, N_INPUT);
    //     ssSetInputPortDirectFeedThrough(S, i, 0);
    //     ssSetOutputPortWidth(S, i, N_STATE);
    // }
    
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

    real_T *ptr_axyz_s = axyz_s;
    real_T            *y1    = ssGetOutputPortRealSignal(S,1);
    for (int id = 0; id < N; id++)
    {
        // real_T *y    = ssGetOutputPortRealSignal(S,id);
        for (int j = 0; j < 3; j++){
            // *y1++ = dstates[id*N_STATE+j+3];
            *y1++ = *ptr_axyz_s++;
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
    //real_T            *dstates2   = ssGetdX(S);
    real_T            *states    = ssGetContStates(S);
    // InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

    // /* xdot=Ax+Bu */
    // Get parameters
    real_T *NPtr = NUMBER_PTR(S);
    real_T *massPtr = MASS_PTR(S);
    real_T *lenArmPtr = LEN_ARM_PTR(S);
    real_T *ctPtr = CT_PTR(S);
    real_T *cmPtr = CM_PTR(S);
    real_T *I = INERTIA_PTR(S);
    real_T Ix = I[0],Iy = I[1],Iz = I[2];
    int_T  N = (int_T)floor(*NPtr + 0.5);
    real_T mass = *massPtr;
    real_T lenArm = *lenArmPtr;
    real_T ct = *ctPtr;
    real_T cm = *cmPtr;

    real_T dx = lenArm*0.7071;
    real_T ct_dx = ct*dx;
    real_T gravity = 9.81;
    // //Poiter to input
    real_T **u = (real_T**) ssGetInputPortSignalPtrs(S,0);
    // // 
    real_T w0 = 0.0, w1 = 0.0, w2 = 0.0, w3 = 0.0, //RPM of each rotor
            x = 0.0,  y = 0.0,  z = 0.0, vx = 0.0,vy = 0.0,vz = 0.0,
           qw = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, p = 0.0, q = 0.0, r = 0.0,
           ax = 0.0, ay = 0.0, az = 0.0,
          dqw = 0.0,dqx = 0.0,dqy = 0.0, dqz = 0.0,
           dp = 0.0,dq  = 0.0,dr  = 0.0,
            T = 0.0, tau_p = 0.0, tau_q = 0.0, tau_r = 0.0;
    real_T w0_2 = 0.0, w1_2 = 0.0, w2_2 = 0.0, w3_2 = 0.0;
    real_T r02 = 0.0, r12 = 0.0, r22 = 0.0;
    real_T qNorm = 1.0, quaterror = 0.0;
    real_T K_quat = 2.0; //this enforces the magnitude 1 constraint for the quaternion
    
    real_T *ptr_axyz_s = axyz_s;
    
    for (int id = 0; id < N; id++)
    {
        w0 = **u++;
        w1 = **u++;
        w2 = **u++,
        w3 = **u++;
        w0_2 = w0*w0;
        w1_2 = w1*w1;
        w2_2 = w2*w2;
        w3_2 = w3*w3;
        // Get the force and moment of each propeller according to the configure
        T     =    ct*( w0_2 + w1_2 + w2_2 + w3_2);
        tau_p = ct_dx*(-w0_2 + w1_2 + w2_2 - w3_2);
        tau_q = ct_dx*(-w0_2 - w1_2 + w2_2 + w3_2);
        tau_r =    cm*(-w0_2 + w1_2 - w2_2 + w3_2);
        // Get state of quadcopter
        x = *states++;
        y = *states++;
        z = *states++;
        vx = *states++;
        vy = *states++;
        vz = *states++;
        qw = *states++;
        qx = *states++;
        qy = *states++;
        qz = *states++;
        p = *states++;
        q = *states++;
        r = *states++;
        // Converts the Quaternion to Rotation matrix
        qNorm = qw*qw + qx*qx + qy*qy + qz*qz;
        qw = qw/qNorm; qx = qx/qNorm; qy = qy/qNorm; qz = qz/qNorm; //Normalize q
        //r00 = - 2*qy^2 - 2*qz^2 + 1;
        //r01 =   2*qx*qy - 2*qw*qz;
        r02 =   2*qw*qy + 2*qx*qz;
        //r10 =   2*qw*qz + 2*qx*qy;
        //r11 = - 2*qx^2 - 2*qz^2 + 1;
        r12 =   2*qy*qz - 2*qw*qx;
        //r20 =   2*qx*qz - 2*qw*qy;
        //r21 =   2*qw*qx + 2*qy*qz;
        r22 = - 2*qx*qx - 2*qy*qy + 1;

        // r02 =   2*qx*qz - 2*qw*qy;
        // r12 =   2*qw*qx + 2*qy*qz;
        // r22 = - 2*qx*qx - 2*qy*qy + 1;
        // Linear acceleration
        ax = 1/mass * r02 * T;
        ay = 1/mass * r12 * T;
        if(z <= 0){
            az = 1/mass * r22 * T;
        }
        else
        {
            az = 1/mass * r22 * T - gravity;
        }
        // Quaternion derivatices 
        quaterror = 1 - qNorm*qNorm;
        dqw = - (p*qx)/2 - (q*qy)/2 - (qz*r)/2 + K_quat * quaterror*qw;
        dqx = (p*qw)/2 - (q*qz)/2 + (qy*r)/2 + K_quat * quaterror*qx;
        dqy = (p*qz)/2 + (q*qw)/2 - (qx*r)/2 + K_quat * quaterror*qy;
        dqz = (q*qx)/2 - (p*qy)/2 + (qw*r)/2 + K_quat * quaterror*qz;
        // Angular acceleration
        dp = (tau_p + Iy*q*r - Iz*q*r)/Ix;
        dq = (tau_q - Ix*p*r + Iz*p*r)/Iy;
        dr = (tau_r + Ix*p*q - Iy*p*q)/Iz;
        // All derivatices
        *dstates++ = vx;
        *dstates++ = vy;
        *dstates++ = vz;
        *dstates++ = ax;
        *dstates++ = ay;
        *dstates++ = az;
        *dstates++ = dqw;
        *dstates++ = dqx;
        *dstates++ = dqy;
        *dstates++ = dqz;
        *dstates++ = dp;
        *dstates++ = dq;
        *dstates++ = dr;

        *ptr_axyz_s++ = ax;
        *ptr_axyz_s++ = ay;
        *ptr_axyz_s++ = az;
        //printf("%lf,%lf,%lf\n",r02,r12,r22);
        //printf("%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf\n",vx,vy,vz,ax,ay,az,dqw,dqx,dqy,dqz,dp,dq,dr);
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
