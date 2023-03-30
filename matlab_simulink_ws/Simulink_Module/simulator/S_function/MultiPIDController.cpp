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


#define S_FUNCTION_NAME MultiPIDController
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include<iostream>

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define SAMPLE_TIME_PTR(S)       (real_T *)mxGetPr(ssGetSFcnParam(S,0))
#define NUMBER_PTR(S)       (real_T *)mxGetPr(ssGetSFcnParam(S,1))
#define WIDTH_PTR(S)        (real_T *)mxGetPr(ssGetSFcnParam(S,2))
#define PROPORTION_PARAM(S)   ssGetSFcnParam(S,6)
#define INTEGRAL_PARAM(S)     ssGetSFcnParam(S,7)
#define DERIVATIVE_PARAM(S)   ssGetSFcnParam(S,8)
#define PROPORTION_PTR(S)   (real_T *)mxGetPr(PROPORTION_PARAM(S))
#define INTEGRAL_PTR(S)     (real_T *)mxGetPr(INTEGRAL_PARAM(S))
#define DERIVATIVE_PTR(S)   (real_T *)mxGetPr(DERIVATIVE_PARAM(S))
#define C_LOW_PASS_PTR(S)   (real_T *)mxGetPr(ssGetSFcnParam(S,9))
#define LOWER_BOUNDARY_PARAM(S)   ssGetSFcnParam(S,11)
#define LOWER_BOUNDARY_PTR(S)   (real_T *)mxGetPr(LOWER_BOUNDARY_PARAM(S))
#define UPPER_BOUNDARY_PARAM(S)   ssGetSFcnParam(S,12)
#define UPPER_BOUNDARY_PTR(S)   (real_T *)mxGetPr(UPPER_BOUNDARY_PARAM(S))

#define N_P   mxGetN(PROPORTION_PARAM(S)) //Col
#define N_I   mxGetN(INTEGRAL_PARAM(S))
#define N_D   mxGetN(DERIVATIVE_PARAM(S))
#define N_LB   mxGetN(LOWER_BOUNDARY_PARAM(S))
#define N_UB   mxGetN(UPPER_BOUNDARY_PARAM(S))

#define getFlagPPrmMXA(S) ssGetSFcnParam(S,3)
#define getFlagIPrmMXA(S) ssGetSFcnParam(S,4)
#define getFlagDPrmMXA(S) ssGetSFcnParam(S,5)
#define getFlagResetPrmMXA(S) ssGetSFcnParam(S,10)
#define getFlagP(S)                       \
    ( mxIsLogicalScalar(getFlagPPrmMXA(S)) ? \
      mxIsLogicalScalarTrue(getFlagPPrmMXA(S)) : \
      (mxGetScalar(getFlagPPrmMXA(S)) != 0) )
#define getFlagI(S)                       \
    ( mxIsLogicalScalar(getFlagIPrmMXA(S)) ? \
      mxIsLogicalScalarTrue(getFlagIPrmMXA(S)) : \
      (mxGetScalar(getFlagIPrmMXA(S)) != 0) )
#define getFlagD(S)                       \
    ( mxIsLogicalScalar(getFlagDPrmMXA(S)) ? \
      mxIsLogicalScalarTrue(getFlagDPrmMXA(S)) : \
      (mxGetScalar(getFlagDPrmMXA(S)) != 0) )
#define getFlagReset(S)                       \
    ( mxIsLogicalScalar(getFlagResetPrmMXA(S)) ? \
      mxIsLogicalScalarTrue(getFlagResetPrmMXA(S)) : \
      (mxGetScalar(getFlagResetPrmMXA(S)) != 0) )

#if defined(__WIN32__)||defined(WIN32)||defined(_WIN32)

#else
#include<math.h>
#endif
class PIDController
{
public:
    unsigned int number;
    unsigned int width;
    real_T sampleTime;
    real_T* proportions;
    real_T* integrals;
    real_T* derivatives;
    real_T c_filter;
    bool enable_P;
    bool enable_I;
    bool enable_D;
    bool enableReset;     // Whether to apply integral reset of rising edge 
    //real_T** errorNow;
    // Note: the dimension of the following variables is a concatenation of number width-dimensional data.  
    real_T* errorFormers; // Error value of the previous step, which used to calculate trapezoidal integrals and derivatives. 
    real_T* errorDFormers; // Error derivatives of the previous step, which used to apply low-pass filtering.
    real_T* errorIFormers; // Error integrals of the previous step 
    // bool* resetFormers;    // 
    bool resetFormer;
public:
    PIDController(unsigned int number, unsigned int width, real_T sampleTime,
        bool enable_P, bool enable_I, bool enable_D, bool enableReset,
        const real_T*proportions, const real_T*integrals, const real_T*derivatives, real_T c_filter);
    ~PIDController();
    // Set the coefficient of proportions, integrals and derivatives.
    void setCoefficient(const real_T*proportions,const real_T*integrals,const real_T*derivatives,const real_T c_filter);
    // Calculate the PID output
    // void calculatePIDOutputs(real_T* outputs,const real_T*errorNows);
};

PIDController::PIDController(unsigned int number, unsigned int width, real_T sampleTime,
    bool enable_P, bool enable_I, bool enable_D, bool enableReset,
    const real_T*proportions, const real_T*integrals, const real_T*derivatives, real_T c_filter)
{
    this->number = number;
    this->width  = width;
    this->sampleTime = sampleTime;
    this->enable_P = enable_P;
    this->enable_I = enable_I;
    this->enable_D = enable_D;
    this->enableReset = enableReset;
    this->proportions = new real_T[width];
    this->integrals = new real_T[width];
    this->derivatives = new real_T[width];
    this->setCoefficient(proportions,integrals,derivatives,c_filter);
    //
    if (this->enableReset)
    {
        // this->resetFormers = new bool[this->width*this->number];
        // bool* ptr = this->resetFormers;
        // for (unsigned int i = 0; i < this->width*this->number; i++)
        // {
        //     *ptr++ = false;
        // }
        this->resetFormer = false;
    }
    
    this->errorFormers    = new real_T[this->width*this->number];
    this->errorDFormers   = new real_T[this->width*this->number];
    this->errorIFormers = new real_T[this->width*this->number];
    real_T* ptr1 = this->errorFormers, *ptr2 = this->errorDFormers, *ptr3 = this->errorIFormers;
    for (unsigned int i = 0; i < this->width*this->number; i++)
    {
        *ptr1++ = 0.0;
        // printf("%f,",*(ptr1-1));
        *ptr2++ = 0.0;
        *ptr3++ = 0.0;
    }
    // printf("\n");
}
PIDController::~PIDController()
{
    delete [] this->proportions;
    delete [] this->integrals;
    delete [] this->derivatives;
    delete [] this->errorFormers;
    delete [] this->errorDFormers;
    delete [] this->errorIFormers;
    this->proportions = nullptr;
    this->integrals = nullptr;
    this->derivatives = nullptr;
    this->errorFormers = nullptr;
    this->errorDFormers = nullptr;
    this->errorIFormers = nullptr;
    if (this->enableReset){
        // delete [] this->resetFormers;
        // this->resetFormers = nullptr;
    }
    
}
void PIDController::setCoefficient
    (const real_T*proportions,const real_T*integrals,const real_T*derivatives,const real_T c_filter){
    for (unsigned int i = 0; i < this->width; i++){
        this->proportions[i] = proportions[i];
        this->integrals[i] = integrals[i];
        this->derivatives[i] = derivatives[i];
    }   
    this->c_filter = c_filter;
}


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
    const mxArray *pVal0 = ssGetSFcnParam(S,0);

    if ( !IS_PARAM_DOUBLE(pVal0)) {
        ssSetErrorStatus(S, "Parameter to S-function must be a double scalar");
        return;
    } 

    real_T *NPtr = NUMBER_PTR(S);
    int_T  N = (int_T)floor(*NPtr + 0.5);
    real_T *widthPtr = WIDTH_PTR(S);
    real_T width = *widthPtr;
    real_T *cPtr = C_LOW_PASS_PTR(S);
    real_T c = *cPtr;
    real_T *proportionPtr = PROPORTION_PTR(S);
    real_T *integralPtr = INTEGRAL_PTR(S);
    real_T *derivativePtr = DERIVATIVE_PTR(S);
    real_T* lbPtr = LOWER_BOUNDARY_PTR(S);
    real_T* ubPtr = UPPER_BOUNDARY_PTR(S);
    
    if (N <= 0)
        {ssSetErrorStatus(S,"The number of quadcopters must be positive!"); return;}
    if (c <= 0 || c>1)
        {ssSetErrorStatus(S,"The coefficient of low-pass filter must be from 0 to 1!"); return;}
    if (N_P != width)
        {ssSetErrorStatus(S,"The dimension of  proportion must be equal to the width of per input!");return;}
    if (N_I != width)
        {ssSetErrorStatus(S,"The dimension of  integral must be equal to the width of per input!");return;}
    if (N_D != width)
        {ssSetErrorStatus(S,"The dimension of  derivative must be equal to the width of per input!");return;}
    if (N_LB != width)
        {ssSetErrorStatus(S,"The dimension of  lower boundary must be equal to the width of per input!");return;}
    if (N_UB != width)
        {ssSetErrorStatus(S,"The dimension of  lower boundary must be equal to the width of per input!");return;}
    for (int i = 0; i < width; i++)
    {
        if (lbPtr[i] > ubPtr[i])
            {ssSetErrorStatus(S,"The lower boundary must be lower than the upper boundary!");return;}
    }
    
    
  }
//#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 13);  /* Number of expected parameters */
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
    real_T *N_Ptr = NUMBER_PTR(S);
    int_T  N = (int_T)floor(*N_Ptr + 0.5);

    real_T *W_Ptr = WIDTH_PTR(S);
    int_T  width = (int_T)floor(*W_Ptr + 0.5);

    bool enableReset = (bool)getFlagReset(S);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    
    // if (!ssSetNumInputPorts(S, N+(int_T)enableReset)) return;
    // if (!ssSetNumOutputPorts(S, N)) return;
    // for (int i = 0; i < N; i++){
    //     // ssSetInputPortRequiredContiguous(S,i,1);
    //     ssSetInputPortWidth(S, i, width);
    //     ssSetInputPortDirectFeedThrough(S, i, 1);
    //     ssSetOutputPortWidth(S, i, width);
    // }
    // if (enableReset)
    // {
    //     // ssSetInputPortRequiredContiguous(S,N,1);
    //     ssSetInputPortWidth(S, N, 1);
    //     ssSetInputPortDirectFeedThrough(S, N, 1);
    // }
    if (!ssSetNumInputPorts(S, 1)) return;
    if (!ssSetNumOutputPorts(S, 1)) return;
        // ssSetInputPortRequiredContiguous(S,i,1);
    ssSetInputPortWidth(S, 0, N*width+(int_T)enableReset);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetOutputPortWidth(S, 0, N*width+(int_T)enableReset);
    
    //ssSetInputPortWidth(S, 0, 4);
    //ssSetInputPortDirectFeedThrough(S, 0, 1);


    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    /* Set this S-function as runtime thread-safe for multicore execution */
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
    
    ssSetOptions(S, 
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we have a continuous sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, mxGetScalar(ssGetSFcnParam(S, 0)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);         
}



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
static void mdlStart(SimStruct *S)
{

}
#endif /*  MDL_START */

#define MDL_INITIALIZE_CONDITIONS /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *num_Ptr = NUMBER_PTR(S);
    unsigned int  number = (unsigned int)floor(*num_Ptr + 0.5);
    real_T *w_Ptr = WIDTH_PTR(S);
    unsigned int  width = (unsigned int)floor(*w_Ptr + 0.5);
    real_T *sampleTime_Ptr = SAMPLE_TIME_PTR(S);
    real_T  sampleTime = *sampleTime_Ptr;
    
    bool enable_P = (bool)getFlagP(S);
    bool enable_I = (bool)getFlagI(S);
    bool enable_D = (bool)getFlagD(S);
    bool enableReset = (bool)getFlagReset(S);
    
    real_T* proportions = PROPORTION_PTR(S);
    real_T* integrals = INTEGRAL_PTR(S);
    real_T* derivatives = DERIVATIVE_PTR(S);

    real_T* cf_Ptr = C_LOW_PASS_PTR(S);
    real_T c_filter = *cf_Ptr;
    
    real_T **inputs = (real_T**) ssGetInputPortSignalPtrs(S,0);

    void **PWork = ssGetPWork(S);
    PWork[0] = new PIDController(number, width, sampleTime, enable_P, enable_I, enable_D,enableReset,
                proportions, integrals, derivatives, c_filter);
    PIDController* mypid = (PIDController*)PWork[0];
    real_T* errorFormers = mypid->errorFormers;
    for (unsigned int i = 0; i < mypid->width*mypid->number; i++)
    {
        *errorFormers++ = **inputs++;
    }
}
#endif /* MDL_INITIALIZE_CONDITIONS */

/* Function: mdlOutputs =======================================================
 * Abstract:
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{

    // real_T* u = (real_T*)ssGetInputPortRealSignal(S,0);
    // real_T *y = (real_T*)ssGetOutputPortRealSignal(S,0);
    // real_T* inputs = nullptr;
    real_T *outputs = (real_T*)ssGetOutputPortRealSignal(S,0);
    real_T **inputs = (real_T**) ssGetInputPortSignalPtrs(S,0);

    void **PWork = ssGetPWork(S);
    PIDController *myPIDController = (PIDController*)PWork[0];
    real_T* pPtr = PROPORTION_PTR(S);
    real_T* iPtr = INTEGRAL_PTR(S);
    real_T* dPtr = DERIVATIVE_PTR(S);
    real_T* lbPtr = LOWER_BOUNDARY_PTR(S);
    real_T* ubPtr = UPPER_BOUNDARY_PTR(S);
    real_T* cf_Ptr = C_LOW_PASS_PTR(S);
    real_T c_filter = *cf_Ptr;
    

    real_T *eFormers = myPIDController->errorFormers;
    real_T *eDFormers = myPIDController->errorDFormers;
    real_T *eIFormers = myPIDController->errorIFormers;
    real_T errorProportionNow = 0.0;
    real_T errorDerivativeNow = 0.0;
    real_T errorIntegralNow = 0.0;
    for (unsigned int id = 0; id < myPIDController->number; id++){
        for (unsigned int w = 0; w < myPIDController->width; w++)
        {
            // Proportions
            errorProportionNow = **inputs;
            // Derivate
            errorDerivativeNow = ((**inputs) - (*eFormers))/myPIDController->sampleTime;
            errorDerivativeNow = c_filter * errorDerivativeNow + (1.0 - c_filter) * (*eDFormers); //Low-pass filtering
            // Integral
            errorIntegralNow = (*eIFormers) + ((*eFormers) + (**inputs))/2.0*myPIDController->sampleTime;
            // Combine
            *outputs = (*pPtr)*errorProportionNow + (*iPtr) * errorIntegralNow + (*dPtr) * errorDerivativeNow;
            if (*outputs > *ubPtr)
            {
                *outputs = *ubPtr;
            }
            else if(*outputs < *lbPtr)
            {
                *outputs = *lbPtr;
            }
            
            //printf("%lf,%lf,%lf,%lf,",*inputs,errorIntegralNow,errorDerivativeNow,*outputs);
            //
            *eFormers  = errorProportionNow;
            *eDFormers = errorDerivativeNow;
            *eIFormers = errorIntegralNow;
            eFormers++;
            eDFormers++;
            eIFormers++;
            outputs++;
            // printf("%lf,",*inputs);
            inputs++;
            //
            pPtr++;
            iPtr++;
            dPtr++;
            lbPtr++;
            ubPtr++;
        }
        
        pPtr = myPIDController->proportions;
        iPtr = myPIDController->integrals;
        dPtr = myPIDController->derivatives;
        lbPtr = LOWER_BOUNDARY_PTR(S);
        ubPtr = UPPER_BOUNDARY_PTR(S);
    }
    
    // if (myPIDController->enableReset)
    // {
    //     //printf("%lf\n",**inputs);
    //     real_T* errorIFormer = myPIDController->errorIFormers;
    //     // if ((bool)**inputs == true && myPIDController->resetFormer == false)
    //     if ((bool)**inputs != myPIDController->resetFormer)
    //     {
    //         for (unsigned int i = 0; i < myPIDController->number*myPIDController->width; i++)
    //         {
    //             *errorIFormer++ = 0.0;
    //         }
    //     }
    //     myPIDController->resetFormer = (bool)**inputs;
    // }
}





/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    void **PWork = ssGetPWork(S);
    delete PWork[0];
    PWork[0] = nullptr;
    UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
