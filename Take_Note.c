typedef struct dcl_df13 {
    // coefficients
    float32_t b0;   //!< b0
    float32_t b1;   //!< b1
    float32_t b2;   //!< b2
    float32_t b3;   //!< b3
    float32_t a0;   //!< a0
    float32_t a1;   //!< a1
    float32_t a2;   //!< a2
    float32_t a3;   //!< a3

    //data
    float32_t d0;   //!< e(k)
    float32_t d1;   //!< e(k-1)
    float32_t d2;   //!< e(k-2)
    float32_t d3;   //!< e(k-3)
    float32_t d4;   //!< u(k)
    float32_t d5;   //!< u(k-1)
    float32_t d6;   //!< u(k-2)
    float32_t d7;   //!< u(k-3)

    DCL_DF13_SPS *sps;   //!< Pointer to the shadow parameter set
    DCL_CSS *css;   //!< Pointer to the common support structure
} DCL_DF13;

//! \brief          Executes an immediate 3rd order Direct Form 1 controller on the FPU32
//!                 Implemented as inline C function
//! \param[in] p    Pointer to the DCL_DF13 controller structure
//! \param[in] ek   The servo error
//! \param[in] vk   The partial pre-computed control effort
//! \return         The control effort
//!
static inline float32_t DCL_runDF13_C5(DCL_DF13 *p, float32_t ek, float32_t vk)
{
    p->d4 = (ek * p->b0) + vk;

    return(p->d4);
}

//! \brief          Executes a partial pre-computed 3rd order Direct Form 1 controller on the FPU32
//!                 Implemented as inline C function
//!                 Note: d0 not used
//! \param[in] p    Pointer to the DCL_DF13 controller structure
//! \param[in] ek   The servo error
//! \param[in] uk   The controller output in the previous sample interval
//! \return         The control effort
//!
static inline float32_t DCL_runDF13_C6(DCL_DF13 *p, float32_t ek, float32_t uk)
{
    float32_t v9;

    v9 = (ek * p->b1) + (p->d1 * p->b2) + (p->d2 * p->b3) - (uk * p->a1) - (p->d5 * p->a2) - (p->d6 * p->a3);
    p->d2 = p->d1;
    p->d1 = ek;
    p->d6 = p->d5;
    p->d5 = uk;

    return(v9);
}

#define CLLLC_GV DCL_DF13
#define CLLLC_GV_IMMEDIATE_RUN DCL_runDF13_C5
#define CLLLC_GV_PRECOMPUTE_RUN DCL_runDF13_C6

static inline void CLLLC_runISR2_primToSecPowerFlow(void)
{
    //
    // Read Current and Voltage Measurements
    //
    CLLLC_readSensedSignalsPrimToSecPowerFlow();
    CLLLC_modeDetect();
    if(CLLLC_closeGvLoop == 1)
    {
        CLLLC_gvError = CLLLC_vSecRefSlewed_pu - CLLLC_vSecSensed_pu;
        CLLLC_gvOut = CLLLC_GV_IMMEDIATE_RUN(&CLLLC_gv,
                                       CLLLC_gvError,
                                       CLLLC_gvPartialComputedValue);
        if(CLLLC_gvOut > CLLLC_GV_OUT_MAX) CLLLC_gvOut = CLLLC_GV_OUT_MAX;
        if(CLLLC_gvOut < CLLLC_GV_OUT_MIN) CLLLC_gvOut = CLLLC_GV_OUT_MIN;
        CLLLC_gvPartialComputedValue = CLLLC_GV_PRECOMPUTE_RUN(&CLLLC_gv,
                                                                CLLLC_gvError,
                                                                CLLLC_gvOut);
        //Added new logic for reducing startup current during soft-start by forcing phase shift operation until Vout rises to a few volts
        //
        //Modified logic to force phase shift operation maintaining non-zero phase shift value until Vout rises to a few volts-up
        if(CLLLC_vSecSensed_pu < (0.1*CLLLC_VSEC_NOMINAL_VOLTS / CLLLC_VSEC_MAX_SENSE_VOLTS))
        {
            //=======CLLLC_pwmPhaseShiftPrimLegsRef_pu is initialized to 0.45 (see theinit in clllc.c file)=================
            //in order to start CLLLC in PS mode with high phase shift and gradually reducing the phase shift from there (as below) until Vout rises to 10% of rated value
            CLLLC_pwmPhaseShiftPrimLegsRef_pu = CLLLC_pwmPhaseShiftPrimLegsRef_pu - CLLLC_MAX_PHASE_STEP_STARTUP_PU;
            if(CLLLC_pwmPhaseShiftPrimLegsRef_pu < 0.1f) CLLLC_pwmPhaseShiftPrimLegsRef_pu = 0.1f;
            //Also during initial part of start-up, limit the max freq to 110kHz/0.29 = 379kHz
            //0.29 corresponds to max freq of 379kHz.
            if(CLLLC_gvOut < 0.29f) CLLLC_gvOut = 0.29f;
        }
        else
        {
            if(CLLLC_gvOut < CLLLC_pwmPeriodMin_pu)
            {
            //
            //
            // Phase shift compensation and clamping
            //
            CLLLC_pwmPhaseShiftPrimLegsRef_pu = (CLLLC_pwmPeriodMin_pu - CLLLC_gvOut) * CLLLC_PHASE_COMP_SCALING;
            if(CLLLC_pwmPhaseShiftPrimLegsRef_pu > CLLLC_PHASE_COMP_MAX) CLLLC_pwmPhaseShiftPrimLegsRef_pu = CLLLC_PHASE_COMP_MAX;
            //
            // Clamp period
            //
            //
             CLLLC_gvOut = CLLLC_pwmPeriodMin_pu;
            }
            else
            {
            //Added logic for reducing current spike during transition from phase to freq mode. This slowly reduces the phase shift if phase shift is high during
            //transition from phase to freq mode. This is new code written after discussion with team
            //
            if(CLLLC_pwmPhaseShiftPrimLegsRef_pu > CLLLC_MAX_PHASE_STEP_TRANSITION_PU)
            CLLLC_pwmPhaseShiftPrimLegsRef_pu = CLLLC_pwmPhaseShiftPrimLegsRef_pu - CLLLC_MAX_PHASE_STEP_TRANSITION_PU;
            else CLLLC_pwmPhaseShiftPrimLegsRef_pu = 0.0f;
            }
        }
        CLLLC_pwmPeriod_pu = CLLLC_gvOut;
    }
    else
    {
        CLLLC_gi.d4 = CLLLC_pwmPeriod_pu;
        CLLLC_gi.d5 = CLLLC_pwmPeriod_pu;
        CLLLC_gi.d6 = CLLLC_pwmPeriod_pu;
        CLLLC_gi.d7 = CLLLC_pwmPeriod_pu;
        CLLLC_gv.d4 = CLLLC_pwmPeriod_pu;
        CLLLC_gv.d5 = CLLLC_pwmPeriod_pu;
        CLLLC_gv.d6 = CLLLC_pwmPeriod_pu;
        CLLLC_gv.d7 = CLLLC_pwmPeriod_pu;
        CLLLC_giError = (CLLLC_iSecRefSlewed_pu - CLLLC_iSecSensed_pu);
        CLLLC_gi.d0 = CLLLC_giError;
        CLLLC_gi.d1 = CLLLC_giError;
        CLLLC_gi.d2 = CLLLC_giError;
        CLLLC_gi.d3 = CLLLC_giError;
        CLLLC_giPartialComputedValue = CLLLC_pwmPeriod_pu;
        CLLLC_gvError = (CLLLC_vSecRefSlewed_pu - CLLLC_vSecSensed_pu);
        CLLLC_gv.d0 = CLLLC_gvError;
        CLLLC_gv.d1 = CLLLC_gvError;
        CLLLC_gv.d2 = CLLLC_gvError;
        CLLLC_gv.d3 = CLLLC_gvError;
        CLLLC_gvPartialComputedValue = CLLLC_pwmPeriod_pu;
        CLLLC_pwmPeriod_pu = CLLLC_pwmPeriodRef_pu;
        if(CLLLC_pwmPeriod_pu < CLLLC_pwmPeriodMin_pu)
        {
            CLLLC_pwmPeriod_pu = CLLLC_pwmPeriodMin_pu;
        }
        else if(CLLLC_pwmPeriod_pu > 1.0f)
        {
            CLLLC_pwmPeriod_pu = 1.0f;
        }
    }
        #if CLLLC_CONTROL_MODE == CLLLC_VOLTAGE_MODE
            //Moved soft-start ramp here from ISR3 to resolve slewed Vout not reaching target Vout
                if(CLLLC_vSecSensed_pu > (0.1f*CLLLC_VSEC_NOMINAL_VOLTS / CLLLC_VSEC_MAX_SENSE_VOLTS))
                {
                    if((CLLLC_vSecRef_pu - CLLLC_vSecRefSlewed_pu) >
                    (2.0f * CLLLC_VOLTS_PER_SECOND_SLEW /
                            CLLLC_VSEC_OPTIMAL_RANGE_VOLTS) *
                    (1.0f / (float32_t)CLLLC_ISR3_FREQUENCY_HZ))
                    {
                        CLLLC_vSecRefSlewed_pu = CLLLC_vSecRefSlewed_pu +
                            ((CLLLC_VOLTS_PER_SECOND_SLEW /
                                    CLLLC_VSEC_OPTIMAL_RANGE_VOLTS) *
                          (1.0f / (float32_t)CLLLC_ISR3_FREQUENCY_HZ));
                    }
                    else if((CLLLC_vSecRef_pu - CLLLC_vSecRefSlewed_pu) <
                        - (2.0f * CLLLC_VOLTS_PER_SECOND_SLEW /
                                CLLLC_VSEC_OPTIMAL_RANGE_VOLTS)
                        * (1.0f / (float32_t)CLLLC_ISR3_FREQUENCY_HZ))
                    {
                        CLLLC_vSecRefSlewed_pu = CLLLC_vSecRefSlewed_pu -
                            ((CLLLC_VOLTS_PER_SECOND_SLEW /
                                    CLLLC_VSEC_OPTIMAL_RANGE_VOLTS) *
                         (1.0f / (float32_t)CLLLC_ISR3_FREQUENCY_HZ));
                    }
                    else
                    {
                        CLLLC_vSecRefSlewed_pu = CLLLC_vSecRef_pu;
                    }
                }
        #endif
        
    if(fabsf(CLLLC_pwmPeriod_pu - CLLLC_pwmPeriodSlewed_pu) >
                        CLLLC_MAX_PERIOD_STEP_PU)
    {
        if(CLLLC_pwmPeriod_pu > CLLLC_pwmPeriodSlewed_pu)
        {
            CLLLC_pwmPeriodSlewed_pu = CLLLC_pwmPeriodSlewed_pu +
                                        CLLLC_MAX_PERIOD_STEP_PU;
        }
        else
        {
            CLLLC_pwmPeriodSlewed_pu = CLLLC_pwmPeriodSlewed_pu -
                                        CLLLC_MAX_PERIOD_STEP_PU;
        }
    }
    else
    {
        CLLLC_pwmPeriodSlewed_pu = CLLLC_pwmPeriod_pu;
    }
    CLLLC_pwmFrequency_Hz = (CLLLC_PWMSYSCLOCK_FREQ_HZ /
                              (CLLLC_pwmPeriodSlewed_pu *
                               CLLLC_pwmPeriodMax_ticks));
}

