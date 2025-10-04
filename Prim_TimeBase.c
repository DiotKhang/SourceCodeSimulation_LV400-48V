#define COUNT_UP							1
#define COUNT_DOWN							0
#define PRECHARGE_TIME						0.05 // seconds
#define PRECHARGE							1
#define SAMPLING_FREQUENCY					100E3
#define SLOW_FREQUENCY						10E3
#define VOLTAGE_MODE 						0
#define CURRENT_MODE						0
#define MAX_PERIOD_STEP_PU					0.05
#define VOLTS_PER_SECOND_SLEW				2.0
// #define VSEC_OPTIMAL_RANGE_VOLTS			60.0
#define VSEC_NOMINAL_VOLTS 					48.0
#define VSEC_MAX_SENSE_VOLTS 				77.15
#define MAX_PWM_SWITCHING_FREQUENCY_HZ		250E3
#define NOMINAL_PWM_SWITCHING_FREQUENCY_HZ	180E3
#define MIN_PWM_SWITCHING_FREQUENCY_HZ		150E3
#define GV_OUT_MIN							-0.1
#define GV_OUT_MAX							0.98
#define Kp_V								0.01
#define Ki_V								0.00001
// Macros
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

//--------------------------------------
// INPUTs
//--------------------------------------
long double PWMSYSCLOCK_FREQ_HZ = 1 / delt;
static int INITIAL = 1;

//--------------------------------------
// Definitions
//--------------------------------------
// Pre-Charge
static long PRECHARGE_MAX_COUNT;
static long PreCharge_Count;
// Time Base
static int PWM_Count_PrimLeg1 = 0, PWM_Count_PrimLeg2 = 0, PWM_Count_Dir_PrimLeg1 = 0, PWM_Count_Dir_PrimLeg2 = 0;
static long Phase_PrimLeg = 0, Pre_Phase_PrimLeg = 0;
static long PWM_TBPRD = 0;
static long pwmFrequencyPrev_Hz = 0, pwmFrequency_Hz = 0;
// Interrupt high frequency
static double ISR_H_COUNT_MAX = 0, ISR_H_Count = 0;
static int ISR_H = 0;
// Interrupt low frequency
static double ISR_L_COUNT_MAX = 0, ISR_L_Count = 0;
static int ISR_L = 0;
// Sensing
static double check = 0;
static double iPrimSensed_pu = 0, iSecSensed_pu = 0, vPrimSensed_pu = 0, vSecSensed_pu = 0, iPrimSensed_TANK = 0, iSecSensed_TANK = 0;
// Control
static double gvOut = 0, gvOutPrev = 0, gvError = 0, gv_SumError = 0, vSecRef_Volts = 0, vSecRefSlewed_pu = 0, vSecRef_pu = 0 ,pwmPeriod_pu = 0, pwmPeriodSlewed_pu = 0, pwmPeriodRef_pu = 0, pwmPeriodMax_ticks = 0, pwmPeriodMin_pu = 0;
//---------------
// Initial
//---------------
if (INITIAL == 1)
{
	INITIAL = 0;
	ISR_H_COUNT_MAX = 1 / (SAMPLING_FREQUENCY * delt);
	ISR_L_COUNT_MAX = 1 / (SLOW_FREQUENCY * delt);
	pwmFrequency_Hz = NOMINAL_PWM_SWITCHING_FREQUENCY_HZ;
	PWM_TBPRD = PWMSYSCLOCK_FREQ_HZ / pwmFrequency_Hz / 2;
	PRECHARGE_MAX_COUNT = SAMPLING_FREQUENCY * PRECHARGE_TIME;
    vSecRef_Volts = VSEC_NOMINAL_VOLTS;
    vSecRef_pu = VSEC_NOMINAL_VOLTS;
	vSecRefSlewed_pu = vSecRef_pu;
	pwmPeriod_pu = (MIN_PWM_SWITCHING_FREQUENCY_HZ /
                          NOMINAL_PWM_SWITCHING_FREQUENCY_HZ);
	pwmPeriodSlewed_pu = pwmPeriod_pu;
	pwmPeriodRef_pu = pwmPeriod_pu;
    pwmPeriodMax_ticks = PWMSYSCLOCK_FREQ_HZ /
                                MIN_PWM_SWITCHING_FREQUENCY_HZ;
	pwmPeriodMin_pu = (MIN_PWM_SWITCHING_FREQUENCY_HZ /
                            MAX_PWM_SWITCHING_FREQUENCY_HZ);

	#if PRECHARGE
	{
		PreCharge_Count = PRECHARGE_MAX_COUNT;
	}
	#else
		PreCharge_Count = 0;
	#endif
}

//--------------------------------------
// Inline function
//--------------------------------------
void readSensedSignalsPrimToSecPowerFlow(void)
{
    vPrimSensed_pu      = x1;
    iPrimSensed_pu      = x2;
    vSecSensed_pu       = x3;
    iSecSensed_pu       = x4;
}

//--------------------------------------
// ISR
//--------------------------------------
ISR_L_Count++;
if (ISR_L_Count > ISR_L_COUNT_MAX)
{
	ISR_L = 1;
	ISR_L_Count = 0;
}
else ISR_L = 0;
if (ISR_L == 1)
{
	readSensedSignalsPrimToSecPowerFlow();
}

ISR_H_Count++;
if (ISR_H_Count > ISR_H_COUNT_MAX)
{
	ISR_H = 1;
	ISR_H_Count = 0;
}
else ISR_H = 0;
if (ISR_H == 1)
{
	// Phase-Shift ramp up
	if (PreCharge_Count > 0) 
	{
		PreCharge_Count--;
		Phase_PrimLeg = PreCharge_Count * PWM_TBPRD / PRECHARGE_MAX_COUNT;
	}
	else
	{
		#if VOLTAGE_MODE
		// Control function in here
		gvError = vSecRefSlewed_pu - vSecSensed_pu;
		// PI control
		gv_SumError += gvError;
		gvOut = Kp_V * gvError + Ki_V * gv_SumError;
		gvOutPrev = gvOut;
		pwmPeriod_pu = gvOut;
    	if(fabs(pwmPeriod_pu) > MAX_PERIOD_STEP_PU)
    	{
    	    // if(pwmPeriod_pu > pwmPeriodSlewed_pu) pwmPeriodSlewed_pu = pwmPeriodSlewed_pu + MAX_PERIOD_STEP_PU/5;
    	    // else pwmPeriodSlewed_pu = pwmPeriodSlewed_pu - MAX_PERIOD_STEP_PU/5;
			pwmPeriodSlewed_pu = pwmPeriodSlewed_pu + pwmPeriod_pu/5;
    	}
		#endif
    	pwmFrequency_Hz = (PWMSYSCLOCK_FREQ_HZ / (pwmPeriodSlewed_pu * pwmPeriodMax_ticks));
		pwmFrequency_Hz = LIMIT(pwmFrequency_Hz, MIN_PWM_SWITCHING_FREQUENCY_HZ, MAX_PWM_SWITCHING_FREQUENCY_HZ);	
	}
}


//--- UPDATE PHASE SHIFT MODULE --- //This condition happens when previous phase is different from present phase
if ((Pre_Phase_PrimLeg != Phase_PrimLeg) && (PWM_Count_PrimLeg1 == Phase_PrimLeg))
{
	Pre_Phase_PrimLeg = Phase_PrimLeg;
	PWM_Count_PrimLeg2 = 0;
}

//--- UPDATE FREQUENCY MODULE --- //This condition happens when previous frequency is different from present frequency
if ((pwmFrequencyPrev_Hz != pwmFrequency_Hz) && (PWM_Count_PrimLeg1 == 0) && (PWM_Count_Dir_PrimLeg1 == COUNT_DOWN) && (PreCharge_Count <= 0))
{
	pwmFrequencyPrev_Hz = pwmFrequency_Hz;
	PWM_TBPRD = (PWMSYSCLOCK_FREQ_HZ / pwmFrequency_Hz / 2);
}

//--- TIMEBASE LEG1 ---
if (PWM_Count_PrimLeg1 >= PWM_TBPRD) PWM_Count_Dir_PrimLeg1 = COUNT_DOWN;
else if (PWM_Count_PrimLeg1 <= 0) PWM_Count_Dir_PrimLeg1 = COUNT_UP;
if (PWM_Count_Dir_PrimLeg1 == COUNT_UP) PWM_Count_PrimLeg1++;
if (PWM_Count_Dir_PrimLeg1 == COUNT_DOWN) PWM_Count_PrimLeg1--;

//--- TIMEBASE LEG2 ---
if (PWM_Count_PrimLeg2 >= PWM_TBPRD) PWM_Count_Dir_PrimLeg2 = COUNT_DOWN;
else if (PWM_Count_PrimLeg2 <= 0) PWM_Count_Dir_PrimLeg2 = COUNT_UP;
if (PWM_Count_Dir_PrimLeg2 == COUNT_UP) PWM_Count_PrimLeg2++;
if (PWM_Count_Dir_PrimLeg2 == COUNT_DOWN) PWM_Count_PrimLeg2--;

//--------------------------------------
// OUTPUTs
//--------------------------------------
y1 = (float)PWM_Count_PrimLeg1 / (float)PWM_TBPRD;
y2 = (float)PWM_Count_PrimLeg2 / (float)PWM_TBPRD;
y3 = vSecSensed_pu;
y4 = gvOutPrev;
y5 = pwmPeriodSlewed_pu;
y6 = gvError;
y7 = gv_SumError;
y8 = vSecRefSlewed_pu;
y9 = pwmPeriod_pu;
y10 = pwmFrequency_Hz;