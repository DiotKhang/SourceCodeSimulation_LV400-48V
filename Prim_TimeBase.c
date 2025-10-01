#define COUNT_UP 1
#define COUNT_DOWN 0
#define PRECHARGE_TIME 0.02 // seconds
#define SAMPLING_FREQUENCY 100000 // 100 kHz
//------------------------------------
//------------- INPUTS -------------
//------------------------------------
long double CLOCK_CYCLE_FREQUENCY = 1 / delt;
long double PWM_FREQUENCY = x1;
// long double PWM_PHASE_SHIFT = x2; // Degree

//------------------------------------
//------- DEFINEATIONS ---------
//------------------------------------
long double PWM_TBPRD_MAX = (CLOCK_CYCLE_FREQUENCY / PWM_FREQUENCY / 2);
long double PRECHARGE_MAX = (SAMPLING_FREQUENCY * PRECHARGE_TIME);
static int PWM_Cnt_PrimLeg1 = 0, PWM_Cnt_PrimLeg2 = 0;
static int PWM_Cnt_dir_PrimLeg1 = 0, PWM_Cnt_dir_PrimLeg2 = 0;
static long PreCharge_Count = 0;
static long Phase_PrimLeg = 0;
static int Enable_PhaseShift = 0, Enable_PreCharge = 0;
static double ISR_COUNT_MAX = 0, ISR_Count = 0;
static int ISR = 0;
static int check = 0;
ISR_COUNT_MAX = 1 / (SAMPLING_FREQUENCY * delt);

//------------------------------------
//-------- ISR HAPPEN ----------
//------------------------------------
ISR_Count++;
if (ISR_Count > ISR_COUNT_MAX)
{
	ISR = 1;
	ISR_Count = 0;
}
else ISR = 0;

if (ISR == 1)
{
	// PHASE-SHIFT RAMP-UP
	if (PreCharge_Count >= PRECHARGE_MAX) Enable_PreCharge = 0;
	else Enable_PreCharge = 1;

	if (Enable_PreCharge == 1)
	{
		Enable_PhaseShift = 1;
		PreCharge_Count++;
		Phase_PrimLeg = (PRECHARGE_MAX - PreCharge_Count) * PWM_TBPRD_MAX / PRECHARGE_MAX;
	}
}

//------------------------------------
//--- PHASE SHIFT MODULE -----
//------------------------------------
if (Enable_PhaseShift == 1)
{
	if (PWM_Cnt_PrimLeg1 == Phase_PrimLeg)
	{
		PWM_Cnt_PrimLeg2 = 0;
		Enable_PhaseShift = 0;
	}
}

//------------------------------------
//------ SAWTOOTH LEG1 -------
//------------------------------------
if (PWM_Cnt_PrimLeg1 >= PWM_TBPRD_MAX) PWM_Cnt_dir_PrimLeg1 = COUNT_DOWN;
else if (PWM_Cnt_PrimLeg1 <= 0) PWM_Cnt_dir_PrimLeg1 = COUNT_UP;
if (PWM_Cnt_dir_PrimLeg1 == COUNT_UP) PWM_Cnt_PrimLeg1++;
if (PWM_Cnt_dir_PrimLeg1 == COUNT_DOWN) PWM_Cnt_PrimLeg1--;

//------------------------------------
//-------- SAWTOOTH LEG2 -----
//------------------------------------
if (PWM_Cnt_PrimLeg2 >= PWM_TBPRD_MAX) PWM_Cnt_dir_PrimLeg2 = COUNT_DOWN;
else if (PWM_Cnt_PrimLeg2 <= 0) PWM_Cnt_dir_PrimLeg2 = COUNT_UP;
if (PWM_Cnt_dir_PrimLeg2 == COUNT_UP) PWM_Cnt_PrimLeg2++;
if (PWM_Cnt_dir_PrimLeg2 == COUNT_DOWN) PWM_Cnt_PrimLeg2--;

//------------------------------------
//---------- OUTPUTS -------------
//------------------------------------
y1 = (float)PWM_Cnt_PrimLeg1 / (float)PWM_TBPRD_MAX;
y2 = (float)PWM_Cnt_PrimLeg2 / (float)PWM_TBPRD_MAX;
y3 = Phase_PrimLeg;
y4 = PreCharge_Count;
y5 = PRECHARGE_MAX;