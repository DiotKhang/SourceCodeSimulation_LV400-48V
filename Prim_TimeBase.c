#define COUNT_UP 1
#define COUNT_DOWN 0
#define PRECHARGE_TIME 0.02 // seconds
#define SAMPLING_FREQUENCY 1E5 // 100 kHz
static int INITIAL = 1;
//--------------
//--- INPUTs ---
//--------------
long double CLOCK_CYCLE_FREQUENCY = 1 / delt;
long double PWM_FREQUENCY = x1;

//-------------------
//--- DEFINITIONS ---
//-------------------
// Pre-Charge
static long PWM_TBPRD, PRECHARGE_MAX_COUNT;
static long PreCharge_Count;
// Time Base
static int PWM_Count_PrimLeg1 = 0, PWM_Count_PrimLeg2 = 0, PWM_Count_Dir_PrimLeg1 = 0, PWM_Count_Dir_PrimLeg2 = 0;
static long Phase_PrimLeg = 0, Pre_Phase_PrimLeg = 0;
// Interrupt
static double ISR_COUNT_MAX = 0, ISR_Count = 0;
static int ISR = 0;
// Check
static int check = 0;

//---------------
//--- INITIAL ---
//---------------
if (INITIAL == 1)
{
	INITIAL = 0;
	ISR_COUNT_MAX = 1 / (SAMPLING_FREQUENCY * delt);
	PWM_TBPRD = CLOCK_CYCLE_FREQUENCY / PWM_FREQUENCY / 2;
	PRECHARGE_MAX_COUNT = SAMPLING_FREQUENCY * PRECHARGE_TIME;
	PreCharge_Count = PRECHARGE_MAX_COUNT;
}

//--- ISR HAPPEN ---
ISR_Count++;
if (ISR_Count > ISR_COUNT_MAX)
{
	ISR = 1;
	ISR_Count = 0;
}
else ISR = 0;

if (ISR == 1)
{
	// Phase-Shift ramp up
	if (PreCharge_Count > 0) 
	{
		PreCharge_Count--;
		Phase_PrimLeg = PreCharge_Count * PWM_TBPRD / PRECHARGE_MAX_COUNT;
	}
}

//--- UPDATE PHASE SHIFT MODULE --- //This condition happens when previous phase is different from present phase
if ((Pre_Phase_PrimLeg != Phase_PrimLeg) && (PWM_Count_PrimLeg1 == Phase_PrimLeg))
{
	Pre_Phase_PrimLeg = Phase_PrimLeg;
	PWM_Count_PrimLeg2 = 0;
}

//--- UPDATE FREQUENCY MODULE --- //This condition happens when previous frequency is different from present frequency
// if (Enable_Frequency_Change == 1)		
// {
// 	if (PWM_Count_PrimLeg1 == 0) && (PWM_Count_Dir_PrimLeg1 == COUNT_UP)
// 	{
// 		Enable_Frequency_Change = 0;
// 		PWM_TBPRD = (CLOCK_CYCLE_FREQUENCY / PWM_FREQUENCY / 2);
// 	}
// }

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

//---------------
//--- OUTPUTs ---
//---------------
y1 = (float)PWM_Count_PrimLeg1 / (float)PWM_TBPRD;
y2 = (float)PWM_Count_PrimLeg2 / (float)PWM_TBPRD;
y3 = Phase_PrimLeg;
y4 = PreCharge_Count;
y5 = PRECHARGE_MAX_COUNT;