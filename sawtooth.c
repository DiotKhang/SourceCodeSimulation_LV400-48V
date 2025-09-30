#define fsampling		50000 // 50kHz
#define COUNT_UP        1
#define COUNT_DOWN      0

//------------------------------------
//------------- INPUTS -------------
//------------------------------------
long double CLOCK_CYCLE_FREQUENCY = 1 / delt;
long double PWM_FREQUENCY = x1;

//------------------------------------
//------- DEFINEATIONS ---------
//------------------------------------
static int PWM_Cnt = 0;
static int PWM_Cnt_dir = 0;

//------------------------------------
//---------- SAWTOOTH ----------
//------------------------------------
long PWM_TBPRD_MAX = (long)(CLOCK_CYCLE_FREQUENCY / PWM_FREQUENCY / 2);
if (PWM_Cnt >= PWM_TBPRD_MAX)
{
	PWM_Cnt_dir = COUNT_DOWN;
}
else if (PWM_Cnt <= 0)
{
	PWM_Cnt_dir = COUNT_UP;
}
if (PWM_Cnt_dir == COUNT_UP) {
	PWM_Cnt++;
}
if (PWM_Cnt_dir == COUNT_DOWN) {
	PWM_Cnt--;
}

//------------------------------------
//---------- OUTPUTS -------------
//------------------------------------
y1 = (float)PWM_Cnt / (float)TBPRD_MAX;
y2 = PWM_Cnt;