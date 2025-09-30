#define fsampling		50000 // 50kHz
#define COUNT_UP        1
#define COUNT_DOWN      2

//------------------------------------
//-------------- INPUT ---------------
//------------------------------------
long double f_sample = 1/delt;
long double PWM_frequency = x1;

//------------------------------------
//---------- DEFINEATIONS ------------
//------------------------------------
static int countTs = 0;
static int interruptCnt = 0;
static int interruptCntMax = 0;
static long PWM_TsCnt = 0;
static int PWM_Cnt = 0;
static int PWM_Cnt_dir = 0;
static int PWM_Update = 0;

//------------- SAWTOOTH -------------
float PWM_Ts = 1 / PWM_frequency;
PWM_TsCnt = (long)((PWM_Ts / delt) / 2);
if (PWM_Cnt >= PWM_TsCnt)
{
	PWM_Cnt_dir = COUNT_DOWN;
}
else if (PWM_Cnt <= 0)
{
	PWM_Cnt_dir = COUNT_UP;
}
if (PWM_Cnt_dir == 1) {
	PWM_Cnt++;
}
if (PWM_Cnt_dir == 0) {
	PWM_Cnt--;
}
//------------------------------------
//-------------- OUTPUT --------------
//------------------------------------
y1 = PWM_Cnt;
y2 = delt;
y3 = PWM_Ts;