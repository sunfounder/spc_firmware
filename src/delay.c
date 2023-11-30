#include "delay.h"
#include "intrins.h"

#ifdef _STC_Y1_12MHz
/*--------------------------------------------------------------
_STC_Y1
时钟频率为 12MHz, 则 1机器周期 = 12*1/12MHz = 1us
即一个  _nop_() 耗时 1us
--------------------------------------------------------------*/
void delayMs(u32 ms)
{
	u32 i;
	while (ms--)
	{
		for (i = 0; i < 250; i++)
		{
			_nop_();
			_nop_();
			_nop_();
			_nop_();
		}
	}
}

void delayUs(u32 us)
{
	while (us--)
	{
		_nop_();
	}
}

void delayS(u32 s)
{
	while (s--)
	{
		delayMs(1000);
	}
}
#endif

#ifdef _STC_Y6_12MHz
/*--------------------------------------------------------------
_STC_Y6
时钟频率为 12MHz, 则 1机器周期 = 1/12MHz = 0.083us
即一个  _nop_() 耗时 0.083us
12 个 _nop_() 耗时 1us
--------------------------------------------------------------*/
void delayUs(u32 us)
{
	u8 i;
	while (us--)
	{
		_nop_();
		_nop_();
		_nop_();
		_nop_();
	}
}

void delayMs(u32 ms)
{
	u16 i;
	while (ms--)
	{
		for (i = 0; i < 3000; i++)
		{
			_nop_();
			_nop_();
			_nop_();
			_nop_();
		}
	}
}

void delayS(u32 s)
{
	while (s--)
	{
		delayMs(1000);
	}
}
#endif

#ifdef _STC32_12MHz
/*--------------------------------------------------------------
_STC_32
--------------------------------------------------------------*/
void delayUs(u32 us) //@12.000MHz
{
	u32 edata i;

	_nop_();
	_nop_();
	_nop_();
	i = 1UL + us * 3;
	while (i)
		i--;
}

void delayMs(u32 ms) //@12.000MHz
{
	u32 edata i;

	_nop_();
	_nop_();
	_nop_();
	i = 2998UL + ms * 3000;
	while (i)
		i--;
}

void delayS(u32 s)
{
	while (s--)
	{
		delayMs(1000);
	}
}
#endif

#ifdef _STC32_35MHz
/*--------------------------------------------------------------
_STC_32
--------------------------------------------------------------*/
void delayUs(u32 us) //@35MHz
{
	u32 edata i;

	_nop_();
	_nop_();
	i = 9UL * (us - 1) + 7UL;
	while (i)
		i--;
}

void delayMs(u32 ms) //@35MHz
{
	u32 edata i;

	_nop_();
	_nop_();
	_nop_();
	i = 8750UL * (ms - 1) + 8748UL;
	while (i)
		i--;
}
#endif

#ifdef _STC32_40MHz
/*--------------------------------------------------------------
_STC_32
--------------------------------------------------------------*/
void delayUs(u32 us) //@40.000MHz
{
	u32 edata i;

	_nop_();
	_nop_();
	_nop_();
	i = 10UL * us - 2;
	while (i)
		i--;
}

void delayMs(u32 ms) //@40.000MHz
{
	u32 edata i;

	_nop_();
	_nop_();
	_nop_();
	i = 10000UL * ms - 2;
	while (i)
		i--;
}
#endif

/*--------------------------------------------------------------
all
--------------------------------------------------------------*/
void delayS(u32 s)
{
	while (s--)
	{
		delayMs(1000);
	}
}
