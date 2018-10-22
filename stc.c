#include <reg52.h>
 
 
unsigned int percent = 0;//控制50HZ的关键变量
unsigned int pwm_value_1;
unsigned int pwm_value_2;
unsigned int pwm_value_3;
unsigned int pwm_value_4;
 
 
 
 
//引脚位定义
sbit pwm1 = P2^0;
sbit pwm2 = P2^1;
sbit pwm3 = P2^2;
sbit pwm4 = P2^3;
 
 
 
 
void initialTimer0()
{
	TH0 = (65536-66)/256;
	TL0 = (65536-66)%256;
}//定时器装初值
 
void initial()
{
	EA=1;//打开总中断
	ET0 =1;//开中断1
	TMOD = 0x01; //T0工作，采用方式1,16位定时器
	initialTimer0();
	TR0 = 1;//开启T0定时器
	P2=0x00;//P2置低电平
	//4路PWM初始值设定，3号为油门通道其它为舵机通道
	percent = 0;
	pwm_value_1 = 14;
	pwm_value_2 = 14;
	pwm_value_3 = 9;
	pwm_value_4 = 14;
}//启动初始化函数
 
void Timer0() interrupt 1 using 0
{
	percent +=1;
	//每次中断加1
	if(percent == 198)//到此数值是20ms，50hz的周期
	{
		pwm1 = 1;//PWM引脚统一置高电平
		pwm2 = 1;
		pwm3 = 1;
		pwm4 = 1;
		percent = 0;
	}
	//以下4路是PWM的正频宽到期时置低电平
	if(percent == pwm_value_1)
	{
		pwm1 = 0;
	}
	if(percent == pwm_value_2)
	{
		pwm2 = 0;
	}
	if(percent == pwm_value_3)
	{
		pwm3 = 0;
	}
	if(percent == pwm_value_4)
	{
		pwm4 = 0;
	}
	initialTimer0();//重新装初值
}
void delay(unsigned int x)
{
	unsigned i,j;
	for (i = x; i >= 0; --i) {
		for (j = 120; j >= 0; --j) {
			;
		}
	}
}
void main()
{
	initial();
	while(1)
	{//insert you code here...
		;
	}
}
