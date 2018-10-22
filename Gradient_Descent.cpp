#include<math.h>
#include<iostream>
using namespace std;
void newton(int n,double x[],double y[],double eps);
double fn(int n,double x[],double y[]);
const double PI = 3.14159265358979323846;
const double  E=  2.718282;
int main()
{
	int i,n=2;
	double y[2],x[2]={0.5,0.5};
	double eps=1.e-08;
	newton(n,x,y,eps);	
	for(i=0;i<n;i++)
	{
		cout<<i<<" "<<x[i]<<" "<<y[i]<<endl;
	}
	return 0;
}
double fn(int n,double x[],double y[])
{
	double s2=0.0;
	y[0]=x[0]-pow(E, x[1]);
	y[1]=x[1]-pow(x[2], 3);
	for(int i=0;i<n;i++)
	{
		s2+=y[i]+y[i];
	}
	return s2;
}
void newton(int n,double x[],double y[],double eps)
{
	double s[3],s0,s1,s2,t,alpha,h=1.e-05;
	while(1)
	{
		s2=fn(n,x,y);
		s0=s2;
		if(s0<eps)
			break;
		s1=0.0;
		for(int i=0;i<n;i++)
		{
			t=x[i];
			x[i]=(1.0+h)*t;
			s2=fn(n,x,y);
			s[i]=(s2-s0)/(h*t);///???
			s1+=s[i]*s[i];
			x[i]=t;
		}
		alpha=s0/s1;
		for(int i=0;i<n;i++)
		{
			x[i]-=alpha*s[i];
			cout<<i<<" "<<x[i];
		}
		cout<<endl;
	}
 
}
