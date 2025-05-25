#include "huamo/pwm_data.h"

using namespace std;

int lim(const int min,int data,const int max)
{
	if(data < min)
		data = min;
	else if(data >  max)
		data = max;
	else
		data = data;
	return data;
}

float float_lim(const float minimum, const float data, const float maximum)
{
	float temp = 0;
	if(data >= maximum)
	{
		temp = maximum;
	}
	else if(data <= minimum)
	{
		temp = minimum;
	}
	else
	{
		temp = data;
	}
	
	return temp;
}

vector<float> remote_data_deal(const float forward, const float turn)
{
    vector<float> output(2);
	float	left_temp_data	=	0, right_temp_data = 0, over_data = 0;
	
	if(turn >= 0)
	{
		left_temp_data	=	forward	+	turn;	//左后桨
		if(left_temp_data > 100)
		{
			left_temp_data = 100;
			over_data = left_temp_data - 100;
			right_temp_data = forward - turn - over_data;
		}
		else
		{
			right_temp_data = forward - turn;
		}		
	}
	else if(turn < 0)
	{
		right_temp_data = forward - turn;
		if(right_temp_data > 100)
		{
			right_temp_data = 100;
			over_data = right_temp_data - 100;
			left_temp_data = forward + turn - over_data;
		}
		else
		{
			left_temp_data = forward + turn;
		}
	}
	
	if(left_temp_data	>	1)
	{
		output[0]	=	1550	+	left_temp_data	*	2.5;
	}
	else if(left_temp_data	<	-1)
	{
		output[0]	=	1450	+	left_temp_data	*	2.5;
	}
	else
	{
		output[0]	=	1500;
	}
	output[0] = float_lim(1200,output[0],1800);
	
	right_temp_data	=	forward	-	turn;	//右后
	if(right_temp_data	>	1)
	{
		output[1]	=	1550	+	right_temp_data	*	2.5;
	}
	else if(right_temp_data	<	-1)
	{
		output[1]	=	1450	+	right_temp_data	*	2.5;
	}
	else
	{
		output[1]	=	1500;
	}
	output[1] = float_lim(1200,output[1],1800);

    return output;
}
