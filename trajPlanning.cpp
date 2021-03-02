//Source file file with the implementations of the trajectory planning for the Smart Controls projects

#include "trajPlanning.h"

#include<iostream>

using namespace std;

// Sign function
int sign (double a)
{
	int s = 1;
	if (a>0) 
	{
		s=1;
	}
	else if (a<0)
	{
		s=-1;
	}
	return s;
}

// Trajectory planner function
vector<double> trajectoryPlanner (double time, 
								  double x_0, 
							      double v_0, 
							      int v_max, 
								  int a_max, 
						  		  double x_goal)

{		
	
	//cout << "Input parameters are, time, pos, vel, max vel, max acc, goal " << time << " , " << x_0 << " , " << v_0 << " , " << v_max << " , " << a_max << " , " << x_goal << endl;
	
	vector<double> triplet(3,0); // Acc, Vel and Pos container initialized to 0 
	
	// d: default cruise-phase direction 
	double t_stop = abs(v_0/a_max);
	double x_stop;
	if (v_0 > 0) 
	{
		x_stop = x_0 + v_0 * t_stop + 0.5 * (-a_max) * pow(t_stop,2);
	}
	else 
	{
		x_stop = x_0 + v_0 * t_stop + 0.5 * (a_max) * pow(t_stop,2);
	}
	int d = sign (x_goal - x_stop); 

	double t1, t2, t3;
	double d_t1,d_t2,d_t3;
	double d_x1,d_x3, x_1, x_2;

	double a_acc =  d * a_max;
	double a_dec = -d * a_max;
	double v     =  d * v_max;
	double v_abs = 0; 
	double inside_root = 0;

	// Acc and dec stages times
	d_t1 = (v-v_0)/a_acc;
	d_t3 = -v/a_dec; 

	d_x1 = v_0 * d_t1 + 0.5 * a_acc * pow(d_t1,2);
	d_x3 = v * d_t3   + 0.5 * a_dec * pow(d_t3,2);
    
	d_t2 = (x_goal - (x_0 + d_x1 + d_x3))/v;
	

	
	
	
	
	// If d_t2 < 0 -> Overshoot, vel profile is wedge-shaped 
	if (d_t2 < 0)
	{
		//cout << "Overshoot" << endl;
		inside_root = (d * a_max * (x_goal-x_0) + 0.5 * pow(v_0,2));
		if (inside_root < 0)
		{
			inside_root = 0;
		}
		double v_abs = sqrt (inside_root);


		//double v_abs = sqrt (d * a_max * (x_goal-x_0) + 0.5 * pow(v_0,2));
		//cout << "inside sqrt is " << (d * a_max * (x_goal-x_0) + 0.5 * pow(v_0,2)) << endl;
		//cout << "v_abs is " << v_abs << endl;
		d_t1 = abs((d*v_abs - v_0)/a_acc); 
		//cout << "dt1 is " << d_t1 <<endl;
		d_t2 = 0;
		//cout << "dt2 is " << d_t2 <<endl;
		d_t3 = -(d*v_abs)/a_dec; 
		//cout << "dt3 is " << d_t3 <<endl;
				
	}
	
	
	
	
	
	// if (d_t2 < 0)
	// {
	// 	//cout << "Overshoot" << endl;
	// 	double v_abs = sqrt (d * a_max * (x_goal-x_0) + 0.5 * pow(v_0,2));
	// 	d_t1 = abs((d*v_abs - v_0)/a_acc); 
	// 	//cout << "dt1 is " << d_t1 <<endl;
	// 	d_t2 = 0;
	// 	//cout << "dt2 is " << d_t2 <<endl;
	// 	d_t3 = -(d*v_abs)/a_dec; 
	// 	//cout << "dt3 is " << d_t3 <<endl;
				
	// }

	t1 = d_t1;
	t2 = d_t1 + d_t2;
	t3 = d_t1 + d_t2 + d_t3;

	//cout << "t1, t2, t3 are " << t1 << " , " << t2 << " , " << t3 << endl;
	
	int stage;
	if (time < t1)
	{
		stage = 1;
	}
	else if (time >= t1 && time < t2 )
	{
		stage = 2;
	}
	else if (time >= t2 && time < t3)
	{
		stage = 3;
	}
	else if (time >= t3)
	{
		stage = 4;
	}


	switch(stage) 
	{

    	case 1 : 
		//cout << "Acceleration phase" << endl; 
		triplet[0] = a_acc;
		triplet[1] = v_0 + a_acc * time;
		triplet[2] = x_0 + v_0   * time + 0.5 * a_acc * pow(time,2) ;
		break;    

		case 2 : 
		//cout << "Cruise phase" << endl; 
		triplet[0] = 0;
		triplet[1] = v;
		//cout << "Loaded velocity is " << v << endl; 
		x_1 = x_0 + v_0 * t1 + 0.5 * a_acc * pow(t1,2);
		triplet[2] = x_1  + v * (time-t1) ;
		break; 

		case 3 : 
		if (d_t2 != 0)
		{
			//cout << "Deceleration phase" << endl; 
			triplet[0]  = a_dec;
			triplet[1]  = v + a_dec * (time-t2);
			x_1 = x_0 + v_0 * t1 + 0.5 * a_acc * pow(t1,2);
			x_2 = x_1 + v * (d_t2);
			triplet[2]  = x_2 + v * (time-t2) + 0.5 * a_dec * pow((time-t2),2) ;
		}
		else
		{
			//cout << "Deceleration phase" << endl; 
			triplet[0]  = a_dec;
			triplet[1]  = v_0 + a_acc * t1 + a_dec * (time-t2);
			x_1 = x_0 + v_0 * t1 + 0.5 * a_acc * pow(t1,2);
			triplet[2]  = x_1 + (v_0 + a_acc * t1) * (time-t2) + 0.5 * a_dec * pow((time-t2),2)  ; 
		}
		break; 

		case 4 : 
		//cout << "At Target phase" << endl; 
		triplet[0] = 0;
		triplet[1] = 0;
		triplet[2] = x_goal ;
		break; 
	}	

	return triplet;

}

		 
				 



