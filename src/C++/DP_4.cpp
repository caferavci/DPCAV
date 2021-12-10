#include <math.h>       /* floor */
#include <algorithm> 
#include <iostream>
#include <list>
#include <vector>
#include <fstream>
#include "DynamicArray.h"
#include <omp.h>
#include <stdio.h>
#include <tchar.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h>


using namespace std;

const int _TIME_HORIZON = 150;
const int _TIME_RESOLUTION = 2;
const int _DISTANCE_HORIZON = 1000;
const int _TAO_HORIZON = 5;

const int _MAX_LABEL_COST = 99999999;

float VOT = 0.01;
float VOG = 1;
float WEIGHT = 0.2;

const int _RED_LIGHT_LOCATION = 250;
const int _RED_LIGHT_LOCATION_2 = 750;

int get_Vf(int i);
int getD(int m);
int taoLimit1 = 500;
int taoLimit2 = 1500;
int taoLimit3 = 325* _TIME_RESOLUTION;

enum pathType { rural, highway, urban };

void writeFiles(int time_index, std::string file_name);//traceback, CSV files
char comaType = ',';
std::vector<float> av_cost, path_loss_urban, path_loss_highway, path_loss_rural, risk_probability;
std::vector<int> node_sequence, time_sequence, tao_sequence, capacity;
int leaderTime = 0 * _TIME_RESOLUTION;
//TAZ


int zones[5][4] = { { 0, taoLimit1 -1 , 0, 16 },
{ taoLimit1, taoLimit2 -1, 0, 8 },
{ taoLimit2, 4000, 0, 16 },
{ 4001, 5000 , 0, 5},
{ 5000, 19999, 0, 16 }};


//Followers initial positions
int vehiclesPos[5] = { -7, -21, -28, -35,-42 };
//Followers Di
int vehiclesD[5] = { 14, 14, 14, 14, 14};

float ***g_label_cost_makeSpan = NULL;
float ***g_label_cost_makeSpan_travel = NULL;
int ***g_node_predecessor = NULL;
int ***g_time_predecessor = NULL;
int ***g_tao_predecessor = NULL;

int **g_node_M_vehicles = NULL;
int *lower_limit_tao = NULL;
int *upper_limit_tao = NULL;
int **followersTaus = NULL;
float *riskProbability = NULL;
bool *compareCrash = NULL;
float *Ps = NULL;
bool *redLight = NULL;
bool *redLight2 = NULL;

int Origin = 0;

int Di = 14;
int M = 4;
int TAO = _TAO_HORIZON;
int D = (_DISTANCE_HORIZON + getD(M - 1)) * _TIME_RESOLUTION;//meters
//int D = (_DISTANCE_HORIZON + getD(M - 1)) ;//meters
int T = _TIME_HORIZON * _TIME_RESOLUTION;//second

int main()
{
	g_label_cost_makeSpan = Allocate3DDynamicArray<float>(T, D, TAO);
	g_tao_predecessor = Allocate3DDynamicArray<int>(T, D, TAO);  // pointer to previous SPEED INDEX from the current label at current node and time
	g_node_predecessor = Allocate3DDynamicArray<int>(T, D, TAO); // pointer to previous NODE INDEX from the current label at current node and time
	g_time_predecessor = Allocate3DDynamicArray<int>(T, D, TAO); // pointer to previous TIME INDEX from the current label at current node and time

	g_node_M_vehicles = AllocateDynamicArray<int>(M, _TIME_HORIZON*_TIME_RESOLUTION + 250);  // pointer to previous SPEED INDEX from the current label at current node and time
	redLight = AllocateDynamicVector<bool>(T);
	redLight2 = AllocateDynamicVector<bool>(T);

	int cycle_clock, cycle_clock2;
	for (int t = 0; t < T; t++)
	{
		cycle_clock = t % (100 * _TIME_RESOLUTION);
		if (cycle_clock < (10 * _TIME_RESOLUTION))
			redLight[t] = true;
		else
			redLight[t] = false;
		cycle_clock2 = t % (250 * _TIME_RESOLUTION);
		if (cycle_clock2 < (10 * _TIME_RESOLUTION))
			redLight2[t] = true;
		else
			redLight2[t] = false;
		for (int d = 0; d < D; d++)
		{
			for (int tao = 0; tao < TAO; tao++)
			{
				g_label_cost_makeSpan[t][d][tao] = { (float)_MAX_LABEL_COST }; // 9999.0f;
				g_node_predecessor[t][d][tao] = { _MAX_LABEL_COST };  // pointer to previous NODE INDEX from the current label at current node and time
				g_time_predecessor[t][d][tao] = { _MAX_LABEL_COST };  // pointer to previous TIME INDEX from the current label at current node and time
				g_tao_predecessor[t][d][tao] = { _MAX_LABEL_COST };  // pointer to previous TAO INDEX from the current label at current node and time
			}
		}
	}
	lower_limit_tao = AllocateDynamicVector<int>(D);
	upper_limit_tao = AllocateDynamicVector<int>(D);

	//for (int i = 0; i < D; i++)
	//{
	//	lower_limit_tao[i] = TAO - 1;
	//	upper_limit_tao[i] = TAO - 1;
	//}
	//for (int i = taoLimit1; i < taoLimit2; i++)
	//	lower_limit_tao[i] = TAO - 1;
	//for (int i = taoLimit3; i < taoLimit2; i++)
	//	lower_limit_tao[i] = TAO - 1;
	//	//lower_limit_tao[i] = TAO - 1;

	for (int i = 0; i < D; i++)
	{
		lower_limit_tao[i] = TAO - 1;
		upper_limit_tao[i] = TAO - 1;
	}

	// 1st Approach
	//for (int i = taoLimit1; i < taoLimit2; i++)
	//	lower_limit_tao[i] = 0;
	//int taoLimit1low = taoLimit1 - (315);
	//int taoLimit1middle = taoLimit1 + ((M + 2) * Di);
	//int taoLimit1upper = taoLimit1middle + (get_Vf(taoLimit1 + 1) * (TAO - 2));
	//int taoLimit2low = taoLimit2 - (get_Vf(taoLimit2 - 1) * 1 * (TAO - 1) *(M + 1));
	//int taoLimit2middle = taoLimit2 + ((M + 1) * Di);
	//int taoLimit2upper = taoLimit2middle + (get_Vf(taoLimit2 + 1) * (TAO - 2));

	// 2nd Approach
	int taoLimit1low = taoLimit1 - (get_Vf(taoLimit1 - 1) * 1 * (TAO - 2));
	int taoLimit1middle = taoLimit1 + ((M + 1) * Di);
	int taoLimit1upper = taoLimit1middle + (get_Vf(taoLimit1 + 1) * (TAO - 1));
	int taoLimit2low = taoLimit2 - (get_Vf(taoLimit1 - 1) * 1 * (TAO - 2));
	int taoLimit2middle = taoLimit2 + ((M + 1) * Di);
	int taoLimit2upper = taoLimit2middle + (get_Vf(taoLimit2 + 1) * (TAO - 2));
	for (int i = taoLimit1low; i <= taoLimit1upper; i++)
		lower_limit_tao[i] = 0;
	for (int i = taoLimit2low; i <= taoLimit2upper; i++)
		lower_limit_tao[i] = 0;
	//for (int i = taoLimit3; i < taoLimit2; i++)
		//lower_limit_tao[i] = 0;
	//lower_limit_tao[i] = TAO - 1;

	g_label_cost_makeSpan[leaderTime][Origin][TAO - 1] = 0;

	int  Vf = 0, arc_makespan = 0, tao_change = 0;
	bool red_flag, red_flag2;
	int taoTimeStep = 0;
	int taoChange2 = 1, tauCount = 1, deltaK = 0, deltaTau = 0;
	{//DP:
		for (int t = leaderTime; t < T - 1; t++)
		{
			//cout << "TIME :" << t << endl;
			for (int i = Origin; i < D; i++)
			{
				Vf = (get_Vf(i));
				if ((i <= taoLimit1 && i >= taoLimit1low)|| (i <= taoLimit2 && i >= taoLimit2low))
					Vf = 3;
				//for (int j = i+ (int)(Vf*0.85); j < min((i + Vf), D); j++)
				for (int j = i + 1; j < min((i + Vf), D); j++)
				{
					arc_makespan = 0;
					tao_change = 0;
					taoTimeStep = 1;
					if ((i <= taoLimit2middle && i >= taoLimit2low)|| (i <= taoLimit1middle && i >= taoLimit1low))
						taoTimeStep = 5;
					if ((t % (taoTimeStep)) == 0)// assignment of tao changing time
						tao_change = 1;
					for (int tao1 = lower_limit_tao[i]; tao1 <= upper_limit_tao[i]; tao1++)//first tao loop
					{
						for (int tao2 = max(tao1 - tao_change, lower_limit_tao[i]); tao2 <= min(tao1 + tao_change, upper_limit_tao[i]); tao2++)
						{
							arc_makespan += t + 1 + (tao2 * M);//adding makespan cost
							for (int tx = t + 1; tx <= t + 1 + (tao2 * M); tx++)
							{
								for (int ty = j - getD(M - 1); ty <= j; ty++)
								{
									if (ty == _RED_LIGHT_LOCATION && redLight[tx])
										arc_makespan = _MAX_LABEL_COST;
									if (ty == _RED_LIGHT_LOCATION_2 && redLight2[tx])
										arc_makespan = _MAX_LABEL_COST;
								}
							}
							if (g_label_cost_makeSpan[t + 1][j][tao2] > g_label_cost_makeSpan[t][i][tao1] + arc_makespan)
							{
								g_label_cost_makeSpan[t + 1][j][tao2] = g_label_cost_makeSpan[t][i][tao1] + arc_makespan;
								g_node_predecessor[t + 1][j][tao2] = i;
								g_time_predecessor[t + 1][j][tao2] = t;
								g_tao_predecessor[t + 1][j][tao2] = tao1;
								//cout << "TIME = " << t << " TAO_CHANGE = " << tao_change << " TAOTIMESTEP = " << taoTimeStep << " TAO1 = " << tao1 << " TAO2 = " << tao2 << endl;
							}
						}
					}
				}
			}
		}//end of the leader
		//cout << "now traceback" << endl;
		//traceback
		float min_cost = (float)_MAX_LABEL_COST;
		int min_cost_index = -1;
		int capacity_index = -1;
		int min_cost_value = _MAX_LABEL_COST;
		int capacity_value = _MAX_LABEL_COST;
		int minimum_time_index = -1;

		for (int t = leaderTime; t < T; t++)
		{
			float cost2 = (float)(t)*VOT*WEIGHT;
			float cost1 = g_label_cost_makeSpan[t][D - 1][TAO - 1] * VOG*(1 - WEIGHT);
			//finding minimum index
			if (cost1 + cost2 < min_cost)
			{
				min_cost = cost1 + cost2;//comprehensive
				//min_cost = g_label_cost_makeSpan[D - 1][t];//min-fuel-consumption
				//min_cost = t*VOT*WEIGHT;//fastest
				minimum_time_index = t;
			}
			//finding capacity index
			if (capacity_value == _MAX_LABEL_COST &&  g_label_cost_makeSpan[t][D - 1][TAO - 1] < _MAX_LABEL_COST)
			{
				capacity_value = g_label_cost_makeSpan[t][D - 1][TAO - 1];
				capacity_index = t;
			}
			//cout << "cost " << t << " :" << g_label_cost_makeSpan[t][D - 1][TAO - 1] << endl;
		}
		cout << "time_index :" << minimum_time_index << endl;
		cout << "capacity_time_index :" << capacity_index << endl;
		//writeFiles(minimum_time_index, "minimum");
		//writeFiles(capacity_index, "capacity");
	}




	system("pause");
	return 0;
}