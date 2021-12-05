"""
Created on Fri Nov 27 14:48:30 2020

Dr. Cafer Avci wrote this dynamic programming model in Python to optimize CAVs behaviour by using Newell's car following model as presented paper in:
<Wei, Y., Avcı, C., Liu, J., Belezamo, B., Aydın, N., Li, P. T., & Zhou, X. (2017). 
Dynamic programming-based multi-vehicle longitudinal trajectory optimization with simplified car following models. 
Transportation research part B: methodological, 106, 102-129.>

Users are welcome to modify and then share it with all the transportation community. 

The example results can be visualized with MatplotLib 

The code has been tested in Spyder, a Pyhon IDE environment

This code is open-source under GNU general public License (GPL)
@author: cafer.avci@aalto.fi
"""
import math
import csv
import numpy as np

_TIME_HORIZON = 150
_TIME_RESOLUTION = 2
_DISTANCE_HORIZON = 1000
_TAO_HORIZON = 5

_MAX_LABEL_COST = 99999999

_INITIAL_SPEED = 10.0
_ACC_LIMIT = 3
_DECC_LIMIT = 4

_RED_LIGHT_LOCATION = 250
_RED_LIGHT_LOCATION_2 = 750

VOT = 0.01
VOG = 1
WEIGHT = 0.2
taoLimit1 = 500
taoLimit2 = 1500
taoLimit3 = 325* _TIME_RESOLUTION
Di = 14
M = 4
TAO = _TAO_HORIZON
D = (_DISTANCE_HORIZON + (M * Di)) * _TIME_RESOLUTION
T = _TIME_HORIZON * _TIME_RESOLUTION
Origin = 0

g_label_cost_makeSpan = np.zeros((T,D,TAO))
g_time_predecessor = np.zeros((T,D,TAO)) #pointer to previous TIME INDEX from the current label at current node and time
g_node_predecessor = np.zeros((T,D,TAO)) #pointer to previous NODE INDEX from the current label at current node and time
g_tao_predecessor = np.zeros((T,D,TAO)) #pointer to previous SPEED INDEX from the current label at current node and time
g_node_M_vehicles = np.zeros((M, T + (Di * M * _TIME_RESOLUTION)))
redLight = np.zeros((T + _TAO_HORIZON * M + 1), dtype = int)
redLight2 = np.zeros((T + _TAO_HORIZON * M + 1), dtype = int)
#zones = np.zeros((5,4))
zones = np.array([[0, taoLimit1 - 1, 0, 16], [taoLimit1, taoLimit2 - 1, 0, 8], [taoLimit2, 4000, 0, 16], [4001,5000, 0, 5], [5000, 19999, 0, 16]], dtype=int)
#ACC = _ACC_LIMIT #acceleration limit
leaderTime = 0 * _TIME_RESOLUTION
def emission(speed, time_interval):
    speed_km_h = speed * 3600.0 / 1000
    fuel = 0
    if speed_km_h >= 0 and speed_km_h <= 50:
        fuel = (0.0467 * pow(speed_km_h, 2.0) - 2.734 * speed_km_h + 50.0)
    else:
        fuel = 999999
    fuel = ((fuel / 3600) * time_interval) / _TIME_RESOLUTION
    return fuel

def get_Vf(space):
    Vf = int(0)
    for x in range (0, zones.shape[0]):
        if (space >= zones[x, 0] and space <= zones[x, 1]):
            Vf = int(zones[x, 3])
    return Vf
for t in range(0, T + _TAO_HORIZON * M + 1):
    cycle_clock = int(t % (100 * _TIME_RESOLUTION))
    if cycle_clock < (10 * _TIME_RESOLUTION):
    	redLight[t] = 1
    else:
        redLight[t] = 0
    cycle_clock2 = int(t % (250 * _TIME_RESOLUTION))
    if cycle_clock2 < (10 * _TIME_RESOLUTION):
    	redLight2[t] = 1
    else:
        redLight2[t] = 0
for t in range(0, T):
    for d in range(0, D):
    	for tao in range(0, TAO):
            g_label_cost_makeSpan[t,d,tao] = _MAX_LABEL_COST #9999.0f;
            g_node_predecessor[t,d,tao] = _MAX_LABEL_COST #pointer to previous NODE INDEX from the current label at current node and time
            g_time_predecessor[t,d,tao] = _MAX_LABEL_COST #pointer to previous TIME INDEX from the current label at current node and time
            g_tao_predecessor[t,d,tao] = _MAX_LABEL_COST #pointer to previous SPEED INDEX from the current label at current node and time
for t in range(0, T):
    print("t = ", t, " redlight 1 = ", redLight[t])
lower_limit_tao = np.zeros((D), dtype=int)
upper_limit_tao = np.zeros((D), dtype=int)
for d in range(0, D):
    lower_limit_tao[d] = TAO - 1
    upper_limit_tao[d] = TAO - 1

g_label_cost_makeSpan[leaderTime, Origin, TAO - 1] = 0



taoLimit1low = taoLimit1 - (get_Vf(taoLimit1 - 1) * 1 * (TAO - 2))
taoLimit1middle = taoLimit1 + ((M + 1) * Di)
taoLimit1upper = taoLimit1middle + (get_Vf(taoLimit1 + 1) * (TAO - 1))
taoLimit2low = taoLimit2 - (get_Vf(taoLimit1 - 1) * 1 * (TAO - 2))
taoLimit2middle = taoLimit2 + ((M + 1) * Di)
taoLimit2upper = taoLimit2middle + (get_Vf(taoLimit2 + 1) * (TAO - 2))
print("D = ", D, " T = ", T,  " TAO = ", TAO, " taoLimit2low = ", leaderTime, " leaderTime = ", taoLimit2middle,  " taoLimit2upper = ", taoLimit2upper)
print("taoLimit1low = ", taoLimit1low, " taoLimit1middle = ", taoLimit1middle,  " taoLimit1upper = ", taoLimit1upper, " taoLimit2low = ", taoLimit2low, " taoLimit2middle = ", taoLimit2middle,  " taoLimit2upper = ", taoLimit2upper)
for i in range(taoLimit1low, taoLimit1upper + 1):
    lower_limit_tao[i] = 0
for i in range(taoLimit2low, taoLimit2upper + 1):
    lower_limit_tao[i] = 0
Vf = int(0)
arc_makespan = 0
arc_cost = _MAX_LABEL_COST
#j = 0
tao_change = int(0)
red_flag = int(0)
red_flag2 = int(0)
taoTimeStep = int(0)
taoChange2 = int(1)
tauCount = int(1)
deltaK = int(0)
deltaTau = int(0)
print('simulating DP')
asddd = 0
################################################################################################################################
###                                                                                                                          ###
###                                              Dynamic Programming Section                                                 ###
###                                                                                                                          ###
################################################################################################################################
for t in range(leaderTime, T - 1):
    print("TIME = ",t)
    for i in range (Origin, D): 
        Vf = (get_Vf(i))
        if ((i <= taoLimit1 and i >= taoLimit1low) or (i <= taoLimit2 and i >= taoLimit2low)):
            Vf = 3
        for j in range(i + math.floor(Vf*0.85), min(i + Vf, D)):
            arc_makespan = 0
            tao_change = int(0)
            taoTimeStep = int(1)
            if ((i <= taoLimit2middle and i >= taoLimit2low) or (i <= taoLimit1middle and i >= taoLimit1low)):
                taoTimeStep = int(5)
            if ((t % (taoTimeStep)) == 0): #assignment of tao changing time:
                tao_change = 1
            for tao1 in list(np.linspace(lower_limit_tao[i],  upper_limit_tao[i], num = 1 + abs(upper_limit_tao[i] - lower_limit_tao[i]), endpoint=True, dtype = int)):
                for tao2 in list(np.linspace(max(tao1 - tao_change, lower_limit_tao[i]),  min(tao1 + tao_change, upper_limit_tao[i]), num = 1 + abs(min(tao1 + tao_change, upper_limit_tao[i]) - max(tao1 - tao_change, lower_limit_tao[i])), endpoint=True, dtype=int)):
                    arc_makespan += t + 1 + (tao2 * M)
                    for tx in range(t + 1, t + 1 + (tao2 * M) + 1):
                        for ty in range(j - (Di * M), j + 1): #14 * (M - 1) it is total length from the leader to last follower
                            if (ty == _RED_LIGHT_LOCATION):
                                if(redLight[tx] == 1):
                                    arc_makespan = _MAX_LABEL_COST
                            if (ty == _RED_LIGHT_LOCATION_2):
                                if(redLight2[tx] == 1):
                                    arc_makespan = _MAX_LABEL_COST
                    if (g_label_cost_makeSpan[t + 1, j, tao2] > g_label_cost_makeSpan[t, i, tao1] + arc_makespan):
                        g_label_cost_makeSpan[t + 1, j, tao2] = g_label_cost_makeSpan[t, i, tao1] + arc_makespan
                        g_node_predecessor[t + 1, j, tao2] = i
                        g_time_predecessor[t + 1, j, tao2] = t
                        g_tao_predecessor[t + 1, j, tao2] = tao1
                        #print("TIME = ",t," TAO_CHANGE = ",tao_change," TAOTIMESTEP = ",taoTimeStep," TAO1 = ",tao1," TAO2 = ",tao2)

################################################################################################################################     
print('traceback')
#traceback
min_cost = _MAX_LABEL_COST
min_cost_index = -1
capacity_index = -1
min_cost_value = _MAX_LABEL_COST
capacity_value = _MAX_LABEL_COST
minimum_time_index = -1
for t in range(leaderTime, T):
    cost2 = (t)*VOT*WEIGHT
    cost1 = g_label_cost_makeSpan[t, D - 1, TAO - 1] * VOG * (1 - WEIGHT)
    #finding minimum index
    if (cost1 + cost2 < min_cost):
        min_cost = cost1 + cost2 #comprehensive
        #min_cost = g_label_cost_makeSpan[D - 1][t];//min-fuel-consumption
        #min_cost = t*VOT*WEIGHT;//fastest
        minimum_time_index = t
    #finding capacity index
    if (capacity_value == _MAX_LABEL_COST and  g_label_cost_makeSpan[t, D - 1, TAO - 1] < _MAX_LABEL_COST):
        capacity_value = g_label_cost_makeSpan[t, D - 1, TAO - 1]
        capacity_index = t
    print('cost = ', g_label_cost_makeSpan[t, D - 1, TAO - 1])
print('minimum_time_index =', minimum_time_index)

		
		