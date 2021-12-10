# DPCAV
Dynamic Programming based trajectory optimization of multiple CAVs

This package mainly introduces some pieces of my PhD dissertation. Users can find more information in the following paper published in Transportation Research Part B: Methodologies.

[Wei, Y., Avcı, C., Liu, J., Belezamo, B., Aydın, N., Li, P. T., & Zhou, X. (2017). Dynamic programming-based multi-vehicle longitudinal trajectory optimization with simplified car following models. Transportation research part B: methodological, 106, 102-129.]

## Motivation

While previous studies in the areas of vehicle motion planning have made important contributions in various aspects, it is still critically important to develop mathematically rigorous optimization models and computationally tractable algorithms to (1) consider the dynamic effect of vehicle interactions and also (2) reach the full potential of many system-level performance measures such as throughput, capacity, stability and safety. Along this line, this paper aims to address the following theoretical research questions.

1. How to adapt the current car-following models to model traffic interactions of automated vehicles based on available connectivity and automated functions, and in particular the dynamic process of tight platoon formation and systemlevel control?

2. How to develop a theoretically rigorous optimization model (e.g., in the form of mixed integer programming models) which could be solvable using standard optimization software such as CPLEX? Desirable multi-vehicle trajectory optimization models should be able to not only satisfy critical operational constraints such as obstacle avoidance, but also recognize the inherent nature of car following behavior to optimize platoon-level or system-level performance.

3. How to design on-line trajectory optimization algorithms to improve the performance of coupled AVs in a platoon, under complex traffic conditions with time-dependent capacity bottlenecks and obstacles of moving trajectories?

[Wei, Y., Avcı, C., Liu, J., Belezamo, B., Aydın, N., Li, P. T., & Zhou, X. (2017). Dynamic programming-based multi-vehicle longitudinal trajectory optimization with simplified car following models. Transportation research part B: methodological, 106, 102-129.]: <https://www.sciencedirect.com/science/article/pii/S0191261517301078?casa_token=-R53IsF9YmoAAAAA:41Azyn3bA7OtX_YR2t5pB7dWDlWtFfJOIJi0FoXDFGep4xvtKKpdt4bhMfrgdpsEENs_DPFSY7s>

## Methodology

In our research, using the human-driver car following behavior as the baseline, we are interested in how to adopt a linear car following model to approximate the time-continuous AV trajectories while maintaining the minimum safe driving distances. In the original classical paper by Newell, he derived the linear car following model as an approximation of high-order trajectories (through the mean-value theorem), while our focuses below are on how the underlying AV collision-avoidance behavior leads to space-time relationship between a pair of leading and following vehicles, as the collision-avoidance constraint between a pair of vehicles is a building block of the proposed optimization models.

![alt text](https://github.com/caferavci/DPCAV/blob/main/Media/Newell.png)

By using Newell's foundations on simplified car following models as adapted from his original paper in Fig.1, we can derive the generalized form of car following behaviour for related vehicles in a platoon as follows:

![alt text](https://github.com/caferavci/DPCAV/blob/main/Media/Newell_General_Form.jpg)

For more or deep understanding please read the paper.

## Experiments

In this section, you can find introductory experimantal results focusing on the headway optimization through the controlling reaction time of vehicles.

![alt text](https://github.com/caferavci/DPCAV/blob/main/Media/Experiment_Layout.jpg)

Above Figure shows the layout of hypothetical environment, where the total length of the road segment is 1000 m and total time horizon is 150 s; the free flow speed is 60 kilometers per hour in segment 1 and segment 3, from origin to 250 meters and from 750 meters to 1000 meters, respectively. Segment 2 is a speed-reduction zone with the speed of 30 km/h from 250 meters to 750 meters. At 250 meters is a traffic signal where the red phase duration is 20 s and green is 15 s. In addition, there is a traffic signal at 750 meters where the red phase duration is 35 s and green is 20 s.

The trajectory of the leader vehicle and reaction time in a platoon have been optimized to obtain the minimum total system travel time after reaching its destination. For the configuration of Newell’s car-following model, we set the rear-to-end distance d_0=2 m and τ is variable at intersection regions between 2 s to 0 s with the step size of reaction time change is 0.2 s. There are 5 tightly coupled autonomous and connected vehicles along a one-lane roadway. Following Figures show a typical vehicle trajectory as a black dash line and optimal AVs trajectories as solid blue lines.

![alt text](https://github.com/caferavci/DPCAV/blob/main/Media/Experiment_Results_1.jpg)

While human-driving vehicles have to wait at intersections (shown was dash lines), all AVs can pass through the intersections without waiting by reducing reaction time and optimized speed along the journey. Since the reaction time and speed have been changed at different segments to find the optimal solution, maximum service rates can be also calculated analytically at different locations. To derive more practically useful capacity estimation results under different communication standards, a more systematically designed study should be conducted to provide the guidelines using realistic settings and real-world geometry features. 

In addition, the last vehicle in above Figure appears to violate the speed limit. As noted from paper, it is possible for the following vehicle when the reaction time is changeable in the state transition process. However, as a control variable, the speed of lead vehicle cannot exceed the maximum speed in this mode. Different simulation results can be obtained by assigning different settings such as enforcing speed limit for leading vehicle in transition process in Figure below. The last vehicle keeps its speed and the leading vehicles slowdown in transition process in this approach.

![alt text](https://github.com/caferavci/DPCAV/blob/main/Media/Experiment_Results_2.jpg)

It should be remarked that, the solution obtained by DP may be just one of optimal solutions to reach system-level minimal travel cost. The result with an optimal reaction time greater than the minimum reaction time can improve the safety, to some extent, when using an unchanged minimum reaction time can also reach the minimal system cost.

## Conclusions

The main focus of this research was to present a novel control strategy for autonomous and connected vehicles. The answers to the research questions of this research have been given as below:

1. How to adapt the current car-following models to model traffic interactions of automated vehicles based on available connectivity and automated functions, and in particular the dynamic process of tight platoon formation and system-level control? 

This study constructs a family of efficient optimization models and algorithms to embed vehicle kinematics and minimum safe distance between consecutive following vehicles by using extended Newell’s simplified car-following model. The main advantage of the proposed model is to enhance the service rate by adjusting vehicle trajectory speed and system level platoon reaction time at critical bottlenecks. Unlike similar control strategies, which handle only closed boundary conditions, the proposed model solves efficiently semi-open boundary conditions by using dynamic programming models with travel time, throughput and fuel consumption optimization objectives. The two aspects namely (i) adjustment of system level reaction time in discretized space-time-reaction time network and (ii) focusing on the optimality of system through controlling simultaneously all vehicles in the platoon, contribute new knowledge to the existing vehicle trajectory optimization studies. 

2. How to develop a theoretically rigorous optimization model (e.g., in the form of dynamic programming models) which could be programmable using different programming languages? Desirable multi-vehicle trajectory optimization models should be able to not only satisfy critical operational constraints such as obstacle avoidance, but also recognize the inherent nature of car following behavior to optimize platoon-level or system-level performance.

The results of numerical experiments have revealed efficiency of controlling reaction time as a new control variable, and increasing system level flexibility and optimality by forming vehicle platoons adaptively at critical bottlenecks. Also, these results give projections for communication connectivity considering team based safety when reaction time dynamically changes under communication support conditions. The approaches for dynamic programming under typical time-dependent bottleneck scenarios are examined to show the benefits of optimizing AVs trajectories with achievement of desired goals.

3. How to design on-line trajectory optimization algorithms to improve the performance of coupled AVs in a platoon, under complex traffic conditions with time-dependent capacity bottlenecks and obstacles of moving trajectories?

Through the numerical experiments for configuring platoon level reaction time under complex driving conditions by using real world trajectories data, it can be noted that multiple AVs increase the service rate under supported communication conditions. 

##	Future works and discussions

Future work will mainly focus on 
* using backward and forward DP algorithms to better refine the feasible space prim and iteratively search the feasible ending states for ultimate optimal solutions, 
* better selecting the discretization granularity for our multi-dimensional discretized networks, 
* smoothing vehicle trajectories generated by Newell’s car-following model, 
* performance evaluation of the current model by using different car-following models, 
* developing the merge-diverge algorithms for different platoon level of multiple AVs,
* developing control models for the formation of swarms consisting of multiple AVs in a large scale application.
