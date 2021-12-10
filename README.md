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

