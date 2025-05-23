# Reachability_2DRobot

This repository builds on [DeepReach](https://github.com/smlbansal/deepreach/tree/public_release) to compute and visualize the backward reachable tube (BRT) for a 2D autonomous navigation robot.

---

## Environment Setup

Follow the [DeepReach](https://github.com/smlbansal/deepreach/tree/public_release) instructions to create the Python environment and install dependencies:  

## High-Level Structure
* `plotSafeset/plot_safeSet.py` defines the Plot_safeset class thats generate plots of the _**safe set**_ for time points: $0s,0.5s,$ and $1s$.
* `results` contains saved plots and trained model checkpoints.
* `Plot safe sets.ipynb` is a Jupyter notebook script for _**safe set**_ plots.
  
For the rest of the code structure, see the DeepReach repo above.

## Problem description
We consider a 2D autonomous navigation robot moving at constant speed $v = 1m/s$ in the 2D plane with dynamics

$$\dot{p}_x = v \mathrm{ } cos \theta, \quad \dot{p}_y = v \mathrm{ } cos \theta,$$

where $(p_x, p_y)$ is the robot's position and $\theta \in [-\pi,\pi]$ is a control input (the heading). Using Hamilton-Jacobi reachability (via the DeepReach Toolbox), we solve two scenarios over a $1s$ horizon:

1. Obstacle Avoidance
    * Obstacle: circular region of radius $0.5m$ at the origin
    * Compute the backward reachable tube (BRT): the set of states from which a collision is unavoidable within $1s$.
    * Plot the _**safe set**_ for different time points: $0s, 0.5s,$ and $1s$
2. Goal Reaching
    * Goal: circular region of radius $0.25m$ at the origin
    * Compute the set of states from which the robot can surely reach the goal region within $1s$.
    * Plot the _**safe set**_ for different time points: $0s, 0.5s,$ and $1s$
## Train the value function
1. Obstacle Avoidance
```
    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_avoid_r0.5 --minWith target --radius 0.5 --velocity 1.0 --set_mode avoid
```    
2. Goal Reaching
```
    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_reach_r0.25 --minWith target --radius 0.25 --velocity 1.0 --set_mode reach
```    
## Plot the safe set using the trained value function

For both scenarios below, the safe set (shown in **blue**) is plotted for time points: $t = 0s, 0.5s,$ and $1s$, with a time horizon of $T = 1s$

1. Obstacle Avoidance
```
    python run_plot_safeSet.py --mode avoid
```
![Safe set at t=0s](results/AutoRobot2D_avoid_r0.5/BRS_safeSet_avoid0.5_plot.png "t = 0 s") 
    
2. Goal Reaching
```
    python run_plot_safeSet.py --mode reach
```
![Safe set at t=0s](results/AutoRobot2D_reach_r0.25/BRS_safeSet_reach0.25_plot.png "t = 0 s") 
