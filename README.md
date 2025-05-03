# Reachability_2DRobot
This repository uses [DeepReach](https://github.com/smlbansal/deepreach/tree/public_release).

## Environment Setup
To set up the environment, please take a look at [DeepReach](https://github.com/smlbansal/deepreach/tree/public_release).


## High-Level Structure
* `plotSafeset` plot the _**safe set**_ for time points: 0*s*, 0.5*s*, 1*s*
* `results`: saved the plots and trained models
* `Plot safe sets.ipynb` jupyter notebook for plotting the _**safe set**_
The other structure is explained in [DeepReach](https://github.com/smlbansal/deepreach/tree/public_release)

## Problem description
We consider a 2-dimensional autonomous navigation robot moving at constant speed $v = 1m/s$ in the 2D plane with dynamics
$$\dot{p}_x = v\;cos \theta, \quad \dot{p}_y = v\;cos \theta,$$
where $(p_x, p_y)$ is the robot's position and $\theta \in [-\pi,\pi]$ is a control input (the heading). Using Hamilton-Jacobi reachability (via the DeepReach toolbox), we solve two scenarios over a 1-second horizon:

1. Obstacle Avoidance
    * Obstacle: circular region of radius $0.5m$ at the origin
    * Compute the backward reachable tube (BRT): the set of states from which a collision is unavoidable within $1s$.
    * Plot the safe set for different time points: $0s, 0.5s, and 1s$
2. Goal Reaching
    * Goal: circular region of radius $0.25m$ at the origin
    * Compute the set of states from which the robot can surely reach the goal region within $1s$.
    * Plot the safe set for different time points: $0s, 0.5s, and 1s$
## Train the value function
Obstacle Avoidance

    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_avoid_r0.5 --minWith target --radius 0.5 --velocity 1.0 --set_mode avoid
    
Goal Reaching

    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_reach_r0.25 --minWith target --radius 0.25 --velocity 1.0 --set_mode reach
    
## Plot the safe set using the trained value function
Obstacle Avoidance

    python run_plot_safeSet.py --mode avoid
    
Goal Reaching

    python run_plot_safeSet.py --mode reach
