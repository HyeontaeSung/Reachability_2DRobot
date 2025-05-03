# Reachability_2Drobot
## Environment Setup
[DeepReach](https://github.com/smlbansal/deepreach/tree/public_release)


## High-Level Structure
* `plotSafeset` plot the _**safe set**_ for time points: 0*s*, 0.5*s*, 1*s*
* `results`: saved the plots and trained models
* `Plot safe sets.ipynb`: jupyter notebook for plotting the _**safe set**_
The other structure is explained in [DeepReach](https://github.com/smlbansal/deepreach/tree/public_release)

## Train
avoid

    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_avoid_r0.5 --minWith target --radius 0.5 --velocity 1.0 --set_mode avoid
    
reach

    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_reach_r0.25 --minWith target --radius 0.25 --velocity 1.0 --set_mode reach
    
## Plot
avoid

    python run_plot_safeSet.py --mode avoid
    
reach

    python run_plot_safeSet.py --mode reach
