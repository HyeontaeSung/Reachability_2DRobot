# Reachability_2Drobot
## High-Level Structure
* 'plotSafeset'
* results
* Plot safe sets.ipynb


## Environment Setup
[DeepReach]https://github.com/smlbansal/deepreach/tree/public_release

## Train
avoid

    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_avoid_r0.5 --minWith target --radius 0.5 --velocity 1.0 --set_mode avoid
    
reach

    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_reach_r0.25 --minWith target --radius 0.25 --velocity 1.0 --set_mode reach
    
## Plot
