# Reachability_2Drobot
## Environment Setup
## Train
avoid

    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_avoid_r0.5 --minWith target --goalR 0.5 --velocity 1.0 --set_mode avoid
    
reach

    python run_experiment.py --mode train --experiment_class DeepReach --dynamics_class AutoRobot2D --experiment_name AutoRobot2D_reach_r0 --minWith target --goalR 0.25 --velocity 0.6 --set_mode reach
    
## Plot
