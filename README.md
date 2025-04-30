# Reachability_2Drobot

# SIA-Lab Reachability Assessment for 2D Planar Robot

This repository contains the code to compute backward reachable tubes (BRTs)
for a planar robot using the DeepReach toolbox, as assigned by PI Somil Bansal’s SIA Lab.

## 📂 Repository Structure

- `dynamics/`  
  - `SimplePlanarRobot.py` — Dubins‐car kinematics (turn-rate control)  
  - `InstantHeadingRobot.py` — direct-heading control model  
- `run_experiment.py` — experiment driver (DeepReach training & validation)  
- `config.yaml` — example training/validation configuration  
- `plot_results/` — sample output plots at t = 0,0.5,1.0 s  
- `requirements.txt` — Python dependencies

## 🚀 Getting Started

1. **Clone** the repo:
   ```bash
   git clone https://github.com/USER/sia-reachability-2d-robot.git
   cd sia-reachability-2d-robot
