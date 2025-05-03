import torch
import matplotlib.pyplot as plt
import os
from dynamics.dynamics import AutoRobot2D
from utils import modules, dataio

class Plot_safeset():
    """
    Plot the BRT safe set for a 2D autonomous robot using a trained DeepReach model.
    Args:

        set_mode (str): 'avoid' for obstacle-avoidance, 'reach' for goal-reaching.
        time_points (list of float): 
            backward times from T = 1.0s (in seconds) at which to evaluate/plot the safe set.
    """
    def __init__(
        self,
        set_mode='avoid',
        time_points = [1.0, 0.5, 0.0]
    ):
        self.set_mode = set_mode
        self.time_points = time_points

        # 1) Dynamics
        if self.set_mode == 'avoid':
            radius = 0.5
        elif self.set_mode == 'reach':
            radius = 0.25
        else:
            raise RuntimeError('Choose avoid or reach !')
    
        self.dynamics = AutoRobot2D(radius=radius,
                                   velocity=1.0,
                                   set_mode=self.set_mode)
        
        # 2) Dataset
        dataset = dataio.ReachabilityDataset(
            dynamics=self.dynamics,
            numpoints=0,          
            pretrain=False,       
            pretrain_iters=0,     
            tMin=0.0,
            tMax=1.0,
            counter_start=0,
            counter_end=0,        
            num_src_samples=0,
            num_target_samples=0, 
        )

        # 3) Model
        self.model = modules.SingleBVPNet(
            in_features=self.dynamics.input_dim,
            out_features=1,
            type='sine',
            mode='mlp',
            hidden_features=512,
            num_hidden_layers=3
        )
        if self.set_mode == 'avoid':
            self.model.load_state_dict(torch.load('runs/AutoRobot2D_avoid_r0.5/training/checkpoints/model_epoch_100000.pth')['model'])
        elif self.set_mode == 'reach':
            self.model.load_state_dict(torch.load('runs/AutoRobot2D_reach_r0.25/training/checkpoints/model_epoch_100000.pth')['model'])
        self.model.eval()

    def plot(self):
        """
        Render and save the safe set for each time in `time_points`.
        """

        xs = torch.linspace(-2, 2, 200)
        ys = torch.linspace(-2, 2, 200)
        PX, PY = torch.meshgrid(xs, ys, indexing='xy')
        time_points = self.time_points
        
        fig = plt.figure(figsize=(5*len(time_points), 5))
        for idx, t in enumerate(time_points):
            coords = torch.stack([
                torch.full_like(PX, t),
                PX, PY
            ], dim=-1).view(-1, 3)

            with torch.no_grad():
                inp = self.dynamics.coord_to_input(coords)
                out = self.model({'coords': inp})['model_out'].squeeze(-1)
                V = self.dynamics.io_to_value(inp, out).view(200, 200)
            # 1) Dynamics
            if self.set_mode == 'avoid':
                safe_mask = (V > 0).float()
            elif self.set_mode == 'reach':
                safe_mask = (V < 0).float()
            
            ax = fig.add_subplot(1, len(time_points), 1 + idx)
            ax.contourf(xs.numpy(), ys.numpy(), safe_mask.T.numpy(), levels=[0.5,1.5], colors=['b'])
            ax.set_title(f"Safe set for t={1.0 - t:.1f}s when T = {1.0}s")
            ax.set_xlabel('p_x [m]'); plt.ylabel('p_y [m]')
            ax.set_aspect('equal')

        if self.set_mode == 'avoid':
            fig.savefig(os.path.join('BRS_safeSet_avoid0.5_plot.png'))
            print("Saved the set of states from which a collision is avoidable!")
        elif self.set_mode == 'reach':
            fig.savefig(os.path.join('BRS_safeSet_reach0.25_plot.png'))
            print("Saved the set of states from which the robot can surely reach the goal region!")