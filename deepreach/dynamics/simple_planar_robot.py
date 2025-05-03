import torch
import math
from dynamics.dynamics import Dynamics

class SimplePlanarRobot(Dynamics):
    """
    A 2D planar robot with constant speed and heading control.
    Can be used for both reachability (goal) and avoidance (obstacle) BRT computations.
    """
    def __init__(self,
                 radius: float,
                 velocity: float = 1.0,
                 set_mode: str = 'avoid'):
        # radius: goal or obstacle radius
        # velocity: constant forward speed
        # omega_max: max angular speed (control bound)
        self.radius = radius
        self.velocity = velocity
        super().__init__(
            loss_type='brt_hjivi',
            set_mode=set_mode,         # 'avoid' for obstacle, 'reach' for goal
            state_dim=2,               # [px, py]
            input_dim=3,               # [t, px, py]
            control_dim=1,             # u = heading angle
            disturbance_dim=0,
            state_mean=[0.0, 0.0],
            state_var=[2.0, 2.0],
            value_mean=0.0,
            value_var=1.0,
            value_normto=0.02,
            deepreach_model='exact',
        )

    def state_test_range(self):
        return [[-2.0, 2.0], [-2.0, 2.0]]  # [px_min, px_max], [py_min, py_max]

    def equivalent_wrapped_state(self, state):
        # no wrapping needed for px, py
        return torch.clone(state)
      
    def dsdt(self, state, control, disturbance):
        # dynamics: dot(px)=v*cos(theta), dot(py)=v*sin(theta), control[...,0] is heading angle u
        dsdt = torch.zeros_like(state)
        dsdt[..., 0] = self.velocity * torch.cos(control[..., 0])
        dsdt[..., 1] = self.velocity * torch.sin(control[..., 0])
        return dsdt

    def boundary_fn(self, state):
        # returns signed distance: <0 inside set, =0 on boundary
        # for avoid: norm <= radius are unsafe
        # for reach: norm <= radius are goal
        return torch.norm(state[..., :1], dim=-1) - self.radius
    '''
    def sample_target_state(self, num_samples): # CHECK
        # Uniformly sample states within the test range to initialize training or evaluation.
        # 1. Get the list of [min, max] ranges for each state dimension (px, py, theta).
        ranges = self.state_test_range()
        # 2. Extract lower bounds into a tensor: [px_min, py_min, theta_min]
        lows = torch.tensor([r[0] for r in ranges], device='cpu')
        # 3. Extract upper bounds into a tensor: [px_max, py_max, theta_max]
        highs = torch.tensor([r[1] for r in ranges], device='cpu')
        # 4. Create num_samples × state_dim random matrix in [0,1)
        #    and scale/shift it to lie within [lows, highs].
        #    Each row is one state sample.
        samples = torch.rand(num_samples, self.state_dim, device=lows.device)
        return lows + (highs - lows) * samples
    ''' 
    def sample_target_state(self, num_samples):
        raise NotImplementedError

    def cost_fn(self, state_traj):
        # tube cost (min over time of boundary function)
        return torch.min(self.boundary_fn(state_traj), dim=-1).values

    def hamiltonian(self, state, dvds):
        # H = max_u v*(cos(u)*V_x + sin(u)*V_y) 
        grad_norm = torch.sqrt(dvds[..., 0]**2 + dvds[..., 1]**2)
        base = self.velocity * grad_norm
        # control contribution: bang-bang on θ
        if self.set_mode == 'reach':
            ham = -base
        else: # self.set_mode == 'reach'
            ham =  base
        return ham

    def optimal_control(self, state, dvds):
        # Optimal heading control:
        # For avoid-mode: steer along the gradient direction (maximize value)
        # For reach-mode: steer opposite gradient (minimize value)
        # Base angle = atan2(dV/dy, dV/dx)
        angle = torch.atan2(dvds[..., 1], dvds[..., 0])
        if self.set_mode == 'reach':
            # reverse direction for reaching
            angle = angle + math.pi
        # wrap into [-pi,pi]
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        return angle.unsqueeze(-1)

    def optimal_disturbance(self, state, dvds):
        # no disturbances
        return 0

    def plot_config(self):
        return {
            'state_slices': [0.0, 0.0],
            'state_labels': ['px', 'py'],
            'x_axis_idx': 0,
            'y_axis_idx': 1,
            'z_axis_idx': -1,
        }