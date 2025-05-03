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
                 omega_max: float = math.pi,
                 set_mode: str = 'avoid'):
        # radius: goal or obstacle radius
        # velocity: constant forward speed
        # omega_max: max angular speed (control bound)
        self.radius = radius
        self.velocity = velocity
        self.omega_max = omega_max
        super().__init__(
            loss_type='brt_hjivi',
            set_mode=set_mode,               # 'avoid' for obstacle, 'reach' for goal
            state_dim=3,                     # [px, py, theta]
            input_dim=4,                     # [t, px, py, theta]
            control_dim=1,                   # heading rate u = omega
            disturbance_dim=0,
            state_mean=[0.0, 0.0, 0.0],
            state_var=[2.0, 2.0, math.pi],
            value_mean=0.0,
            value_var=1.0,
            value_normto=0.02,
            deepreach_model='exact',
        )

    def state_test_range(self):
        # define [[min, max] for each of px, py, theta]
        return [
            [-2.0, 2.0],  # px
            [-2.0, 2.0],  # py
            [-math.pi, math.pi],  # theta
        ]

    def equivalent_wrapped_state(self, state):
        # wrap heading into [-pi, pi]
        w = state.clone()
        w[..., 2] = (w[..., 2] + math.pi) % (2*math.pi) - math.pi
        return w

    def dsdt(self, state, control, disturbance):
        # dynamics: dot(px)=v*cos(theta), dot(py)=v*sin(theta), dot(theta)=u
        d = torch.zeros_like(state)
        d[..., 0] = self.velocity * torch.cos(state[..., 2])
        d[..., 1] = self.velocity * torch.sin(state[..., 2])
        d[..., 2] = control[..., 0]
        return d

    def boundary_fn(self, state):
        # returns signed distance: <0 inside set, =0 on boundary
        # for avoid: states with norm <= radius are unsafe
        # for reach: states with norm <= radius are goal
        return torch.norm(state[..., :2], dim=-1) - self.radius

    def sample_target_state(self, num_samples):
        # not used for BRT-only experiments
        raise NotImplementedError

    def cost_fn(self, state_traj):
        # tube cost (min over time of boundary function)
        return torch.min(self.boundary_fn(state_traj), dim=-1).values

    def hamiltonian(self, state, dvds):
        # Hamiltonian of HJ PDE: f·dv + control term
        # f·dv = v*cosθ * dv_x + v*sinθ * dv_y
        base = self.velocity * (torch.cos(state[..., 2]) * dvds[..., 0]
                                 + torch.sin(state[..., 2]) * dvds[..., 1])
        # control contribution: maximize or minimize dv_theta * u
        if self.set_mode == 'reach':
            # choose u to decrease value (reach target)
            ctrl = -self.omega_max * torch.abs(dvds[..., 2])
        else:
            # choose u to increase value (avoid obstacle)
            ctrl = +self.omega_max * torch.abs(dvds[..., 2])
        return base + ctrl

    def optimal_control(self, state, dvds):
        # bang-bang control on heading rate
        sign = torch.sign(dvds[..., 2])
        if self.set_mode == 'reach':
            # u* = -omega_max * sign(dV/dθ)
            return (-self.omega_max * sign)[..., None]
        else:
            # u* = +omega_max * sign(dV/dθ)
            return ( self.omega_max * sign)[..., None]

    def optimal_disturbance(self, state, dvds):
        # no disturbances
        return 0

    def plot_config(self):
        return {
            'state_slices': [0.0, 0.5, 1.0],
            'state_labels': ['px', 'py', 'θ'],
            'x_axis_idx': 0,
            'y_axis_idx': 1,
            'z_axis_idx': 2,
        }

# Example instantiation:
# For obstacle avoidance BRT (radius=0.5m):
# robot_avoid = SimplePlanarRobot(radius=0.5, set_mode='avoid')
# For goal-reaching BRT (radius=0.25m):
# robot_reach = SimplePlanarRobot(radius=0.25, set_mode='reach')
