import configargparse
from plotSafeset import plot_safeSet


p = configargparse.ArgumentParser()
p.add_argument('--mode',
                type=str, 
                required=True, 
                choices=['avoid', 'reach'], 
                help="Plot the safe set in avoid or reach for time points: 0s 0.5s 1.0s")

mode = p.parse_known_args()[0].mode
plotter = plot_safeSet.Plot_safeset(set_mode = mode)
plotter.plot()
