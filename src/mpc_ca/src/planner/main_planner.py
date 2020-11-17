from planner.mpc import NonLinearMPC
from planner.ros_interface import Logger as logging
import numpy as np


class Planner(object):
    """Planner wrapper to handle the different planner algorithms available.

    :param occupancy_grid: ocuppancy grid representation of the map
    :type occupancy_grid: OccupancyGrid object
    :param planner_name: name of the planner algorithm
    :type planner_name: str
    :param planner_parameters: ROS parameters for the planner algorithm
    :type planner_parameters: dict
    :raises RuntimeError: An invalid planner name was provided
    """

    available_planners = ['mpc']
    available_initial_guess_planners = []

    
    
    def __init__(self, occupancy_grid, planner_name, planner_parameters):
        if planner_name not in self.available_planners:
            logging.error('"{}" is not a valid planner!'.format(planner_name))
            raise RuntimeError

        self.occupancy_grid = occupancy_grid

        self.planner = NonLinearMPC(planner_parameters)
        self.planner.setup_optimization_problem(occupancy_grid.obstacles)
        
       

    def compute_path(self, initial_state, goal_state):
        """Uses the planner to compute the path

        :param initial_state: initial pose [x, y, yaw]
        :type initial_state: list
        :param goal_state: goal pose [x, y, yaw]
        :type goal_state: list
        :return: path, where each pose is of the format [x, y, yaw]
        :rtype: list of tuples or numpy array
        """
        xs = [-7.4 , -2.33, 10.3, 5.9]#, -7.2, -2.5]
        ys = [-15.3,  -7.09, 11.4, 15.0]#, -4.2]#, -7.6]
        ds = [1.12, 1.12, 3*np.pi/4, np.pi]
        #initial_guess = [[xs[0],ys[0],1.12],[xs[1],ys[1],1.12],[xs[2],ys[2],3*np.pi/4],[xs[3],ys[3],np.pi]]#,[xs[4],ys[4],2.685,[xs[5],ys[5],2.685]]
        initial_guess = []
        '''
        traj_x = []
        traj_y = []
        traj_d = []
        initial_guess = []
        for i in range(0,len(xs)-1):
            traj_x += np.linspace(xs[i], xs[i+1],5).tolist()
            traj_y += np.linspace(ys[i], ys[i+1],5).tolist()
            traj_d += np.linspace(ds[i], ds[i+1],5).tolist()
            for j in range(0,len(traj_d)-1):
                initial_guess.append([traj_x[j],traj_y[j],traj_d[j]])
        print(initial_guess)'''
        planner_kwargs = {
            'initial_state': initial_state,
            'goal_state': goal_state,
            'initial_guess': initial_guess
        }
        return self.planner.compute_path(**planner_kwargs)

