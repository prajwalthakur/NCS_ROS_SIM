#test-1
import numpy as np
import pdb
from params import MPCParams
import sys
sys.path.append('/root/workspace/src/vehicle/scripts/')
from car_dynamics_pid import car_model
mpc_params = MPCParams()


def test_car_dynamics_init():
    init_ego_pose_array =mpc_params.init_ego_pose
    pdb.set_trace()
    vehicle_obj = car_model(initial_states=init_ego_pose_array[...,np.newaxis])



if __name__=='__main__':
    test_car_dynamics_init()