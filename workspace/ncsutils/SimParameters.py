import numpy as np
class SimParameters:
    def __init__(self):
        self.MapLocation = "/root/workspace/src/ncsutils/exp_track.npy" #"/root/workspace/src/ncsutils/straight_path.npy" #"/root/workspace/src/maps_database/customRaceTrack/track_final.npy"
        self.safetyScaling = 1 #0.2 # safety scaling for the map
         
        
class CarParams:
    def __init__(self):
        #params (dict, default={'mu': 1.0489, 'C_Sf':, 'C_Sr':, 'lf': 0.15875, 'lr': 0.17145, 'h': 0.074, 'm': 3.74, 'I': 0.04712, 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2, 'v_switch':7.319, 'a_max': 9.51, 'v_min':-5.0, 'v_max': 20.0, 'width': 0.31, 'length': 0.58}): dictionary of vehicle parameters.
        self.model_parameters ={}
        self.model_parameters["sx"] = 4  #"no of state which needs to integrate" x,y,yaw,v,theta
        self.model_parameters["su"]  = 2 
        self.model_parameters["ax"] = 6  #"augmented no of state" #"no of state which needs to integrate" x,y,yaw,v,theta
        self.model_parameters["au"]  = 3 
        self.model_parameters["stateindex_x"] = 0
        self.model_parameters["stateindex_y"] = 1 
        self.model_parameters["stateindex_psi"]  = 2 
        self.model_parameters["stateindex_v"]  = 3
        self.model_parameters["stateindex_theta"]  = 4
        self.model_parameters["stateindex_delta"]  = 5
        self.model_parameters["stateindex_vdot"]  = 6
        self.model_parameters["stateindex_prevIndex"]  = 7
        self.model_parameters["stateindex_currentIndex"]  = 8
        
        self.model_parameters["inputindex_vdot"] = 0
        self.model_parameters["inputindex_delta"] = 1    
        self.model_parameters["inputindex_thetadot"] = 2      
        self.model_parameters["m"] = 1573
        self.model_parameters["Iz"] = 2873
        self.model_parameters["lf"] = 1.25 #1.35
        self.model_parameters["lr"] = 1.25 #1.35
        
        self.model_parameters["Cm1"] = 17303
        self.model_parameters["Cm2"] = 175
         
        self.model_parameters["Cro"] = 120
        self.model_parameters["Cr2"] = 0.5*1.225*0.35*2.5

        self.model_parameters["Br"] = 13
        self.model_parameters["Cr"] = 2

        self.model_parameters["Bf"] = 13
        self.model_parameters["Cf"] = 2

        
        self.model_parameters["L"] = 2
        self.model_parameters["W"] = 0.9
                
        self.model_parameters["scale"] = 1 
                
