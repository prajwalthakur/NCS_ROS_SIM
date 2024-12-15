#test-1
#units tests
import numpy as np
import pdb
from base_classes import RobotHospitalBed
robot_obj = RobotHospitalBed(vehicle_type="hospital_bed",current_time=0.0,control_seq_length=10,control_update_rate=1/0.1)
def test_base_classes_update_state()->None:
    
    X=0.0
    Y=1.0;theta=0.1;Vx=0.0;Vy=0.1;Vf=0.2;Vr=0.3;Sf=0.0;Sr=0.0;feedback_time = 0.5;msg_id_feedback = 1;control_idx_in_seq = 2
    try:
        robot_obj._update_state(x=X,y=Y,yaw=theta,v_x=Vx,v_y=Vy,v_front=Vf,v_rear=Vr,steer_front = Sf, steer_rear=Sr,time_sampled=feedback_time,msg_id = msg_id_feedback , control_idx_in_seq = control_idx_in_seq)
    except AssertionError:
        # If the assertion fails, print the mismatch message
        print("Assertion failed: msg_id in buffer does not match msg_id from perception")
        print(f"Expected msg_id_feedback: {msg_id_feedback}, "
              f"Actual msg_id in buffer: {robot_obj.robot_control_obj.msg_id}")


def test_base_classes_update_state_test2()->None:
    
    X=0.0
    Y=1.0;theta=0.1;Vx=0.0;Vy=0.1;Vf=0.2;Vr=0.3;Sf=0.0;Sr=0.0;feedback_time = 0.5;msg_id_feedback = None ;control_idx_in_seq = None
    robot_obj._update_state(x=X,y=Y,yaw=theta,v_x=Vx,v_y=Vy,v_front=Vf,v_rear=Vr,steer_front = Sf, steer_rear=Sr,time_sampled=feedback_time,msg_id = msg_id_feedback , control_idx_in_seq = control_idx_in_seq)
    assert robot_obj.robot_control_obj.time_seqs[0] == 0.5 
    print(f"feedback msg sampled at time {feedback_time}  , update seqs in the the buffer starts from the time {robot_obj.robot_control_obj.time_seqs[0]}" )


def test_base_classes_append_control_seqs()->None:
    
    X=0.0
    Y=1.0;theta=0.1;Vx=0.0;Vy=0.1;Vf=0.2;Vr=0.3;Sf=0.0;Sr=0.0;feedback_time = 0.5;msg_id_feedback = None ;control_idx_in_seq = None
    robot_obj._update_state(x=X,y=Y,yaw=theta,v_x=Vx,v_y=Vy,v_front=Vf,v_rear=Vr,steer_front = Sf, steer_rear=Sr,time_sampled=feedback_time,msg_id = msg_id_feedback , control_idx_in_seq = control_idx_in_seq)
    control_seqs = np.zeros((4,10))
    curr_time = 0.5
    assumed_delay = 0.05
    robot_obj._append_control_seqs(control_seqs=control_seqs,curr_time=curr_time,assumed_delay=assumed_delay)
    assert robot_obj.robot_control_obj.time_seqs[0] - 0.6 < 1e-4
    assert robot_obj.robot_control_obj.a_f.shape == robot_obj.robot_control_obj.a_r.shape == robot_obj.robot_control_obj.s_f.shape == robot_obj.robot_control_obj.s_r.shape
    print(f"feedback msg sampled at time {feedback_time}  , update seqs in the the buffer starts from the time {robot_obj.robot_control_obj.time_seqs[0]}" )
