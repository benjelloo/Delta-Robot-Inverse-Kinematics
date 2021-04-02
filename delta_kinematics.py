import math
import numpy as np

def inverse_kinematics(x,y,z):
    # [0,0,0] is defined to be the centre of the baseplate
    bicep_endeff_point = [x,y+BotParams.end_effector_radius,z]
    forearm_baseplate_point = [0,BotParams.base_radius,0]
    # we want to find the intersection point between the bicep circle and the forearm circle, which will
    # be the location of the joint
    return 0

@dataclass
class BotParams:
    base_radius: float # center of baseplate to servo
    bicep_length: float
    forearm_length: float
    end_effector_radius: float 
    height: float # from the ground to the servo
