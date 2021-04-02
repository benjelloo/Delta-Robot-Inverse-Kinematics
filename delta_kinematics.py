import math
import numpy as np
from scipy.optimize import fsolve

def inverse_kinematics(x,y,z):
    # [0,0,0] is defined to be the centre of the baseplate
    forearm_endeff_point = [x,y+BotParams.end_effector_radius,z]
    bicep_baseplate_point = [0,BotParams.base_radius,0]
    
    forearm_proj = math.sqrt(BotParams.forearm_length^2-x^2) #project forearm onto x=0 axis

    # we want to find the intersection point between the bicep circle and the forearm circle, which will
    # be the location of the joint

    # forearm_endeff_point to joint_point = sqrt(forearm_length^2-x^2)
    # (y-bicep_baseplate_point_y)^2+(z-bicep_baseplate_point_z)^2=BotParams.bicep_length^2
    # (y-forearm_endeff_point_y)^2+(z-forearm_endeff_point_z)^2=forearm_proj^2
    # solve for y,z!
    yj,zj = fsolve(equations,(base_radius,0),forearm_endeff_point,bicep_baseplate_point,forearm_proj)
    print(equations(yj,zj))
    print(yj,zj)
    return 0

def equations(p,forearm_endeff_point,bicep_baseplate_point,forearm_proj):
    y,z=p
    return((y-bicep_baseplate_point[1])^2+(z-bicep_baseplate_point[2])^2-BotParams.bicep_length^2,
            (y-forearm_endeff_point[1])^2+(z-forearm_endeff_point[2])^2-forearm_proj^2)
@dataclass
class BotParams:
    base_radius: float # center of baseplate to servo
    bicep_length: float
    forearm_length: float
    end_effector_radius: float 
    height: float # from the ground to the servo
