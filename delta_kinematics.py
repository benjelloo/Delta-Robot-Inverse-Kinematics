import math
import numpy as np
from scipy.optimize import fsolve
from dataclasses import dataclass

@dataclass
class BotParams:
    base_radius: float = 0.5 # center of baseplate to servo
    bicep_length: float = 0.4
    forearm_length: float =0.7
    end_effector_radius: float =0.14
    height: float=2 # from the ground to the servo

def angle_calc(x,y,z):
    # [0,0,0] is defined to be the centre of the baseplate
    forearm_endeff_point = [x,y+BotParams.end_effector_radius,z]
    bicep_baseplate_point = [0,BotParams.base_radius,0]

    forearm_proj = math.sqrt(BotParams.forearm_length**2-x**2) #project forearm onto x=0 axis
   
    print("forearm endeff point:",forearm_endeff_point)
    print("bicep baseplate point:",bicep_baseplate_point)
    # we want to find the intersection point between the bicep circle and the forearm projection circle, which will
    # be the location of the joint

    # forearm_endeff_point to joint_point = sqrt(forearm_length^2-x^2)
    # (y-bicep_baseplate_point_y)^2+(z-bicep_baseplate_point_z)^2=BotParams.bicep_length^2
    # (y-forearm_endeff_point_y)^2+(z-forearm_endeff_point_z)^2=forearm_proj^2
    # solve for y,z!
    yj,zj = fsolve(circle_intersect,(BotParams.base_radius,0),(forearm_endeff_point,bicep_baseplate_point,forearm_proj))
    print(yj,zj)
    angle = math.atan(zj/(yj-BotParams.base_radius))*180/math.pi
    return angle

def inverse_kinematics(x,y,z):
    angles = []
    angles.append(angle_calc(x,y,z))
    x1=x*math.cos(2/3*math.pi)+y*math.sin(2/3*math.pi)
    y1=-1*x*math.sin(2/3*math.pi)+y*math.cos(2/3*math.pi)
    angles.append(angle_calc(x1,y1,z))
    x2=x*math.cos(-2/3*math.pi)+y*math.sin(-2/3*math.pi)
    y2=-1*x*math.sin(-2/3*math.pi)+y*math.cos(-2/3*math.pi)
    angles.append(angle_calc(x2,y2,z))
    return angles
def circle_intersect(p,forearm_endeff_point,bicep_baseplate_point,forearm_proj):
    y,z=p
    return((y-bicep_baseplate_point[1])**2+(z-bicep_baseplate_point[2])**2-BotParams.bicep_length**2,
            (y-forearm_endeff_point[1])**2+(z-forearm_endeff_point[2])**2-forearm_proj**2)

print(inverse_kinematics(0,0.05,-0.01))

