from spatialmath import SE3, SO3
from scipy.spatial.transform import Rotation 
from linearPath import buildABBIRB, robotFowardKinematics
import math
import numpy as np
import mpmath as mp

pi = np.pi


def configRobot(Px=765,Py=0,Pz=99):
    (L1, L2, L3, Of, r) = (352, 360, 380, 70, np.sqrt(Px**2 + Py**2))
    d = Pz-L1
    cos_alfa = ((Pz-L1)**2 + (r-Of)**2 - L2**2 - L3**2) / (-2*L2*L3)
    cos_alfa = 1 if cos_alfa>=1 else (-1 if cos_alfa<=-1 else cos_alfa)
    alfa1, alfa2 = math.atan2(np.sqrt(1-cos_alfa**2),cos_alfa), np.arctan2(-np.sqrt(1-cos_alfa**2),cos_alfa)
    pos_alfa, neg_alfa = max(alfa1,alfa2), min(alfa1,alfa2)
    
    t3_1 = pi/2 + pos_alfa 
    t3_2 = pi/2 + neg_alfa
    t2_1 = np.arctan2((r-Of),(d)) - np.arctan2((L3*np.sin(neg_alfa)) , (L2 - L3*cos_alfa)) 
    t2_2 = np.arctan2((r-Of),(d)) - np.arctan2((L3*np.sin(pos_alfa)) , (L2 - L3*cos_alfa))
    t1 = np.arctan2(Py,Px)
       
    return [(t1,t2_1,t3_1) , (t1,t2_2,t3_2)]

def obtenerCinematicaR60(Des_pos = [0.07,0,1.092], direction = SE3([0,0,0]), Robot_ets = None):

    Robot_ets = robotFowardKinematics()[1] if Robot_ets is None else Robot_ets
    
    #parametros
    Robot_01_ets, Robot_12_ets, Robot_23_ets = Robot_ets[0], Robot_ets[1], Robot_ets[2]
    Robot_34_ets, Robot_45_ets, Robot_56_ets = Robot_ets[3], Robot_ets[4], Robot_ets[5]
    Dfin, position = 0.065, SE3(Des_pos)
    R06 = position*direction

    #matrices
    Des_rot = direction.rpy(unit='rad',order='xyz')
    full_matrix = direction.A

    #transformacion efector a muñeca
    Rd = direction.R[:,2]
    oc = Des_pos - Rd * Dfin

    #cinematica de las 3 primeras articulaciones para ambas configuraciones
    [(t1,t2,t3),(t1b,t2b,t3b)] = configRobot(oc[0]*1000,oc[1]*1000,oc[2]*1000)
    fkine_A = Robot_01_ets.fkine([t1b])*Robot_12_ets.fkine([t2b])*Robot_23_ets.fkine([t3b])*Robot_34_ets.fkine([0])*Robot_45_ets.fkine([0])

    #transformacion de muñeca a final de efector R36R 
    #(R06 = R03*R36     R36 = inv(R03)*R06)
    R06R = np.matrix(R06.R)
    R03 = np.matrix(fkine_A.R)
    Inv = np.linalg.inv(R03)
    
    R36R = Inv*R06R
    r =  Rotation.from_matrix(R36R)
    Robot, L = buildABBIRB()

    #obtencion de angules euler, alias articulaciones finales y cinematica directa como demostracion
    [t4,t5,t6] = r.as_euler("xyz",degrees=False) #R36.eul(unit='rad', flip=False)
    return (Robot.fkine([t1b,t2b,t3b,t4,t5,t6]),[t1b,t2b,t3b,t4,t5,t6], Des_pos, direction)