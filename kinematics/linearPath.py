from spatialmath import SE3, SO3
from spatialmath.base.symbolic import *
#from sympy import nsimplify, Matrix
from inverseKinematics import *
import roboticstoolbox as rtb
import spatialmath.base as base
import numpy as np

pi=np.pi

def buildABBIRB():
    limits1, limits2, limits3 = [np.radians(-180),np.radians(180)], [np.radians(-90),np.radians(110)], [np.radians(-230),np.radians(50)]
    limits4, limits5, limits6 = [np.radians(-200),np.radians(200)], [np.radians(-120),np.radians(120)], [np.radians(-400),np.radians(400)]
    limits = limits1 + limits2 + limits3 + limits4 + limits5 + limits6
    
    L1 =  rtb.robot.DHLink(d=0.352, a=0.07, alpha=-pi/2, offset=0,qlim=limits1)  
    L2 =  rtb.robot.DHLink(d=-0.065, alpha=0, a=0.36, offset=-pi/2,qlim=limits2) 
    L3 =  rtb.robot.DHLink(d=0.065, alpha=-pi/2, a=0.0, offset=0,qlim=limits3)
    L4 =  rtb.robot.DHLink(d=0.38, alpha=-pi/2, a=0.0, offset=0,qlim=limits4)
    L5 =  rtb.robot.DHLink(d=0.0, alpha=pi/2, a=0.0, offset=0,qlim=limits5)
    L6 =  rtb.robot.DHLink(d=0.065, alpha=0, a=0.0, offset=0,qlim=limits6)
    
    R1, R2, R3, R4, R5, R6 = rtb.Robot([L1],name='L1'), rtb.Robot([L2],name='L2'), rtb.Robot([L3],name='L3'), rtb.Robot([L4],name='L4'), rtb.Robot([L5],name='L5'), rtb.Robot([L6],name='L6')
    
    Robot = rtb.DHRobot([L1,L2,L3,L4,L5,L6],name="ABB_IRB") #,L6

    return [Robot, [L1,L2,L3,L4,L5,L6]]

def robotFowardKinematics():
    (q1,q2,q3,q4,q5,q6) = symbol('q_:6')
    RobotFkineEval, L = buildABBIRB()
    Robot_01 = rtb.DHRobot([L[0]],name="ABB_IRB_01")
    Robot_12 = rtb.DHRobot([L[1]],name="ABB_IRB_12")
    Robot_23 = rtb.DHRobot([L[2]],name="ABB_IRB_23")
    Robot_34 = rtb.DHRobot([L[3]],name="ABB_IRB_34")
    Robot_45 = rtb.DHRobot([L[4]],name="ABB_IRB_45")
    Robot_56 = rtb.DHRobot([L[5]],name="ABB_IRB_56")

    Robot_01_ets, Robot_12_ets, Robot_23_ets = Robot_01.ets(), Robot_12.ets(), Robot_23.ets()
    Robot_34_ets, Robot_45_ets, Robot_56_ets = Robot_34.ets(), Robot_45.ets(), Robot_56.ets()
    fow_kin_01, fow_kin_12, fow_kin_23 = Robot_01_ets.fkine([q1]),Robot_12_ets.fkine([q2]), Robot_23_ets.fkine([q3])
    fow_kin_34, fow_kin_45, fow_kin_56 = Robot_34_ets.fkine([q4]), Robot_45_ets.fkine([q5]), Robot_56_ets.fkine([q6])
    fow_kins = [fow_kin_01,fow_kin_12,fow_kin_23,fow_kin_34,fow_kin_45,fow_kin_56]
    Robot_etss = [Robot_01_ets, Robot_12_ets, Robot_23_ets, Robot_34_ets, Robot_45_ets, Robot_56_ets]

    return fow_kins, Robot_etss


def angle_in_range(alpha, lower, upper):
    return (alpha - lower) % 360 <= (upper - lower) % 360

def getRefinedKinematics(Des_pos = [0.07,0,1.092], direction = SE3([0,0,0])):
    Robot, L = buildABBIRB()
    
    position = SE3(Des_pos)
    R06 = position*direction
    
    Robot_fkine, q, Des_pos, direction2 = obtenerCinematicaR60(Des_pos = Des_pos, direction = direction)
    
    return Robot.ikine_LM(R06, q0=q)

def generatePath(start=[0.5, 0.3, 0.6], end=[0.5, -0.3, 0.6], size = 50):
    if size <= 1:
        return []
    stepX = (start[0]-end[0])/size
    stepY = (start[1]-end[1])/size 
    stepZ = (start[2]-end[2])/size

    Path = []

    for step in range(size+1):
        Path.append([start[0]-stepX*step, start[1]-stepY*step, start[2]-stepZ*step])

    return Path

def buildPath(Path):
    trajectory = []
    for point in range(len(Path)):
        tPoint = getRefinedKinematics(Des_pos = Path[point], direction = SE3([0,0,0]))
        trajectory.append(np.array(tPoint.q))
        
    trajectory = np.array(trajectory)
    Finaltray = rtb.tools.trajectory.mstraj(viapoints=trajectory, dt=1, tacc=1, tsegment=[2]*30)
    return Finaltray

def buildPathFeedback(Path):
    Robot, L = buildABBIRB()
    origin = Path[0]
    originq = getRefinedKinematics(Des_pos = origin, direction = SE3([0,0,0])).q
    trajectory = []

    for point in range(len(Path)):
        position = SE3(Path[point])
        direction = SE3([0,0,0])
        finalPose = position*direction

        originq = Robot.ikine_GN(finalPose, q0=originq).q

        trajectory.append(np.array(originq))
        
    trajectory = np.array(trajectory)
    Finaltray = rtb.tools.trajectory.mstraj(viapoints=trajectory, dt=1, tacc=1, tsegment=[2]*30)
    return Finaltray


def main():
    Robot, L = buildABBIRB()
    #DENAVIT HARTENBERG FOWARD KIN FOR EACH LINK
    Robot_01_ets, Robot_12_ets, Robot_23_ets, Robot_34_ets, Robot_45_ets, Robot_56_ets = robotFowardKinematics()[1]

    #simplification = nsimplify(fow_kin_06.data[0],tolerance=1e-5,rational=False)

    [(t1,t2,t3),(t1b,t2b,t3b)] = configRobot()
    (t3, t2, t1), (t3b, t2b, t1b) = (np.round([t3,t2,t1], decimals = 5), np.round([t3b,t2b,t1b], decimals = 5))

    Robot_etss = [Robot_01_ets, Robot_12_ets, Robot_23_ets, Robot_34_ets, Robot_45_ets, Robot_56_ets]

    defaultEndPoseRPY = SE3.RPY([pi,0,0],unit='rad', order='xyz')

    startPath = [0.5, -0.3, 0.7]
    endPath = [0.5, 0.3, 0.7]
    RobotPath = generatePath(start = startPath, end = endPath, size=30)

    Finaltray = buildPathFeedback(RobotPath)
    print(Finaltray.q)

    Robot.plot(Finaltray.q, dt=0.1, limits=[-0.6,0.6,-0.6,0.6,0,1.2])

if __name__ == '__main__':
    main()