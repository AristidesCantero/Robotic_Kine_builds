import roboticstoolbox as rtb
import numpy as np
import time
from spatial_trajectories import *
from roboticstoolbox import trapezoidal 
from spatialmath import SE3, Twist3
from spatialmath.base import rotx,roty,rotz

#robot = rtb.robot.DHRobot()

def buildAbbIrb140():
    pi = np.pi
    #define robot joints limits
    limits1 = [np.radians(-180),np.radians(180)]
    limits2 = [np.radians(-90),np.radians(110)]
    limits3 = [np.radians(-230),np.radians(50)]
    limits4 = [np.radians(-200),np.radians(200)]
    limits5 = [np.radians(-120),np.radians(120)]
    limits6 = [np.radians(-400),np.radians(400)]
    limits = limits1 + limits2 + limits3 + limits4 + limits5 + limits6

    #define robot links
    L1 =  rtb.robot.DHLink(d=0.352, a=0.07, alpha=-pi/2, offset=0,qlim=limits1)  
    L2 =  rtb.robot.DHLink(d=-0.065, alpha=0, a=0.36, offset=-pi/2,qlim=limits2) 
    L3 =  rtb.robot.DHLink(d=0.065, alpha=-pi/2, a=0.0, offset=0,qlim=limits3)
    L4 =  rtb.robot.DHLink(d=0.38, alpha=-pi/2, a=0.0, offset=0,qlim=limits4)
    L5 =  rtb.robot.DHLink(d=0.0, alpha=pi/2, a=0.0, offset=0,qlim=limits5)
    L6 =  rtb.robot.DHLink(d=0.065, alpha=0, a=0.0, offset=0,qlim=limits6)


    return rtb.SerialLink([L1,L2,L3,L4,L5,L6],name="ABB_IRB_140")


def getRobotLinearProfile(Robot, start=[0.4,0.4,0.5], end=[0.4,-0.4,0.5], direction = SE3([0,0,0])):
    #SE3.OA([1,0,0],[0,0,1]

    Tst = SE3(start)*direction
    Ted = SE3(end)*direction
    qt = rtb.tools.trajectory.ctraj(Tst, Ted, 50)
    return Robot.ikine_LM(qt)


def main():
    pi=np.pi

    #Robot.teach([0,0,0,0,0,0],limits=[-0.6,0.6,0.6,-0.6,0,1.2]) #,0
    Robot = buildAbbIrb140()

    #Robot pose 
    poses = getRobotLinearProfile(Robot,direction=SE3.AngleAxis(pi/2, [0,1,0], unit='rad'))

    T = SE3(0.5, 0.2, 0.5) * SE3.OA([0,0,1], [1,0,0])

    #Robot.plot([0,0],limits=[-0.5,0.5,0.2,-0.2,0,0.7])
    sol = Robot.ikine_LM(T)

    q_pickup = sol.q
    check_fkine = Robot.fkine(q_pickup)

    qt = rtb.tools.trajectory.jtraj([0,0,0,0,0,0], sol.q, 50)

    visionLimits = [-0.6,0.6,-0.6,0.6,0,1.2]

    Robot.plot(poses.q,dt=0.1,limits=visionLimits)
    print(poses.q)

    #Robot.teach([0,0,0,0,0,0],limits=visionLimits)


if __name__=='__main__':
    main()