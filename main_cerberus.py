import swift
import roboticstoolbox as rtb
from roboticstoolbox.robot.ERobot import ERobot, ERobot2
import spatialgeometry as sg
import spatialmath as sm
import numpy as np

class My_robot(ERobot):
     def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "/home/samiul/Documents/IK_solver/example_arm.urdf.xacro"
        )

        super().__init__(
            links,
            name=name,
            manufacturer="cerberus_robotoics",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )



        self.qr = np.array([1.3, .5, .2, .1 , 1.5 ])
        self.qz = np.zeros(5)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


# robot = rtb.models.Panda()
robot = My_robot()
robot.q = robot.qr
env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

arrived = False
dt = .1



print(robot)

while True:
    
    try:
        x = float(input("x cordinate\n"))
    except:
        print("not float is assinged to x")
    try:
        y = float(input("y cordinate\n"))
    except:
        print("not float is assinged to y")
    try:
        z = float(input("z cordinate\n"))
    except:
        print("not float assigned to z")
    print(x, y , z)
    tap = sm.SE3(x=x,y=y,z=z) * sm.SE3.OA([0, 1, 0], [0, 0, -1])
    axes = sg.Axes(length= .1 ,base=tap)
    env.add(axes)
    loop = 0
    arrived= False
    while not arrived and loop < 200 :
        s_error, arrived = rtb.p_servo(robot.fkine(robot.q),tap,gain = 1, threshold= .05)
        j = robot.jacobe(robot.q)
        robot.qd = np.linalg.pinv(j) @ s_error
        print("error is ",s_error, "at loop", loop , "is arrived ", arrived)
        loop =  loop + 1
        env.step(dt)
    print("previous position", x,y,z)

