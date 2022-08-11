import swift
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import numpy as np


robot = rtb.models.Panda()
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
    while not arrived and loop < 100 :
        s_error, arrived = rtb.p_servo(robot.fkine(robot.q),tap,gain = 1,threshold= .05)
        j = robot.jacobe(robot.q)
        robot.qd = np.linalg.pinv(j) @ s_error
        print("error is ",s_error, "at loop", loop , "is arrived ", arrived)
        loop =  loop + 1
        env.step(dt)
    print("previous position", x,y,z)

