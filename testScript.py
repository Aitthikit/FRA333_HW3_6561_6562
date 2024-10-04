# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ex: ธนวัฒน์_6461)
1.อิทธิกฤต_6562
2.
3.
'''
import HW3_utils
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi,radians
#    d_1 = 0.0892
#     a_2 = -0.425
#     a_3 = -0.39243
#     d_4 = 0.109
#     d_5 = 0.093
#     d_6 = 0.082
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def proofOne(q:list[float])->list[float]:
    robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=0.0892 , offset = pi),
        rtb.RevoluteMDH(alpha = pi/2 ),
        rtb.RevoluteMDH(a=-0.425),
    ],
    name = "RRR_Robot"
    )
    tool_frame = SE3(-0.47443, -0.093,0.109) @ SE3.RPY(0.0,-pi/2,0.0)  # Adjust the position and orientation as needed

    # Assign the tool frame to the robot
    robot.tool = tool_frame
    # print(robot)
    # print(robot.fkine([0.0,0.0,0.0]))
    J_e = [robot.jacob0(q)[0],robot.jacob0(q)[1],robot.jacob0(q)[2]]
    return robot.jacobe(q)
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def proofThree(q:list[float], w:list[float])->list[float]:
    robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d=0.0892 , offset = pi),
        rtb.RevoluteMDH(alpha = pi/2 ),
        rtb.RevoluteMDH(a=-0.425),
    ],
    name = "RRR_Robot"
    )
    tool_frame = SE3(-0.47443, -0.093,0.109) @ SE3.RPY(0.0,-pi/2,0.0)  # Adjust the position and orientation as needed
    robot.tool = tool_frame
    dummy = np.zeros(3)
    g = np.array([0, 0, -9.81])
    w = np.array(w)
    tau = robot.rne(q,dummy,dummy,g,w)
    return tau
#==============================================================================================================#
print(proofOne([0.0,0.0,0.0]))
print(proofThree([0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,1.0]))