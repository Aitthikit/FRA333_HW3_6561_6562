# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.อสมา_6561
2.อิทธิกฤต_6562
'''
# Import Library
import HW3_utils
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3,base
from math import pi,radians
import FRA333_HW3_6561_6562 as hand
# Config robot link
d_1 = 0.0892
a_2 = -0.425
a_3 = -0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
#Create robot from roboticstoolbox DHRobot
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(d= d_1 , offset = pi),
        rtb.RevoluteMDH(alpha = pi/2 ),
        rtb.RevoluteMDH(a=a_2),
    ],
    name = "RRR_Robot"
    )
tool_frame = SE3((a_3-d_6),-d_5,d_4) @ SE3.RPY(0.0,-pi/2,0.0) #Transformation Matrix from last joint to end-effector
robot.tool = tool_frame #add End-effector to robot
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here
def proofOne(q:list[float],robot:rtb.DHRobot,ref:int)->bool:
    J_e = hand.endEffectorJacobianHW3(q,ref)
    if ref == 0:
        J_ertb = robot.jacob0(q)
    else:    
        J_ertb = robot.jacobe(q)
    allow_error = 0.0001
    print("-----------Hand Jacobian-----------")
    print(J_e)
    print("-----------RTB Jacobian------------")
    print(J_ertb)
    print("------------Jacobian True or False------------")
    return np.allclose(J_e, J_ertb, atol=allow_error)
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def proofTwo(q:list[float],robot:rtb.DHRobot,ref:int)->bool:
    if ref == 0:
        J = robot.jacob0(q)
    else:    
        J = robot.jacobe(q)
    singularity = hand.checkSingularityHW3(q,ref)
    if singularity == 1:
        issingula = True
    else:
        issingula = False  
    # singularity_rtb = robot.manipulability(J=J)
    J_r = J[:3,:] #Reduce Jacobian Matrix to made it can find det and inverse
    singularity_rtb = base.det(J_r)
    if abs(singularity_rtb) < 0.001:
        issingula_rtb = True
    else:
        issingula_rtb = False
    print("-----------Hand Singularity-----------")
    print(issingula)
    print("-----------RTB Singularity------------")
    print(issingula_rtb)
    print("------------Singularity True or False------------")
    return issingula == issingula_rtb
    
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def proofThree(q:list[float], w:list[float],robot:rtb.DHRobot,ref:int)->bool:
    tau = hand.computeEffortHW3(q,w,ref)
    w = np.array(w)
    if ref == 0:
        J = robot.jacob0(q)
    else:    
        J = robot.jacobe(q)
    tau_rtb = robot.pay(w,J=J,frame = 0)
    allow_error = 0.0001
    print("-----------Hand Effort-----------")
    print(tau)
    print("-----------RTB Effort------------")
    print(-tau_rtb)
    print("------------Effort True or False------------")
    return np.allclose(tau, -tau_rtb, atol=allow_error)
#==============================================================================================================#
q = hand.q
w = hand.w
ref = hand.ref
print(proofOne(q,robot,ref))
print(proofTwo(q,robot,ref))
print(proofThree(q,w,robot,ref))