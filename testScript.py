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
    J_e = hand.endEffectorJacobianHW3(q,ref)#Get Jacobian Matrix from endEffectorJacobianHW3 function
    if ref == 0:# Frame selection
        J_ertb = robot.jacob0(q) # Jacobian Matrix of each joint reference to base frame
    else:    
        J_ertb = robot.jacobe(q) # Jacobian Matrix of each joint reference to end-effector frame
    allow_error = 0.0001 # Allowable error in Jacobian Matrix compare
    print("-----------Hand Jacobian-----------")
    print(J_e)
    print("-----------RTB Jacobian------------")
    print(J_ertb)
    print("------------Jacobian True or False------------")
    return np.allclose(J_e, J_ertb, atol=allow_error) # Compare Jacobian Matrix from ans and from roboticstoolbox 
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here
def proofTwo(q:list[float],robot:rtb.DHRobot,ref:int)->bool:
    if ref == 0:# Frame selection
        J = robot.jacob0(q) # Jacobian Matrix of each joint reference to base frame
    else:    
        J = robot.jacobe(q) # Jacobian Matrix of each joint reference to end-effector frame
    singularity = hand.checkSingularityHW3(q,ref)# Get Singularity flag from checkSingularityHW3 function
    if singularity == 1:# Near Singularity == 1
        issingula = True
    else:
        issingula = False  
    # singularity_rtb = robot.manipulability(J=J) #Check from manipulability(can test in only some case)
    J_r = J[:3,:] #Reduce Jacobian Matrix to made it can find det and inverse
    singularity_rtb = base.det(J_r) # Find det of Jacobian Matrix from roboticstoolbox 
    if abs(singularity_rtb) < 0.001:# Near Singularity < 0.001
        issingula_rtb = True
    else:
        issingula_rtb = False
    print("-----------Hand Singularity-----------")
    print(issingula)
    print("-----------RTB Singularity------------")
    print(issingula_rtb)
    print("------------Singularity True or False------------")
    return issingula == issingula_rtb #Compare Between checkSingularityHW3 and singularity_rtb
    
#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def proofThree(q:list[float], w:list[float],robot:rtb.DHRobot,ref:int)->bool:
    tau = hand.computeEffortHW3(q,w,ref)# Get tau from computeEffortHW3 function
    w = np.array(w) #Change w to numpy array
    if ref == 0:# Frame selection
        J = robot.jacob0(q) # Jacobian Matrix of each joint reference to base frame
    else:    
        J = robot.jacobe(q) # Jacobian Matrix of each joint reference to end-effector frame
    tau_rtb = robot.pay(w,J=J,frame = 0)# Find tau from roboticstoolbox function (pay)
    allow_error = 0.0001 # Allowable error in tau compare
    print("-----------Hand Effort-----------")
    print(tau)
    print("-----------RTB Effort------------")
    print(-tau_rtb)
    print("------------Effort True or False------------")
    return np.allclose(tau, -tau_rtb, atol=allow_error) # Compare tau from ans and from roboticstoolbox 
    # tau_rtb need to multiply -1 because in library roboticstoolbox use - Jacobian Matrix transpose(in rtbPay.png)
#==============================================================================================================#
q = hand.q # Get q value from FRA333_HW3_6561_6562
w = hand.w # Get w value from FRA333_HW3_6561_6562
ref = hand.ref # Get ref value from FRA333_HW3_6561_6562

print(proofOne(q,robot,ref))
print(proofTwo(q,robot,ref))
print(proofThree(q,w,robot,ref))
#print each function output 