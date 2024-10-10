# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.อสมา_6561
2.อิทธิกฤต_6562
'''
import HW3_utils
import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3,base
import math
#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    R,P,R_e,p_e = HW3_utils.FKHW3(q)
    p_01 = P[:,0]
    p_02 = P[:,1]
    p_03 = P[:,2]
    p_0e = p_e
    R_e0 = np.linalg.inv(R_e)
    R_e1 = R_e0 @ R[:,:,0]
    R_e2 = R_e0 @ R[:,:,1]
    R_e3 = R_e0 @ R[:,:,2]
    # print (R_e1,R_e2)
    z_1 = np.array([0.0,0.0,1.0]).reshape(3,1)
    # z_2 = np.array([math.sin(q[1]),-math.cos(q[1]),0.0]).reshape(3,1)
    # z_3 = np.array([math.sin(q[1]),-math.cos(q[1]),0.0]).reshape(3,1)
    z_2 = np.array([0.0,0.0,1.0]).reshape(3,1)
    z_3 = np.array([0.0,0.0,1.0]).reshape(3,1)
    z_01 = R[:,:,0] @ z_1
    z_02 = R[:,:,1] @ z_2
    z_03 = R[:,:,2] @ z_3
    z_e1 = R_e1 @ z_1
    z_e2 = R_e2 @ z_2
    z_e3 = R_e3 @ z_3
    J_01 = np.vstack((R_e0 @ base.cross(z_01,base.vector_diff(p_0e,p_01,'r')).reshape(3,1),z_e1))
    J_02 = np.vstack((R_e0 @ base.cross(z_02,base.vector_diff(p_0e,p_02,'r')).reshape(3,1),z_e2))
    J_03 = np.vstack((R_e0 @ base.cross(z_03,base.vector_diff(p_0e,p_03,'r')).reshape(3,1),z_e3))
    J_e = np.hstack((J_01,J_02,J_03))
    return J_e
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J_e = endEffectorJacobianHW3(q)
    J_re = J_e[:3,:]
    value = base.det(J_re)
    # print(abs(value))
    if abs(value) < 0.001:
        return 1
    else:
        return 0
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J_e = endEffectorJacobianHW3(q)
    J_ret = np.transpose(J_e)
    w_t = np.array(w)
    print(w_t)
    tau = J_ret @ w_t
    return tau
#==============================================================================================================#
q = [0.0,-math.pi/2,-0.2] #joint [joint1,joint2,joint3] recommend[-0.6,-math.pi/2,0.0] for singularity test
w = [1.0,1.0,5.0,1.0,2.0,1.0]# wrench [Fx,Fy,Fz,Mx,My,Mz]
print(endEffectorJacobianHW3(q))
print(checkSingularityHW3(q))
print(computeEffortHW3(q,w))
