# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.อิทธิกฤต_6562
2.
3.
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
    z_1 = np.array([0.0,0.0,1.0]).reshape(3,1)
    z_2 = np.array([math.sin(-q[1]),-math.cos(-q[1]),0.0]).reshape(3,1)
    z_3 = np.array([math.sin(-q[1]),-math.cos(-q[1]),0.0]).reshape(3,1)
    J_01 = np.vstack((base.cross(z_1,base.vector_diff(p_0e,p_01,'r')).reshape(3,1),z_1))
    J_02 = np.vstack((base.cross(z_2,base.vector_diff(p_0e,p_02,'r')).reshape(3,1),z_2))
    J_03 = np.vstack((base.cross(z_3,base.vector_diff(p_0e,p_03,'r')).reshape(3,1),z_3))
    J_e = np.hstack((J_01,J_02,J_03))
    
    # J_e = 
    
    return J_01,J_02,J_03,J_e,R_e
#==============================================================================================================#
#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    pass
#==============================================================================================================#
#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    pass
#==============================================================================================================#
print(endEffectorJacobianHW3([0.0,0.0,0.0]))
