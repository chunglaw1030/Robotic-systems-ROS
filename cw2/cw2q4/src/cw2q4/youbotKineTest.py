import numpy as np
from youbotKineBase import YoubotKinematicBase
from youbotKineStudent import YoubotKinematicStudent
from youbotKineKDL import YoubotKinematicKDL

YKB = YoubotKinematicBase()
YKS = YoubotKinematicStudent()
YKK= YoubotKinematicKDL()


joint = []

for i in range(5):
    joint_min = YKB.joint_limit_min[i]
    joint_max = YKB.joint_limit_max[i]
    joint_val = np.random.uniform(low = joint_min, high = joint_max )
    joint.append(joint_val)


# jac_true = YKK.get_jacobian(joint)

# jac_stu = YKS.get_jacobian(joint)

# jac_diff = jac_true - jac_stu

# print (jac_diff)

singularity = YKS.check_singularity(joint)
print(singularity)

# print(joint)



# jacobian = np.zeros((6,5))
# jacobian1 = np.zeros((6,5))
# T0n = []
# z0i = []
# O0i = []
# # Jp = np.zeros((3,1))
# # JO = np.zeros((3,1))
# z0 = [0, 0, -1]
# z0i.append(z0)

# # row, col = jacobian.shape

# for i in range(0,len(joint)+1):
#     T0n.append(YKS.forward_kinematics(joint, i)) #return T00 up to T05

# for i in range(1,len(joint)):
#     z0i.append(T0n[i][0:3,2])

# for i in range(0,len(joint)):    
#     O0i.append(T0n[i][0:3,3])
    
# P0e = T0n[5][0:3,3]

# for i in range(0,len(joint)):

#     jacobian[0:3,i] = np.cross(z0i[i] , (P0e - O0i[i] ))
#     jacobian[3:6,i] = z0i[i] 
#     # Jp = np.cross(z0i[i] , (P0e - O0i[i] ))
#     # JO = np.array(z0i[i])
#     # JOP = np.vstack((Jp.reshape(3,1) , JO.reshape(3,1)))
#     # for j in range(0,5):
#         # jacobian[:,i] = JOP

# print(jacobian)


# jac_true = YKK.get_jacobian(joint)







# for i in range(0,len(joint)):

#     Jp = np.cross(z0i[i] , (P0e - O0i[i] ))
#     JO = z0i[i]

#     for col in range(col):
#         jacobian[col]= np.vstack((Jp , JO))






