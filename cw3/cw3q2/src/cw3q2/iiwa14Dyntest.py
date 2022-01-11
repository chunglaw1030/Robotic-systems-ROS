from iiwa14DynBase import Iiwa14DynamicBase
from iiwa14DynKDL import Iiwa14DynamicKDL
from iiwa14DynStudent import Iiwa14DynamicRef
import numpy as np
stu = Iiwa14DynamicRef()
kdl = Iiwa14DynamicKDL(Iiwa14DynamicBase)
base = Iiwa14DynamicBase()
# current_pos = kdl.current_joint_position
# current_vel = kdl.current_joint_velocity

current_pos = []

for i in range(7):
    joint_min = base.joint_limit_min[i]
    joint_max = base.joint_limit_max[i]
    joint_val = np.random.uniform(low = joint_min, high = joint_max )
    current_pos.append(joint_val)



# current_pos = [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
current_vel = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

print('The current joint position: ')
print(current_pos)
print('The current joint velocity: ')
print(current_vel)
 
print('B(q): ')
print(kdl.get_B(current_pos))
print('Student B(q): ')
print(stu.get_B(current_pos))

print('C(q, qdot) * qdot: ')
print(kdl.get_C_times_qdot(current_pos, current_vel))
print('Student C(q, qdot) * qdot: ')
print(stu.get_C_times_qdot(current_pos, current_vel))

print('g(q): ')
print(kdl.get_G(current_pos))
print('Student g(q): ')
print(stu.get_G(current_pos))


B = kdl.get_B(current_pos)
Cxqdot = kdl.get_C_times_qdot(current_pos, current_vel)
G = kdl.get_G(current_pos)

jk = Cxqdot - G
print(jk)