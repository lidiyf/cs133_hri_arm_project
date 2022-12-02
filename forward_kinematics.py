# Here is a program that reads urdf and generates a kinematic chain.

import kinpy as kp
import math

chain = kp.build_chain_from_urdf(open("kuka_iiwa/model.urdf").read())
print(chain)
# Displays the parameter names of joint angles included in the chain.
print(chain.get_joint_parameter_names())
th = {'lbr_iiwa_joint_2': math.pi / 4.0, 'lbr_iiwa_joint_4': math.pi / 2.0}
# 'joint name': joint angle value

ret = chain.forward_kinematics(th)

@software{kinpy,
    author = {{Kenta-Tanaka et al.}},
    title = {kinpy},
    url = {https://github.com/neka-nat/kinpy},
    version = {0.0.3},
    date = {2019-10-11},
}

# To load the Gen3 lite description, you would put in your launch file : 
# <param name="robot_description" command="$(find xacro)/xacro --inorder $(find kortex_description)/robots/gen3_lite_gen3_lite_2f.xacro sim:=false"/>