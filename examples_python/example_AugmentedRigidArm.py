from softtrunk_pybind_module import AugmentedRigidArm, State, SoftTrunkParameters
from time import sleep

# because of the current implementation, an instance of SoftTrunkModel must be run beforehand to generate the URDF / XACRO model of the robot.
st_params = SoftTrunkParameters()
st_params.finalize()

ara = AugmentedRigidArm(st_params)
state = st_params.getBlankState()
q = state.q

for i in range(10):
    q[0] += 0.1
    state.q = q
    ara.update(state)
    print(f"first segment pose is :{ara.get_H(0)}")
    print(f"tip pose is :{ara.get_H_tip()}")
    print(f"base pose is : {ara.get_H_base()}")
    sleep(0.5)