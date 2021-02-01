from softtrunk_pybind_module import AugmentedRigidArm
from time import sleep

ara = AugmentedRigidArm()
q = [0] * 6
dq = [0] * 6

for i in range(10):
    q[0] += 0.1
    ara.update(q, dq)
    print(f"tip pose is :{ara.get_H_tip()}")
    sleep(0.5)