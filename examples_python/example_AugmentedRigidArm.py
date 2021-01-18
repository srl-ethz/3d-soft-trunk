from softtrunk_pybind_module import AugmentedRigidArm
from time import sleep

aar = AugmentedRigidArm()
q = [0] * 6
dq = [0] * 6

for i in range(10):
    q[0] += 0.1
    aar.update(q, dq)
    sleep(0.5)