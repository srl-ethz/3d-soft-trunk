from os import write
from softtrunk_pybind_module import Simulator, State, SoftTrunkModel
import numpy as np

x_des = np.array([0.11, 0., 0.2])
dx_des = np.zeros(3)
k_p = 0.5
k_d = 2 * np.sqrt(k_p)  # critically damped approach to target

stm = SoftTrunkModel()
state = State()

stm.updateState(state)
_, _, _, _, _, A, _ = stm.getModel()
num_segments = A.shape[1]//3
print(f"number of segments: {num_segments}")
assert len(state.q) == 2 * num_segments, "assuming that N_PCC is set to 1..."

# construct input matrix for pseudopressure
A_pseudo = np.zeros((num_segments*2, num_segments*2))
for i in range(num_segments):
    A_pseudo[2*i, 2*i] = A[2*i, 3*i]
    A_pseudo[2*i+1, 2*i+1] = A_pseudo[2*i, 2*i]
print(f"A:\n{A}\nA_pseudo:\n{A_pseudo}")

# create function to map it back
inv = np.linalg.pinv(np.array([[1, -0.5, -0.5], [0, np.sqrt(3)/2, -np.sqrt(3)/2]]))
print(f"pinv:{inv}")


def pseudo2real(p_pseudo):
    p = np.zeros(num_segments*3)
    for i in range(num_segments):
        p_seg = np.matmul(inv, p_pseudo[2*i: 2*i+2])
        # convert to positive pressure
        min_p = np.min(p_seg)
        if min_p < 0:
            p_seg -= min_p
        p[3*i:3*i+3] = p_seg
    return p


initial_state = State()
sim = Simulator(stm, 0.01, 1, initial_state)
ddx_ref = np.zeros(3)  # reference acceleration in operational space
p = np.zeros(3*num_segments)
while True:
    state = sim.getState()
    stm.updateState(state)
    B, c, g, K, D, A, J = stm.getModel()
    # calculate inertia matrix in operational space
    B_op = np.linalg.inv(np.linalg.multi_dot([J, np.linalg.inv(B), J.transpose()]))
    x = stm.get_H(num_segments-1)[0:3, 3]
    dx = np.matmul(J, state.dq)
    ddx_ref = k_p * (x_des - x) + k_d * (dx_des - dx)
    tau_ref = np.linalg.multi_dot([J.transpose(), B_op, ddx_ref])
    p_pseudo = np.matmul(np.linalg.inv(A_pseudo), tau_ref)
    sim.simulate(pseudo2real(p_pseudo))
    print(f"x:{x}")

# isn't nearly stable at all...