from os import write
from softtrunk_pybind_module import Simulator, State, SoftTrunkModel
import numpy as np

# rather carefully determined configuration of the task movement
x_des_center = np.array([0., 0.1, -0.2])  # center of task space circular movement, in global coordinates
amplitude = 0.03  # amplitude of task space movement
T = 2  # cycle of movement
k_p = 50.
k_d = 2. * np.sqrt(k_p)  # critically damped approach to target
alpha = 0.1  # determine "cost" for deviating from straight

dt = 0.01  # timestep. Currently this is set to be both the control & sim frequency.

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

# get rotation matrix of base
H_base = stm.get_H_base()
R_base = H_base[:3, :3]


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


# set initial state
# when this is changed, the movement could get unstable - an area for improvement!
initial_state = State()
initial_q = 0.1
q = initial_state.q
for i in range(num_segments):
    q[2*i + 1] = -initial_q
initial_state.q = q

sim = Simulator(stm, dt, 1, initial_state)
t = 0.
while True:
    # compute desired trajectory
    coef = 2 * np.pi / T
    x_des = x_des_center + amplitude * np.array([np.cos(coef*t), 0, np.sin(coef*t)])
    dx_des = amplitude * coef * np.array([- np.sin(coef*t), 0, np.cos(coef*t)])

    state = sim.getState()
    stm.updateState(state)
    B, c, g, K, D, A, J = stm.getModel()
    B_inv = np.linalg.inv(B)

    # calculate parameters in operational space
    B_op = np.linalg.inv(np.linalg.multi_dot([J, B_inv, J.transpose()]))
    g_op = np.linalg.multi_dot([B_op, J, B_inv, g])

    # inertia-weighted pseudoinverse
    J_inv = np.linalg.multi_dot([np.linalg.inv(B), J.transpose(), B_op])

    x = np.matmul(R_base, stm.get_H(num_segments-1)[0:3, 3])  # tip position in world frame
    dx = np.matmul(J, state.dq)  # tip speed in world frame
    ddx_ref = k_p * (x_des - x) + k_d * (dx_des - dx)  # critically damped approach to target
    f = np.matmul(B_op, ddx_ref) + g_op
    tau_null = - alpha * state.q
    tau_ref = np.matmul(J.transpose(), f) + np.matmul(K, state.q) + np.matmul(D, state.dq) + \
        np.matmul((np.eye(num_segments*2) - np.matmul(J.transpose(), J_inv.transpose())), tau_null)

    p_pseudo = np.matmul(np.linalg.inv(A_pseudo), tau_ref)
    p = pseudo2real(p_pseudo)

    sim.simulate(p)
    np.set_printoptions(precision=3)
    print(f"P [mbar]: {p/100}")
    t += dt
