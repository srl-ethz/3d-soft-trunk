from softtrunk_pybind_module import Simulator, State, SoftTrunkModel

stm = SoftTrunkModel()
state = State()

# set initial state
q = state.q
for i in range(len(q)):
    q[i] = 2. / len(q)
state.q = q

# set pressure (use A matrix to figure out number of chambers)
stm.updateState(state)
_, _, _, _, _, A, _ = stm.getModel()
p = [0] * A.shape[1]

sim = Simulator(stm, 0.01, 1, state)
sim.start_log("sim_Python")
i = 0
while True:
    sim.simulate(p)
    if i == 10:
        # once every 10 steps, print the state of robot
        state = sim.getState()
        print(state.q)
        i = 0
    i += 1
sim.end_log()