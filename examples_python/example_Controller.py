from softtrunk_pybind_module import ControllerPCC, State, CurvatureCalculator, SoftTrunkParameters
import numpy as np

st_params = SoftTrunkParameters()
st_params.finalize()

initial_state = st_params.getBlankState()
# set initial state
q = initial_state.q
q = 2. / q.size * np.ones(q.size)
initial_state.q = q

# set pressure (use size of state to figure out number of chambers)
num_segments = initial_state.q.size // 2
print(f"number of segements: {num_segments}")
p = np.zeros(3 * num_segments)

# simulator functionality is provided as part of the Controller class
ctrl = ControllerPCC(st_params, CurvatureCalculator.SensorType.simulator, True)
dt = 0.01
ctrl.set_frequency(1./dt)
ctrl.set_state(initial_state)

i = 0
while True:
    ctrl.simulate(p)
    if i == 10:
        # once every 10 steps, print the state of robot
        state = ctrl.get_state()
        print(state.q)
        i = 0
    i += 1