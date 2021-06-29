from softtrunk_pybind_module import SoftTrunkModel, State, SoftTrunkParameters

st_params = SoftTrunkParameters()
# currently the ability to edit parameter values from the Python interface (as is done in the equivalend C++ example) is not implemented, because of lack of demand & still changing specifications.
# plz implement yourself to src/python_bindings.cpp if needed...
st_params.finalize()

stm = SoftTrunkModel(st_params)
state = st_params.getBlankState()

# editing elements in state
state.q[0] = 0.1 # this method of setting elements does not work
print(state.q)

q = state.q
q[0] = 0.1
state.q = q # this method works
print(state.q)

stm.updateState(state)

B, c, g, K, D, A, J = stm.getModel()
print(f"B:{B}\nc:{c}\ng:{g}\nK:{K}\nD:{D}\nA:{A}\nJ:{J}")

H = stm.get_H(0)
print(f"pose of tip of first segment (relative to base):\n{H}")