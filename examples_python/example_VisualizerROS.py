import rospy
from softtrunk_ROS_pybind_module import VisualizerROS, ros_init_custom, ros_ok_custom
from softtrunk_pybind_module import SoftTrunkModel, State, SoftTrunkParameters
from time import sleep
import math

st_params = SoftTrunkParameters()
st_params.finalize()
stm = SoftTrunkModel(st_params)
ros_init_custom("example_VisualizerROS")  # must be called before creating VisualizerROS
vis = VisualizerROS(stm)
state = st_params.getBlankState()

t = 0.
while ros_ok_custom():
    q = state.q
    q[0] = 1. * math.sin(t)
    state.q = q
    stm.updateState(state)
    vis.publishState()
    vis.publishArrow(1, [0.1, 0, 0], [1, 0, 0], True)
    sleep(0.1)
    t += 0.1
