# ------------------------------------------------------------------------------
# Example for controlling Sopra. Transcription of the C++ example.
# ------------------------------------------------------------------------------

import sys
sys.path.append('../')
from softtrunk_pybind_module import OSC, SoftTrunkParameters, CurvatureCalculator
from detectron_utils import loadNetwork, detectronTargetPosition

import numpy as np

import threading
import time


def getTargetPosition ():
    # The 3D point might be out of reach for the manipulation configuration space of the gripper. Does this cause any issues?
    pos = np.array([0,0,0])
    return pos


def key_inputs (osc):
    """
    Keyboard events are handled here. 
    """
    global stopping
    while not stopping:
        c = input('').split(" ")[0]
        if c == 'q':
            osc.set_kp(osc.get_kp()*1.1)
        elif c == 'a':
            osc.set_kp(osc.get_kp()*0.9)

        elif c == 'e':
            osc.set_kd(osc.get_kd()*1.1)
        elif c == 'd':
            osc.set_kd(osc.get_kd()*0.9)


        elif c == 't':
            osc.toggleGripper()

        elif c == 'l':
            osc.toggle_log()

        elif c == 'f':
            osc.freeze = not osc.freeze
            print("Freeze status: {}".format(osc.freeze))


        elif c == 'x':
            stopping = True



if __name__ == '__main__':
    ### Initialization
    # Sopra
    st_params = SoftTrunkParameters()
    st_params.finalize()
    osc = OSC(st_params, CurvatureCalculator.SensorType.qualisys, 0)    # 0 obstacles in view
    initial_state = st_params.getBlankState()
    
    # Detectron
    demo, pipeline, align, depth_scale = loadNetwork()


    # Set variables 
    # Starting in singularity bad
    target_pos = np.array([0.1,0,0])
    stopping = False
    
    osc.gripperAttached = True
    osc.loadAttached = 0

    # Start thread for printing out information
    inputs_thread = threading.Thread(target=key_inputs, name="Keyboard_Input_Thread", args=(osc,))
    inputs_thread.start()


    try:
        while not stopping:
            start_time = time.time()
            # Get the 3D point to track
            res = detectronTargetPosition(demo, pipeline, align, depth_scale)
            target_pos = res if res is not None else target_pos

            osc.set_ref(target_pos, np.zeros(3), np.zeros(3))

            x = osc.get_x()
            print('-'*10)
            print(f"Processing time: {1000 * (time.time() - start_time)}ms")
            print("x tip: {}".format(x.transpose()))
            print("x error: {}".format((target_pos-x).transpose()))
            print("x error normalized: {}".format(np.linalg.norm(target_pos-x)))

    finally:
        # Join the threads
        inputs_thread.join()


