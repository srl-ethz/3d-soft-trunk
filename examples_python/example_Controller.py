# ------------------------------------------------------------------------------
# Example for controlling Sopra. Transcription of the C++ example.
# ------------------------------------------------------------------------------

import sys
sys.path.append('../')
from softtrunk_pybind_module import OSC, SoftTrunkParameters, CurvatureCalculator

import numpy as np

import threading
import time


def gain (osc):
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

        elif c == 'g':
            osc.set_ref(osc.get_objects()[0])

        elif c == 't':
            osc.toggleGripper()

        elif c == 'm':
            osc.toggle_log()

        elif c == 'b':
            osc.loadAttached = 0.2

        elif c == 'f':
            osc.freeze = not osc.freeze
            print("Freeze status: {}".format(osc.freeze))

        elif c == 'x':
            stopping = True

        print("kp = {}, kd = {}".format(osc.get_kp(), osc.get_kd()))
    


def printer (osc):
    """
    Periodically print information
    """
    rate = 0.3
    
    global stopping
    while not stopping:
        x = osc.get_x()

        print("------------------------------------")
        for i, obj in enumerate(osc.get_objects()):
            print("extra object {}: {}\n".format(i, obj.transpose()))
        print("x tip: {}".format(x.transpose()))
        print("x error: {}".format((x_ref-x).transpose()))
        print("x error normalized: {}".format(np.linalg.norm(x_ref-x)))

        time.sleep(rate)



if __name__ == '__main__':
    ### Initialization
    st_params = SoftTrunkParameters()
    st_params.finalize()
    # 0 obstacles in view
    osc = OSC(st_params, CurvatureCalculator.SensorType.qualisys, 0)
    initial_state = st_params.getBlankState()


    # Set variables
    global x_ref, dx_ref, ddx_ref, stopping   # TODO: Ideally we don't want global variables
    stopping = False
    x_ref = np.array([0.15, 0.0, -0.2])
    dx_ref = np.array([0.0, 0.0, 0.0])
    ddx_ref = np.array([0.0, 0.0, 0.0])
    t = 0
    dt = 0.1
    
    r = 0.13
    coef = 2 * 3.1415 / 8
    osc.gripperAttached = True
    osc.loadAttached = 0

    # Start thread for printing out information
    printer_thread = threading.Thread(target=printer, name="Printer_Thread", args=(osc,))
    printer_thread.start()

    input("Press ENTER for gripper...")

    osc.toggleGripper()

    input("Press ENTER for initial x_ref...")

    osc.set_ref(x_ref, dx_ref, ddx_ref)

    input("Press ENTER for starting everything...")

    # Start events for receiving and processing keyboard inputs
    gain_thread = threading.Thread(target=gain, name="Gain", args=(osc,))
    gain_thread.start()

    try:
        while not stopping:
            circle = np.array([r * np.cos(coef*t), r * np.sin(coef*t), -0.215])
            d_circle = np.array([-r * coef * np.sin(coef*t), r * coef * np.cos(coef*t), 0])
            dd_circle = np.array([-r * coef * coef * np.cos(coef*t), -r * coef * coef * np.sin(coef*t), 0])

            osc.set_ref(circle, d_circle, dd_circle)

            t += dt
            time.sleep(dt)


        ### Throwing motion
        # Throw
        x_ref = np.array([-0.15, 0.0, -0.2])
        dx_ref[0] = -10

        osc.set_ref(x_ref, dx_ref, ddx_ref)
        time.sleep(0.2)

        # Release
        osc.toggleGripper()
        time.sleep(0.1)

        # Reset
        dx_ref[0] = 0
        osc.set_ref(x_ref, dx_ref, ddx_ref)
        time.sleep(3)

    finally:
        # Join the threads
        printer_thread.join()
        gain_thread.join()


