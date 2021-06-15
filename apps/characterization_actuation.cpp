#include <mobilerack-interface/ValveController.h>
#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/CurvatureCalculator.h>
#include <cmath>
#include <iostream>
const double duration = 10;

/**
 * @brief swing a single segment back and forth in a planar fashion
 */
void sinusoid(double t, VectorXd& p) {
    double phase = 2*t;
    assert(p.size() == 3);
    p(0) = 250. - 250. * cos(phase);
    p(1) = 250. - 250. * sin(phase);
    if (phase < 3.1415 / 2)
        p(1) = 0;
    p(2) = p(1);
    p += 50. * VectorXd::Ones(3); // offset pressure
}

VectorXd p = VectorXd::Zero(3);

/**
 * @brief actuate a single segment of the arm in a simple swinging motion, and record the pressure and curvature, to be used for characterization of unknown parameters.
 * Use the Python script characterize.py for the actual calculation of the unknown params.
 * See IROS2021 paper (toshimitsu et al., 2021) for details.
 */
int main() {
    SoftTrunkParameters st_params;
    st_params.finalize();
    std::vector<int> map = {0, 1, 2};
    ValveController vc{"192.168.0.100", map, 600};
    CurvatureCalculator cc{st_params, CurvatureCalculator::SensorType::qualisys, "192.168.0.101"};

    // The current characterization process assumes constant curvature across the entire length of the actuated segment.
    // This assumption is not actually true, especially when affected by gravitational forces when the arm is configured with multiple segments.
    // so this process only works for a single segment characterization with nothing connected at its tip.
    assert(st_params.num_segments == 1);

    srl::sleep(1);
    unsigned long long int timestamp = cc.get_timestamp();
    fmt::print("time: {}\n", timestamp/1000);
    vc.syncTimeStamp(timestamp/1000);

    // set the offset pressure
    for (int i = 0; i < 3; i++)
    {
        vc.setSinglePressure(i, 50);
    }
    srl::sleep(2);

    double timestep = 0.01;
    srl::Rate r{1. / timestep};
    for (double time = 0; time < duration; time += timestep) {
        sinusoid(time, p);
        for (int i=0; i<6; i++)
            vc.setSinglePressure(i, (int)p(i));
        r.sleep();
    }
    for (double i = 0; i < 6; i++)
    {
        vc.setSinglePressure(i, 50);
    }
    srl::sleep(6);
    
    return 1;
}