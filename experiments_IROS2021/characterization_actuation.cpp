#include <mobilerack-interface/ValveController.h>
#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/CurvatureCalculator.h>
#include <cmath>
#include <iostream>
const double duration = 20;

/**
 * @brief swing planar
 */
void sinusoid(double t, VectorXd& p) {
    double phase = 2*t;
    assert(p.size() == 6);
    p(0) = 250. - 250. * cos(phase);
    p(1) = 250. - 250. * sin(phase);
    if (phase < 3.1415 / 2)
        p(1) = 0;
    p(2) = p(1);
    // opposite actuation for lower segment
    p(3) = p(1);
    p(4) = p(0);
    p(5) = p(0);
    p += 50. * VectorXd::Ones(6); // offset pressure
}

/**
 * @brief swing around
 */
void sinusoid2(double t, VectorXd& p) {
    double phase = 2*t;
    assert(p.size() == 6);
    p(0) = 250. - 250. * cos(phase);
    p(1) = 250. - 250. * cos(phase - 3.14 * 2. / 3.);
    p(2) = 250. - 250. * cos(phase - 3.14 * 4. / 3.);
    if (phase - 3.14 * 2./3 < 0)
        p(1) = 0;
    if (phase - 3.14 * 4./3 < 0)
        p(2) = 0;
    // opposite actuation for lower segment
    p(3) = p(0);
    p(4) = p(1);
    p(5) = p(2);
    p += 50. * VectorXd::Ones(6); // offset pressure
}

VectorXd p = VectorXd::Zero(6);

int physicalRobot() {
    std::vector<int> map = {0, 1, 2, 3, 6, 4};
    ValveController vc{"192.168.0.100", map, 600};
    CurvatureCalculator cc{CurvatureCalculator::SensorType::qualisys, "192.168.0.101"};

    srl::sleep(1);
    unsigned long long int timestamp = cc.get_timestamp();
    fmt::print("time: {}\n", timestamp/1000);
    vc.syncTimeStamp(timestamp/1000);

    double timestep = 0.01; // ValveController was temporarily modified to 100Hz for this (with wired connection)
    srl::Rate r{1. / timestep};
    for (double time = 0; time < duration; time += timestep) {
        sinusoid2(time, p);
        for (int i=0; i<6; i++)
            vc.setSinglePressure(i, (int)p(i));
        r.sleep();
    }
    return 1;
}

int simulatedRobot(){
    SoftTrunkModel stm{};

    VectorXd q = VectorXd::Zero(2*st_params::num_segments*st_params::sections_per_segment);
    VectorXd dq = VectorXd::Zero(q.size());

    VectorXd ddq;
    VectorXd q_mid;
    VectorXd dq_mid;
    VectorXd ddq_mid;

    double dt = 0.0002;
}


int main(){
    physicalRobot();
}