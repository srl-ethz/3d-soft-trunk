// Copyright 2018 ...
#include <mobilerack-interface/ValveController.h>
#include <3d-soft-trunk/SoftTrunkModel.h>
#include <mobilerack-interface/QualisysClient.h>
#include <cmath>
#include <iostream>
const double duration = 20;

void sinusoid(double t, VectorXd& p) {
    double phase = 2*t;
    assert(p.size() == 6);
    p(0) = 250. + 250. * sin(phase);
    p(1) = 250. + 250. * cos(phase);
    if (phase < 3.1415 / 2)
        p(1) == 0;
    p(2) = p(1);
    // opposite actuation for lower segment
    p(3) = p(1);
    p(4) = p(0);
    p(5) = p(0);
    p += 50. * VectorXd::Ones(6); // offset pressure
}

VectorXd p = VectorXd::Zero(6);

int physicalRobot() {
    std::vector<int> map = {0, 1, 2, 3, 6, 4};
    ValveController vc{"192.168.0.100", map, 400};
    QualisysClient qc{"192.168.0.101", 1};

    srl::sleep(1);
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> frames_;
    unsigned long long int timestamp;
    qc.getData(frames_, timestamp);
    fmt::print("time: {}\n", timestamp/1000);
    vc.syncTimeStamp(timestamp/1000);

    double timestep = 0.03;
    srl::Rate r{1. / timestep};
    for (double time = 0; time < duration; time += timestep) {
        sinusoid(time, p);
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