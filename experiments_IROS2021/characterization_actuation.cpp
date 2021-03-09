#include <mobilerack-interface/ValveController.h>
#include <3d-soft-trunk/SoftTrunkModel.h>
#include <3d-soft-trunk/CurvatureCalculator.h>
#include <cmath>
#include <iostream>
const double duration = 10;

/**
 * @brief swing planar
 */
void sinusoid(double t, VectorXd& p) {
    double phase = 2*t;
    assert(p.size() == 3);
    p(0) = 250. - 250. * cos(phase);
    p(1) = 250. - 250. * sin(phase);
    if (phase < 3.1415 / 2)
        p(1) = 0;
    p(2) = p(1);
    // opposite actuation for lower segment
    //p(3) = p(1);
    //p(4) = p(0);
    //p(5) = p(0);
    p += 50. * VectorXd::Ones(3); // offset pressure
}

/**
 * @brief swing around
 * this is for the feedforward actuation experiment, isn't related to characterization...
 */
void sinusoid2(double t, VectorXd& p) {
    double phase;
    double P = 240;
    if (t < duration/2)
        phase = 3*t;
    else
        phase = 3*(duration-t);
    p(0) = P *(1 - cos(phase));
    p(1) = P *(1 - cos(phase - 3.14 * 2. / 3.));
    p(2) = P *(1 - cos(phase - 3.14 * 4. / 3.));
    if (phase - 3.14 * 2./3 < 0)
        p(1) = 0;
    if (phase - 3.14 * 4./3 < 0)
        p(2) = 0;
    
    phase = 3*t;
    p(3) = P * (1 -  cos(phase));
    p(4) = P * (1 - cos(phase - 3.14 * 2. / 3.));
    p(5) = P * (1- cos(phase - 3.14 * 4. / 3.));
    if (duration * 0.75 < t)
        p.segment(3,3) *= (1. - (t - duration*0.75)/duration/0.25);
    if (phase - 3.14*2./3. < 0)
        p(4) = 0;
    if (phase - 3.14*4./3. < 0)
        p(5) = 0;
    
    p += 50. * VectorXd::Ones(6); // offset pressure
}

VectorXd p = VectorXd::Zero(6);

int physicalRobot() {
    std::vector<int> map = {0, 1, 6, 3, 4, 2};
    ValveController vc{"192.168.0.100", map, 600};
    CurvatureCalculator cc{CurvatureCalculator::SensorType::qualisys, "192.168.0.101"};

    srl::sleep(1);
    unsigned long long int timestamp = cc.get_timestamp();
    fmt::print("time: {}\n", timestamp/1000);
    vc.syncTimeStamp(timestamp/1000);

    for (int i = 0; i < 6; i++)
    {
        vc.setSinglePressure(i, 50);
    }
    srl::sleep(2);
    

    double timestep = 0.01; // ValveController was temporarily modified to 100Hz for this (with wired connection)
    srl::Rate r{1. / timestep};
    for (double time = 0; time < duration; time += timestep) {
        sinusoid2(time, p);
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

int simulatedRobot(){
    SoftTrunkModel stm{};

    Pose pose;
    Pose pose_mid;

    double dt = 0.0002;
}


int main(){
    physicalRobot();
}
