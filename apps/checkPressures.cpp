#include "3d-soft-trunk/ControllerPCC.h"

int main() {
    ControllerPCC cpcc{CurvatureCalculator::SensorType::qualisys};
    VectorXd pressures = VectorXd::Zero(2*st_params::num_segments);
    srl::State state;
    std::fstream log_file;
    std::string filename = "pressureMapping";

    filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
    fmt::print("Starting log to {}\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "angle";

    //write header
    log_file << fmt::format(", angle_measured, r, x, y, z");
    for (int i=0; i < st_params::q_size; i++)
        log_file << fmt::format(", q_{}", i);
    log_file << "\n";

    const double deg2rad = 0.01745329;
    pressures << 500, 0, 0, 0;
    cpcc.actuate(cpcc.stm->pseudo2real(pressures));
    srl::sleep(5);
    srl::Rate r{1};

    for (int i = 0; i < 360; i++){
        pressures << 500*cos(i*deg2rad), -500*sin(i*deg2rad), 0, 0;
        cpcc.actuate(cpcc.stm->pseudo2real(pressures));
        fmt::print("angle: {}, pressure: {}\n", i, cpcc.stm->pseudo2real(pressures).transpose());

        cpcc.cc->get_curvature(state);
        cpcc.stm->updateState(state);
        Vector3d x = cpcc.stm->get_H_base().rotation()*cpcc.stm->get_H(st_params::num_segments-1).translation();
        fmt::print("{}\n",x.transpose());
        double angle = atan2(x(1),x(0))*180/3.14156;
        if (angle < 0) angle+=360;
        log_file << fmt::format("{},{},{},{},{}", i, angle, sqrt(x(0)*x(0)+x(1)*x(1)), x(0), x(1), x(2));

        for (int i=0; i < st_params::q_size; i++)               //log q
            log_file << fmt::format(", {}", cpcc.state.q(i));
        log_file << "\n";
        r.sleep();
    }
    log_file.close();
    fmt::print("\n finished logging");

}