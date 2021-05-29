#include "3d-soft-trunk/Characterize.h"

Characterize::Characterize(CurvatureCalculator::SensorType sensor_type) : ControllerPCC(sensor_type){

}

void Characterize::logRadialPressureDist(int segment){
    VectorXd pressures = VectorXd::Zero(2 * st_params::num_segments);
    filename = "radialPressureDist";

    filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
    fmt::print("Starting radial log to {}\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "angle";

    //write header
    log_file << fmt::format(", angle_measured, r, x, y, z");
    for (int i=0; i < st_params::q_size; i++)
        log_file << fmt::format(", q_{}", i);
    log_file << "\n"; 
    
    pressures(2*segment) = 500;

    actuate(stm->pseudo2real(pressures));
    srl::sleep(5);
    srl::Rate r{3};

    for (int i = 0; i < 360; i++){
        pressures(2*segment) = 500*cos(i*deg2rad);
        pressures(2*segment+1) = -500*sin(i*deg2rad);

        actuate(stm->pseudo2real(pressures));
        fmt::print("angle: {}, pressure: {}\n", i, stm->pseudo2real(pressures).transpose());

        cc->get_curvature(state);
        stm->updateState(state);
        x = stm->get_H_base().rotation()*stm->get_H(st_params::num_segments-1).translation();

        double angle = atan2(x(1),x(0))*180/3.14156;
        if (angle < 0) angle+=360;
        log_file << fmt::format("{},{},{},{},{}", i, angle, sqrt(x(0)*x(0)+x(1)*x(1)), x(0), x(1), x(2));

        for (int i=0; i < st_params::q_size; i++)               //log q
            log_file << fmt::format(", {}", state.q(i));
        log_file << "\n";
        r.sleep();
    }
    log_file.close();
    fmt::print("\n Finished radial logging\n");
}

void Characterize::calcGravK(int segment, int directions){
    MatrixXd K = VectorXd::Zero(2*directions*5);
    VectorXd tau = VectorXd::Zero(2*directions*5);
    double angle = 0;
    VectorXd pressures = VectorXd::Zero(2*st_params::num_segments);
    fmt::print("Starting coefficient characterization in {} directions\n",directions);
    for (int i = 0; i < directions; i ++){
        angle = i*360/directions; 

        for (int j = 0; j < 5; j++){                                //iterate through multiple heights for the respective angle
            pressures(2*segment) = (100+100*j)*cos(angle*deg2rad);
            pressures(2*segment+1) = -(100+100*j)*sin(angle*deg2rad);
            actuate(stm->pseudo2real(pressures));
            fmt::print("angle = {}, intensity = {}\n", angle, 100+100*j);

            srl::sleep(10); //wait to let swinging subside
            cc->get_curvature(state);
            stm->updateState(state);

            tau(10*i + 2*j) = pressures(2*segment) - stm->g(2*segment);;
            tau(10*i + 2*j + 1) = pressures(2*segment+1) - stm->g(2*segment+1);
            //fill in matrix containing g, K
            K(10*i + 2*j) = (stm->K*state.q)(2*segment);
            K(10*i + 2*j + 1) = (stm->K*state.q)(2*segment + 1);

        }
    }

    VectorXd Kcoeff = (K.transpose()*K).inverse()*K.transpose()*tau;
    fmt::print("Finished coeffient characterization. Best fit is g + {}*K*q", Kcoeff.transpose());
}