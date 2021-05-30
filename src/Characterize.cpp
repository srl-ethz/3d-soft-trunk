#include "3d-soft-trunk/Characterize.h"

Characterize::Characterize(CurvatureCalculator::SensorType sensor_type) : ControllerPCC(sensor_type){

}

void Characterize::logRadialPressureDist(int segment, std::string fname){
    VectorXd pressures = VectorXd::Zero(2 * st_params::num_segments);
    filename = fname;

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
    srl::Rate r{2};

    for (int i = 0; i < 720; i++){
        pressures(2*segment) = 500*cos(i*deg2rad/2);
        pressures(2*segment+1) = -500*sin(i*deg2rad/2);

        actuate(stm->pseudo2real(pressures));
        fmt::print("angle: {}, pressure: {}\n", (i+0.0)/2, stm->pseudo2real(pressures).transpose());

        cc->get_curvature(state);
        stm->updateState(state);
        x = stm->get_H_base().rotation()*stm->get_H(st_params::num_segments-1).translation();

        double angle = atan2(x(1),x(0))*180/3.14156;
        if (angle < 0) angle+=360;
        log_file << fmt::format("{},{},{},{},{}", (i+0.0)/2, angle, sqrt(x(0)*x(0)+x(1)*x(1)), x(0), x(1), x(2));

        for (int i=0; i < st_params::q_size; i++)               //log q
            log_file << fmt::format(", {}", state.q(i));
        log_file << "\n";
        r.sleep();
    }
    log_file.close();
    fmt::print("\n Finished radial logging\n");
}

void Characterize::calcGravK(int segment, int directions, int verticalsteps){
    MatrixXd gK = MatrixXd::Zero(2*directions*verticalsteps,1);
    VectorXd tau = VectorXd::Zero(2*directions*verticalsteps);
    double angle = 0;
    VectorXd pressures = VectorXd::Zero(2*st_params::num_segments);
    fmt::print("Starting coefficient characterization in {} directions\n", directions);
    for (int i = 0; i < directions; i ++){
        angle = 45+i*360/directions; 

        for (int j = 0; j < verticalsteps; j++){                                //iterate through multiple heights for the respective angle
            pressures(2*segment) = (500/verticalsteps+500*j/verticalsteps)*cos(angle*deg2rad);
            pressures(2*segment+1) = -(500/verticalsteps+500*j/verticalsteps)*sin(angle*deg2rad);
            actuate(stm->pseudo2real(pressures));
            fmt::print("angle = {}, intensity = {}\n", angle, 500/verticalsteps+500*j/verticalsteps);

            srl::sleep(10); //wait to let swinging subside
            cc->get_curvature(state);
            stm->updateState(state);

            tau(2*verticalsteps*i + 2*j) = pressures(2*segment) - (stm->A_pseudo.inverse()*stm->K*state.q/100)(2*segment);
            tau(2*verticalsteps*i + 2*j + 1) = pressures(2*segment+1) - (stm->A_pseudo.inverse()*stm->K*state.q/100)(2*segment + 1);
            //fill in matrix containing g, K
            //gK(2*verticalsteps*i + 2*j, 0) = (stm->A_pseudo.inverse()*stm->g/100)(2*segment);
            //gK(2*verticalsteps*i + 2*j + 1, 0) = (stm->A_pseudo.inverse()*stm->g/100)(2*segment+1);

            gK(2*verticalsteps*i + 2*j, 0) = (stm->A_pseudo.inverse()*stm->g/100)(2*segment);
            gK(2*verticalsteps*i + 2*j + 1, 0) = (stm->A_pseudo.inverse()*stm->g/100)(2*segment+1);

        }
    }

    VectorXd gKcoeff = (gK.transpose()*gK).inverse()*gK.transpose()*tau;
    
    fmt::print("Finished coeffient characterization. Best fit is {}*g + K*q\n\n", gKcoeff(0));
    fmt::print("tau: \n{}\n gK Matrix: \n{}\n", tau.transpose(), gK);
}