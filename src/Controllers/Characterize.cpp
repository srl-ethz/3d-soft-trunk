#include "3d-soft-trunk/Controllers/Characterize.h"

Characterize::Characterize(const SoftTrunkParameters st_params) : ControllerPCC(st_params){

}

void Characterize::logRadialPressureDist(int segment, std::string fname){
    VectorXd pressures = VectorXd::Zero(st_params_.p_pseudo_size);
    filename_ = fname;

    filename_ = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename_);
    fmt::print("Starting radial log to {}\n", filename_);
    log_file_.open(filename_, std::fstream::out);
    log_file_ << "angle";
    //write header
    log_file_ << fmt::format(", angle_measured, r");
    log_file_ << "\n"; 
    
    pressures(2*segment) = 500;

    actuate(mdl_->pseudo2real(pressures));
    srl::sleep(5);
    srl::Rate r{1};

    VectorXd ang_err = VectorXd::Zero(360);
    VectorXd radii = VectorXd::Zero(360);
    MatrixXd angle_vals = MatrixXd::Zero(360,4);

    for (double i = 0; i < 360; i+=1.){
        pressures(2*segment+st_params_.prismatic*2) = 500*cos(i*deg2rad);
        pressures(2*segment+1+st_params_.prismatic*2) = -500*sin(i*deg2rad);

        actuate(mdl_->pseudo2real(pressures));        

        x_ = state_.tip_transforms[st_params_.num_segments+st_params_.prismatic].translation();
        double angle = atan2(x_(1),x_(0))*180/3.14156;
        if (angle < 0) angle+=360;

        fmt::print("angle: {}, angle_measured: {} radius: {}\n", (double) i, angle, sqrt(x_(1)*x_(1) + x_(0)*x_(0)));

        log_file_ << fmt::format("{},{},{}", (double) i, angle, sqrt(x_(0)*x_(0)+x_(1)*x_(1)));
        ang_err(i) = i - angle;
        if (abs(ang_err(i)) > 180) ang_err(i) -= ((ang_err(i) > 0) - (ang_err(i) < 0))*360;
        angle_vals(i,0) = i*i*i;
        angle_vals(i,1) = i*i;
        angle_vals(i,2) = i;
        angle_vals(i,3) = 1;
        radii(i) = sqrt(x_(0)*x_(0)+x_(1)*x_(1));

        log_file_ << "\n";
        r.sleep();
    }
    log_file_.close();
    fmt::print("\n Finished radial logging\n");   

    std::string poly_location = fmt::format("{}/polynomial_{}.txt", SOFTTRUNK_PROJECT_DIR,segment);
    std::fstream polynomial_out;
    polynomial_out.open(poly_location, std::fstream::out);

    for(int p = 0; p < 3; p++){ 
        MatrixXd angval120 = angle_vals.block(0,0,120,4);
        VectorXd poly_coeffs = (angval120.transpose()*angval120).inverse()*angval120.transpose()*ang_err.segment(p*120,120); //calculate polynomial coeffs using least squares
        polynomial_out << fmt::format("angular: {}*pow(angle-{},3) + {}*pow(angle-{},2) + {}*(angle-{}) + {}\n",poly_coeffs(0),p*120,poly_coeffs(1),p*120,poly_coeffs(2),p*120,poly_coeffs(3));
        poly_coeffs = (angval120.transpose()*angval120).inverse()*angval120.transpose()*radii.segment(p*120,120); //calculate polynomial coeffs using least squares
        polynomial_out << fmt::format("radial: {}*pow(angle-{},3) + {}*pow(angle-{},2) + {}*(angle-{}) + {}\n",poly_coeffs(0),p*120,poly_coeffs(1),p*120,poly_coeffs(2),p*120,poly_coeffs(3));

    }
}

void Characterize::calcK(int segment, int directions, int verticalsteps){
    VectorXd K = VectorXd::Zero(2*directions*verticalsteps,1);
    VectorXd tau = VectorXd::Zero(2*directions*verticalsteps);
    double angle = 0;
    VectorXd pressures = VectorXd::Zero(2*st_params.num_segments);
    fmt::print("Starting coefficient characterization in {} directions\n", directions);
    for (int i = 0; i < directions; i ++){
        angle = 45+i*360/directions; 

        for (int j = 0; j < verticalsteps; j++){                                //iterate through multiple heights for the respective angle
            pressures(2*segment) = (500/verticalsteps+500*j/verticalsteps)*cos(angle*deg2rad);
            pressures(2*segment+1) = -(500/verticalsteps+500*j/verticalsteps)*sin(angle*deg2rad);
            if(sensor_type == CurvatureCalculator::SensorType::simulator) simulate(pressures);
                else actuate(stm->pseudo2real(pressures));
            fmt::print("angle = {}, intensity = {}\n", angle, 500/verticalsteps+500*j/verticalsteps);

            srl::sleep(10); //wait to let swinging subside
            cc->get_curvature(state);
            stm->set_state(state);

            tau(2*verticalsteps*i + 2*j) = pressures(2*segment) - (stm->A_pseudo.inverse()*stm->g/100)(2*segment);
            tau(2*verticalsteps*i + 2*j + 1) = pressures(2*segment+1) - (stm->A_pseudo.inverse()*stm->g/100)(2*segment+1);

            K(2*verticalsteps*i + 2*j) = (stm->A_pseudo.inverse()*stm->K*state.q/100)(2*segment);
            K(2*verticalsteps*i + 2*j + 1) = (stm->A_pseudo.inverse()*stm->K*state.q/100)(2*segment + 1);

        }
    }

    VectorXd Kcoeff = (K.transpose()*K).inverse()*K.transpose()*tau;
    
    fmt::print("Finished coeffient characterization. Best fit is g + {}*K*q\n\n", Kcoeff(0));
}