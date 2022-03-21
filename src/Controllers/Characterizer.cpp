#include "3d-soft-trunk/Controllers/Characterizer.h"

Characterize::Characterize(const SoftTrunkParameters st_params) : ControllerPCC(st_params){
    new_params = st_params_;
}

void Characterize::angularError(int segment, std::string fname){
    VectorXd pressures = VectorXd::Zero(st_params_.p_pseudo_size);
    
    filename_ = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, fname);
    fmt::print("Starting radial log to {}\n", fname);
    log_file_.open(filename_, std::fstream::out);
    log_file_ << "angle";
    //write header
    log_file_ << fmt::format(", angle_measured, r");
    log_file_ << "\n"; 
    
    pressures(2*(segment+st_params_.prismatic)) = 500;

    actuate(mdl_->pseudo2real(pressures));
    srl::sleep(5);
    srl::Rate r{1};

    VectorXd ang_err = VectorXd::Zero(360);
    VectorXd radii = VectorXd::Zero(360);
    MatrixXd angle_vals = MatrixXd::Zero(360,4);

    for (double i = 0; i < 360; i+=1.){ //Draw a circle while logging measured angle vs desired angle
        pressures(2*(segment+st_params_.prismatic)+st_params_.prismatic) = 500*cos(i*deg2rad);
        pressures(2*(segment+st_params_.prismatic)+1+st_params_.prismatic) = 500*sin(i*deg2rad);

        actuate(mdl_->pseudo2real(pressures));        

        double angle = atan2(state_.q(2*segment + st_params_.prismatic + 1),state_.q(2*segment + st_params_.prismatic))*180/3.14156;
        if (angle < 0) angle+=360;

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

    std::vector<double> angOffsetCoeffs(12);
    
    for (int i = 0; i < 3; i++){ //least squares solve to obtain polynomials describing the angular error
        VectorXd poly_coeffs = (angle_vals.block(120*i,0,120,4).transpose()*angle_vals.block(120*i,0,120,4)).inverse()
            *angle_vals.block(120*i,0,120,4).transpose()*ang_err.segment(120*i,120); //calculate polynomial coeffs using least squares
            
        for (int j = 0; j < 4; j++){
            angOffsetCoeffs[4*i+j] = poly_coeffs(4*i+j);
        }
    }
    
    new_params.angOffsetCoeffs = angOffsetCoeffs;
}

void Characterize::stiffness(int segment, int verticalsteps, int maxpressure){
    VectorXd K = VectorXd::Zero(4*verticalsteps);
    VectorXd tau = VectorXd::Zero(4*verticalsteps);
    double angle = 0;
    VectorXd pressures = VectorXd::Zero(st_params_.p_pseudo_size);
    fmt::print("Estimating stiffness for segment {} in {} directions\n", segment, 4);
    for (int i = 0; i < 4; i++){
        angle = i*360/4; 

        for (int j = 0; j < verticalsteps; j++){        //iterate through multiple heights for the respective angle
            pressures(2*segment+st_params_.prismatic) = (maxpressure/verticalsteps+maxpressure*j/verticalsteps)*cos(angle*deg2rad);
            pressures(2*segment+st_params_.prismatic+1) = -(maxpressure/verticalsteps+maxpressure*j/verticalsteps)*sin(angle*deg2rad);
            actuate(mdl_->pseudo2real(pressures));

            srl::sleep(10); //wait to let swinging subside

            //log values of the dynamic equation (assumption: no movement, since we waited 10 seconds)
            tau(verticalsteps*i+j) = pressures(2*segment+st_params_.prismatic + i%2) - (dyn_.A_pseudo.inverse()*dyn_.g/100)(2*segment+st_params_.prismatic+i%2);
            K(verticalsteps*i+j) = (dyn_.A_pseudo.inverse()*dyn_.K*state_.q/100)(2*segment+st_params_.prismatic+i%2);
        }
    }

    VectorXd Kcoeff = (K.transpose()*K).inverse()*K.transpose()*tau; //least squares solve for stiffness
    
    new_params.shear_modulus[segment] = Kcoeff(0)*st_params_.shear_modulus[segment];
    fmt::print("Shear Modulus for segment {}: {}\n", segment, Kcoeff(0)*st_params_.shear_modulus[segment]);
}

bool Characterize::valveMap(int maxpressure){
    std::vector<int> newMap(st_params_.p_size);

    for (int i = 0; i < st_params_.p_size - 2*st_params_.prismatic; i++){
        vc_->setSinglePressure(i+2*st_params_.prismatic,300);
        srl::sleep(7);
        int segment = 0;
        double largest = 0;

        //find which segment contains the largest curvature
        for (int j = 0; j < st_params_.num_segments; j++){
            if (state_.q.segment(2*j+st_params_.prismatic,2).norm() > largest){
                largest = state_.q.segment(2*j+st_params_.prismatic,2).norm();
                segment = j;
            }
        }
        fmt::print("q: {}\n", state_.q.transpose());

        if (largest < 0.18) { //if nothing is bending, it must be the gripper
            fmt::print("Gripper\n");
            newMap[st_params_.p_size - 1] = st_params_.valvemap[i+2*st_params_.prismatic];
        } else { //otherwise, determine which chamber it is
            fmt::print("Segment {}, direction: ", segment);
            double angle = atan2(state_.q(2*segment+st_params_.prismatic+1),
                state_.q(2*segment+st_params_.prismatic))*180/3.14156; //angle of bending
            
            if (angle > 120 or angle <= -120){
                fmt::print("1\n");
                newMap[3*segment+2*st_params_.prismatic] = st_params_.valvemap[i+2*st_params_.prismatic];
            }
            else if (angle > 0 and angle <= 120){
                fmt::print("2\n");
                newMap[3*segment+1+2*st_params_.prismatic] = st_params_.valvemap[i+2*st_params_.prismatic];
            }
            else if (angle <= 0 and angle > -120){
                fmt::print("3\n");
                newMap[3*segment+2+2*st_params_.prismatic] = st_params_.valvemap[i+2*st_params_.prismatic];
            }
            else return false;
        }

        vc_->setSinglePressure(i+2*st_params_.prismatic,0);
    }

    if(st_params_.prismatic){
        newMap[0] = st_params_.valvemap[0];
        newMap[1] = st_params_.valvemap[1];
    }
    
    fmt::print("New map: ");
    for (int i = 0; i < newMap.size(); i++){
        fmt::print("{} ", newMap[i]);
    }
    fmt::print("\n");



    new_params.valvemap = newMap;
    return true;
}

void Characterize::actuation(int segment, int pressure){
    filename_ = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename_);
    log_file_.open(fmt::format("{}/actuationEstimate.csv", SOFTTRUNK_PROJECT_DIR), std::fstream::out);
    fmt::print("Estimating A for segment {}\n",segment);
    int rotation = 360;
    MatrixXd p = MatrixXd::Zero(3,rotation);
    MatrixXd Kqg = MatrixXd::Zero(2,rotation);

    vc_->setSinglePressure(2*st_params_.prismatic+segment*3, pressure);
    srl::sleep(8);
    srl::Rate r{2};

    for (int i = 0; i < rotation; i++){ //draw a circle while logging dynamic equation coefficients
        
        if (i >= 0 && i < 120) p.block(0,i,3,1) << cos(i*deg2rad*90/120)*pressure, sin(i*deg2rad*90/120)*pressure, 0;
        if (i >= 120 && i < 240) p.block(0,i,3,1) << 0, sin(i*deg2rad*90/120)*pressure, sin((i-120)*90*deg2rad/120)*pressure;
        if (i >= 240 && i < 360) p.block(0,i,3,1) << sin((i-240)*deg2rad*90/120)*pressure, 0, sin((i-120)*90*deg2rad/120)*pressure;

        vc_->setSinglePressure(2*st_params_.prismatic+segment*3, p(0,i));
        vc_->setSinglePressure(2*st_params_.prismatic+segment*3+1, p(1,i));
        vc_->setSinglePressure(2*st_params_.prismatic+segment*3+2, p(2,i));

        Kqg.block(0,i,2,1) = (dyn_.g + dyn_.K*state_.q).segment(st_params_.prismatic + 2*segment, 2);

        r.sleep();
    }
    vc_->setSinglePressure(2*st_params_.prismatic+segment*3, 0);
    vc_->setSinglePressure(2*st_params_.prismatic+segment*3+1, 0);
    vc_->setSinglePressure(2*st_params_.prismatic+segment*3+2, 0);

    p = p*100*dyn_.A_pseudo(st_params_.prismatic + segment*2, st_params_.prismatic + segment*2);

    p = p*100;
    Kqg = Kqg*100;


    log_file_ << "i,kqg0,kqg1,p0,p1,p2\n";
    for (int i = 0; i < rotation; i++){
        log_file_ << fmt::format("{},{},{},{},{},{}\n",i, Kqg(0,i), Kqg(1,i), p(0,i), p(1,i), p(2,i));
    }
    log_file_.close();

    MatrixXd A_top = p.transpose().bdcSvd(ComputeThinU | ComputeThinV).solve(Kqg.block(0,0,1,rotation).transpose()).transpose(); //least squares to determine A (x direction)
    MatrixXd A_bot = p.transpose().bdcSvd(ComputeThinU | ComputeThinV).solve(Kqg.block(1,0,1,rotation).transpose()).transpose(); // least squares to determine A (y direction)

    MatrixXd A = MatrixXd::Zero(2,3);
    A << A_top, A_bot;
    fmt::print("A for segment {}:\n {}\n",segment,A);
    for (int i = 0; i < 6; i++){
        new_params.chamberConfigs[6*segment+i] = A(i/3,i%3);
    }
}
