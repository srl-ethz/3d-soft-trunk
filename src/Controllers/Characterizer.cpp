#include "3d-soft-trunk/Controllers/Characterizer.h"

Characterize::Characterize(const SoftTrunkParameters st_params) : ControllerPCC(st_params){
    new_params = st_params_;
}

void Characterize::angularError(int segment, std::string fname){
    VectorXd pressures = VectorXd::Zero(st_params_.p_pseudo_size);
    
    filename_ = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, fname);
    fmt::print("Starting radial log to {}\n", fname);
    log_file_.open(fname, std::fstream::out);
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

    for (double i = 0; i < 360; i+=1.){
        pressures(2*(segment+st_params_.prismatic)+st_params_.prismatic*2) = 500*cos(i*deg2rad);
        pressures(2*(segment+st_params_.prismatic)+1+st_params_.prismatic*2) = -500*sin(i*deg2rad);

        actuate(mdl_->pseudo2real(pressures));        

        x_ = state_.tip_transforms[st_params_.num_segments+st_params_.prismatic].translation();
        double angle = atan2(x_(1),x_(0))*180/3.14156;
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

    std::string poly_location = fmt::format("{}/polynomial_{}.txt", SOFTTRUNK_PROJECT_DIR,segment);
    std::fstream polynomial_out;
    polynomial_out.open(poly_location, std::fstream::out);

    for(int p = 0; p < 3; p++){ 
        MatrixXd angval120 = angle_vals.block(0,0,120,4);
        VectorXd poly_coeffs = (angval120.transpose()*angval120).inverse()*angval120.transpose()*ang_err.segment(p*120,120); //calculate polynomial coeffs using least squares
        /** @todo log these values to yaml */
    }
}

void Characterize::stiffness(int segment, int directions, int verticalsteps, int maxpressure){
    assert(st_params_.model_type==ModelType::augmentedrigidarm);
    assert(segment < st_params_.num_segments); //sanity check

    VectorXd K = VectorXd::Zero(2*directions*verticalsteps);
    VectorXd tau = VectorXd::Zero(2*directions*verticalsteps);
    
    double angle = 0;
    VectorXd pressures = VectorXd::Zero(st_params_.p_pseudo_size);
    fmt::print("Starting stiffness coefficient characterization in {} directions for segment {}\n", directions,segment);

    for (int i = 0; i < directions; i ++){
        angle = i*360/directions; 

        for (int j = 0; j < verticalsteps; j++){   //iterate through multiple heights for the respective angle
            pressures(2*segment+st_params_.prismatic) = (maxpressure/verticalsteps+maxpressure*j/verticalsteps)*cos(angle*deg2rad);
            pressures(2*segment+st_params_.prismatic+1) = -(maxpressure/verticalsteps+maxpressure*j/verticalsteps)*sin(angle*deg2rad);

            actuate(mdl_->pseudo2real(pressures));

            srl::sleep(10); //wait to let swinging subside

            tau(2*verticalsteps*i + 2*j) = pressures(2*segment+st_params_.prismatic) - (dyn_.A_pseudo.inverse()*dyn_.g/100)(2*segment+st_params_.prismatic);
            tau(2*verticalsteps*i + 2*j + 1) = pressures(2*segment+st_params_.prismatic+1) - (dyn_.A_pseudo.inverse()*dyn_.g/100)(2*segment+st_params_.prismatic+1);

            K(2*verticalsteps*i + 2*j) = (dyn_.A_pseudo.inverse()*dyn_.K*state_.q/100)(2*segment+st_params_.prismatic);
            K(2*verticalsteps*i + 2*j + 1) = (dyn_.A_pseudo.inverse()*dyn_.K*state_.q/100)(2*segment+st_params_.prismatic + 1);
            fmt::print("q: {}\n",state_.q.transpose());
        }
    }

    VectorXd Kcoeff = (K.transpose()*K).inverse()*K.transpose()*tau;
    fmt::print("Finished coeffient characterization. Best fit is {}\n", Kcoeff(0)*st_params_.shear_modulus[segment]);

    new_params.shear_modulus[segment] = st_params_.shear_modulus[segment]*Kcoeff(0);
}

bool Characterize::valveMap(int maxpressure){
    std::vector<int> newMap(st_params_.p_size);

    for (int i = 0; i < st_params_.p_size - 2*st_params_.prismatic; i++){
        vc_->setSinglePressure(i,300);
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
            newMap[st_params_.p_size - 1] = st_params_.valvemap[i];
        } else { //otherwise, determine which chamber it is
            fmt::print("Segment {}, direction: ", segment);
            double angle = atan2(state_.q(2*segment+st_params_.prismatic+1),
                state_.q(2*segment+st_params_.prismatic))*180/3.14156; //angle of bending
            
            if (angle > 120 or angle <= -120){
                fmt::print("1\n");
                newMap[3*segment+2*st_params_.prismatic] = st_params_.valvemap[i];
            }
            else if (angle > 0 and angle <= 120){
                fmt::print("2\n");
                newMap[3*segment+1+2*st_params_.prismatic] = st_params_.valvemap[i];
            }
            else if (angle <= 0 and angle > -120){
                fmt::print("3\n");
                newMap[3*segment+2+2*st_params_.prismatic] = st_params_.valvemap[i];
            }
            else return false;
        }

        vc_->setSinglePressure(i,0);
    }
    
    new_params.valvemap = newMap;
    fmt::print("New map: ");
    for (int i = 0; i < newMap.size(); i++){
        fmt::print("{} ", newMap[i]);
    }
    fmt::print("\n");

    //remove this in favor of yaml vomiter
    /*
    std::string file = fmt::format("{}/config/{}",SOFTTRUNK_PROJECT_DIR,this->yaml_name_);
    YAML::Node params = YAML::LoadFile(file);
    std::ofstream fout(file);
    params["valveMap"] = newMap;
    params["valveMap"].SetStyle(YAML::EmitterStyle::Flow);
    fout << params;
    return true;
    */
}
