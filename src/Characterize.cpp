#include "3d-soft-trunk/Characterize.h"
#include "time.h"

Characterize::Characterize(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type) : ControllerPCC(st_params, sensor_type, 1){
}

void Characterize::logRadialPressureDist(int segment, std::string fname){
    VectorXd pressures = VectorXd::Zero(2 * st_params.num_segments+1);
    filename = fname;

    filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
    fmt::print("Starting radial log to {}\n", filename);
    log_file.open(filename, std::fstream::out);
    log_file << "angle, angle_measured, q_1, q_2\n";

    pressures(2*segment+1) = 500;

    if(sensor_type == CurvatureCalculator::SensorType::simulator) simulate(pressures);
        else actuate(stm->pseudo2real(pressures));
    srl::sleep(5);
    srl::Rate r{2};

    VectorXd ang_err = VectorXd::Zero(360);
    VectorXd radii = VectorXd::Zero(360);
    MatrixXd angle_vals = MatrixXd::Zero(360,4);

    for (double i = 0; i < 360; i+=1.){
        pressures(2*segment+1) = 500*cos(i*deg2rad);
        pressures(2*segment+2) = -500*sin(i*deg2rad);

        if(sensor_type == CurvatureCalculator::SensorType::simulator) simulate(pressures);
        else actuate(stm->pseudo2real(pressures));
        

        cc->get_curvature(state);
        stm->updateState(state);
        x = stm->get_H_base().rotation()*cc->get_frame(1).rotation()*(cc->get_frame(st_params.num_segments+1).translation()-cc->get_frame(1).translation());

        double angle = atan2(x(1),x(0))*180/3.14156;
        if (angle < 0) angle+=360;

        fmt::print("angle: {}, angle_measured: {} q_1: {}, q_2: {} \n", (double) i, angle, state.q(1), state.q(2));

        log_file << fmt::format("{},{},{},{}", (double) i, angle, state.q(1), state.q(2));
        ang_err(i) = i - angle;
        if (abs(ang_err(i)) > 180) ang_err(i) -= ((ang_err(i) > 0) - (ang_err(i) < 0))*360;
        angle_vals(i,0) = i*i*i;
        angle_vals(i,1) = i*i;
        angle_vals(i,2) = i;
        angle_vals(i,3) = 1;
        radii(i) = sqrt(x(0)*x(0)+x(1)*x(1));

        log_file << "\n";
        r.sleep();
    }
    log_file.close();
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
    VectorXd pressures = VectorXd::Zero(2*st_params.num_segments+1);
    fmt::print("Starting coefficient characterization in {} directions\n", directions);
    for (int i = 0; i < directions; i ++){
        angle = 45+i*360/directions; 

        for (int j = 0; j < verticalsteps; j++){                                //iterate through multiple heights for the respective angle
            pressures(2*segment+1) = (500/verticalsteps+500*j/verticalsteps)*cos(angle*deg2rad);
            pressures(2*segment+2) = -(500/verticalsteps+500*j/verticalsteps)*sin(angle*deg2rad);
            if(sensor_type == CurvatureCalculator::SensorType::simulator) simulate(pressures);
                else actuate(stm->pseudo2real(pressures));
            fmt::print("angle = {}, intensity = {}\n", angle, 500/verticalsteps+500*j/verticalsteps);

            srl::sleep(10); //wait to let swinging subside
            cc->get_curvature(state);
            stm->updateState(state);

            tau(2*verticalsteps*i + 2*j) = pressures(2*segment+1) - (stm->A_pseudo.inverse()*stm->g/100)(2*segment+1);
            tau(2*verticalsteps*i + 2*j + 1) = pressures(2*segment+2) - (stm->A_pseudo.inverse()*stm->g/100)(2*segment+2);

            K(2*verticalsteps*i + 2*j) = (stm->A_pseudo.inverse()*stm->K*state.q/100)(2*segment+1);
            K(2*verticalsteps*i + 2*j + 1) = (stm->A_pseudo.inverse()*stm->K*state.q/100)(2*segment + 2);

        }
    }

    VectorXd Kcoeff = (K.transpose()*K).inverse()*K.transpose()*tau;
    
    fmt::print("Finished coeffient characterization. Best fit is g + {}*K*q\n\n", Kcoeff(0));
}

void Characterize::TaskSpaceAnalysis(int points_per_height){
    set_log_filename("prismatic_taskspace");
    srand(time(NULL)); //initialize randomness
    toggle_log();
    VectorXd p_ps = VectorXd::Zero(2*st_params.num_segments+1);
    fmt::print("Starting task space analysis with {} points per height\n",points_per_height);
    for (int h = 0; h < 2001; h+=200){ //go through the different prismatic heights
        fmt::print("Height: {}\n",h);
        p_ps(0) = h;

        for(int pi = 0; pi < points_per_height; pi++){ //go to pi different positions
            fmt::print("Point: {} ",pi);
            VectorXd p_prev = p_ps;

            
            for(int i = 0; i < st_params.num_segments; i++){ //generate random x,y coords
                p_ps(2+i*2) = (rand()%1400)-700;
            }
            
           
           p_ps(2) = 700 - 1400*(pi==2 or pi==3 or pi==6);
           p_ps(4) = 700 - 1400*(pi==2 or pi==5 or pi==6);
           p_ps = p_ps * (pi!=0) ;
            p_ps(0) = h;
            VectorXd delta = p_ps - p_prev;

            double traversal_time = 0;
            if (abs(delta.minCoeff()) > abs(delta.maxCoeff())){ //determine time required to traverse
                traversal_time = abs(delta.minCoeff()) / 100;
            } else {
                traversal_time = abs(delta.maxCoeff()) / 100;
            }
            fmt::print("Traversal Time: {} \n",traversal_time);

            double time = 0;
            int counter = 0;

            while (time < traversal_time){                  //traverse to new point, normed to 100mbar/s max
                cc->get_curvature(state);
                p = stm->pseudo2real(p_prev+(time/traversal_time)*delta);
                for(int i = 0; i < p.size(); i++){
                    vc->setSinglePressure(i+1,p(i));
                }
                time += 1./50;
                counter++;
                if(!(counter%50)) log(time);
                srl::sleep(1./50);
            }

        }
    }
    toggle_log();
}