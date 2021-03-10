#include "3d-soft-trunk/Simulator.h"


Simulator::Simulator(SoftTrunkModel &softtrunk, Pose &pose_passed, Simulator::SimType sim_type) : stm(softtrunk), pose(pose_passed), sim_type(sim_type){
    pose_prev = pose;
    t = 0;
}



bool Simulator::simulate(const VectorXd &p, const double &dt, const int &steps){
    assert(steps>=1);
    double simtime = dt/steps;

    for (int i=0; i < steps; i++){
        if (sim_type == Simulator::SimType::euler) {
            if(!Euler(p, simtime)) {
                std::cout << "Simulator has crashed, terminating...\n";
                return false;
            };
        } else if (sim_type == Simulator::SimType::beeman) {
            if(!Beeman(p, simtime)) {
                std::cout << "Simulator has crashed, terminating...\n";
                return false;
            };
        } else {
            std::cout << "No valid sim type specified! Check your Simulator constructor\n";
            return false;
        }
    if (logging){
        log_file << t;
        for (int i=0; i < st_params::num_segments; i++){
            VectorXd x_tip = stm.get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        log_file << "\n";
    }

    }

    return true;

};




bool Simulator::Euler(const VectorXd &p, const double &simtime){
    stm.updateState(pose);

    pose.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * pose.q  -stm.D * pose.dq); //get ddq

    pose_mid.dq = pose.dq + pose.ddq * simtime / 2; //half step forward approximation
    pose_mid.q = pose.q + pose.dq * simtime / 2;

    stm.updateState(pose_mid);
    pose_mid.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * pose_mid.q  -stm.D * pose_mid.dq); 
       
    pose.dq = pose.dq + pose_mid.dq * simtime;
    pose.q = pose.q + pose_mid.dq * simtime;

    t+=simtime;
    return !(pose.ddq[0]>pow(10.0,10.0) or pose.dq[0]>pow(10.0,10.0) or pose.q[0]>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing
}


bool Simulator::Beeman(const VectorXd &p, const double &simtime){
    stm.updateState(pose);

    pose.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * pose.q  -stm.D * pose.dq);

    pose.q = pose.q + pose.dq*simtime + (simtime*simtime*(4*pose.ddq - pose_prev.ddq) / 6);
    pose.dq = pose.dq + simtime*(2*(2*pose.ddq - pose_prev.ddq) + 5*pose.ddq - pose_prev.ddq)/6;  //2*ddq(n) - ddq(n-1) is approximately ddq(n+1)

    pose_prev.ddq = pose.ddq;

    t+=simtime;
    return !(pose.ddq[0]>pow(10.0,10.0) or pose.dq[0]>pow(10.0,10.0) or pose.q[0]>pow(10.0,10.0));
}


void Simulator::start_log(std::string filename){

    this->filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
    fmt::print("Starting log to {}, timestamp: {}s: \n", this->filename, this->t);

    log_file.open(this->filename, std::fstream::out);
    log_file << "timestamp";

    for (int i=0; i<st_params::num_segments; i++)
        log_file << fmt::format(", x_{}, y_{}, z_{}", i, i, i);

    log_file << "\n";
    logging = true;

    log_file << t; // log t = 0
        for (int i=0; i < st_params::num_segments; i++){
            VectorXd x_tip = stm.get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        log_file << "\n";
}

void Simulator::end_log(){
    fmt::print("Ending log to {}, timestamp: {}s\n", this->filename, this->t);
    logging = false;
    log_file.close();
}
