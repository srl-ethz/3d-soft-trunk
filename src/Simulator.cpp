#include "3d-soft-trunk/Simulator.h"


Simulator::Simulator(SoftTrunkModel &stm, Simulator::SimType sim_type, const double &dt, const int &steps) : stm(stm), sim_type(sim_type), steps(steps), dt(dt), simtime(dt/steps){
    t = 0;
    assert(steps>=1);
}



bool Simulator::simulate(const VectorXd &p, Pose &pose){

    

    if (logging){
        log_file << t;
        for (int i=0; i < st_params::num_segments; i++){        //log tip pos
            VectorXd x_tip = stm.get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        for (int i=0; i < st_params::q_size; i++)               //log q
            log_file << fmt::format(", {}", pose.q(i));
        log_file << "\n";
    }

    for (int i=0; i < steps; i++){
        if (sim_type == Simulator::SimType::euler) {
            if (!Euler(p, pose)) return false;
        } else if (sim_type == Simulator::SimType::beeman) {
            if(!Beeman(p, pose)) return false;
        }
    }

    return true;

};




bool Simulator::Euler(const VectorXd &p, Pose &pose){
    stm.updateState(pose);

    pose.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * pose.q  -stm.D * pose.dq); //get ddq

    pose_mid.dq = pose.dq + pose.ddq * simtime / 2; //half step forward approximation
    pose_mid.q = pose.q + pose.dq * simtime / 2;

    stm.updateState(pose_mid);
    pose_mid.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * pose_mid.q  -stm.D * pose_mid.dq); 
       
    pose.dq = pose.dq + pose_mid.dq * simtime;
    pose.q = pose.q + pose_mid.dq * simtime;

    t+=simtime;
    return !(abs(pose.ddq[0])>pow(10.0,10.0) or abs(pose.dq[0])>pow(10.0,10.0) or abs(pose.q[0])>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing
}


bool Simulator::Beeman(const VectorXd &p, Pose &pose){
    stm.updateState(pose);

    pose.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * pose.q  -stm.D * pose.dq);

    pose.q = pose.q + pose.dq*simtime + (simtime*simtime*(4*pose.ddq - pose_prev.ddq) / 6);
    pose.dq = pose.dq + simtime*(2*(2*pose.ddq - pose_prev.ddq) + 5*pose.ddq - pose_prev.ddq)/6;  //2*ddq(n) - ddq(n-1) is approximately ddq(n+1)

    pose_prev.ddq = pose.ddq;

    t+=simtime;
    return !(abs(pose.ddq[0])>pow(10.0,10.0) or abs(pose.dq[0])>pow(10.0,10.0) or abs(pose.q[0])>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing
}


void Simulator::start_log(std::string filename){
    logging = true;
    this->filename = fmt::format("{}/{}.csv", SOFTTRUNK_PROJECT_DIR, filename);
    fmt::print("Starting log to {}, timestamp: {}s: \n", this->filename, this->t);

    log_file.open(this->filename, std::fstream::out);
    log_file << "timestamp";

    for (int i=0; i<st_params::num_segments; i++)               //write header
        log_file << fmt::format(", x_{}, y_{}, z_{}", i, i, i);
    for (int i=0; i < st_params::q_size; i++)
        log_file << fmt::format(", q_{}", i);

    log_file << "\n";
}

void Simulator::end_log(){
    fmt::print("Ending log to {}, timestamp: {}s\n", this->filename, this->t);
    logging = false;
    log_file.close();
}
