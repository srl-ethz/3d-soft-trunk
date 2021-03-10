#include "3d-soft-trunk/Simulator.h"


Simulator::Simulator(SoftTrunkModel &stm, Simulator::SimType sim_type, const double &dt, const int &steps) : stm(stm), sim_type(sim_type), steps(steps), dt(dt), simtime(dt/steps){
    t = 0;
    assert(steps>=1);
}



bool Simulator::simulate(const VectorXd &p, srl::State &state){

    

    if (logging){
        log_file << t;
        for (int i=0; i < st_params::num_segments; i++){        //log tip pos
            VectorXd x_tip = stm.get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        for (int i=0; i < st_params::q_size; i++)               //log q
            log_file << fmt::format(", {}", state.q(i));
        log_file << "\n";
    }

    for (int i=0; i < steps; i++){
        if (sim_type == Simulator::SimType::euler) {
            if (!Euler(p, state)) return false;
        } else if (sim_type == Simulator::SimType::beeman) {
            if(!Beeman(p, state)) return false;
        }
    }

    return true;

};




bool Simulator::Euler(const VectorXd &p, stl::State &state){
    stm.updateState(state);

    state.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * state.q  -stm.D * state.dq); //get ddq

    state_mid.dq = state.dq + state.ddq * simtime / 2; //half step forward approximation
    state_mid.q = state.q + state.dq * simtime / 2;

    stm.updateState(state_mid);
    state_mid.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * state_mid.q  -stm.D * state_mid.dq); 
       
    state.dq = state.dq + state_mid.dq * simtime;
    state.q = state.q + state_mid.dq * simtime;

    t+=simtime;
    return !(abs(state.ddq[0])>pow(10.0,10.0) or abs(state.dq[0])>pow(10.0,10.0) or abs(state.q[0])>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing
}


bool Simulator::Beeman(const VectorXd &p, srl::State &state){
    stm.updateState(state);

    state.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * state.q  -stm.D * state.dq);

    state.q = state.q + state.dq*simtime + (simtime*simtime*(4*state.ddq - state_prev.ddq) / 6);
    state.dq = state.dq + simtime*(2*(2*state.ddq - state_prev.ddq) + 5*state.ddq - state_prev.ddq)/6;  //2*ddq(n) - ddq(n-1) is approximately ddq(n+1)

    state_prev.ddq = state.ddq;

    t+=simtime;
    return !(abs(state.ddq[0])>pow(10.0,10.0) or abs(state.dq[0])>pow(10.0,10.0) or abs(state.q[0])>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing
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
