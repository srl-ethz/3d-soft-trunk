#include "3d-soft-trunk/Simulator.h"


Simulator::Simulator(SoftTrunkModel &stm, double control_step, int steps, const srl::State& initialState) : stm(stm), steps(steps), control_step(control_step), sim_step(control_step/steps), state(initialState){
    fmt::print("Simulator object created with steps: {}\tcontrol_step: {}\tsim_step:{}\n", steps, control_step, sim_step);
    t = 0;
    assert(steps>=1);
}



void Simulator::simulate(const VectorXd &p){
    for (int i=0; i < steps; i++){
        if (!Beeman(p)) throw std::runtime_error("simulated value has overflown");
    }

    if (logging){                                               //log once per control timestep
        log_file << t;
        for (int i=0; i < st_params::num_segments; i++){        //log tip pos
            VectorXd x_tip = stm.get_H(i).translation();
            log_file << fmt::format(", {}, {}, {}", x_tip(0), x_tip(1), x_tip(2));
        }
        for (int i=0; i < st_params::q_size; i++)               //log q
            log_file << fmt::format(", {}", state.q(i));
        log_file << "\n";
        
    }
}

void Simulator::get_state(srl::State& state){
    state = this->state;
}


bool Simulator::Beeman(const VectorXd &p){
    
    stm.updateState(state);
    state_prev.ddq = state.ddq;
    if(sim_step>0.00001){                                                                            //normal case, assume matrixes to be constant and compute with high resolution
        
        VectorXd b_inv_rest = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * state.q);      //set up constant terms to not constantly recalculate
        MatrixXd b_inv_d = -stm.B.inverse() * stm.D;
        VectorXd ddq_prev;

        for (int i=0; i < int (sim_step/0.00001); i++){                                              //forward integrate dq with high resolution
            ddq_prev = state.ddq;
            state.ddq = b_inv_rest + b_inv_d*state.dq;
            state.dq += 0.00001*(2*(2*state.ddq - ddq_prev) + 5*state.ddq - ddq_prev)/6;  
        }

    } else {                                                                                        //if incredibly high resolution is chosen, use that instead
        state.ddq = stm.B.inverse() * (stm.A * p - stm.c - stm.g - stm.K * state.q + stm.D * state.dq);
        state.dq += sim_step*(2*(2*state.ddq - state_prev.ddq) + 5*state.ddq - state_prev.ddq)/6;
    }

    state.q = state.q + state.dq*sim_step + (sim_step*sim_step*(4*state.ddq - state_prev.ddq) / 6);

    t+=sim_step;
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
