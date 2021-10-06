#include "3d-soft-trunk/Model.h"

Model::Model(const SoftTrunkParameters& st_params, int update_frequency) : st_params_(st_params), update_frequency_(update_frequency){
    switch (st_params_.model_type) {
        case ModelType::augmentedrigidarm: 
            stm_ = std::make_unique<SoftTrunkModel>(st_params_);     
            break;
        case ModelType::lagrange: 
            break;
    }

    update_thread = std::thread(&Model::update_loop, this);
    fmt::print("Model initialized at {}Hz with {} model.\n",update_frequency_,st_params_.model_type);
}

void Model::update_loop(){
    srl::Rate r{(double) update_frequency_};
    /*while (true){
        r.sleep();
        switch (st_params_.model_type){
            case ModelType::augmentedrigidarm: 
                stm_->set_state(state_);
                stm_->get_dynamic_params(dyn_);
                break;
        }
    }*/
}

void Model::get_dynamic_params(DynamicParams& dyn){
    dyn = this->dyn_;
}

bool Model::get_x(std::vector<Vector3d>& x){
    if(this->x_.size() <= 0){
        return false;
    }
    x = this->x_;
    return true;
}

bool Model::get_x(Vector3d& x, int segment){
    if (this->x_.size() > segment){
        return false;
    }
    x = this->x_[segment];
    return true;
}

void Model::set_state(const srl::State& state){
    switch (st_params_.model_type){
            case ModelType::augmentedrigidarm: 
                stm_->set_state(state);
                stm_->get_dynamic_params(dyn_);
                break;
        }
}

void Model::get_state(srl::State& state){
    state = this->state_;
}

bool Model::simulate(srl::State& state, const VectorXd &p, double dt){
    set_state(state);
    VectorXd p_adjusted = 100*p; //convert from mbar
    VectorXd ddq_init = state.ddq;
    VectorXd ddq_prev;

    VectorXd b_inv_rest = dyn_.B.inverse() * (dyn_.A * p_adjusted - dyn_.c - dyn_.g - dyn_.K * state.q);      //set up constant terms to not constantly recalculate
    MatrixXd b_inv_d = -dyn_.B.inverse() * dyn_.D; //dq will be changing for the loop so seperate
    

    for (int i=0; i < int (dt/0.00001); i++){                                              //forward integrate dq with very small steps
        ddq_prev = state.ddq;
        state.ddq = b_inv_rest + b_inv_d*state.dq;
        state.dq += 0.00001*(2*(2*state.ddq - ddq_prev) + 5*state.ddq - ddq_prev)/6;
        }
    state.q = state.q + state.dq*dt + (dt*dt*(4*state.ddq - ddq_init) / 6);

    return !(abs(state.ddq[0])>pow(10.0,10.0) or abs(state.dq[0])>pow(10.0,10.0) or abs(state.q[0])>pow(10.0,10.0)); //catches when the sim is crashing, true = all ok, false = crashing
}

void Model::force_dyn_update(){
    switch (st_params_.model_type){
            case ModelType::augmentedrigidarm: 
                stm_->get_dynamic_params(dyn_);
                break;
        }
}