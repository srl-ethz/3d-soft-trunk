//created by oliver 3.10.21

#include "3d-soft-trunk/Model.h"


Model::Model(const SoftTrunkParameters& st_params) : st_params_(st_params){
    switch (st_params_.model_type) {
        case ModelType::augmentedrigidarm: 
            stm_ = std::make_unique<SoftTrunkModel>(st_params_);  
            this->dyn_ = stm_->dyn_;   
            break;
        case ModelType::lagrange: 
            lag_ = std::make_unique<Lagrange>(st_params_);
            this->dyn_ = lag_->dyn_;
            break;
    }
    chamber_config_ << -1, 0.5, 0.5, 0, sqrt(3) / 2, -sqrt(3) / 2; //default "correct" configuration
    
    state_ = st_params_.getBlankState();
    fmt::print("Model initialized at {}Hz with {} model.\n",st_params_.model_update_rate,st_params_.model_type);
}

Model::~Model(){
}

void Model::update(const srl::State& state){
    switch (st_params_.model_type){
            case ModelType::augmentedrigidarm: 
                stm_->set_state(state);
                this->dyn_ = stm_->dyn_;
                assert (st_params_.coord_type == dyn_.coordtype);
                break;
            case ModelType::lagrange:
                lag_->set_state(state);
                this->dyn_ = lag_->dyn_;
                assert (st_params_.coord_type == CoordType::phitheta);
                assert (st_params_.num_segments == 2);    //lagrange is hardcoded for a 2seg phitheta robot
                break;
        }
}

VectorXd Model::pseudo2real(VectorXd p_pseudo){
    assert(p_pseudo.size() == st_params_.p_pseudo_size);
    VectorXd output = VectorXd::Zero(st_params_.p_size);
    MatrixXd inverter = MatrixXd::Zero(2,2);
    VectorXd truePressure = VectorXd::Zero(2);

    for (int i = 0; i < st_params_.num_segments; i++){
        double angle = atan2(p_pseudo(2*i+st_params_.prismatic+1), p_pseudo(2*i+st_params_.prismatic))*180/3.14156; //determine direction the pressure wants to actuate in

        if (angle < -60){ //now, based on angle determine which chamber receives zero pressure and which must be calculated
            inverter.col(0) = chamber_config_.col(0);
            inverter.col(1) = chamber_config_.col(2);
            truePressure = inverter.inverse()*p_pseudo.segment(2*i+st_params_.prismatic,2);
            output.segment(3*i+st_params_.prismatic,3) << truePressure(0), 0, truePressure(1);
        } else if (angle > -60 and angle < 60){ //depends on angle
            inverter.col(0) = chamber_config_.col(1);
            inverter.col(1) = chamber_config_.col(2);
            truePressure = inverter.inverse()*p_pseudo.segment(2*i+st_params_.prismatic,2);
            output.segment(3*i+st_params_.prismatic,3) << 0, truePressure(0), truePressure(1);
        } else if (angle > 60){
            inverter.col(0) = chamber_config_.col(0);
            inverter.col(1) = chamber_config_.col(1);
            truePressure = inverter.inverse()*p_pseudo.segment(2*i+st_params_.prismatic,2);
            output.segment(3*i+st_params_.prismatic,3) << truePressure(0), truePressure(1), 0;
        } else {
            fmt::print("Actuation angle out of bounds. This should never trigger.\n");
            assert(false);
        }

        if (st_params_.prismatic){
            output(0) = p_pseudo(0);
        }

    }
    return output;
}
