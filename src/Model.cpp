//created by oliver 3.10.21

#include "3d-soft-trunk/Model.h"


Model::Model(const SoftTrunkParameters& st_params) : st_params_(st_params){
    //determine which model is being used
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

    //read in the chamber configurations
    chamber_config_.resize(st_params_.num_segments);
    for (int i = 0; i < st_params_.num_segments; i++){
        chamber_config_[i] = MatrixXd::Zero(2,3);
        chamber_config_[i] << st_params_.chamberConfigs[i*6], st_params_.chamberConfigs[i*6+1], st_params_.chamberConfigs[i*6+2], 
            st_params_.chamberConfigs[i*6+3], st_params_.chamberConfigs[i*6+4], st_params_.chamberConfigs[i*6+5];
    }

    state_ = st_params_.getBlankState();
    fmt::print("Model initialized at {}Hz.\n",st_params_.model_update_rate);
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
    VectorXd truePressure = VectorXd::Zero(2);

    /** @todo add the prismatic pseudo2real */

    if(st_params_.prismatic){
        output(0) = 0;
        output(1) = 1;
    }

    for (int i = 0; i < st_params_.num_segments; i++){

        double angle = atan2(p_pseudo(2*i+st_params_.prismatic+1), p_pseudo(2*i+st_params_.prismatic))*180/3.14156; //determine direction the pressure wants to actuate in

        if (p_pseudo.segment(2*i+st_params_.prismatic,2).norm() > st_params_.p_max){ //constrain the pressure to max allowed value
            p_pseudo.segment(2*i+st_params_.prismatic,2) = st_params_.p_max * p_pseudo.segment(2*i+st_params_.prismatic,2).normalized();
        }
        MatrixXd inverter = MatrixXd::Zero(2,2);

        if (angle < -60){ //now, based on angle we know which chamber receives zero pressure and which must be calculated
            inverter.col(0) = chamber_config_[i].col(0);
            inverter.col(1) = chamber_config_[i].col(2); //read in the correct chamberes
            truePressure = inverter.inverse()*p_pseudo.segment(2*i+st_params_.prismatic,2); //invert to obtain desired pressure
            output.segment(3*i+2*st_params_.prismatic,3) << truePressure(0), 0, truePressure(1); //map to output
        } else if (angle > -60 and angle < 60){ //depends on angle
            inverter.col(0) = chamber_config_[i].col(1);
            inverter.col(1) = chamber_config_[i].col(2);
            truePressure = inverter.inverse()*p_pseudo.segment(2*i+st_params_.prismatic,2);
            output.segment(3*i+2*st_params_.prismatic,3) << 0, truePressure(0), truePressure(1);
        } else if (angle > 60){
            inverter.col(0) = chamber_config_[i].col(0);
            inverter.col(1) = chamber_config_[i].col(1);
            truePressure = inverter.inverse()*p_pseudo.segment(2*i+st_params_.prismatic,2);
            output.segment(3*i+2*st_params_.prismatic,3) << truePressure(0), truePressure(1), 0;
        } else {
            fmt::print("Actuation angle out of bounds. This should never trigger.\n");
            assert(false);
        }
    }
    return output;
}
