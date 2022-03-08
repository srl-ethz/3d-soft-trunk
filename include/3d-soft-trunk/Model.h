#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/Models/SoftTrunkModel.h"
#include "3d-soft-trunk/Models/Lagrange.h"
//#include "3d-soft-trunk/Model/Lagrange.h"

class Model{
public:
    /** @brief model constructor */
    Model(const SoftTrunkParameters& st_params);

    ~Model();

    /** @brief forces model to grab latest dynamic parameters from submodules */
    void update(const srl::State& state);

    /** @brief forward simulate using the model
     * @param state the state which will be forward simulated
     * @param p pressure vector which will be input to the vector during the forward simulation
     * @param dt timespan which will be simulated for the constant pressure input */
    bool simulate(srl::State& state, const VectorXd& p, double dt);

    /** @brief takes x,y pseudopressures and transforms them to real pressures */
    VectorXd pseudo2real(VectorXd p_pseudo);

    /** @brief returns a pressure vector which compensates for gravity, coriolis, damping and stiffness */
    VectorXd gravity_compensate(const srl::State state);

    /** @brief state of model */
    srl::State state_;

    /** @brief dynamic parameters of model */
    DynamicParams dyn_;

    MatrixXd chamber_config_ = MatrixXd::Zero(2,3);

    const SoftTrunkParameters st_params_;

private:
    std::unique_ptr<Lagrange> lag_;
    std::unique_ptr<SoftTrunkModel> stm_;

    MatrixXd chamber_inv_;

    std::mutex mtx;
};