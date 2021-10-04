#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/Models/SoftTrunkModel.h"
//#include "3d-soft-trunk/Model/Lagrange.h"

class Model{
public:
    /** @brief model constructor 
     * @param update_frequency frequency at which the model self-updates */
    Model(const SoftTrunkParameters& st_params, int update_frequency);

    /** @brief updates given DynamicParameter dyn with new values from model */
    void get_dynamic_params(DynamicParams& dyn);

    /** @brief updates model's configuration and forces computation of new dynamic parameters */
    void set_state(const srl::State& state);

    /** @brief update handed state with the one stored in this class */
    void get_state(srl::State& state);

    /** @brief update vector with forward kinematics of all manipulator tips */
    bool get_x(std::vector<Vector3d>& x);

    /** @brief update vector with forward kinematics of specific tip position 
     * @param segment segment of which tip is desired, starts with 0 at base segment (thickest) */
    bool get_x(Vector3d &x, int segment);

    /** @brief forces model to grab latest dynamic parameters from submodules */
    void force_dyn_update();

    /** @brief forward simulate using the model
     * @param state the state which will be forward simulated
     * @param p pressure vector which will be input to the vector during the forward simulation
     * @param dt timespan which will be simulated for the constant pressure input */
    bool simulate(srl::State& state, const VectorXd& p, double dt);

    /** @brief loop which automatically updates model params */
    void update_loop();

private:
    srl::State state_;
    DynamicParams dyn_;
    std::vector<Vector3d> x_;

    SoftTrunkParameters st_params_;

    //std::unique_ptr<Lagrange> lag_;
    std::unique_ptr<SoftTrunkModel> stm_;

    std::thread update_thread;
    int update_frequency_;
};