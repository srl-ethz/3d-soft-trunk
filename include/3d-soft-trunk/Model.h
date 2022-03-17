#pragma once

#include "3d-soft-trunk/SoftTrunk_common.h"
#include "3d-soft-trunk/Models/SoftTrunkModel.h"
#include "3d-soft-trunk/Models/Lagrange.h"

/** @brief The model object's purpose is determining matrices of the dynamic equation. It does NOT estimate state, it uses state to estimate inertia, gravity etc.
 * @details The model object acts as a funnel for all possible models. Currently, choices are an Augmented Rigid Arm (variable segments) or Lagrangian Energy (2 segment hardcoded)
*/
class Model{
public:
    Model(const SoftTrunkParameters& st_params);

    ~Model();

    /** @brief Forces model object to update dynamic parameters
     * @param state Configuration of the arm for which dynamic parameters should be obtained*/
    void update(const srl::State& state);

    /** @brief Converts x,y pseudopressures to "real" pressures which can be sent to the chambers */
    VectorXd pseudo2real(VectorXd p_pseudo);

    srl::State state_;
    DynamicParams dyn_;
    std::vector<MatrixXd> chamber_config_;

    const SoftTrunkParameters st_params_;

private:
    std::unique_ptr<Lagrange> lag_;
    std::unique_ptr<SoftTrunkModel> stm_;

    MatrixXd chamber_inv_;

    std::mutex mtx;
};