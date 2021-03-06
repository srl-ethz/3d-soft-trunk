#pragma once

#include "3d-soft-trunk/ControllerPCC.h"

/** @brief State Space LQR Controller */
class LQR: public ControllerPCC
{
public:
    LQR(const SoftTrunkParameters st_params);

    /** @brief realinearize the LQR controller, takes quite a while */
    void relinearize();

    /** @brief set the LQR cost matrices */
    void setcosts(MatrixXd &Q, MatrixXd &R);

private:
    void control_loop();
    /** @brief solve the riccati equation for given matrixes, stolen from https://github.com/TakaHoribe/Riccati_Solver
    *   @details Overview of LQR/Riccati is here: https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Infinite-horizon,_continuous-time_LQR
    */
    void solveRiccati(const MatrixXd &A, const MatrixXd &B, const MatrixXd &Q, const MatrixXd &R, Eigen::MatrixXd &K);

    /** @brief due to LQR formulation we need to combine states, fullstate = [state.dq, state.q] */
    VectorXd fullstate;
    VectorXd fullstate_ref;

    MatrixXd K;     /* gain matrix for LQR */
    VectorXd u0;    /* input offset for  */
    MatrixXd A;     /* linearized system dynamics  */
    MatrixXd B;     /* actuation matrix */
    MatrixXd R;     /* actuation cost */
    MatrixXd Q;     /* state deviation cost */

};