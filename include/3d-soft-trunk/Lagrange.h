#pragma once

#include "SoftTrunk_common.h"
#include <cstddef>
#include <cstdlib>

class Lagrange{
public:
    /** @brief inertia matrix mapped to q */
    MatrixXd B;
    /** @brief coreolis & centrifugal forces mapped to q */
    VectorXd c;
    /** @brief gravity mapped to q */
    MatrixXd g;
    /** @brief Jacobian of tip position vs q */
    std::vector<Eigen::MatrixXd> J;
    /* @brief Coriolis factorization matrix mapped to q */
    MatrixXd S;
private:


    void A_update(const double in1[4], MatrixXd& A);
    void c_update(const double in1[4], const double in2[4], const double in3[2], const double in4[2], VectorXd& c);
    void d_update(const double in1[2], const double in2[4], const double in3[4], double D[4]);
    void g_update(const double in1[4], const double in2[2], const double in3[2], double g0, double G[4]);
    void J_update(const double in1[4], const double in2[2], double J[12]);
    void JDot_update(const double in1[4], const double in2[4], const double in3[2], double JDot[12]);
    void k_update(const double in1[2], const double in2[4], double K[4]);
    void B_update(const double in1[4], const double in2[2], const double in3[2], double M[16]);
    void p_update(const double in1[4], const double in2[2], double p_FK[3]);

    const double g0 = 9.80665; //earth's gravity constant

}

