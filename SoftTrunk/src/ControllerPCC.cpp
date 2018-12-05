//
// Created by yasu and rkk on 26/10/18.
//

#include "ControllerPCC.h"

MiniPID ZieglerNichols(double Ku, double period){
    // https://en.wikipedia.org/wiki/Ziegler–Nichols_method
    double Kp = 0.33 * Ku;
    double Ki = Kp/ (period / 2.0) * CONTROL_PERIOD;
    double Kd = Kp * period/3 / CONTROL_PERIOD;
    return MiniPID(Kp, Ki, Kd);
}

ControllerPCC::ControllerPCC(AugmentedRigidArm* augmentedRigidArm, SoftArm* softArm) : ara(augmentedRigidArm), sa(softArm){
    // set up PID controllers
    if (USE_PID_CURVATURE_CONTROL){
        miniPIDs.push_back(MiniPID(700,0,0));// PID for phi. Z-N doesn't seem to work very well, so just doing P control...
        miniPIDs.push_back(ZieglerNichols(500, 0.6)); // PID for theta
        miniPIDs.push_back(MiniPID(0,0,0)); // PID for phi
        miniPIDs.push_back(MiniPID(0,0,0)); // PID for theta
        miniPIDs.push_back(MiniPID(0,0,0)); // PID for phi
        miniPIDs.push_back(MiniPID(0,0,0)); // PID for theta
    }
    else{
//        no PID controller necessary when not doing PID control
    }
}


void ControllerPCC::curvatureDynamicControl(const Vector2Nd &q_ref,
                                            const Vector2Nd &dq_ref,
                                            const Vector2Nd &ddq_ref,
                                            Vector2Nd *tau) {

    curvatureDynamicControl(sa->curvatureCalculator->q, sa->curvatureCalculator->dq, q_ref, dq_ref, ddq_ref, tau);
}


void ControllerPCC::updateBCG(const Vector2Nd &q, const Vector2Nd &dq) {
    ara->update(q, dq);
    B = ara->Jxi.transpose() * ara->B_xi * ara->Jxi;
    C = ara->Jxi.transpose() * ara->B_xi * ara->dJxi;
    G = ara->Jxi.transpose() * ara->G_xi;
}

void ControllerPCC::curvatureDynamicControl(
        const Vector2Nd &q_meas,
        const Vector2Nd &dq_meas,
        const Vector2Nd &q_ref,
        const Vector2Nd &dq_ref,
        const Vector2Nd &ddq_ref,
        Vector2Nd *tau) {
    if(USE_FEEDFORWARD_CONTROL)
        updateBCG(q_ref, dq_ref);
    else
        updateBCG(q_meas, dq_meas);

    *tau = sa->k*q_ref + sa->d* dq_ref+ G + C*dq_ref + B*ddq_ref;
}


void ControllerPCC::curvaturePIDControl(const Vector2Nd &q_ref, Vector2Nd *output) {
    for (int i = 0; i < 2 * NUM_ELEMENTS; ++i) {
        (*output)(i) = miniPIDs[i].getOutput(sa->curvatureCalculator->q(i), q_ref(i));
    }
}
