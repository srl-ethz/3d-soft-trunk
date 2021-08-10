#include <stdio.h>
#include <chrono>
#include <thread>
#include "3d-soft-trunk/Lagrange.h"
#include "3d-soft-trunk/CurvatureCalculator.h"
#include "mobilerack-interface/ValveController.h"
#include "3d-soft-trunk/SoftTrunkModel.h"
/**
 * demo to showcase how to use the SoftTrunkModel class, as well as create a custom arm configuration youself. 
 */
using namespace std;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
void avoid_singularity(srl::State &state)
{
    double r;
    double eps_custom = 0.0001;
    for (int idx = 0; idx < state.q.size(); idx++)
    {
        r = std::fmod(std::abs(state.q[idx]), 6.2831853071795862); //remainder of division by 2*pi
        state.q[idx] = (r < eps_custom) ? eps_custom : state.q[idx];
    }
}

//compute damped pesudo inverse with a variable damping
//Deo, A. S., & Walker, I. D. (1995). Overview of damped least-squares methods for inverse kinematics of robot manipulators. Journal of Intelligent and Robotic Systems, 14(1), 43-68.
//Flacco, Fabrizio, and Alessandro De Luca. "A reverse priority approach to multi-task control of redundant robots." 2014 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2014.
template <typename Derived1, typename Derived2>
void dampedPseudoInverse(const Eigen::MatrixBase<Derived1> &A, double e, double dampingFactor, Eigen::MatrixBase<Derived2> &Apinv, unsigned int computationOptions)
{
    int m = A.rows(), n = A.cols(), k = (m < n) ? m : n;
    JacobiSVD<typename MatrixBase<Derived1>::PlainObject> svd = A.jacobiSvd(computationOptions);
    const typename JacobiSVD<typename Derived1::PlainObject>::SingularValuesType &singularValues = svd.singularValues();
    MatrixXd sigmaDamped = MatrixXd::Zero(k, k);
    double damp = dampingFactor * dampingFactor;

    for (int idx = 0; idx < k; idx++)
    {
        if (singularValues(idx) >= e)
            damp = 0;
        else
            damp = (1 - ((singularValues(idx) / e) * (singularValues(idx) / e))) * dampingFactor * dampingFactor;
        sigmaDamped(idx, idx) = singularValues(idx) / ((singularValues(idx) * singularValues(idx)) + damp);
    }
    Apinv = svd.matrixV() * sigmaDamped * svd.matrixU().transpose(); // damped pseudoinverse
}

//return pesudo inverse computed in dampedPseudoInverse function
Eigen::MatrixXd computePinv(Eigen::MatrixXd j, double e, double lambda)
{

    Eigen::MatrixXd pJacobian(j.cols(), j.rows());
    dampedPseudoInverse(j, e, lambda, pJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    return pJacobian;
}

int main()
{
    SoftTrunkParameters st_params_l{};
    st_params_l.load_yaml("softtrunkparams_Lagrange.yaml");
    st_params_l.finalize(); // run sanity check and finalize parameters
    Lagrange lag(st_params_l);
    srl::State state = st_params_l.getBlankState();     // get blank state with appropriate size
    srl::State state_ref = st_params_l.getBlankState(); // get blank state with appropriate size
    srand((unsigned int)time(0));
    /*
    state.q = VectorXd::Random(4);
    state.dq = VectorXd::Random(4);
    VectorXd x_ref = VectorXd::Random(3);
    VectorXd dx_ref = VectorXd::Random(3);
    VectorXd ddx_ref = VectorXd::Random(3);
    */

    VectorXd x_ref = VectorXd::Zero(3);
    VectorXd dx_ref = VectorXd::Zero(3);
    VectorXd ddx_ref = VectorXd::Zero(3);
    state.q << -1.5473  ,  0.3544 ,  -1.5922 ,   0.6483;
    state.dq << 0.1313  , -0.6384 ,  -0.4817  ,  2.0370;
    x_ref << 0.119947078490299 ,  0.003563476061649 , -0.2;
    dx_ref << -0.001069042818495 ,  0.035984123547090      ,             0;
    ddx_ref <<  -0.010795237064127 , -0.000320712845548      ,             0;
    VectorXd a = VectorXd::Zero(11);
    VectorXd Ka = VectorXd::Zero(11);
    VectorXd Kp = VectorXd::Zero(3);
    VectorXd Kd = VectorXd::Zero(3);
    Ka << 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.;
    Kp << 1., 1., 1.;
    Kd << 0.1, 0.1, 0.1;
    a <<   -0.000982687429715,
  -0.003930749718861,
  -0.004156732757044,
  -0.008681972932283,
   0.000230906118091,
   0.000461812236183,
   0.012118394924136,
   0.000664379983305,
   0.023943867337249,
   0.000157480540086,
   0.005497786818221;
    double dt = 1. / 1000;
    double eps = 1e-3;
    double lambda = 0.5e-3;
    avoid_singularity(state);

    lag.update(state, state_ref);

    VectorXd x = lag.p;
    VectorXd dx = lag.J * state.dq;
    VectorXd ddx_d = ddx_ref + Kp.asDiagonal() * (x_ref - x) + Kd.asDiagonal() * (dx_ref - dx);
    MatrixXd J_inv = computePinv(lag.J, eps, lambda);
    //cout << "\n J_inv \n " << J_inv << "\n";
    state_ref.dq = J_inv * (dx_ref + Kp.asDiagonal() * (x_ref - x));
    state_ref.ddq = J_inv * (ddx_d - lag.JDot * state.dq); // + ((MatrixXd::Identity(st_params.q_size, st_params.q_size) - J_inv * J)) * (-kd * state.dq);
    lag.update(state, state_ref);

    cout << "Ytranspose:\n" << lag.Y.transpose() << "\n\n";
    cout << "dqr \n" << state_ref.dq << "\n";
    //cout << "\n dq \n" << state.dq;
    //cout << "\n dqr-dq: \n" << state_ref.dq - state.dq;
    VectorXd aDot = Ka.asDiagonal() * lag.Y.transpose() * (state_ref.dq - state.dq);
    cout << "\naDot:\n" << aDot.transpose() << "\n";
    a += dt * aDot;
    cout << "\na:\n" << a.transpose() << "\n";

    VectorXd tau = lag.A.inverse() * lag.Y * a;
    std::cout << "tau:\n" << tau << "\n";
    //VectorXd p = stm->pseudo2real(stm->A_pseudo.inverse() * tau / 100);
    //cout << "pressure:\n" << p << "\n";
/*
    std::cout << "q:\n"
              << state.q << std::endl;
    std::cout << "dq:\n"
              << state.dq << std::endl;
    std::cout << "ddq:\n"
              << state.ddq << std::endl;
    std::cout << "A:\n"
              << lag.A << std::endl;
    std::cout << "Cdq:\n"
              << lag.Cdq << std::endl;
    std::cout << "M:\n"
              << lag.M << std::endl;
    std::cout << "g:\n"
              << lag.g << std::endl;
    std::cout << "k:\n"
              << lag.k << std::endl;
    std::cout << "d:\n"
              << lag.d << std::endl;
    std::cout << "p:\n"
              << lag.p << std::endl;
    std::cout << "J:\n"
              << lag.J << std::endl;
    std::cout << "JDot:\n"
              << lag.JDot << std::endl;
    //std::cout << "par:\n" << st_params.masses << std::endl;
    std::cout << "Y:\n"
              << lag.Y << std::endl;
    //std::cout << X << std::endl;
    return 1;
    */
}
