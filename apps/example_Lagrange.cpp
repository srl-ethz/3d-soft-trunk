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
int main()
{
    /*
    SoftTrunkParameters st_params_l{};
    st_params_l.load_yaml("softtrunkparams_Lagrange.yaml");
    st_params_l.finalize();  // run sanity check and finalize parameters

    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_example.yaml");
    st_params.finalize();

    CurvatureCalculator cc(st_params, CurvatureCalculator::SensorType::qualisys);
    SoftTrunkModel stm(st_params);

    std::vector<int> map = {1,2,5,3,6,4,0};
    ValveController vc("192.168.0.100", map, 600);
    

    Lagrange lag(st_params_l);
    srl::State state = st_params.getBlankState();  // get blank state with appropriate size

    VectorXd tau = VectorXd::Zero(4);
    VectorXd p = VectorXd::Zero(6);
    double t = 0.2;
    while(true){

        for (int i = 0; i < 3; i++){
            p(i) = sinusoid()
        }

        for (int i = 0; i < st_params.num_segments*3; i++){
            vc.setSinglePressure(i, p(i));
        }
        srl::sleep(0.2);
        t += 0.2;
    }
*/
    SoftTrunkParameters st_params_l{};
    st_params_l.load_yaml("softtrunkparams_Lagrange.yaml");
    st_params_l.finalize(); // run sanity check and finalize parameters
    Lagrange lag(st_params_l);
    srl::State state = st_params_l.getBlankState(); // get blank state with appropriate size
    srl::State state_r = st_params_l.getBlankState(); // get blank state with appropriate size
    srand((unsigned int)time(0));
    state.q = VectorXd::Random(4);
    state.dq = VectorXd::Random(4);
    state.ddq = VectorXd::Random(4);
    state_r.q = VectorXd::Random(4);
    state_r.dq = VectorXd::Random(4);
    state_r.ddq = VectorXd::Random(4);
    std::cout << "q:\n"
              << state.q << std::endl;
    avoid_singularity(state);
    lag.update(state,state_r);
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
}
