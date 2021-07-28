#include <stdio.h>
#include <chrono>
#include <thread>
#include "3d-soft-trunk/Lagrange.h"

/**
 * demo to showcase how to use the SoftTrunkModel class, as well as create a custom arm configuration youself. 
 */
int main(){
    SoftTrunkParameters st_params{};
    st_params.load_yaml("softtrunkparams_Lagrange.yaml");
    st_params.finalize();  // run sanity check and finalize parameters
    Lagrange lag = Lagrange(st_params);
    srl::State state = st_params.getBlankState();  // get blank state with appropriate size
    srand((unsigned int) time(0));
    state.q = VectorXd::Random(4);
    state.dq = VectorXd::Random(4);
    std::cout << "q:\n" << state.q << std::endl;
    std::cout << "dq:\n" << state.dq << std::endl;
    lag.update(state);
    std::cout << "A:\n" << lag.A << std::endl;
    std::cout << "Cdq:\n" << lag.Cdq << std::endl;
    std::cout << "M:\n" << lag.M << std::endl;
    std::cout << "g:\n" << lag.g << std::endl;
    std::cout << "k:\n" << lag.k << std::endl;
    std::cout << "d:\n" << lag.d << std::endl;
    std::cout << "p:\n" << lag.p << std::endl;
    std::cout << "J:\n" << lag.J << std::endl;
    std::cout << "JDot:\n" << lag.JDot << std::endl;
    //std::cout << "par:\n" << st_params.masses << std::endl;
    /*
    */
    // print parameters of model
    //fmt::print("B:{}\nc:{}\ng:{}\nK:{}\nD:{}\nA:{}\nA_pseudo:{}\nJ:{}\n S:{}\n", stm.B, stm.c, stm.g, stm.K, stm.D, stm.A, stm.A_pseudo, stm.J[st_params.num_segments-1], stm.S);
    //fmt::print("A:{}\n", lag.A);
    //converting between pseudopressure to real pressure
    //VectorXd v = VectorXd::Zero(2*st_params.num_segments);
    //v(0) = 200;

    return 1;
}
