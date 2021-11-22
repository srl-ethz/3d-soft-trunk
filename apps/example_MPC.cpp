#include <3d-soft-trunk/ControllerPCC.h>
#include "3d-soft-trunk/MPC.h"
#include <chrono>

int main(){

    SoftTrunkParameters st_params;
    st_params.finalize();
    ControllerPCC cpcc = ControllerPCC(st_params, CurvatureCalculator::SensorType::simulator);
    MPC mpc(st_params, CurvatureCalculator::SensorType::simulator);
    srl::State state = st_params.getBlankState();
    srl::State state_ref = st_params.getBlankState(); 
    VectorXd p = VectorXd::Zero(3*st_params.num_segments);
    double time = 5.0;
    const double dt = 0.01;

    for (int i = 0; i < st_params.num_segments; i++) {
        // set to have about the same curvature as a whole regardless of scale
        double rand = -2.093 / st_params.sections_per_segment / st_params.num_segments / 10;
        for (int j = 0; j < st_params.sections_per_segment; j++){
            state.q(2*i*st_params.sections_per_segment + 2*j + 1) = -rand;
            state.q(2*i*st_params.sections_per_segment + 2*j ) = rand * 0.022 / 0.19;
        }

    }

    std::unique_ptr<SoftTrunkModel> stm;
    std::unique_ptr<CurvatureCalculator> cc;


    //Vector3d q_ref, q_act; 
    //Vector3d dq_ref;

    double coef = 2 * 3.1415 / time;
    double r = 0.1;
    double tol = 0.1;
    double t = 0;
    bool flag = 0; // to have 1 disturbance, and one only
    double disturbance = 2.093 / st_params.sections_per_segment / st_params.num_segments / 10;

    int counter = 0;
    int new_counter;

    int avg = 20;

    VectorXd error_avg = VectorXd::Zero(avg); 


    mpc.set_state(state);
    const double hz = 1./dt;
    mpc.set_frequency(hz);

    auto start = std::chrono::steady_clock::now();

    while (t < time/2){
        
        state_ref.q << -0.440825, 0.373653, -0.0481017, -0.105963;
        mpc.set_ref(state_ref);

        mpc.get_state(state); 

        if (counter >= avg){
            counter = 0;
        }

        if (counter < avg){
            error_avg(counter) = (state_ref.q - state.q).norm(); 
            counter ++; 
        }

        std::cout << "Reference : "<< state_ref.q.transpose()<< "\nCurrent state : " << state.q.transpose() << std::endl;  
        // std::cout << "Error vector : " << error_avg.transpose() << std::endl;
        
        std::cout << "Error : " << error_avg.mean() << std::endl; 

        std::cout<< "time : " << t <<std::endl; 

        // if ( (state.q - state_ref.q).norm() < tol){
        //     std::cout << "Convergerged in " << t << std::endl;
        // }

        t += dt; 
    }

    counter = 0;
    error_avg = VectorXd::Zero(avg);

    while (t < time){

        state_ref.q << 0.0633816, 0.290002, 0.655952, -0.0951374;
        mpc.set_ref(state_ref);

        mpc.get_state(state); 

        if (counter >= avg){
            counter = 0;
        }

        if (counter < avg){
            error_avg(counter) = (state_ref.q - state.q).norm(); 
            counter ++; 
        }

        std::cout << "Reference : "<< state_ref.q.transpose()<< "\nCurrent state : " << state.q.transpose() << std::endl;  
        // std::cout << "Error vector : " << error_avg.transpose() << std::endl;

        std::cout << "Error : " << error_avg.mean() << std::endl;
        
        std::cout<< "time : " << t <<std::endl; 
        // if ( (state.q - state_ref.q).norm() < tol){
        //     std::cout << "Convergerged in " << t << std::endl;
        // }

        t += dt; 
    }

    std::cout << "Simulation end" << std::endl; 

}