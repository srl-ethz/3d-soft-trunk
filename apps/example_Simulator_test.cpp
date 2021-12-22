#include <3d-soft-trunk/ControllerPCC.h>
#include "3d-soft-trunk/QuasiStatic.h"
#include "3d-soft-trunk/PID.h"
#include "3d-soft-trunk/MPC_ts.h"
#include <chrono>

int main(){

    SoftTrunkParameters st_params;
    st_params.finalize();
    ControllerPCC cpcc = ControllerPCC(st_params, CurvatureCalculator::SensorType::simulator);
    QuasiStatic qs(st_params, CurvatureCalculator::SensorType::simulator);
    //PID pid(st_params, CurvatureCalculator::SensorType::simulator);
    //MPC_ts mpc2(st_params, CurvatureCalculator::SensorType::simulator);
    srl::State state = st_params.getBlankState();
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

    Vector3d x_ref, x_act, trajectory; 
    Vector3d dx_ref;
    Vector3d ddx_ref;

    std::unique_ptr<SoftTrunkModel> stm;
    std::unique_ptr<CurvatureCalculator> cc;

    double coef = 4 * 3.1415 / time;
    double r = 0.1;
    double tol = 0.05;
    double t = 0;
    bool flag = 0; // to have 1 disturbance, and one only
    double disturbance = 2.093 / st_params.sections_per_segment / st_params.num_segments / 10;

    int counter = 0;
    int new_counter = 0; 

    
    qs.set_state(state);
    const double hz = 1./dt;
    qs.set_frequency(hz);

    qs.toggle_log(); 

    auto start = std::chrono::steady_clock::now();

    while ( t < time){

        trajectory << r*cos(coef*t), r*sin(coef*t),-0.215;  // circular trajectory
        //trajectory << r*cos(coef*t), 0, -0.200;               // linear trajectory
        x_ref = trajectory; 

        qs.set_ref(x_ref); 

        qs.get_x(x_act); 

        if ( (x_ref - x_act).norm() < tol){
            t += dt;  
            //std::cout << trajectory.transpose() << std::endl;
            counter = 0;
        }

        // if ((time/2.1 < t ) & (t < time/2) & (!flag)){
        //     state.q(1) += disturbance;
        //     state.q(1) += disturbance; 
        //     std::cout << "Disturbance occurred" << std::endl;
        //     flag = 1; 
        // }

        if ( counter%10000 == 0){
            new_counter = counter/10000; 
            if (( new_counter > 9 ) & (new_counter % 5 == 0)){
                std::cout << "Number of iteration = " << new_counter << std::endl;
            }  
        }
        
        counter++;
            
    }


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulated " << time << "s of motion in " << elapsed.count() <<"s realtime using timestep " << dt << "\n";

    qs.toggle_log(); 

    //return 0;
}
