#include <3d-soft-trunk/ControllerPCC.h>
#include "3d-soft-trunk/QuasiStatic.h"
#include "3d-soft-trunk/PID.h"
#include "3d-soft-trunk/MPC_ts.h"
#include <chrono>

int main(){

    bool logger = true; 

    SoftTrunkParameters st_params;
    st_params.finalize();
    ControllerPCC cpcc = ControllerPCC(st_params, CurvatureCalculator::SensorType::simulator);
    //QuasiStatic qs(st_params, CurvatureCalculator::SensorType::simulator);
    MPC_ts mpc2(st_params, CurvatureCalculator::SensorType::simulator);
    srl::State state = st_params.getBlankState();
    VectorXd p = VectorXd::Zero(3*st_params.num_segments);
    double time = 10.0;  //10 for 6 turns
    const double dt = 0.01;

    for (int i = 0; i < st_params.num_segments; i++) {
        // set to have about the same curvature as a whole regardless of scale
        double rand = -2.093 / st_params.sections_per_segment / st_params.num_segments / 10;
        for (int j = 0; j < st_params.sections_per_segment; j++){
            state.q(2*i*st_params.sections_per_segment + 2*j + 1) = -rand;
            state.q(2*i*st_params.sections_per_segment + 2*j ) = rand * 0.022 / 0.19;
        }

    }

    Vector3d x_act; 
    MatrixXd x_ref;
    //Vector3d trajectory; 
    MatrixXd trajectory = MatrixXd::Zero(3, mpc2.Horizon+1); 
    Vector3d dx_ref;
    Vector3d ddx_ref;

    std::unique_ptr<SoftTrunkModel> stm;
    std::unique_ptr<CurvatureCalculator> cc;

    double coef = 12 * 3.1415 / time;
    double r = 0.1;
    double tol = 0.2;
    double t = 0;
    bool flag = 0; // to have 1 disturbance, and one only
    double disturbance = 2.093 / st_params.sections_per_segment / st_params.num_segments / 10;

    int counter = 0;
    int new_counter = 0; 

    
    mpc2.set_state(state);
    const double hz = 1./dt;
    mpc2.set_frequency(hz);

    auto start = std::chrono::steady_clock::now();

    if (logger){
        mpc2.toggle_log();
    }
     

    while ( t < 1.1*time){

        //trajectory << r*cos(coef*t), r*sin(coef*t),-0.215;  // circular trajectory
        //trajectory << r*cos(coef*t), 0, -0.200;               // linear trajectory

        for (int i = 0; i< mpc2.Horizon +1; i++){
            trajectory(0,i) = r*cos(coef*(t+3*i*dt));
            trajectory(1,i) = r*sin(coef*(t+3*i*dt));
            trajectory(2,i) = -0.215;
        }

        // for (int i = 0; i<mpc2.Horizon +1; i++){
        //     if (t + 3*i*dt < time/4){
        //         //trajectory(0,i) = 0.08;
        //         trajectory(0,i) = 0.08*((t+3*i*dt) / (time/4)); // provide a slow approach 
        //         //trajectory(1,i) = -0.08 + 0.16*((t+3*i*dt) / (time/4)); 
        //         trajectory(1,i) = 0.08*((t+3*i*dt) / (time/4)); 
        //         trajectory(2,i) = -0.25; 
        //     }
        //     if ((time/4 < t + 3*i*dt) && (t + 3*i*dt < time/2)){
        //         trajectory(0,i) = 0.08 - 0.16*((t+3*i*dt-time/4) / (time/4));
        //         trajectory(1,i) =  0.08; 
        //         trajectory(2,i) = -0.25; 
        //     }
        //     if ((time/2 < t + 3*i*dt) && (t + 3*i*dt < 3*time/4)){
        //         trajectory(0,i) = - 0.08; 
        //         trajectory(1,i) = 0.08 - 0.16*((t+3*i*dt-time/2) / (time/4)); 
        //         trajectory(2,i) = -0.25; 
        //     }
        //     if (3*time/4 < t + 3*i*dt){
        //         trajectory(0,i) = -0.08 + 0.16*((t+3*i*dt-3*time/4) / (time/4));
        //         trajectory(1,i) = -0.08; 
        //         trajectory(2,i) = -0.25; 
        //     }
        // }

        // std::cout << "Future trajectory" << trajectory << std::endl; 

        // for (int i = 0; i< mpc2.Horizon +1; i++){
        //     trajectory(0,i) = r*cos(coef*(t+i*dt));
        //     trajectory(1,i) = 0;
        //     trajectory(2,i) = -0.250;
        // }

        x_ref = trajectory; 

        mpc2.set_ref(x_ref); 

        mpc2.get_x(x_act); 

        srl::sleep(0.03);  // 0.03 for circular 
        t+= dt; 


        // if ( (x_ref.col(0) - x_act).norm() < tol){   // wrong dimensions
        //     t += dt;  
        //     //std::cout << trajectory.transpose() << std::endl;
        //     counter = 0;
        // }

        // if ((time/2.1 < t ) & (t < time/2) & (!flag)){
        //     state.q(1) += disturbance;
        //     state.q(1) += disturbance; 
        //     std::cout << "Disturbance occurred" << std::endl;
        //     flag = 1; 
        // }

        if ( counter%10000 == 0){
            new_counter = counter/10000; 
            if (( new_counter > 9 ) & (new_counter % 5 == 0)){
                counter = 0; 
                std::cout << "Number of iteration = " << new_counter << std::endl;
            }  
        }
        
        counter++;
            
    }

    if (logger){
        mpc2.toggle_log();
    }


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulated " << time << "s of motion in " << elapsed.count() <<"s realtime using timestep " << dt << "\n";

    //return 0;
}