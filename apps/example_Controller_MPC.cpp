#include <3d-soft-trunk/ControllerPCC.h>
#include "3d-soft-trunk/MPC.h"
#include "3d-soft-trunk/MPC_ts.h"
#include "3d-soft-trunk/MPC_robust.h"
#include "3d-soft-trunk/MPC_obstacles.h"
#include <chrono>


MatrixXd x_ref(3,1);
Vector3d x;

void printer(MPC_obstacles& mpc){
    srl::Rate r{0.3};
    while(true){
        Vector3d x;
        mpc.get_x(x);
        fmt::print("------------------------------------\n");
        fmt::print("x tip: {}\n", x.transpose());
        fmt::print("x error: {}\n", (x_ref.col(0)-x).transpose());
        fmt::print("x error normalized: {}\n", (x_ref.col(0)-x).norm());
        r.sleep();
    }
}


int main(){
    SoftTrunkParameters st_params;
    st_params.finalize();
    MPC_obstacles mpc(st_params, CurvatureCalculator::SensorType::qualisys);
    VectorXd p = VectorXd::Zero(3*st_params.num_segments);
    srl::State state = st_params.getBlankState();

    //x_ref = x_ref_center;
    x_ref(0,0) = 0.02;
    x_ref(1,0) = 0;
    x_ref(2,0) = -0.26;
    //std::thread print_thread(printer, std::ref(mpc));

    
    double t = 0;
    double dt = 0.07;
    double time = 15;  // 25 for circle
    double coef = 4 * 3.1415 / time;
    double r = 0.1;
    //MatrixXd trajectory(3,1);
    MatrixXd trajectory = MatrixXd::Zero(3, mpc.Horizon);

    // mpc.set_ref(x_ref);
    srl::sleep(0.5);
    // arguments to pass by reference must be explicitly designated as so
    // https://en.cppreference.com/w/cpp/thread/thread/thread


    mpc.toggle_log();
    while (t < time){
        //x_ref = osc.get_objects()[0];

        for (int i = 0; i< mpc.Horizon; i++){
            trajectory(0,i) = r*cos(coef*(t+i*dt));
            trajectory(1,i) = r*sin(coef*(t+i*dt));
            trajectory(2,i) = -0.25;
        }

        // for (int i = 0; i<mpc.Horizon; i++){
        //     if (t + i*dt < time/4){
        //         //trajectory(0,i) = 0.08;
        //         trajectory(0,i) = 0.10*((t+i*dt) / (time/4)) - 0.02; // provide a slow approach 
        //         //trajectory(1,i) = -0.08 + 0.16*((t+3*i*dt) / (time/4)); 
        //         trajectory(1,i) = 0.10*((t+i*dt) / (time/4)) - 0.02; 
        //         trajectory(2,i) = -0.26; 
        //     }
        //     if ((time/4 < t + i*dt) && (t + i*dt < time/2)){
        //         trajectory(0,i) = 0.08 - 0.16*((t+i*dt-time/4) / (time/4));
        //         trajectory(1,i) =  0.08; 
        //         trajectory(2,i) = -0.26; 
        //     }
        //     if ((time/2 < t + i*dt) && (t + i*dt < 3*time/4)){
        //         trajectory(0,i) = - 0.08; 
        //         trajectory(1,i) = 0.08 - 0.16*((t+i*dt-time/2) / (time/4)); 
        //         trajectory(2,i) = -0.26; 
        //     }
        //     if (3*time/4 < t + i*dt){
        //         trajectory(0,i) = -0.08 + 0.16*((t+i*dt-3*time/4) / (time/4));
        //         trajectory(1,i) = -0.08; 
        //         trajectory(2,i) = -0.26; 
        //     }
        // }

        // trajectory(0,0) = 0.09; 
        // trajectory(1,0) = -0.01; 
        // trajectory(2,0) = -0.215; 
        // x_ref = trajectory;       // also for long trajectory would work

        mpc.set_ref(trajectory);
        /*osc.get_x(x);
        if ((x_ref - x).norm() < 0.07){
            freedom = true;
            osc.toggleGripper();
        }*/
        
        t+=dt;
        srl::sleep(dt);
        std::cout << t << std::endl;
    }

    x_ref(0,0) = 0.02;
    x_ref(1,0) = 0;
    x_ref(2,0) = -0.26;
    
    // mpc.set_ref(x_ref);
    // srl::sleep(1);
    mpc.toggle_log();
    srl::sleep(1);
    return 1;
}

// For joint control
// int main(){
//     SoftTrunkParameters st_params;
//     st_params.finalize();
//     MPC mpc(st_params, CurvatureCalculator::SensorType::qualisys);
//     VectorXd p = VectorXd::Zero(3*st_params.num_segments);
//     srl::State state = st_params.getBlankState();
//     srl::State state_ref1 = st_params.getBlankState();
//     srl::State state_ref2 = st_params.getBlankState();


//     double t = 0;
//     double dt = 0.07;
//     double time = 20;
    

//     mpc.toggle_log();
//     while (t < time/2){
        
//         state_ref1.q << 0.222, -0.116, -0.077, -0.866;
//         state_ref2 = state_ref1; 
//         int edge = 0;
//         for (int i = 0; i < mpc.Horizon + 1; i++){
//             if (t + i*dt >= time/2){   //time/2
//                 state_ref2.q << -0.133, 0.075, -0.003, 0.670;
//                 edge = i;
//                 break;
//             }
//         }

//         mpc.set_ref(state_ref1, state_ref2, edge);

        
//         t+=dt;
//         srl::sleep(dt);
//         std::cout << t << std::endl;
//     }

//     while (t < time){
        
//         state_ref1.q << -0.133, 0.075, -0.003, 0.670;
//         state_ref2 = state_ref1;

//         mpc.set_ref(state_ref1, state_ref2, 0);

        
//         t+=dt;
//         srl::sleep(dt);
//         std::cout << t << std::endl;
//     }

//     mpc.toggle_log();
//     srl::sleep(1);
//     return 1;
// }