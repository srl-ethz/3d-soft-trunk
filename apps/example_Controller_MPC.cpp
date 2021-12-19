#include <3d-soft-trunk/ControllerPCC.h>
#include "3d-soft-trunk/MPC.h"
#include "3d-soft-trunk/MPC_ts.h"
#include <chrono>


MatrixXd x_ref(3,1);
MatrixXd trajectory(3,1);
Vector3d x;

void printer(MPC_ts& mpc){
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
    MPC_ts mpc(st_params, CurvatureCalculator::SensorType::qualisys);
    VectorXd p = VectorXd::Zero(3*st_params.num_segments);
    srl::State state = st_params.getBlankState();

    //x_ref = x_ref_center;
    x_ref(0,0) = 0.02;
    x_ref(1,0) = 0.02;
    x_ref(2,0) = -0.25;
    std::thread print_thread(printer, std::ref(mpc));

    
    double t = 0;
    double dt = 0.1;

    mpc.set_ref(x_ref);
    srl::sleep(0.5);
    // arguments to pass by reference must be explicitly designated as so
    // https://en.cppreference.com/w/cpp/thread/thread/thread
    

    mpc.toggle_log();
    while (t < 10){
        //x_ref = osc.get_objects()[0];

        trajectory(0,0) = 0.09; 
        trajectory(1,0) = -0.01; 
        trajectory(2,0) = -0.215; 
        x_ref = trajectory;

        mpc.set_ref(x_ref);
        /*osc.get_x(x);
        if ((x_ref - x).norm() < 0.07){
            freedom = true;
            osc.toggleGripper();
        }*/
        
        t+=dt;
        srl::sleep(dt);
    }

    x_ref(0,0) = 0.03;
    x_ref(1,0) = 0.03;
    x_ref(2,0) = -0.23;
    
    mpc.set_ref(x_ref);
    srl::sleep(3);
    mpc.toggle_log();
    srl::sleep(2);
    return 1;
}