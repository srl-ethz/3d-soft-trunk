#include <3d-soft-trunk/ControllerPCC.h>
#include <3d-soft-trunk/Simulator.h>
int main(){

    srl::State state_ref, state;

    double control_step = 0.005;
    int steps = 10;
    double time = 5;
    double kp = 50;
    double kd = 2*sqrt(kp);


    VectorXd p = VectorXd::Zero(3*st_params::num_segments);
    VectorXd p_pseudo = VectorXd::Zero(2*st_params::num_segments);

    
    char c;

    SoftTrunkModel stm = SoftTrunkModel();
    state.q = -0.2*VectorXd::Ones(st_params::q_size);

    Simulator sim = Simulator(stm, control_step, steps, state);
    stm.updateState(state);

   
    sim.start_log("lqr_exp");
    auto start = std::chrono::steady_clock::now();
    stm.updateState(state);


    
    Vector3d x_des;
    x_des << 0, 0.1, -0.2;
    Vector3d dx_des = Vector3d::Zero();
    Vector3d x = stm.ara->get_H_base().rotation()*stm.ara->get_H_tip().translation();
    Vector3d dx = stm.J*state.dq;
    Vector3d ddx_ref = kp*(x_des - x) + kd*(dx_des - dx);
    /*Vector3d norm = stm.J.col(0).cross(stm.J.col(1)).normalized();
    Vector3d x_des_adjusted = x_des - (x_des - x).dot(norm)*norm;*/

    MatrixXd B_op = stm.J*stm.B.inverse()*stm.J.transpose();
    MatrixXd g_op = B_op*stm.J*stm.B.inverse()*stm.g;
    MatrixXd J_inv = stm.B.inverse()*stm.J.transpose()*B_op;

    

    VectorXd f = B_op*ddx_ref + g_op;
    VectorXd tau_null = VectorXd::Zero(st_params::q_size);
    VectorXd tau_ref = stm.J.transpose()*f + stm.K * state.q + stm.D * state.dq + (MatrixXd::Identity(st_params::q_size, st_params::q_size) - stm.J.transpose()*J_inv.transpose())*tau_null;
    
    
    for (double t=0; t+control_step < time; t+=control_step){
        sim.get_state(state);
        stm.updateState(state);

        //norm = stm.J.col(0).cross(stm.J.col(1)).normalized();
        //x_des_adjusted = x_des - (x_des - x).dot(norm)*norm;
        x = stm.ara->get_H_base().rotation()*stm.ara->get_H_tip().translation();
        dx = stm.J*state.dq;
        ddx_ref = kp*(x_des - x) + kd*(dx_des - dx);
        B_op = stm.J*stm.B.inverse()*stm.J.transpose();
        g_op = B_op*stm.J*stm.B.inverse()*stm.g;
        J_inv = stm.B.inverse()*stm.J.transpose()*B_op;
        f = B_op*ddx_ref + g_op;
        tau_ref = stm.J.transpose()*f + stm.K * state.q + stm.D * state.dq + (MatrixXd::Identity(st_params::q_size, st_params::q_size) - stm.J.transpose()*J_inv.transpose())*tau_null;
        p_pseudo = stm.A_pseudo.inverse()*tau_ref;

        p = stm.pseudo2real(p_pseudo);
        sim.get_state(state);
        std::cout << "\n----------\n Pressures: \n" << p.transpose() << "\n----------\n dq:" << state.dq.transpose();
        sim.simulate(p);
        std::cin >> c;
    }

    
    sim.end_log();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulated " << time << "s of motion in " << elapsed.count() <<"s realtime using timestep " << control_step << "\n";
    
    //while(1); for real robot
}
