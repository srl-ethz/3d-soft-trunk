#include <3d-soft-trunk/ControllerPCC.h>
#include <3d-soft-trunk/Simulator.h>
int main(){

    srl::State state_ref, state;

    double control_step = 0.005;
    int steps = 1;
    double time = 5;
    double kp = 50;
    double kd = 2*sqrt(kp);

    VectorXd input_linearize = VectorXd::Zero(3*st_params::num_segments);
    VectorXd ill = VectorXd::Zero(3*st_params::num_segments*st_params::sections_per_segment); //input linearize long, shortened so the statements are more readable

    VectorXd p = VectorXd::Zero(3*st_params::num_segments);
    VectorXd p_pseudo = VectorXd::Zero(2*st_params::num_segments);

    MatrixXd inv_pseudo = MatrixXd::Zero(2,3);         
    inv_pseudo << 1, -0.5, -0.5, 0, sqrt(3)/2, -sqrt(3)/2; //set up constants for the 2-to-3 solving
    MatrixXd inv = inv_pseudo.transpose()*(inv_pseudo*inv_pseudo.transpose()).inverse();

    
    for (int i = 0; i < st_params::num_segments; i++) {
        state_ref.q(2 * i) = -0.3;
        state_ref.q(2 * i + 1) = 0.;
    }
    
    /*
    for (int i = 0; i < st_params::num_segments; i++) {
        // set to have about the same curvature as a whole regardless of scale
        double rand = -2.093 / st_params::sections_per_segment / st_params::num_segments;
        for (int j = 0; j < st_params::sections_per_segment; j++){
            state_ref.q(2*i*st_params::sections_per_segment + 2*j + 1) = -rand;
            state_ref.q(2*i*st_params::sections_per_segment + 2*j ) = rand * 0.022 / 0.19;
        }
            
    }*/

    SoftTrunkModel stm = SoftTrunkModel();
    Simulator sim = Simulator(stm, control_step, steps, state);
    ControllerPCC cpcc{CurvatureCalculator::SensorType::qualisys};
    stm.updateState(state);
    cpcc.set_ref(state_ref);

   
    sim.start_log("lqr_exp");

    auto start = std::chrono::steady_clock::now();
    stm.updateState(state);

    MatrixXd A_pseudo = MatrixXd::Zero(st_params::q_size, st_params::q_size);
    for (int i = 0; i < st_params::num_segments; i++) {
        A_pseudo.col(2*i) = stm.A.col(3*i);
        A_pseudo.col(2*i+1) = stm.A.col(3*i+1) - stm.A.col(3*i+2);
    }


    Vector3d x_des = stm.ara->get_H_tip().translation();
    Vector3d dx_des = Vector3d::Zero();
    Vector3d x = stm.ara->get_H_tip().translation()+stm.ara->get_H_base().translation();
    Vector3d dx = stm.J*state.dq;
    Vector3d ddx_ref = kp*(x_des - x) + kd*(dx_des - dx);

    MatrixXd B_op = stm.J*stm.B.inverse()*stm.J.transpose();
    MatrixXd g_op = B_op*stm.J*stm.B.inverse()*stm.g;
    MatrixXd J_inv = stm.B.inverse()*stm.J.transpose()*B_op;

    VectorXd f = B_op*ddx_ref + g_op;
    VectorXd tau_null = VectorXd::Zero(st_params::q_size);
    VectorXd tau_ref = stm.J.transpose()*f + stm.K * state.q + stm.D * state.dq + (MatrixXd::Identity(st_params::q_size, st_params::q_size) - stm.J.transpose()*J_inv)*tau_null;

    
    for (double t=0; t+control_step < time; t+=control_step){
        dx = stm.J*state.dq;
        ddx_ref = kp*(x_des - x) + kd*(dx_des - dx);
        B_op = stm.J*stm.B.inverse()*stm.J.transpose();
        g_op = B_op*stm.J*stm.B.inverse()*stm.g;
        J_inv = stm.B.inverse()*stm.J.transpose()*B_op;
        f = B_op*ddx_ref + g_op;
        tau_ref = stm.J.transpose()*f + stm.K * state.q + stm.D * state.dq + (MatrixXd::Identity(st_params::q_size, st_params::q_size) - stm.J.transpose()*J_inv)*tau_null;
        p_pseudo = A_pseudo.inverse()*tau_ref;

        for (int i = 0; i < st_params::num_segments; i++){
            p.segment(3*i,3) = inv*p_pseudo.segment(2*i,2);
            if (p.segment(3*i,3).minCoeff() < 0) p.segment(3*i,3) -= p.segment(3*i,3).minCoeff()*Vector3d::Ones();
        }

        sim.simulate(p);
        sim.get_state(state);
        stm.updateState(state);
    }

    
    sim.end_log();
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Simulated " << time << "s of motion in " << elapsed.count() <<"s realtime using timestep " << control_step << "\n";
    
    //while(1); for real robot
}
