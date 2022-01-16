#include "3d-soft-trunk/MPC.h"

// works in end-effector space

MPC::MPC(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(st_params, sensor_type){
    filename = "MPC_logger";

    dt = 1./15;

    // Take model
    ctrl = define_problem();   // define matrices as parameters, so we can update them without re-initalizing the problem
    solved = false; 

    control_thread = std::thread(&MPC::control_loop, this);
}

void MPC::control_loop(){
    srl::Rate r{1./dt};

    while(true){

        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
            
        stm->updateState(state);
            
        if (!is_initial_ref_received){
            //only control after receiving a reference position
            continue;
        } 

        // auto start = std::chrono::steady_clock::now();

        // Vector3d ee_x = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-1).translation();

        // std::cout << "End-effector position : " << ee_x.transpose() << std::endl; 

        // Define state space system : https://x-engineer.org/graduate-engineering/signals-systems/control-systems/state-space-model-dynamic-system/
        // Define discrete system


        // need a coversion DM to MatrixXd and viceversa
        
        get_state_space(stm->B, stm->c, stm->g, stm->K, stm->D, stm->A_pseudo, sp_A, sp_B, sp_w, dt);

        //https://github.com/casadi/casadi/issues/2563
        //https://groups.google.com/g/casadi-users/c/npPcKItdLN8
        // DM <--> Eigen

        std::copy(sp_A.data(), sp_A.data() + sp_A.size(), sp_A_temp.ptr());
        std::copy(sp_B.data(), sp_B.data() + sp_B.size(), sp_B_temp.ptr());
        std::copy(sp_w.data(), sp_w.data() + sp_w.size(), sp_w_temp.ptr());
        std::copy(state_ref.q.data(), state_ref.q.data() + state_ref.q.size(), q_r_temp.ptr());
        std::copy(state_ref.dq.data(), state_ref.dq.data() + state_ref.dq.size(), q_dot_r_temp.ptr());
        std::copy(state.q.data(), state.q.data() + state.q.size(), q_0_temp.ptr());
        std::copy(state.dq.data(), state.dq.data() + state.dq.size(), q_dot_0_temp.ptr());

        for (int i = 0; i < Horizon+1; i++){
            q_0_large(Slice(),i) = q_0_temp; 
            q_dot_0_large(Slice(),i) = q_dot_0_temp; 
        }

        // solve problem

        if (!solved){
            u_temp = DM::zeros(2*st_params.num_segments,1); 
        }

        ctrl.set_value(A, sp_A_temp);   // need to define them already as global variables
        ctrl.set_value(B, sp_B_temp);
        ctrl.set_value(w, sp_w_temp); 
        ctrl.set_value(q_r, q_r_temp); 
        ctrl.set_value(q_dot_r, q_dot_r_temp); 
        ctrl.set_value(q_0, q_0_temp); 
        ctrl.set_value(q_dot_0, q_dot_0_temp); 
        ctrl.set_value(u_prev, u_temp); 

        ctrl.set_initial(q, q_0_large);
        ctrl.set_initial(q_dot, q_dot_0_large);

        OptiSol sol = ctrl.solve(); 
        
        if (!solved){
            solved = TRUE; 
        }

        
        //std::cout << "complete u : " <<sol.value(u) << std::endl; 
        //std::cout << "-----------------" << std::endl;
        //std::cout << stm->A_pseudo << std::endl;
        //std::cout << stm-> A << std::endl;

        u_temp = sol.value(u)(Slice(),0); 

        std::cout << "solution :" << u_temp << std::endl;

        p_temp = Eigen::VectorXd::Map(DM::densify(u_temp).nonzeros().data(),2*st_params.num_segments,1 ); 

        ///// DEBUG
        /*
        std::cout << "DEBUG" << std::endl; 
        std::cout << stm->A_pseudo.size() << std::endl;   // size 16? 
        std::cout << p_temp.size() << std::endl;   // size 2
        */
        /////

        // tau_ref = stm->A_pseudo* p_temp; 

        // // apply input                     // need to check how to do this, seems really small
        // pxy = stm->A_pseudo.inverse()*tau_ref/10000;
        // p = stm->pseudo2real(pxy);  // possibly add gravity compensation

        p = stm->pseudo2real(p_temp); 

        //std::cout << "pressure input : " << p.transpose() << std::endl; 

        if (sensor_type != CurvatureCalculator::SensorType::simulator) {actuate(p);}
        else {
            assert(simulate(p));
        }

        // auto end = std::chrono::steady_clock::now();
        // std::chrono::duration<double> elapsed = end - start;

        // counter += 1; 
        // total_time += elapsed.count(); 
        // if (counter < 5){
        //     total_time -= elapsed.count(); 
        // }
        // if (elapsed.count() > slow_execution(4)){
        //     for (int i = 0; i<5; i++){
        //         if (elapsed.count() > slow_execution(i)){
        //             for(int k = 4; k>i; k--){
        //                 slow_execution(k) = slow_execution(k-1); 
        //             }
        //             slow_execution(i) = elapsed.count(); 
        //             break; 
        //         }
        //     }
        // }

        // if (counter%1 == 0){
        //     std::cout << "==================" << std::endl; 
        //     std::cout << "Average time : " << total_time / counter << std::endl; 
        //     std::cout << "Slowest execution :" << slow_execution.transpose() << std::endl; 
        //     std::cout << "==================" << std::endl; 
        // }
        
    }
    
}

Opti MPC::define_problem(){
    
    Opti prob = casadi::Opti();

    q = prob.variable(st_params.q_size, Horizon+1);
    q_dot = prob.variable(st_params.q_size, Horizon+1);
    u = prob.variable(2*st_params.num_segments, Horizon);  //pressure

    q_0 = prob.parameter(st_params.q_size,1);
    q_dot_0 = prob.parameter(st_params.q_size,1); 

    // define as parameters the model matrices and set point

    A = prob.parameter(2*st_params.q_size, 2*st_params.q_size);
    B = prob.parameter(2*st_params.q_size, 2*st_params.num_segments);
    w = prob.parameter(2*st_params.q_size, 1); 

    q_r = prob.parameter(st_params.q_size, 1);
    q_dot_r = prob.parameter(st_params.q_size,1); 

    u_prev = prob.parameter(2*st_params.num_segments,1); 



    MX J = 0;
    MX A1 = MX::zeros(st_params.q_size, st_params.q_size);
    MX b1 = MX::ones(st_params.q_size,1);
    MX A2 = MX::zeros(st_params.q_size, st_params.q_size);
    MX b2 = MX::ones(st_params.q_size,1);
    MX T1 = MX::zeros(st_params.q_size,1);
    MX T2 = MX::zeros(st_params.q_size,1);

    MX p_min = MX::ones(2*st_params.num_segments,1)*-500;
    MX p_max = MX::ones(2*st_params.num_segments,1)*500; 

    MX Du = MX::ones(2*st_params.num_segments,1)*5; 

    //std::cout << "Delta u = " << Du << std::endl; 

    T1 = q_r; 
    T2 = q_dot_r;  //terminal condition with delta formulation

    MX Q1 = MX::eye(st_params.q_size)*1e8; 
    MX Q2 = MX::eye(st_params.q_size)*1e0; 
    MX R = MX::eye(2*st_params.num_segments)*1e-3;

    std::cout << "Variables initialized" << std::endl; 

    // use delta-formulation with state reference  <<<<--------------<<<<<<<<<<<<----------<<<<<<<<<<<<----------

    for (int k = 0; k < Horizon; k++) 
    {
        J += mtimes((q(Slice(),k)-q_r).T(), mtimes(Q1, (q(Slice(),k)-q_r)));   // probaly Slice(0,st_params.q_size,1) has same effect
        J += mtimes((q_dot(Slice(),k)-q_dot_r).T(), mtimes(Q2, (q_dot(Slice(),k)-q_dot_r)));
        J += mtimes(u(Slice(),k).T(), mtimes(R, u(Slice(),k)));     // if want to penalize input, need to use the ss one
    }

    J += mtimes((q(Slice(),Horizon)-q_r).T(), mtimes(Q1, (q(Slice(),Horizon)-q_r))); 
    J += mtimes((q_dot(Slice(),Horizon)-q_dot_r).T(), mtimes(Q2, (q_dot(Slice(),Horizon)-q_dot_r)));  // terminal cost
    
    prob.minimize(J);

    std::cout << "Cost function initialized" << std::endl;

    ///////////////// DEBUG STUFF 
    /*
    std::cout << "DEBUG" << std::endl; 
    std::cout << A.size() << std::endl;
    std::cout << A(Slice(0, st_params.q_size), Slice(0, st_params.q_size)).size() << std::endl;
    std::cout << A(Slice(0, st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)).size()<< std::endl;
    std::cout << A(Slice(st_params.q_size,2*st_params.q_size), Slice(0, st_params.q_size)).size() << std::endl;
    std::cout << A(Slice(st_params.q_size,2*st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)).size() << std::endl; 
    std::cout << B(Slice(0, st_params.q_size), Slice()).size() << std::endl;
    std::cout << B(Slice(st_params.q_size, 2*st_params.q_size), Slice()).size() << std::endl; 
    */
    //////////////////

    // Slice function (0,2) select 0-1 only

    prob.subject_to(q(Slice(),0) == q_0);
    prob.subject_to(q_dot(Slice(),0) == q_dot_0);

    for (int k = 0; k < Horizon; k++)
    {
        prob.subject_to(q(Slice(),k+1) == mtimes(A(Slice(0, st_params.q_size), Slice(0, st_params.q_size)), q(Slice(),k)) + mtimes(A(Slice(0, st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)), q_dot(Slice(),k)) + mtimes(B(Slice(0, st_params.q_size), Slice()), u(Slice(),k)) + w(Slice(0,st_params.q_size))); 
        prob.subject_to(q_dot(Slice(),k+1) == mtimes(A(Slice(st_params.q_size, 2*st_params.q_size), Slice(0, st_params.q_size)), q(Slice(),k)) + mtimes(A(Slice(st_params.q_size,2*st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)), q_dot(Slice(),k)) + mtimes(B(Slice(st_params.q_size, 2*st_params.q_size), Slice()), u(Slice(),k)) + w(Slice(st_params.q_size, 2*st_params.q_size)));
        //prob.subject_to(mtimes(A1, q(Slice(),k)) <= b1);
        //prob.subject_to(mtimes(A2, q_dot(Slice(),k)) <= b2);
        // some stuff on pressure
    }

    prob.subject_to(-Du <= (u(Slice(),0)-u_prev) <= Du); 
    for (int k = 0; k < Horizon/4; k++){
        prob.subject_to(-Du <= (u(Slice(),k+1)-u(Slice(),k)) <= Du); 
        prob.subject_to(p_min < u(Slice(), k) < p_max);   // need to put it here, otherwise with some configurations it doesn't work (since model is fixed along prediction)
    }

    // terminal constraint
    prob.subject_to(q(Slice(),Horizon) == T1);
    prob.subject_to(q_dot(Slice(),Horizon) == T2);

    std::cout << "Constraints initialized" << std::endl;

    Dict opts_dict=Dict();   // to stop printing out solver data
    opts_dict["ipopt.print_level"] = 0;
    opts_dict["ipopt.sb"] = "yes";
    opts_dict["print_time"] = 0;

    prob.solver("ipopt", opts_dict);
    //prob.solver("ipopt"); 

    std::cout<< "MPC problem initialized correctly."<< std::endl; 

    return prob; 
}

void MPC::get_state_space(MatrixXd B, MatrixXd c, MatrixXd g, MatrixXd K, MatrixXd D, MatrixXd A, MatrixXd &sp_A, MatrixXd &sp_B, MatrixXd &sp_w, double Ts){
    // to get state space matrices for state evolution, given dynamics equation
    // already discretized
    // missing the constant coriolis + stuff, check paper

    sp_A_c << MatrixXd::Zero(st_params.q_size,st_params.q_size), MatrixXd::Identity(st_params.q_size, st_params.q_size), - B.inverse() * K, -B.inverse() * D;
    sp_B_c << MatrixXd::Zero(st_params.q_size, 2*st_params.num_segments), B.inverse()*A;
    sp_w_c << MatrixXd::Zero(st_params.q_size,1), - B.inverse()*(c+g); 
    
    // sp_A = MatrixXd::Identity(2*st_params.q_size, 2*st_params.q_size) + Ts*sp_A; 
    // sp_B = Ts*sp_B; 
    // sp_w = Ts*sp_w; 

    sp_A = matrix_exponential(sp_A_c*Ts, 2*st_params.q_size); 
    sp_B = sp_A_c.inverse() * (sp_A - MatrixXd::Identity(2*st_params.q_size, 2*st_params.q_size)) * sp_B_c; 
    sp_w = Ts*sp_w; 

}


MatrixXd MPC::matrix_exponential(MatrixXd A, int size){

    Ad = MatrixXd::Identity(size,size) + A + (A*A)/2 + (A*A*A)/6 + (A*A*A*A)/24 + (A*A*A*A*A)/120;   //up to 5th order
    return Ad; 
}