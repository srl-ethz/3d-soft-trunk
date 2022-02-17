#include "3d-soft-trunk/MPC_obstacles.h"

// derived from MPC_robust, just check there


void compute_disturbance(DM q_sim, DM q_real, DM q_dot_sim, DM q_dot_real, const SoftTrunkParameters st_params, DM &dist){

    DM dist_temp = DM::zeros(2*st_params.q_size, 1);

    dist_temp = fabs( vertcat(q_sim, q_dot_sim) - vertcat(q_real, q_dot_real) );

    dist = fmax(dist, dist_temp); 
} 

MPC_obstacles::MPC_obstacles(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(st_params, sensor_type){
    filename = "MPC_logger";

    dt = 1./15;

    // Take model
    ctrl = define_problem();   // define matrices as parameters, so we can update them without re-initalizing the problem
    solved = false; 

    //OptiAdvanced ctrl_deb = ctrl.debug(); 

    Q << 1e5*MatrixXd::Identity(st_params.q_size, st_params.q_size), MatrixXd::Zero(st_params.q_size, st_params.q_size), MatrixXd::Zero(st_params.q_size, st_params.q_size), 1e0*MatrixXd::Identity(st_params.q_size, st_params.q_size) ;

    control_thread = std::thread(&MPC_obstacles::control_loop, this);
}

void MPC_obstacles::control_loop(){
    srl::Rate r{1./dt};

    while(true){

        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        // auto start = std::chrono::steady_clock::now();

        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
            
        stm->updateState(state);
            
        if (!is_initial_ref_received){
            //only control after receiving a reference position
            continue;
        } 

        if (sensor_type != CurvatureCalculator::SensorType::simulator){
            x = stm->get_H_base().rotation()*cc->get_frame(0).rotation()*(cc->get_frame(st_params.num_segments).translation()-cc->get_frame(0).translation());
        }
        else {
            x = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-1).translation();
        }


        // need a coversion DM to MatrixXd and viceversa
        
        get_state_space(stm->B, stm->c, stm->g, stm->K, stm->D, stm->A_pseudo, sp_A, sp_B, sp_w, dt);


        // auto start = std::chrono::steady_clock::now();

        P_old = P;
         
        res = solveDARE(sp_A, sp_B, Q, R, P); 

        if (res != 0){
            std::cout << "Error in solving DARE, using old solution" << std::endl; 
            P = P_old; 
        }

        K = -(sp_B.transpose()*P*sp_B + R).inverse()*(sp_B.transpose()*P*sp_A); 

        // std::cout << "size of matrix K : " << K.rows() << "x" << K.cols()<< std::endl; 
        // std::cout << K << std::endl;

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
        
        // auto start = std::chrono::steady_clock::now();

        std::copy(sp_A.data(), sp_A.data() + sp_A.size(), sp_A_temp.ptr());
        std::copy(sp_B.data(), sp_B.data() + sp_B.size(), sp_B_temp.ptr());
        std::copy(sp_w.data(), sp_w.data() + sp_w.size(), sp_w_temp.ptr());
        std::copy(K.data(), K.data() + K.size(), K_temp.ptr());
        //std::copy(x_ref.data(), x_ref.data() + x_ref.size(), x_r_temp.ptr());
        std::copy(traj_ref.data(), traj_ref.data() + traj_ref.size(), tr_r_temp.ptr()); 
        std::copy(state.q.data(), state.q.data() + state.q.size(), q_0_temp.ptr());
        std::copy(state.dq.data(), state.dq.data() + state.dq.size(), q_dot_0_temp.ptr());

        //std::cout << K_temp << std::endl;

        for (int i = 0; i < Horizon+1; i++){
            q_0_large(Slice(),i) = q_0_temp; 
            q_dot_0_large(Slice(),i) = q_dot_0_temp; 
        }


        // solve problem

        if (!solved){
            u_temp = DM::zeros(2*st_params.num_segments,1);
            q_old = q_0_temp;
            q_dot_old = q_dot_0_temp;  
        }

        for (int i = 0; i < Horizon; i++){
            u_large(Slice(),i) = u_temp; 
        }

        // function to find disturbance with q_old and q_0_temp

        // compute_disturbance(q_old, q_0_temp, q_dot_old, q_dot_0_temp, st_params, disturbance); 

        // std::cout << "disturbance = " << disturbance.T() << std::endl; 

        ctrl.set_value(A, sp_A_temp);   // need to define them already as global variables
        ctrl.set_value(B, sp_B_temp);
        ctrl.set_value(w, sp_w_temp); 
        //ctrl.set_value(x_r, x_r_temp);
        ctrl.set_value(tr_r, tr_r_temp);
        ctrl.set_value(q_0, q_0_temp); // q_old for simplified tube
        ctrl.set_value(q_dot_0, q_dot_0_temp); // q_dot_old for simplified tube
        ctrl.set_value(u_prev, u_temp); 

        ctrl.set_initial(q, q_0_large);
        ctrl.set_initial(q_dot, q_dot_0_large);
        //ctrl.set_initial(u, u_large); 

        OptiSol sol = ctrl.solve(); 
         
        
        if (!solved){
            solved = true; 
        }

        //ctrl.set_initial(sol.value_variables());   // to be investigated, as it is doesn't work

        
        //std::cout << "complete u : " <<sol.value(u) << std::endl; 
        //std::cout << "-----------------" << std::endl;
        //std::cout << stm->A_pseudo << std::endl;
        //std::cout << stm-> A << std::endl;

       
        u_temp = sol.value(u)(Slice(),0); 

        q_old = sol.value(q)(Slice(),1);
        q_dot_old = sol.value(q_dot)(Slice(),1);

        //std::cout << mtimes(sol.value(q_dot)(Slice(),Horizon).T(), sol.value(q_dot)(Slice(),Horizon)) << std::endl; 

        //std::cout << "solution :" << u_temp << std::endl;


        // provide robust input

        v_temp = vertcat(sol.value(q)(Slice(),0), sol.value(q_dot)(Slice(),0) ); 
        x_temp = vertcat(q_0_temp, q_dot_0_temp);

        // std::cout << "v : " << v_temp << std::endl;
        // std::cout << "x : " << x_temp << std::endl;

        u_robust = u_temp + robust_correction(mtimes(K_temp, (x_temp - v_temp)));

        std::cout << "u_temp = " << u_temp << " -- u_robust = " << u_robust << std::endl; 
        

        p_temp = Eigen::VectorXd::Map(DM::densify(u_robust).nonzeros().data(),2*st_params.num_segments,1 ); 

        ///// DEBUG
        /*
        std::cout << "DEBUG" << std::endl; 
        std::cout << stm->A_pseudo.size() << std::endl;   // size 16? 
        std::cout << p_temp.size() << std::endl;   // size 2
        */
        /////

        p = stm->pseudo2real(p_temp ); //+ gravity_compensate(state)/2

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

Opti MPC_obstacles::define_problem(){
    
    Opti prob = casadi::Opti();

    q = prob.variable(st_params.q_size, Horizon+1);
    q_dot = prob.variable(st_params.q_size, Horizon+1);
    u = prob.variable(2*st_params.num_segments, Horizon);  //pressure

    // MX slack = prob.variable(2*st_params.q_size, Horizon);   // soft mpc

    q_0 = prob.parameter(st_params.q_size,1);
    q_dot_0 = prob.parameter(st_params.q_size,1); 

    // define as parameters the model matrices and set point

    A = prob.parameter(2*st_params.q_size, 2*st_params.q_size);
    B = prob.parameter(2*st_params.q_size, 2*st_params.num_segments);
    w = prob.parameter(2*st_params.q_size, 1); 

    //x_r = prob.parameter(3,1); 
    tr_r = prob.parameter(3, Horizon); 

    u_prev = prob.parameter(2*st_params.num_segments,1); 



    MX J = 0;
    DM A1 = DM::zeros(2*st_params.q_size, st_params.q_size);
    DM b1 = DM::ones(2*st_params.q_size,1);                     // b1 and b2 for initial set
    DM A2 = DM::zeros(2*st_params.q_size, st_params.q_size);
    DM b2 = DM::ones(2*st_params.q_size,1);
    MX T1 = MX::zeros(3,1);
    MX T2 = MX::zeros(st_params.q_size,1);
    //MX T3 = MX::ones(st_params.q_size,1)*0.5; 

    DM b3 = DM::zeros(2*st_params.q_size,1);    // b3 for feasible set (MPC_constraints_finder)

    A1(0,0) = 1;
    A1(1,0) = -1;
    A1(2,1) = 1; 
    A1(3,1) = -1;
    A1(4,2) = 1;
    A1(5,2) = -1;
    A1(6,3) = 1;
    A1(7,3) = -1;

    A2 = A1; 

    //std::cout << A1 << std::endl; 
    // Simulation
    // 0.00942188, 0.00552891, 0.0277994, 0.0238107, 1.2437, 0.944875, 4.58187, 3.41322
    // 0.00678449, 0.00518904, 0.0217856, 0.0206506, 0.722802, 0.791403, 2.51938, 3.51999
    // 0.00745796, 0.0051715, 0.0193878, 0.0199429, 0.730912, 0.851556, 3.15132, 3.63464
    // 0.00595452, 0.00610513, 0.0229777, 0.0228444, 0.950628, 1.09753, 3.28104, 4.09674
    // 0.00646077, 0.00610885, 0.0199081, 0.0255239, 0.736116, 1.01822, 3.21705, 4.20152

    // Real
    // 0.114308, 0.130749, 0.39742, 0.416883, 2.33043, 3.00624, 6.40119, 7.64612
    // 0.0852791, 0.111846, 0.419061, 0.413883, 3.25302, 3.08219, 7.16393, 7.14721
    // 0.254572, 0.133217, 0.442591, 0.410043, 4.59241, 3.55436, 9.93646, 8.1169

    // b1 = {0.002, 0.002, 0.001, 0.001, 0.003, 0.003, 0.003, 0.003};  
    // b2 = {0.15, 0.15, 0.15, 0.15, 0.5, 0.5, 0.5, 0.5};

    b1 = {0.004, 0.004, 0.002, 0.002, 0.006, 0.006, 0.006, 0.006};  
    b2 = {0.15, 0.15, 0.15, 0.15, 0.5, 0.5, 0.5, 0.5};

    //b3 = {1.20, 1.47, 0.17, 0.64, 1.04, 1.54, 1.38, 1.51};   // upper, - lower
    //b3 = {1.47, 1.20, 0.64, 0.17, 1.54, 1.04, 1.51, 1.38};   // -lower, upper   works
    //b3 = {0.78, 1.33, 0.48, -0.21, 1.52, 1.15, 1.16, 1.55};    // upper, -lower   works
    // b3 = {1.30, 1.29, 1.26, 1.43, 1.40, 1.34, -1.03, 1.04}; 

    MX p_min = MX::ones(2*st_params.num_segments,1)*-500;
    MX p_max = MX::ones(2*st_params.num_segments,1)*500;

    MX Du = MX::ones(2*st_params.num_segments,1)*20;   // 8 for simulation, 20 real

    MX end_effector = MX::zeros(3,1); 

    T1 = fabs(tr_r(Slice(), Horizon-1));
    //T1 = tr_r(Slice(), Horizon); 
    T2 = MX::zeros(st_params.q_size,1);  //terminal condition with delta formulation

    MX Q = MX::eye(3)*5e2; 
    MX Q2 = MX::eye(st_params.q_size)*1e-4; //-8 before
    MX R = MX::eye(2*st_params.num_segments)*1e-10;

    MX thetax = MX::zeros(2,1);
    MX thetay = MX::zeros(2,1);
    MX length1 = MX::zeros(2,1);
    MX length2 = MX::zeros(2,1);

    std::cout << "Variables initialized" << std::endl; 

    int k, kk;

    // MX obstacle = MX::zeros(3,1);
    // obstacle(0,0) = 0;
    // obstacle(1,0) = 0.14;
    // obstacle(2,0) = -0.24;  

    // obstacle(0,0) = 0;
    // obstacle(1,0) = 0.06;
    // obstacle(2,0) = -0.24;  


    // use delta-formulation with state reference  <<<<--------------<<<<<<<<<<<<----------<<<<<<<<<<<<----------
    //end_effector = MX::zeros(3,1);

    J += mtimes((u(Slice(),0)-u_prev).T(), mtimes(R, (u(Slice(),0)-u_prev))); 

    for (k = 0; k < Horizon; k++) 
    {
        // J += mtimes((q(Slice(),k)-q_r).T(), mtimes(Q, (q(Slice(),k)-q_r)));   // probaly Slice(0,st_params.q_size,1) has same effect
        J += mtimes((q_dot(Slice(),k)).T(), mtimes(Q2, (q_dot(Slice(),k))));    // keep limit on speeds 
        J += mtimes((u(Slice(),k)).T(), mtimes(R, (u(Slice(),k)))); 
        if (k>1){
            J += mtimes((u(Slice(),k)-u(Slice(),k-1)).T(), mtimes(R, (u(Slice(),k)-u(Slice(),k-1)))); 
        

            for (kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

                thetax(kk) = q(2*kk,k); 
                thetay(kk) = q(2*kk+1,k);
                length1(kk) = -st_params.lengths[2*kk]; 
                length2(kk) = -st_params.lengths[2*kk+1];
                
                // end_effector += ee_position(q(2*kk,k), q(2*kk+1,k), -st_params.lengths[2*kk] - st_params.lengths[2*kk+1]); 
            }

            end_effector = ee_position(thetax, thetay, length1, length2); 

            J += mtimes((end_effector-tr_r(Slice(),k-1)).T(), mtimes(Q, (end_effector-tr_r(Slice(),k-1)))); 

        }

        //J += mtimes(u(Slice(),k).T(), mtimes(R, u(Slice(),k)));

        // J += 10* mtimes((slack(Slice(),k)).T(), slack(Slice(),k));  // soft formulation (can't use linear, norm1 doesn't work)

        // J += 5e1 / exp( 1e3* mtimes( (end_effector - obstacle(Slice(),0)).T(), (end_effector - obstacle(Slice(),0)) ));   // to account for one obstacle
        // J += 5e1 / exp( 1e3* mtimes( (end_effector - obstacle(Slice(),1)).T(), (end_effector - obstacle(Slice(),1)) ));

    }


    //end_effector = MX::zeros(3,1);    // needed also later for terminal constraint

    for (kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

        thetax(kk) = q(2*kk,Horizon); 
        thetay(kk) = q(2*kk+1,Horizon);
        length1(kk) = -st_params.lengths[2*kk]; 
        length2(kk) = -st_params.lengths[2*kk+1];
        
        // end_effector += ee_position(q(2*kk,Horizon), q(2*kk+1,Horizon), -st_params.lengths[2*kk] - st_params.lengths[2*kk+1]); 
    }

    end_effector = ee_position(thetax, thetay, length1, length2);

    
    J += mtimes((end_effector-tr_r(Slice(),Horizon-1)).T(), mtimes(Q, (end_effector-tr_r(Slice(), Horizon-1))));

    //J += mtimes(q(Slice(),Horizon).T(), mtimes(Q, q(Slice(),Horizon))); 
    //J += mtimes(q_dot(Slice(),Horizon).T(), mtimes(Q, q_dot(Slice(),Horizon)));  // terminal cost
    
    prob.minimize(J);

    std::cout << "Cost function initialized" << std::endl;

    // Slice function (0,2) select 0-1 only

    // prob.subject_to(q(Slice(),0) == q_0);
    // prob.subject_to(q_dot(Slice(),0) == q_dot_0);
    
 
    MX Q0 = MX::zeros(2*st_params.q_size,1);
    MX Q_DOT_0 = MX::zeros(2*st_params.q_size,1);
    for (int i = 0; i < 2*st_params.q_size; i++){
        Q0(i,0) = pow(-1,i)*q_0(int(i/2)) + b1(i,0);
        Q_DOT_0(i,0) = pow(-1,i)*q_dot_0(int(i/2)) + b2(i,0);
    }

    prob.subject_to( mtimes(A1, q(Slice(),0)) <= Q0); 
    prob.subject_to( mtimes(A2, q_dot(Slice(),0)) <= Q_DOT_0); 


    for (k = 0; k < Horizon; k++)
    {
        prob.subject_to(q(Slice(),k+1) == mtimes(A(Slice(0, st_params.q_size), Slice(0, st_params.q_size)), q(Slice(),k)) + mtimes(A(Slice(0, st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)), q_dot(Slice(),k)) + mtimes(B(Slice(0, st_params.q_size), Slice()), u(Slice(),k)) + w(Slice(0,st_params.q_size))); 
        prob.subject_to(q_dot(Slice(),k+1) == mtimes(A(Slice(st_params.q_size, 2*st_params.q_size), Slice(0, st_params.q_size)), q(Slice(),k)) + mtimes(A(Slice(st_params.q_size,2*st_params.q_size), Slice(st_params.q_size, 2*st_params.q_size)), q_dot(Slice(),k)) + mtimes(B(Slice(st_params.q_size, 2*st_params.q_size), Slice()), u(Slice(),k)) + w(Slice(st_params.q_size, 2*st_params.q_size)));
        // prob.subject_to(mtimes(A1, q(Slice(),k)) <= b3 + slack(Slice(),k));
        //prob.subject_to(mtimes(A2, q_dot(Slice(),k)) <= b2);
        // some stuff on pressure
    }

    prob.subject_to(-Du <= (u(Slice(),0)-u_prev) <= Du); 
    for (k = 0; k < Horizon/4; k++){
        prob.subject_to(-Du <= (u(Slice(),k+1)-u(Slice(),k)) <= Du); 
        prob.subject_to(p_min < u(Slice(), k) < p_max);
    }

    // terminal constraint


    //prob.subject_to( fabs(end_effector) >= 0.5*T1); 
    prob.subject_to(q_dot(Slice(),Horizon) == T2); 
    //prob.subject_to( mtimes(q_dot(Slice(),Horizon).T(), q_dot(Slice(),Horizon)) < 1e-23); 
    

    //prob.subject_to(end_effector == T1);
    // prob.subject_to(q_dot(Slice(),Horizon) == T2);

    std::cout << "Constraints initialized" << std::endl;


    Dict opts_dict=Dict();   // to stop printing out solver data
    //opts_dict["ipopt.tol"] = 1e-4;
    opts_dict["ipopt.acceptable_tol"] = 1e2;
    opts_dict["ipopt.print_level"] = 0; 
    opts_dict["ipopt.sb"] = "yes";
    opts_dict["print_time"] = 0;

    prob.solver("ipopt", opts_dict);

    std::cout<< "MPC problem initialized correctly."<< std::endl; 

    return prob; 
}

void MPC_obstacles::get_state_space(MatrixXd B, MatrixXd c, MatrixXd g, MatrixXd K, MatrixXd D, MatrixXd A, MatrixXd &sp_A, MatrixXd &sp_B, MatrixXd &sp_w, double Ts){
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


MatrixXd MPC_obstacles::matrix_exponential(MatrixXd A, int size){

    Ad = MatrixXd::Identity(size,size) + A + (A*A)/2 + (A*A*A)/6 + (A*A*A*A)/24 + (A*A*A*A*A)/120;   //up to 5th order
    return Ad; 
}


int MPC_obstacles::solveDARE(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R, MatrixXd &X){
    
    At = A.transpose();
    Bt = B.transpose();
    X = Q;

    double EPS = 0.001;

    double d = EPS;
    for (int ii=0; ii < 1000 && d >= EPS; ++ii)
    {
        MatrixXd Xp = X;
        
        X = Q + At*(X)*A - At*(X)*B*(Bt*(X)*B+R).inverse()*Bt*(X)*A;
        d = (X - Xp).array().abs().sum();
    }
    
    return d >= EPS;
}


MX MPC_obstacles::Rotx(MX theta){

    rot = MX::zeros(3,3); 
    rot(0,0) = 1;
    rot(1,1) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(1,2) = -sin(theta);
    rot(2,1) = sin(theta); 

    // for (int i = 0; i<3; i++){
    //     std::cout << "rotation x diagonal: " << rot(i,i) << std::endl;
    // }

     

    return rot; 
}

MX MPC_obstacles::Roty(MX theta){

    rot = MX::zeros(3,3); 
    rot(1,1) = 1;
    rot(0,0) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(0,2) = sin(theta);
    rot(2,0) = -sin(theta); 

    //std::cout << "rotation y : " << rot << std::endl; 

    return rot; 
}


MX MPC_obstacles::ee_position(MX thetax, MX thetay, MX length1, MX length2){    // computes position of end effector for each segment (simplified version)



    len1 = MX::zeros(3,2);
    len2 = MX::zeros(3,2);

    len1(2,Slice()) = 0.98*length1;
    len2(2,Slice()) = length2; 


    totRot = mtimes(Roty(-thetax(0)/2),Rotx(-thetay(0)/2));
    

    inter = mtimes(totRot, len1(Slice(),0)); 
    totRot = mtimes(totRot, mtimes(Roty(-thetax(0)/2), Rotx(-thetay(0)/2))); 
    inter += mtimes(totRot, len2(Slice(),0)); 
    totRot = mtimes(totRot, mtimes(Roty(-thetax(1)/2),Rotx(-thetay(1)/2)));  
    inter += mtimes(totRot, len1(Slice(),1));

     
    return inter;  

}


void MPC_obstacles::set_ref(const MatrixXd refx){
    std::lock_guard<std::mutex> lock(mtx);
    // x_r = refx;
    // if (x_ref.colwise().norm() > 0.27) {
    //     x_r = 0.27*x_ref.colwise().normalized();
    // }

    //std::cout << "reference : " << x_r << std::endl;

    length = refx.cols();
    int i; 
    if (length == 0){
        ; 
    }
    else if (length <= Horizon){
        for (i = 0; i < length; i++){
            this->traj_ref.col(i) = refx.col(i); 
        }
        for (i = length; i< Horizon; i++){
            this->traj_ref.col(i) = refx.col(length-1); 
        }
    }
    else{
        for (i = 0; i< Horizon; i++){
            this->traj_ref.col(i) = refx.col(i);
        }
    }


    this->x_ref = traj_ref.col(0); 

    if (!is_initial_ref_received)
        is_initial_ref_received = true;

    // std::cout << "Trajectory : " << traj_ref << std::endl; 
}

DM MPC_obstacles::robust_correction(DM U){     // use normalization ? 

    if (U.is_zero()){
        return U; 
    }

    // std::cout << "Correction from " << U.T() << std::endl; 

    auto U_new = U.get_elements(); 
    auto index  = std::minmax_element(U_new.begin(), U_new.end()); 

    double limit = 0;
    if (abs(*index.second) >= abs(*index.first)){
        limit = abs(*index.second); 
    }
    else{
        limit = abs(*index.first); 
    }

    if (limit < 6){
        return U; 
    }

    //float coeff = (5 - (-5)) / (*index.second - *index.first);   // 5 for simulation
    float coeff = (10 - (-10)) / (limit - (-limit)); 

    int i; 
    for (i = 0; i < U.rows() ; i++ ){
        U(i) = -10 + coeff*(U(i)- (-limit)); 
    } 



    // std::cout << "to "<< U.T() << std::endl; 

    return U; 

}