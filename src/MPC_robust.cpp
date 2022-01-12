#include "3d-soft-trunk/MPC_robust.h"

// Need to solve DARE
// https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c
// https://github.com/wcaarls/grl/blob/master/addons/lqr/src/lqr.cpp
// https://www.mathworks.com/help/control/ref/idare.html
// SLICOT

// Missing:
// retrieve noise bounds for simlation and real experiment 
// simulation : 0.5060399090697567 1.1695533650831549 1.4095219511871493 1.053521060609602

// with new function, one-step simulation error
// 0.00942188, 0.00552891, 0.0277994, 0.0238107, 1.2437, 0.944875, 4.58187, 3.41322
// 0.00678449, 0.00518904, 0.0217856, 0.0206506, 0.722802, 0.791403, 2.51938, 3.51999
// 0.00745796, 0.0051715, 0.0193878, 0.0199429, 0.730912, 0.851556, 3.15132, 3.63464
// 0.00669639, 0.00586572, 0.0271501, 0.0194003, 0.853697, 0.811016, 3.76215, 3.34119

// define somehow a minimum robust invariant set
// change initial constraint with this set and design tube-MPC control law

// now correction is too strong, need to somehow limit it. It's what the restricted set would do. 

void compute_disturbance(DM q_sim, DM q_real, DM q_dot_sim, DM q_dot_real, const SoftTrunkParameters st_params, DM &dist){

    DM dist_temp = DM::zeros(2*st_params.q_size, 1);

    dist_temp = fabs( vertcat(q_sim, q_dot_sim) - vertcat(q_real, q_dot_real) );

    dist = fmax(dist, dist_temp); 
} 

MPC_robust::MPC_robust(const SoftTrunkParameters st_params, CurvatureCalculator::SensorType sensor_type) : ControllerPCC::ControllerPCC(st_params, sensor_type){
    filename = "MPC_logger";

    dt = 1./20;

    // Take model
    ctrl = define_problem();   // define matrices as parameters, so we can update them without re-initalizing the problem
    solved = false; 

    //OptiAdvanced ctrl_deb = ctrl.debug(); 

    control_thread = std::thread(&MPC_robust::control_loop, this);
}

void MPC_robust::control_loop(){
    srl::Rate r{1./dt};

    while(true){

        r.sleep();
        std::lock_guard<std::mutex> lock(mtx);

        auto start = std::chrono::steady_clock::now();

        if (sensor_type != CurvatureCalculator::SensorType::simulator) cc->get_curvature(state);
            
        stm->updateState(state);
            
        if (!is_initial_ref_received){
            //only control after receiving a reference position
            continue;
        } 

        x = stm->get_H_base().rotation()*stm->get_H(st_params.num_segments-1).translation();  // needed to fill global variable 


        // need a coversion DM to MatrixXd and viceversa
        
        get_state_space(stm->B, stm->c, stm->g, stm->K, stm->D, stm->A_pseudo, sp_A, sp_B, sp_w, dt);

        MatrixXd P, P_old; 
        MatrixXd R = 0.0001*MatrixXd::Identity(2*st_params.num_segments, 2*st_params.num_segments);
        MatrixXd Q = 100000*MatrixXd::Identity(2*st_params.q_size, 2*st_params.q_size);
        Q.block(st_params.q_size, st_params.q_size, st_params.q_size, st_params.q_size) *= 0.001; // reduce cost for velocity
        int res; 

        // auto start = std::chrono::steady_clock::now();

        P_old = P;
         
        res = solveDARE(sp_A, sp_B, Q, R, P); 

        if (res != 0){
            std::cout << "Error in solving DARE, using old solution" << std::endl; 
            P = P_old; 
        }

        MatrixXd K = -(sp_B.transpose()*P*sp_B + R).inverse()*(sp_B.transpose()*P*sp_A); 

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


        //https://github.com/casadi/casadi/issues/2563
        //https://groups.google.com/g/casadi-users/c/npPcKItdLN8
        // DM <--> Eigen
        

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
        ctrl.set_value(q_0, q_0_temp); // q_old for tube
        ctrl.set_value(q_dot_0, q_dot_0_temp); // q_dot_old for tube
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

        DM v_temp = vertcat(sol.value(q)(Slice(),0), sol.value(q_dot)(Slice(),0) ); 
        DM x_temp = vertcat(q_0_temp, q_dot_0_temp);

        // std::cout << "v : " << v_temp << std::endl;
        // std::cout << "x : " << x_temp << std::endl;

        DM u_robust = u_temp + robust_correction(mtimes(K_temp, (x_temp - v_temp)));

        //std::cout << "u_temp = " << u_temp << " -- u_robust = " << u_robust << std::endl; 
        

        p_temp = Eigen::VectorXd::Map(DM::densify(u_robust).nonzeros().data(),2*st_params.num_segments,1 ); 

        ///// DEBUG
        /*
        std::cout << "DEBUG" << std::endl; 
        std::cout << stm->A_pseudo.size() << std::endl;   // size 16? 
        std::cout << p_temp.size() << std::endl;   // size 2
        */
        /////

        p = stm->pseudo2real(p_temp ); //+ gravity_compensate(state)/2

        // std::cout << "pressure input : " << p.transpose() << std::endl; 

        if (sensor_type != CurvatureCalculator::SensorType::simulator) {actuate(p);}
        else {
            assert(simulate(p));
        }

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        counter += 1; 
        total_time += elapsed.count(); 
        if (counter < 5){
            total_time -= elapsed.count(); 
        }
        if (elapsed.count() > slow_execution(4)){
            for (int i = 0; i<5; i++){
                if (elapsed.count() > slow_execution(i)){
                    for(int k = 4; k>i; k--){
                        slow_execution(k) = slow_execution(k-1); 
                    }
                    slow_execution(i) = elapsed.count(); 
                    break; 
                }
            }
        }

        if (counter%1 == 0){
            std::cout << "==================" << std::endl; 
            std::cout << "Average time : " << total_time / counter << std::endl; 
            std::cout << "Slowest execution :" << slow_execution.transpose() << std::endl; 
            std::cout << "==================" << std::endl; 
        }
        
    }
    
}

Opti MPC_robust::define_problem(){
    
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

    //x_r = prob.parameter(3,1); 
    tr_r = prob.parameter(3, Horizon+1); 

    u_prev = prob.parameter(2*st_params.num_segments,1); 



    MX J = 0;
    DM A1 = DM::zeros(2*st_params.q_size, st_params.q_size);
    DM b1 = DM::ones(2*st_params.q_size,1);
    DM A2 = DM::zeros(2*st_params.q_size, st_params.q_size);
    DM b2 = DM::ones(2*st_params.q_size,1);
    MX T1 = MX::zeros(3,1);
    MX T2 = MX::zeros(st_params.q_size,1);
    //MX T3 = MX::ones(st_params.q_size,1)*0.5; 

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
    // 0.00942188, 0.00552891, 0.0277994, 0.0238107, 1.2437, 0.944875, 4.58187, 3.41322
    // 0.00678449, 0.00518904, 0.0217856, 0.0206506, 0.722802, 0.791403, 2.51938, 3.51999
    // 0.00745796, 0.0051715, 0.0193878, 0.0199429, 0.730912, 0.851556, 3.15132, 3.63464

    b1 = {0.01, 0.01, 0.006, 0.006, 0.03, 0.03, 0.03, 0.03};
    b2 = {1.3, 1.3, 1.0, 1.0, 4.6, 4.6, 3.7, 3.7};

    MX p_min = MX::ones(2*st_params.num_segments,1)*-500;
    MX p_max = MX::ones(2*st_params.num_segments,1)*500;

    MX Du = MX::ones(2*st_params.num_segments,1)*8;   // 8~10 seems to achieve the best result with 20Hz rate

    MX end_effector = MX::zeros(3,1); 

    T1 = fabs(tr_r(Slice(), Horizon));
    //T1 = tr_r(Slice(), Horizon); 
    T2 = MX::zeros(st_params.q_size,1);  //terminal condition with delta formulation

    MX Q = MX::eye(3)*1; 
    MX Q2 = MX::eye(st_params.q_size)*1e-12; //-8 before
    MX R = MX::eye(2*st_params.num_segments)*1e-14;

    MX thetax = MX::zeros(2,1);
    MX thetay = MX::zeros(2,1);
    MX length1 = MX::zeros(2,1);
    MX length2 = MX::zeros(2,1);

    std::cout << "Variables initialized" << std::endl; 

    int k, kk;

    // use delta-formulation with state reference  <<<<--------------<<<<<<<<<<<<----------<<<<<<<<<<<<----------
    //end_effector = MX::zeros(3,1);

    //J += mtimes((u(Slice(),0)-u_prev).T(), mtimes(R, (u(Slice(),0)-u_prev))); 

    for (k = 0; k < Horizon; k++) 
    {
        // J += mtimes((q(Slice(),k)-q_r).T(), mtimes(Q, (q(Slice(),k)-q_r)));   // probaly Slice(0,st_params.q_size,1) has same effect
        J += mtimes((q_dot(Slice(),k)).T(), mtimes(Q2, (q_dot(Slice(),k))));    // keep limit on speeds
        //J += mtimes((u(Slice(),k+1)-u(Slice(),k)).T(), mtimes(R, (u(Slice(),k+1)-u(Slice(),k)))); 

        for (kk = 0; kk < st_params.num_segments*st_params.sections_per_segment; kk++){

            thetax(kk) = q(2*kk,k); 
            thetay(kk) = q(2*kk+1,k);
            length1(kk) = -st_params.lengths[2*kk]; 
            length2(kk) = -st_params.lengths[2*kk+1];
            
            // end_effector += ee_position(q(2*kk,k), q(2*kk+1,k), -st_params.lengths[2*kk] - st_params.lengths[2*kk+1]); 
        }

        end_effector = ee_position(thetax, thetay, length1, length2); 

        J += mtimes((end_effector-tr_r(Slice(),k)).T(), mtimes(Q, (end_effector-tr_r(Slice(),k)))); 

        //J += mtimes(u(Slice(),k).T(), mtimes(R, u(Slice(),k)));
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

    
    J += mtimes((end_effector-tr_r(Slice(),Horizon)).T(), mtimes(Q, (end_effector-tr_r(Slice(), Horizon))));

    //J += mtimes(q(Slice(),Horizon).T(), mtimes(Q, q(Slice(),Horizon))); 
    //J += mtimes(q_dot(Slice(),Horizon).T(), mtimes(Q, q_dot(Slice(),Horizon)));  // terminal cost
    
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
        //prob.subject_to(mtimes(A1, q(Slice(),k)) <= b1);
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
    //prob.subject_to(q_dot(Slice(),Horizon) == T2); 
    //prob.subject_to( mtimes(q_dot(Slice(),Horizon).T(), q_dot(Slice(),Horizon)) < 1e-23); 
    

    //prob.subject_to(end_effector == T1);
    // prob.subject_to(q_dot(Slice(),Horizon) == T2);

    std::cout << "Constraints initialized" << std::endl;


    Dict opts_dict=Dict();   // to stop printing out solver data
    //opts_dict["ipopt.tol"] = 1e-4;
    opts_dict["ipopt.acceptable_tol"] = 1e4;
    opts_dict["ipopt.print_level"] = 0; 
    opts_dict["ipopt.sb"] = "yes";
    opts_dict["print_time"] = 0;
    // opts_dict["jit"] = true;
    // opts_dict["compiler"] = "shell"; 

    prob.solver("ipopt", opts_dict);

    std::cout<< "MPC problem initialized correctly."<< std::endl; 

    return prob; 
}

void MPC_robust::get_state_space(MatrixXd B, MatrixXd c, MatrixXd g, MatrixXd K, MatrixXd D, MatrixXd A, MatrixXd &sp_A, MatrixXd &sp_B, MatrixXd &sp_w, double Ts){
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


MatrixXd MPC_robust::matrix_exponential(MatrixXd A, int size){

    Ad = MatrixXd::Identity(size,size) + A + (A*A)/2 + (A*A*A)/6 + (A*A*A*A)/24 + (A*A*A*A*A)/120;   //up to 5th order
    return Ad; 
}


int MPC_robust::solveDARE(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R, MatrixXd &X){
    
    MatrixXd At = A.transpose(), Bt = B.transpose();
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


MX MPC_robust::Rotx(MX theta){

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

MX MPC_robust::Roty(MX theta){

    rot = MX::zeros(3,3); 
    rot(1,1) = 1;
    rot(0,0) = cos(theta); 
    rot(2,2) = cos(theta); 
    rot(0,2) = sin(theta);
    rot(2,0) = -sin(theta); 

    //std::cout << "rotation y : " << rot << std::endl; 

    return rot; 
}


MX MPC_robust::ee_position(MX thetax, MX thetay, MX length1, MX length2){    // computes position of end effector for each segment (simplified version)



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


void MPC_robust::set_ref(const MatrixXd refx){
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
    else if (length <= Horizon+1){
        for (i = 0; i < length; i++){
            this->traj_ref.col(i) = refx.col(i); 
        }
        for (i = length; i< Horizon+1; i++){
            this->traj_ref.col(i) = refx.col(length-1); 
        }
    }
    else{
        for (i = 0; i< Horizon+1; i++){
            this->traj_ref.col(i) = refx.col(i);
        }
    }


    this->x_ref = traj_ref.col(0); 

    if (!is_initial_ref_received)
        is_initial_ref_received = true;

    //std::cout << "Trajectory : " << traj_ref << std::endl; 
}

DM MPC_robust::robust_correction(DM U){

    if (U.is_zero()){
        return U; 
    }

    //std::cout << "From " << U.T() << std::endl; 

    auto U_new = U.get_elements(); 
    auto index  = std::minmax_element(U_new.begin(), U_new.end()); 

    float coeff = (5 - (-5)) / (*index.second - *index.first); 

    int i; 
    for (i = 0; i < U.rows() ; i++ ){
        U(i) = -5 + coeff*(U(i)- *index.first); 
    }

    //std::cout << "to "<< U.T() << std::endl; 

    return U; 

}